"""Optional native evidence sidecar. No Python ROS executor or HTTP polling.

Only the diagnostic request file is written outside this recording directory.
It enables bounded file snapshots, never changes navigation or ROS parameters.
"""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

EXECUTABLES = {'controller_server': 'controller_server',
               'velocity_smoother': 'velocity_smoother',
               'collision_monitor': 'collision_monitor',
               'robot_safety_node': 'robot_safety',
               'ranger_base_node': 'ranger_base_node'}
REQUEST_PATH = Path('/tmp/njrh_navlite_capture.request')


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding='utf-8')
    os.replace(temporary, path)


def bounded_command(argv, timeout=4):
    try:
        result = subprocess.run(argv, capture_output=True, timeout=timeout, text=True,
                                errors='replace', stdin=subprocess.DEVNULL)
        return {'status': 'ok' if result.returncode == 0 else 'failed',
                'returncode': result.returncode, 'stdout': result.stdout[:262144],
                'stderr': result.stderr[:8192],
                'truncated': len(result.stdout) > 262144}
    except subprocess.TimeoutExpired:
        return {'status': 'timeout'}
    except OSError as exc:
        return {'status': 'unavailable', 'error': str(exc)}


def parse_process(pid, argv):
    role = EXECUTABLES.get(Path(argv[0]).name)
    remaps, overrides, files = {}, {}, []
    for i, arg in enumerate(argv[:-1]):
        value = argv[i+1]
        if arg in ('-r', '--remap') and ':=' in value:
            key, mapped = value.split(':=', 1)
            remaps[key] = mapped
        elif arg in ('-p', '--param') and ':=' in value:
            key, val = value.split(':=', 1)
            overrides[key] = val
        elif arg == '--params-file':
            files.append(value)
    return {'pid': int(pid), 'role': role, 'executable': argv[0],
            'node': remaps.get('__node', role), 'namespace': remaps.get('__ns', '/'),
            'remaps': remaps, 'overrides': overrides, 'param_files': files}


def absolute_topic(name, proc=None):
    if not isinstance(name, str) or not name:
        return None
    namespace = (proc or {}).get('namespace', '/')
    return name if name.startswith('/') else '/' + '/'.join(
        p for p in (namespace.strip('/'), name) if p)


def resolve_topic(proc, original):
    if proc is None:
        return None
    remaps = proc.get('remaps', {})
    return absolute_topic(remaps.get(original, remaps.get(
        absolute_topic(original, proc), original)), proc)


def parameter(params, key, default=None):
    if key in params:
        return params[key]
    current = params
    for part in key.split('.'):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return current


def resolve_manifest(processes, parameters):
    """Use running command remaps/parameters; graph verification belongs to native helper.

    Known source-coded topic defaults below are labelled as such, never a claim of
    an observed publisher. Duplicate processes are a provenance gap, not selected at random.
    """
    by_role, gaps = {}, []
    for role in EXECUTABLES.values():
        matches = [p for p in processes if p['role'] == role]
        by_role[role] = matches[0] if len(matches) == 1 else None
        if len(matches) != 1:
            gaps.append(role + ': process missing or nonunique')
    def param_topic(role, key):
        proc = by_role[role]
        value = parameter(parameters.get(role, {}), key)
        if value is None and proc:
            value = proc['overrides'].get(key)
        if value is None:
            gaps.append(role + ': missing parameter ' + key)
            return None
        return resolve_topic(proc, value)
    controller = by_role['controller_server']
    smoother = by_role['velocity_smoother']
    chain = {
        'controller_output': resolve_topic(controller, 'cmd_vel'),
        'smoother_input': resolve_topic(smoother, 'cmd_vel'),
        'smoother_output': resolve_topic(smoother, 'cmd_vel_smoothed'),
        'collision_input': param_topic('collision_monitor', 'cmd_vel_in_topic'),
        'collision_output': param_topic('collision_monitor', 'cmd_vel_out_topic'),
        'safety_input': param_topic('robot_safety', 'cmd_vel_in_topic'),
        'safety_output': param_topic('robot_safety', 'cmd_vel_out_topic'),
        'chassis_input': resolve_topic(by_role['ranger_base_node'], '/cmd_vel')}
    for a, b in [('controller_output','smoother_input'), ('smoother_output','collision_input'),
                 ('collision_output','safety_input'), ('safety_output','chassis_input')]:
        if chain[a] is not None and chain[b] is not None and chain[a] != chain[b]:
            gaps.append('chain mismatch: ' + a + ' != ' + b)
    topics = {}
    def add(name, role, typ=None, critical=False, cadence='event', origin='source_default_graph_required'):
        if not name:
            return
        if name not in topics:
            topics[name] = {'name': name, 'role': role, 'roles': [role], 'critical': critical,
                            'cadence': cadence, 'binding_origin': origin}
            if typ:
                topics[name]['expected_type'] = typ
        else:
            topics[name]['roles'].append(role)
            topics[name]['critical'] |= critical
    for role, topic in chain.items():
        add(topic, role, critical=True, cadence='task', origin='running_remaps_and_parameters')
    for role, process_role, direction in [
            ('controller_output','controller_server','publisher'),
            ('smoother_input','velocity_smoother','subscriber'),
            ('smoother_output','velocity_smoother','publisher'),
            ('collision_input','collision_monitor','subscriber'),
            ('collision_output','collision_monitor','publisher'),
            ('safety_input','robot_safety','subscriber'),
            ('safety_output','robot_safety','publisher'),
            ('chassis_input','ranger_base_node','subscriber')]:
        topic, proc = chain[role], by_role[process_role]
        if topic and proc:
            expected = topics[topic].setdefault('endpoint_expected', {'publisher':[], 'subscriber':[]})
            expected[direction].append(absolute_topic(proc['node'], proc))
    add(param_topic('controller_server', 'odom_topic'), 'controller_odom',
        'nav_msgs/msg/Odometry', True, 'continuous', 'running_parameter')
    add(param_topic('ranger_base_node', 'odom_topic_name'), 'wheel_odom',
        'nav_msgs/msg/Odometry', True, 'continuous', 'running_parameter')
    collision = parameters.get('collision_monitor', {})
    sources = collision.get('observation_sources', [])
    if isinstance(sources, str):
        sources = sources.split()
    if not sources:
        gaps.append('collision_monitor: observation_sources unavailable')
    for source in sources:
        typ = parameter(collision, source + '.type', 'scan')
        if typ != 'scan':
            gaps.append('collision source ' + source + ': ' + str(typ) + ' not captured by lightweight profile')
            continue
        add(resolve_topic(by_role['collision_monitor'], parameter(collision, source + '.topic')),
            'collision_scan', 'sensor_msgs/msg/LaserScan', True, 'continuous', 'running_parameter')
    for key, role in [('api_cmd_vel_in_topic','api_terminal_teleop'),
                      ('docking_cmd_vel_in_topic','docking'),
                      ('elevator_entry_cmd_vel_in_topic','elevator_source')]:
        value = parameter(parameters.get('robot_safety', {}), key)
        if value:
            add(resolve_topic(by_role['robot_safety'], value), role)
    # Passive published evidence. No services/actions are sent. Native graph saves absence.
    for topic, role, typ, critical, cadence in [
        ('/tf', 'tf', 'tf2_msgs/msg/TFMessage', True, 'continuous'),
        ('/tf_static', 'tf_static', 'tf2_msgs/msg/TFMessage', True, 'latched'),
        ('/local_costmap/published_footprint', 'footprint','geometry_msgs/msg/PolygonStamped',True,'continuous'),
        ('/local_costmap/costmap', 'published_costmap','nav_msgs/msg/OccupancyGrid',True,'continuous'),
        ('/local_costmap/costmap_updates','published_costmap_updates','map_msgs/msg/OccupancyGridUpdate',False,'event'),
        ('/plan', 'global_reference','nav_msgs/msg/Path',False,'event'),
        ('/ranger_mini3/ordinary_local_repair_path','repaired_reference','nav_msgs/msg/Path',False,'event'),
        ('/speed_limit','speed_limit','nav2_msgs/msg/SpeedLimit',False,'event'),
        ('/ranger_base/status','chassis_status',None,True,'continuous'),
        ('/safety/status','safety_status',None,False,'event'),
        ('/safety/motion_interlock_state','motion_interlock',None,False,'event'),
        ('/safety/dock_interlock_state','bms_interlock',None,False,'event'),
        ('/localization/bridge_status','localization',None,False,'event'),
        ('/battery_state','battery','sensor_msgs/msg/BatteryState',False,'continuous'),
        ('/navigate_to_pose/_action/status','navigation_status','action_msgs/msg/GoalStatusArray',False,'event'),
        ('/navigate_to_pose/_action/feedback','navigation_feedback',None,False,'event'),
        ('/follow_path/_action/status','follow_path_status','action_msgs/msg/GoalStatusArray',False,'event'),
        ('/parameter_events','parameter_events','rcl_interfaces/msg/ParameterEvent',False,'event'),
        ('/rosout','rosout','rcl_interfaces/msg/Log',False,'event')]:
        # Resolve known default topics through relevant running process remaps.
        proc = controller if topic.startswith('/local_costmap/') or topic in ('/tf','/tf_static') else None
        if topic == '/ranger_base/status':
            proc = by_role['ranger_base_node']
            topic = parameter(parameters.get('ranger_base_node', {}), 'mode_status_topic',
                              (proc or {}).get('overrides', {}).get('mode_status_topic', topic))
        add(resolve_topic(proc, topic) if proc else topic, role, typ, critical, cadence)
    return {'schema': 1, 'topics': list(topics.values()), 'chain': chain,
            'chain_gaps': gaps, 'source_binary_equivalence': 'not_verified',
            'internal_mppi_output': 'not a ROS topic; optional internal frame file',
            'footnote': 'Published costmap is not the controller internal snapshot. Graph still required.'}


def hash_file(path):
    try:
        result = hashlib.sha256()
        with Path(path).open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''):
                result.update(block)
        return result.hexdigest()
    except OSError:
        return None


def discover_runtime(destination, workspace=None, live_parameters=False):
    try:
        import yaml  # Existing ROS dependency; never installed by this script.
    except ImportError as exc:
        raise ValueError('Use the existing Humble Python environment (PyYAML unavailable); no installation attempted') from exc
    destination.mkdir(parents=True, exist_ok=True)
    processes, parameters, errors = [], {}, []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        selected = False
        try:
            argv = directory.joinpath('cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
            if not argv or Path(argv[0]).name not in EXECUTABLES:
                continue
            selected = True
            proc = parse_process(directory.name, argv)
            env = dict(item.split('=',1) for item in
                       (directory/'environ').read_bytes().decode(errors='replace').split('\0')
                       if '=' in item)
            proc['ros_environment'] = {key: env.get(key) for key in
                ('ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','ROS_LOCALHOST_ONLY')}
            proc['executable_resolved'] = str((directory/'exe').resolve())
            proc['executable_sha256'] = hash_file(directory/'exe')
            proc['loaded_libraries'] = []
            for line in (directory/'maps').read_text().splitlines():
                if any(key in line for key in ('libranger_dynamics_mppi_controller',
                        'libgoal_scoped_rotation_shim_controller', 'libmppi_controller',
                        'libmppi_critics', 'libcollision_monitor')):
                    path = line.split()[-1]
                    if path not in {x['path'] for x in proc['loaded_libraries']}:
                        proc['loaded_libraries'].append({'path': path, 'sha256': hash_file(path),
                                                        'mapped_file': line.split()[4]})
            processes.append(proc)
            values = {}
            for filename in proc['param_files']:
                path = Path(filename)
                data = path.read_bytes()
                digest = hashlib.sha256(data).hexdigest()
                parsed = yaml.safe_load(data) or {}
                values.update((parsed.get(proc['node'], parsed.get('/'+proc['node'], {})) or {}).get('ros__parameters', {}))
                # Keep only this node; don't copy unrelated API tokens from shared launch files.
                write_json(destination / (str(proc['pid'])+'_'+path.name+'.json'),
                           {'source': filename, 'sha256': digest, 'node': proc['node'], 'parameters': values})
            values.update({key: yaml.safe_load(value) for key, value in proc['overrides'].items()})
            parameters[proc['role']] = values
        except (OSError, ValueError, yaml.YAMLError) as exc:
            # Unrelated short-lived processes routinely disappear during /proc enumeration.
            if selected:
                errors.append(str(directory.name)+': '+str(exc))
    live = {}
    # Optional bounded read-only query, never a set/load/lifecycle operation.
    if live_parameters:
        for proc in processes:
            node = absolute_topic(proc['node'], proc)
            result = bounded_command(['ros2','param','dump', node], 4)
            live[node] = result
            if result['status'] == 'ok' and not result.get('truncated'):
                try:
                    parsed = yaml.safe_load(result['stdout']) or {}
                    data = parsed.get(node, parsed.get(proc['node'], {})).get('ros__parameters')
                    if data is not None:
                        parameters[proc['role']] = data
                    else:
                        errors.append(node+': live parameter response missing node')
                except (ValueError, AttributeError, yaml.YAMLError) as exc:
                    errors.append(node+': live parameter decode '+str(exc))
            else:
                errors.append(node+': live parameters '+result['status'])
    manifest = resolve_manifest(processes, parameters)
    manifest['parameter_origin'] = 'live_where_query_succeeded_else_launch_snapshot' if live_parameters else 'running_launch_snapshot_only'
    manifest['chain_gaps'].extend(errors)
    provenance = {'processes': processes, 'parameters': parameters, 'live_queries': live,
                  'errors': errors, 'wall_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(),
                  'environment': {key: os.environ.get(key) for key in
                                  ('ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','ROS_LOCALHOST_ONLY')},
                  'source_binary_equivalence': 'not_verified'}
    if workspace:
        scope = ['scripts/diagnostics','src/robot_nav_config','src/robot_safety',
                 'src/ranger_base','scripts/jetson/runtime_overlay/config']
        for name, args in [('branch',['branch','--show-current']), ('sha',['rev-parse','HEAD']),
                           ('status',['status','--porcelain','--']+scope),
                           ('diff',['diff','--stat','--']+scope)]:
            provenance['git_'+name] = bounded_command(['git','-C',str(workspace)]+args, 2)
    provenance['versions'] = bounded_command(['dpkg-query','-W','-f=${Package} ${Version}\n',
        'ros-humble-rclcpp','ros-humble-nav2-mppi-controller','ros-humble-nav2-collision-monitor',
        'ros-humble-rosbag2-cpp','ros-humble-rmw-fastrtps-cpp'], 2)
    write_json(destination/'provenance.json', provenance)
    write_json(destination/'topic_manifest.json', manifest)
    return manifest


class DiagnosticRequest:
    def __init__(self, path, directory, session, duration, max_mb):
        self.path, self.session = Path(path), session
        self.payload = {'schema': 1, 'session': session, 'output_dir': str(Path(directory).resolve()),
                        'max_bytes': int(max_mb*1024*1024),
                        'deadline_monotonic_ns': time.monotonic_ns()+int(duration*1e9)}
        self.owned = False

    def start(self):
        # Atomic publication and no replacement: plugin never sees partial JSON.
        # This is a diagnostic artifact, not an interlock for any vehicle action.
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=str(self.path.parent),
                                             prefix='.navlite_request_', delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(self.payload, stream)
            os.link(temporary, self.path)  # Fails if another session already owns path.
            self.owned = True
        finally:
            if temporary:
                temporary.unlink(missing_ok=True)

    def close(self):
        if self.owned:
            try:
                if json.loads(self.path.read_text()).get('session') == self.session:
                    self.path.unlink()
            except (OSError, ValueError):
                pass
            self.owned = False


def evidence_quality(directory, manifest, require_mppi=True):
    directory = Path(directory)
    gaps = list(manifest.get('chain_gaps', []))
    try:
        raw = json.loads((directory/'motion'/'quality.json').read_text())
    except (OSError, ValueError):
        raw = {}
        gaps.append('native capture: quality not available yet')
    topics = raw.get('topics', {})
    if isinstance(topics, list):
        topics = {row.get('name', row.get('topic')): row for row in topics}
    for topic in manifest.get('topics', []):
        if not topic.get('critical'):
            continue
        count = topics.get(topic['name'], {}).get('messages', 0)
        if not count:
            gaps.append(topic['name'] + ': no recorded message')
    try:
        graph = json.loads((directory/'motion'/'topic_map.json').read_text())
        observed = {topic['name']: topic for topic in graph.get('topics', [])}
        for topic in manifest.get('topics', []):
            for direction in ('publisher','subscriber'):
                wanted = topic.get('endpoint_expected', {}).get(direction, [])
                actual = {'/'+'/'.join(part for part in row.get('node','').split('/') if part)
                          for row in observed.get(topic['name'], {}).get(direction+'s', [])}
                for node in wanted:
                    if node not in actual:
                        gaps.append(topic['name']+': graph missing expected '+direction+' '+node)
    except (OSError, ValueError):
        if any(topic.get('endpoint_expected') for topic in manifest.get('topics', [])):
            gaps.append('control chain graph endpoints not available yet')
    gaps.extend(raw.get('incomplete_reasons', []))
    if raw.get('queue_drops', 0):
        gaps.append('native capture: recorder dropped messages')
    snapshots = []
    for path in directory.glob('mppi_*/status.json'):
        try:
            snapshots.append(json.loads(path.read_text()))
        except (ValueError, OSError):
            pass
    frames_exist = any(p.stat().st_size > 0 for p in directory.glob('mppi_*/frames.jsonl'))
    if require_mppi and not frames_exist:
        gaps.append('MPPI: no internal frame yet (idle or diagnostics unavailable)')
    for snapshot in snapshots:
        if (snapshot.get('dropped', 0) or snapshot.get('error') or snapshot.get('write_errors', 0)
                or snapshot.get('stale_generation_frames', 0)):
            gaps.append('MPPI: dropped frame or write failure')
        if snapshot.get('complete') is False:
            gaps.append('MPPI: capture closed incomplete: ' + str(snapshot.get('reason')))
    return {'ready': not gaps, 'gaps': list(dict.fromkeys(gaps)), 'raw': raw,
            'mppi_status': snapshots, 'mppi_has_frames': frames_exist,
            'physical_causality_verified': False}


class EvidenceSession:
    def __init__(self, output, args):
        self.output, self.args = Path(output), args
        self.process = self.log = self.request = None
        self.last_notice = None
        self.last_poll = 0
        self.extra_gaps = []

    def start(self):
        import math
        if not all(math.isfinite(x) for x in (self.args.motion_max_mb, self.args.mppi_max_mb)):
            raise ValueError('Evidence budgets must be finite')
        if not 1 <= self.args.mppi_max_mb <= 2048 or self.args.motion_max_mb < 1:
            raise ValueError('Require 1 <= mppi-max-mb <= 2048 and motion-max-mb >=1')
        if not 0 < self.args.duration <= 3590:
            raise ValueError('Motion/internal recording requires duration <=3590s')
        if Path('/tmp/njrh_reports').resolve() not in self.output.resolve().parents:
            raise ValueError('Motion/internal evidence output must be below /tmp/njrh_reports')
        helper = Path(self.args.native_helper).resolve()
        if not helper.is_file() or not os.access(helper, os.X_OK):
            raise ValueError('Native helper is missing/not executable: ' + str(helper))
        self.manifest = discover_runtime(self.output/'runtime_start',
                                        self.args.workspace, self.args.live_params)
        manifest_path = self.output/'topic_manifest.json'
        write_json(manifest_path, self.manifest)
        self.log = (self.output/'motion_capture.log').open('w', buffering=1)
        self.process = subprocess.Popen([str(helper), '--output',str(self.output/'motion'),
            '--manifest',str(manifest_path),'--duration',str(self.args.duration),
            '--disk-budget-mb',str(self.args.motion_max_mb),'--queue-mb','16',
            '--min-free-mb',str(self.args.min_free_mb)], stdout=self.log, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True)
        self.request = DiagnosticRequest(REQUEST_PATH, self.output,
                                          'navlite_'+str(os.getpid())+'_'+str(time.monotonic_ns()),
                                          self.args.duration+10, self.args.mppi_max_mb)
        try:
            self.request.start()
        except (OSError, ValueError) as exc:
            self.extra_gaps.append('MPPI diagnostic request unavailable: '+str(exc))
        write_json(self.output/'evidence_session.json', {'native_helper': str(helper),
            'native_sha256':hash_file(helper),'native_pid':self.process.pid,
            'mppi_request_owned':self.request.owned,'same_boot_monotonic':True,
            'http':False,'production_mutations':False,'diagnostic_file_request':str(REQUEST_PATH)})

    def poll(self, force=False):
        now = time.monotonic()
        if not force and now-self.last_poll < 1:
            return None
        self.last_poll = now
        quality = evidence_quality(self.output, self.manifest)
        quality['gaps'].extend(self.extra_gaps)
        if self.process and self.process.poll() not in (None, 0):
            quality['gaps'].append('native capture incomplete (exit 3)' if self.process.returncode == 3
                                   else 'native capture exited '+str(self.process.returncode))
        quality['ready'] = not quality['gaps']
        state = ('EVIDENCE_READY' if quality['ready'] else 'EVIDENCE_INCOMPLETE', tuple(quality['gaps']))
        if state != self.last_notice:
            print(state[0]+': '+('; '.join(state[1]) if state[1] else 'required streams and MPPI frames observed; not causal proof'),flush=True)
            self.last_notice = state
        write_json(self.output/'evidence_quality.json', quality)
        return quality

    def close(self):
        if self.request:
            self.request.close()
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.extra_gaps.append('native recorder did not close within 8s; terminated recorder only')
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
        if self.log:
            self.log.close()
        # Producer reads the diagnostic request every 500ms. Wait only for its
        # own final file, never for a ROS callback or robot state.
        deadline = time.monotonic()+1.2
        while time.monotonic() < deadline:
            folders = list(self.output.glob('mppi_*'))
            if all((folder/'status.json').exists() for folder in folders):
                break
            time.sleep(.05)
        # No final ROS queries: record changes passively via parameter_events.
        return self.poll(force=True)
