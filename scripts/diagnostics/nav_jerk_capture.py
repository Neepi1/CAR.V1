#!/usr/bin/env python3
"""Passive navigation recorder. ROS imports are lazy; mark/report need no ROS graph.

One raw subscription per topic -> bounded queue -> one rosbag2 writer.
No action clients, command publishers, parameter setters or recovery calls.
"""
import argparse
import bisect
import collections
import csv
import functools
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import urllib.request
import xml.etree.ElementTree as ET

VELOCITIES = {'geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped'}
LARGE = {'sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage', 'visualization_msgs/msg/MarkerArray'}
SMALL_TYPES = {'nav_msgs/msg/Odometry', 'sensor_msgs/msg/Imu', 'sensor_msgs/msg/BatteryState', 'nav_msgs/msg/Path', 'nav2_msgs/msg/SpeedLimit', 'nav2_msgs/msg/CollisionMonitorState', 'geometry_msgs/msg/PolygonStamped', 'geometry_msgs/msg/PoseWithCovarianceStamped', 'geometry_msgs/msg/Vector3Stamped'}
GAPS = [
    'Internal MPPI-only output/critic rejection/effective constraints/computation duration not exposed; outer controller output is not pure MPPI.',
    'Published costmap is an asynchronous public snapshot, not the exact controller-cycle map.',
    'DDS/source losses unknown; callback counts measure only this observer.',
    'Receive order does not prove internal latency or causality; multi-publisher topic attribution may be unknown.',
    'No passive Action goal/result service interception; target only from existing explicit status/HTTP fields, never path endpoint.',
    'IMU acceleration is in its reported frame; gravity inclusion is unknown unless documented, not longitudinal shock.',
    'Parameter/external speed-limit snapshots do not prove MPPI internal active constraints.',
    'Ranger public header stamps are publication stamps, not CAN sample timestamps; CAN source sequence/age unavailable.',
    'Current Ranger ActuatorStateArray repeats selected SDK fields across eight entries; do not treat these as independently validated wheel channels.',
    'Current-owner selection may be unavailable: safety state/mode/permits are retained, but topic names alone do not prove arbiter ownership.',
    'Snapshot parameters can time out; parameter_events may precede subscription or arrive from multiple nodes. Inspect missing lists.',
    'Endpoints created after discovery are inventoried every 10 s but not automatically subscribed; missing/new topics remain evidence gaps.'
]


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite': str(value)}
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def dumps(value):
    return json.dumps(clean(value), ensure_ascii=False, allow_nan=False, separators=(',', ':'))


def save(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.pending')
    temporary.write_text(dumps(value) + '\n', encoding='utf-8')
    temporary.replace(path)


def load_config(path):
    text = Path(path).read_text(encoding='utf-8-sig')
    try:
        return json.loads(text)  # Shipped config is JSON-compatible YAML; stdlib only.
    except json.JSONDecodeError:
        import yaml
        return yaml.safe_load(text)


def node_matches(endpoint, name):
    return endpoint['node'].strip('/') == name.strip('/')


def node_params(params, node):
    return params.get('/' + node.strip('/'), params.get(node, {}))


def resolved_topic(name, node):
    return name if name.startswith('/') else '/' + '/'.join(node.strip('/').split('/')[:-1] + [name])


def qos_choice(publishers):
    # Least-demanding compatible subscriber; retained data only when ALL offer it.
    rel = 'BEST_EFFORT' if any('BEST_EFFORT' in p.get('reliability', '') for p in publishers) else 'RELIABLE'
    dur = 'TRANSIENT_LOCAL' if publishers and all('TRANSIENT_LOCAL' in p.get('durability', '') for p in publishers) else 'VOLATILE'
    return {'reliability': rel, 'durability': dur, 'depth': 100,
            'mixed_durability_retained_gap': len({p.get('durability') for p in publishers}) > 1}


def bag_qos(endpoint):
    """Serialize offered graph QoS, keeping UNKNOWN (not a guessed queue depth)."""
    def policy(key, values, unknown):
        return values.get(str(endpoint.get(key, '')).rsplit('.', 1)[-1], unknown)
    def duration(key):
        ns = endpoint.get(key, 9223372036854775807)
        return {'sec': ns // 1000000000, 'nsec': ns % 1000000000}
    return {
        'history': policy('history', {'SYSTEM_DEFAULT': 0, 'KEEP_LAST': 1, 'KEEP_ALL': 2}, 3),
        'depth': endpoint.get('depth', 0),
        'reliability': policy('reliability', {'SYSTEM_DEFAULT': 0, 'RELIABLE': 1, 'BEST_EFFORT': 2}, 3),
        'durability': policy('durability', {'SYSTEM_DEFAULT': 0, 'TRANSIENT_LOCAL': 1, 'VOLATILE': 2}, 3),
        'deadline': duration('deadline_ns'), 'lifespan': duration('lifespan_ns'),
        'liveliness': policy('liveliness', {'SYSTEM_DEFAULT': 0, 'AUTOMATIC': 1, 'MANUAL_BY_TOPIC': 3}, 4),
        'liveliness_lease_duration': duration('liveliness_lease_duration_ns'),
        'avoid_ros_namespace_conventions': endpoint.get('avoid_ros_namespace_conventions', False)}


def build_topic_map(graph, params, cfg, deep=False):
    result = {'topics': {}, 'roles': {}, 'missing_roles': {}, 'edges': [],
              'internal_mppi_output_visible': False, 'observability_gaps': list(GAPS)}

    def add(topic, role, critical=False, evidence='graph', cadence='event'):
        g = graph[topic]
        item = result['topics'].setdefault(topic, {**g, 'roles': [], 'critical': False,
            'evidence': [], 'cadence': cadence, 'multi_publisher': len(g['pubs']) > 1,
            'subscription_qos': qos_choice(g['pubs']), 'missing_reason': None if g['pubs'] else 'no_publisher_observed'})
        if role not in item['roles']:
            item['roles'].append(role)
        item['critical'] |= critical
        if evidence not in item['evidence']:
            item['evidence'].append(evidence)
        if cadence != 'event':
            item['cadence'] = cadence

    for role, rule in cfg['roles'].items():
        parameter = node_params(params, rule['node']).get(rule.get('param'))
        hits = []
        for topic, g in graph.items():
            if not VELOCITIES.intersection(g['types']):
                continue
            if not any(node_matches(e, rule['node']) for e in g[rule['direction']]):
                continue
            # Named safety ports need a parameter proof; no guesses among many inputs.
            if rule.get('param') and (not isinstance(parameter, str) or topic != resolved_topic(parameter, rule['node'])):
                continue
            hits.append(topic)
            add(topic, role, rule.get('critical', False), 'endpoint + live_parameter' if parameter else 'endpoint', 'command')
        result['roles'][role] = hits
        if not hits:
            result['missing_roles'][role] = 'parameter_unavailable_or_endpoint_missing' if rule.get('param') else 'endpoint_missing'
    for topic, g in graph.items():
        types = set(g['types'])
        related = any(re.search(cfg['related_nodes_regex'], e['node']) for e in g['pubs'] + g['subs'])
        if types & VELOCITIES and related:
            add(topic, 'other_velocity_source_or_boundary', cadence='command')
        elif types & SMALL_TYPES and related:
            cadence = 'continuous' if types & {'nav_msgs/msg/Odometry', 'sensor_msgs/msg/Imu', 'sensor_msgs/msg/BatteryState'} else 'event'
            add(topic, 'motion_or_navigation_state', cadence=cadence)
        elif 'sensor_msgs/msg/LaserScan' in types and any('collision_monitor' in e['node'] or 'local_costmap' in e['node'] for e in g['subs']):
            add(topic, 'obstacle_scan', True, cadence='continuous')
        elif types & {'nav_msgs/msg/OccupancyGrid', 'map_msgs/msg/OccupancyGridUpdate'} and any('local_costmap' in e['node'] for e in g['pubs']):
            p = node_params(params, 'local_costmap/local_costmap')
            cells = p.get('width', math.inf) * p.get('height', math.inf) / max(p.get('resolution', .05) ** 2, 1e-9)
            if cells <= cfg['max_costmap_cells']:
                add(topic, 'published_local_costmap', cadence='event')
            else:
                result['observability_gaps'].append(topic + ': size unproven or above max_costmap_cells; not subscribed')
        elif topic in ('/tf', '/tf_static', '/rosout', '/parameter_events', '/clock'):
            add(topic, 'tf_or_log_or_parameter_event', cadence='continuous' if topic == '/tf' else 'event')
        elif topic in cfg['state_topics'] or (related and (topic.endswith('/_action/feedback') or topic.endswith('/_action/status') or any(t.startswith(('ranger_msgs/', 'robot_interfaces/')) for t in types))):
            add(topic, 'passive_state_or_action')
        elif related and topic.startswith('/ranger_mini3/') and types & {'std_msgs/msg/Bool', 'std_msgs/msg/UInt8', 'std_msgs/msg/String'}:
            add(topic, 'permit_or_progress')
        elif types & LARGE and related:
            if deep and 'sensor_msgs/msg/PointCloud2' in types:
                add(topic, 'deep_pointcloud_input', cadence='continuous')
            else:
                result['observability_gaps'].append(topic + ': large input excluded (PointCloud2 can be opted into with --deep)')
    for a, b in cfg['edges']:
        one_each = len(result['roles'][a]) == len(result['roles'][b]) == 1
        same_component = cfg['roles'][a]['node'] == cfg['roles'][b]['node']
        connected = one_each and (same_component or result['roles'][a] == result['roles'][b])
        result['edges'].append({'from_role': a, 'to_role': b, 'from_topics': result['roles'][a], 'to_topics': result['roles'][b],
            'verified': connected,
            'missing_reason': None if connected else ('ports_do_not_share_topic' if one_each else 'missing_or_ambiguous_port'),
            'meaning': 'source-audited port relationship + live endpoints/parameters; not measured internal latency'})
    odom = node_params(params, 'controller_server').get('odom_topic')
    result['controller_odom_topic'] = resolved_topic(odom, 'controller_server') if isinstance(odom, str) else None
    if result['controller_odom_topic'] in result['topics']:
        result['topics'][result['controller_odom_topic']]['critical'] = True
    for topic in cfg['state_topics']:
        if topic not in result['topics']:
            result['observability_gaps'].append(topic + ': no endpoint observed')
    return result


class BoundedInbox:
    def __init__(self, messages, byte_limit):
        self.q = queue.Queue(maxsize=messages)
        self.byte_limit = byte_limit
        self.bytes = self.high_bytes = self.high_count = 0
        self.dropped = {}
        self.lock = threading.Lock()

    def put(self, item):
        size = len(item.get('raw', b'')) + len(item.get('json', '').encode('utf-8'))
        with self.lock:
            if size + self.bytes > self.byte_limit or self.q.full():
                key = item.get('topic', item.get('kind', 'auxiliary'))
                row = self.dropped.setdefault(key, {'count': 0, 'first_seq': item.get('seq'), 'last_seq': None})
                row['count'] += 1
                row['last_seq'] = item.get('seq')
                return False
            self.bytes += size
            item['_size'] = size
            self.q.put_nowait(item)
            self.high_bytes = max(self.high_bytes, self.bytes)
            self.high_count = max(self.high_count, self.q.qsize())
            return True

    def get(self, timeout=.1):
        item = self.q.get(timeout=timeout)
        with self.lock:
            self.bytes -= item.pop('_size')
        return item

    def stats(self):
        with self.lock:
            return {'bytes': self.bytes, 'pending': self.q.qsize(), 'max_bytes': self.high_bytes,
                    'max_pending': self.high_count, 'dropped': {k: dict(v) for k, v in self.dropped.items()}}


def continuous_health(topic_map, received, last_receive, now_ns, stale_sec=2.):
    """Observer quality only. Never treat event silence/idle command silence as failure."""
    topics = {}
    for topic, item in topic_map['topics'].items():
        if item.get('cadence') != 'continuous':
            continue
        last = last_receive.get(topic)
        age = (now_ns-last)/1e9 if last is not None else None
        topics[topic] = {'received': received.get(topic, 0), 'last_receive_age_sec': age,
                         'state': 'missing' if last is None else ('stale' if age > stale_sec else 'recent')}
    stale = sorted(t for t, value in topics.items() if value['state'] != 'recent')
    return {'state': 'INCOMPLETE' if stale else 'READY', 'stale_topics': stale, 'topics': topics,
            'meaning': 'observer reception only; not a producer fault or vehicle gate'}


class RotatingLog:
    """Keep the old inode open through rename; copy-truncate is explicitly lossy."""
    def __init__(self, path, initial_tail_bytes):
        self.path, self.tail = Path(path), initial_tail_bytes
        self.handle = None
        self.inode = None
        self.opened_once = False

    def read(self):
        rows = []
        try:
            st = self.path.stat()
        except OSError as exc:
            return [{'path': str(self.path), 'event': 'unavailable', 'error': str(exc)}]
        identity = (st.st_dev, st.st_ino)
        if self.handle and identity != self.inode:
            rows.extend(self._chunk())
            if self.handle.tell() < os.fstat(self.handle.fileno()).st_size:
                return rows  # finish draining old inode in bounded chunks before reopening
            self.handle.close()
            self.handle = None
            rows.append({'path': str(self.path), 'event': 'rotated', 'old_inode': self.inode, 'new_inode': identity})
        if self.handle is None:
            self.handle = self.path.open('rb')
            self.inode = identity
            offset = 0 if self.opened_once else max(0, st.st_size - self.tail)
            self.opened_once = True
            self.handle.seek(offset)
            rows.append({'path': str(self.path), 'event': 'opened', 'inode': identity, 'initial_skipped_bytes': offset})
        elif st.st_size < self.handle.tell():
            self.handle.seek(0)
            rows.append({'path': str(self.path), 'event': 'copytruncate', 'unread_loss': 'unknown'})
        rows.extend(self._chunk())
        return rows

    def _chunk(self):
        offset = self.handle.tell()
        raw = self.handle.read(256 * 1024)
        return [{'path': str(self.path), 'inode': self.inode, 'offset': offset, 'bytes': len(raw),
                 'text': raw.decode('utf-8', errors='replace'), 'source_time': 'embedded_original_text_not_receive_time'}] if raw else []

    def close(self):
        if self.handle:
            self.handle.close()


def digest(path):
    try:
        h = hashlib.sha256()
        with Path(path).open('rb') as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b''):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


SECRET = re.compile(r'token|password|secret|authorization', re.I)


def redact(value):
    if isinstance(value, dict):
        return {k: '<redacted>' if SECRET.search(str(k)) else redact(v) for k, v in value.items()}
    if isinstance(value, list):
        return [redact(v) for v in value]
    return value


def command(args, cwd=None, timeout=3):
    try:
        r = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=timeout, errors='replace')
        return {'returncode': r.returncode, 'stdout': r.stdout, 'stderr': r.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'error': str(exc)}


def processes():
    rows = []
    selected = re.compile(r'controller_server|planner_server|bt_navigator|velocity_smoother|collision_monitor|robot_safety_node|ranger_base_node|ekf_node|robot_api_server_node|docking_manager_node|localization_bridge|imu_.*node|amcl$')
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            exe = os.readlink(proc / 'exe')
            if not selected.search(Path(exe).name):
                continue
            args = (proc / 'cmdline').read_bytes().decode(errors='replace').split('\0')
            # Never persist process environments or credentials from command lines.
            args = ['<redacted>' if SECRET.search(a) else a for a in args]
            rows.append({'pid': int(proc.name), 'exe': exe, 'sha256': digest(proc / 'exe'), 'argv': args,
                         'source_binary_consistency': 'unverified (hash is identity, not a build provenance proof)'})
        except OSError:
            pass
    return rows


def snapshot_files(root, cfg, params, procs, output):
    paths = [root / f for f in cfg['source_files']]
    for p in procs:
        for i, arg in enumerate(p['argv'][:-1]):
            if arg == '--params-file':
                paths.append(Path(p['argv'][i+1]))
    for values in params.values():
        for key, value in values.items():
            if isinstance(value, str) and value.endswith(('.xml', '.yaml', '.yml')) and not SECRET.search(key):
                paths.append(Path(value))
    files = []
    output.mkdir(exist_ok=True)
    for path in dict.fromkeys(paths):
        row = {'path': str(path), 'sha256': digest(path)}
        try:
            if path.stat().st_size > 2 * 1024 * 1024:
                row['copy_gap'] = 'over 2 MiB per-file snapshot bound'
            else:
                text = path.read_text(encoding='utf-8', errors='replace')
                # Preserve file hash but do not disclose secrets in file snapshots.
                filtered = '\n'.join('<redacted credential line>' if SECRET.search(line) else line for line in text.splitlines())
                target = output / (hashlib.sha256(str(path).encode()).hexdigest()[:12] + '_' + path.name)
                target.write_text(filtered, encoding='utf-8')
                row.update(copy=str(target.name), redacted=filtered != '\n'.join(text.splitlines()))
        except OSError as exc:
            row['error'] = str(exc)
        files.append(row)
    return files


def provenance(root, cfg, params, output):
    procs = processes()
    versions = {}
    for package in cfg['packages']:
        try:
            from ament_index_python.packages import get_package_share_directory
            versions[package] = ET.parse(Path(get_package_share_directory(package)) / 'package.xml').findtext('version')
        except Exception as exc:
            versions[package] = {'unavailable': str(exc)}
    env = {key: os.getenv(key) for key in ('ROS_DISTRO', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'FASTDDS_BUILTIN_TRANSPORTS', 'FASTRTPS_DEFAULT_PROFILES_FILE')}
    libraries = {}
    for proc in procs:
        try:
            for line in Path('/proc', str(proc['pid']), 'maps').read_text().splitlines():
                path = line.split()[-1]
                if path.startswith('/') and any(s in Path(path).name for s in ('robot_nav_config', 'ranger_mppi', 'rotation_shim', 'fastrtps', 'rmw_fastrtps')):
                    if path not in libraries:
                        libraries[path] = digest(path)
        except OSError:
            pass
    return {'wall_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(), 'workspace': str(root),
        'branch': command(['git', 'branch', '--show-current'], root), 'sha': command(['git', 'rev-parse', 'HEAD'], root),
        'status': command(['git', 'status', '--short'], root),
        'relevant_diff': command(['git', 'diff', 'HEAD', '--'] + cfg['source_files'], root),
        'environment': env, 'versions': versions, 'processes': procs, 'loaded_libraries': libraries,
        'files': snapshot_files(root, cfg, params, procs, output / 'files'),
        'source_binary_consistency': 'unverified unless accompanied by independently matched build manifest',
        'use_sim_time': {node: p.get('use_sim_time') for node, p in params.items()}}


def process_sample(pid):
    # Reuse the existing reader, plus RSS/I/O; never call this a compute-cycle timer.
    helper = Path(__file__).resolve().parents[1] / 'jetson/runtime_overlay/scripts'
    if str(helper) not in sys.path:
        sys.path.insert(0, str(helper))
    try:
        from navigation_observer_evidence import controller_process_counters
        row = controller_process_counters(pid)
    except ImportError:
        row = {'pid': pid, 'available': False, 'reuse_helper_missing': True}
        try:
            fields = Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()
            row.update(available=True, utime_ticks=int(fields[11]), stime_ticks=int(fields[12]),
                       process_start_ticks=int(fields[19]), clock_ticks_per_sec=os.sysconf('SC_CLK_TCK'))
        except (OSError, ValueError, IndexError): pass
    try:
        proc = Path('/proc') / str(pid)
        status = (proc / 'status').read_text()
        row['rss_kib'] = int(re.search(r'VmRSS:\s+(\d+)', status)[1])
        row['io'] = {k: int(v.strip()) for k, v in (s.split(':', 1) for s in (proc / 'io').read_text().splitlines())}
        row['cpus_allowed'] = re.search(r'Cpus_allowed_list:\s*(.*)', status)[1]
    except (OSError, TypeError, AttributeError, ValueError):
        row['rss_or_io_gap'] = True
    return row


def get_message_class(typename):
    from rosidl_runtime_py.utilities import get_message, get_action
    if '/action/' in typename and typename.endswith('_FeedbackMessage'):
        return get_action(typename.removesuffix('_FeedbackMessage')).Impl.FeedbackMessage
    return get_message(typename)


@functools.lru_cache(maxsize=256)
def message_fields(cls):
    return tuple(cls.get_fields_and_field_types())


def ros_dict(value):
    # Generated public fields have stable schemas; don't parse the type description
    # for every element of every message. Original CDR is always retained separately.
    if hasattr(value, 'get_fields_and_field_types'):
        return {name:ros_dict(getattr(value,name)) for name in message_fields(type(value))}
    if hasattr(value,'tolist'):
        return value.tolist()
    if isinstance(value,(list,tuple)):
        return [ros_dict(v) for v in value]
    if isinstance(value,bytes):
        return value.decode('latin1')  # rosidl_runtime_py's byte-scalar JSON convention
    return value


def stamp_fields(msg):
    """Only declared source stamps; never manufacture one for Twist/String/API."""
    def header(h):
        return {'stamp_ns': int(h.stamp.sec) * 1000000000 + int(h.stamp.nanosec), 'frame': h.frame_id}
    if hasattr(msg, 'header'):
        return [header(msg.header)]
    if hasattr(msg, 'transforms'):
        return [{**header(t.header), 'child_frame': t.child_frame_id} for t in msg.transforms]
    if hasattr(msg, 'stamp') and hasattr(msg.stamp, 'sec'):
        return [{'stamp_ns': int(msg.stamp.sec)*1000000000+int(msg.stamp.nanosec), 'frame': None}]
    if hasattr(msg, 'feedback') and hasattr(msg.feedback, 'current_pose'):
        return [header(msg.feedback.current_pose.header)]
    return []


class BagSink:
    def __init__(self, directory, topic_map, cfg, stop):
        import rosbag2_py
        self.output, self.cfg, self.stop = Path(directory), cfg, stop
        self.inbox = BoundedInbox(cfg['queue_messages'], cfg['queue_bytes'])
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(self.output / 'bag'), storage_id=cfg['storage_id']),
                         rosbag2_py.ConverterOptions('', ''))
        self.types, self.classes = {}, {}
        for topic, item in topic_map['topics'].items():
            if item.get('excluded'):
                continue
            typename = item['types'][0]
            self.classes[topic] = get_message_class(typename)
            self.types[topic] = typename
            # rosbag2 Humble expects serialized QoS; preserve exact inventory separately too.
            import yaml
            qos = [bag_qos(p) for p in item['pubs']]
            self.writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typename,
                serialization_format='cdr', offered_qos_profiles=yaml.safe_dump(qos)))
        self.handles = {name: (self.output / (name + '.jsonl')).open('x', encoding='utf-8')
                        for name in ('index', 'auxiliary')}
        self.counts = collections.Counter()
        self.last_bag_ns = 0
        self.failure = None
        self.http_quality = {'enabled': cfg['http_enabled'], 'attempts': 0, 'successes': 0,
                             'failures': 0, 'last_error': None}
        self.finished = threading.Event()
        self.thread = threading.Thread(target=self.run, name='nav_jerk_bag_writer')
        self.thread.start()

    def auxiliary(self, kind, value):
        if kind == 'http':
            self.http_quality['attempts'] += 1
            good = value.get('status') == 200
            self.http_quality['successes' if good else 'failures'] += 1
            error = None if good else value.get('error', 'HTTP status '+str(value.get('status')))
            if self.http_quality['attempts'] == 1 or error != self.http_quality['last_error']:
                print(('HTTP_READY' if good else 'HTTP_INCOMPLETE')+' '+value['path']+
                      (' '+error if error else '')+'; recorder quality only', flush=True)
            self.http_quality['last_error'] = error
        return self.inbox.put({'kind': kind, 'json': dumps({'kind': kind, 'monotonic_ns': time.monotonic_ns(), 'wall_ns': time.time_ns(), **value})})

    def run(self):
        from rclpy.serialization import deserialize_message
        try:
            last_flush = time.monotonic()
            while not self.finished.is_set() or not self.inbox.q.empty():
                try:
                    item = self.inbox.get()
                except queue.Empty:
                    continue
                if 'raw' not in item:
                    self.handles['auxiliary'].write(item['json'] + '\n')
                    continue
                topic, raw = item['topic'], item.pop('raw')
                # Unique, ordered bag key independent of backwards wall-clock steps.
                # This index time is NOT a source stamp or an exact receive wall time.
                bag_ns = max(self.last_bag_ns + 1, item['wall_ns'])
                self.last_bag_ns = bag_ns
                self.writer.write(topic, raw, bag_ns)
                self.counts[topic] += 1
                item.update(bag_timestamp_ns=bag_ns, topic_ordinal=self.counts[topic], type=self.types[topic], serialized_bytes=len(raw))
                try:
                    msg = deserialize_message(raw, self.classes[topic])
                    item['source_headers'] = stamp_fields(msg)
                    # Full original bytes remain in bag, including ranges/grid/path/covariances.
                    # Small decoded data in the index makes reports ROS-independent.
                    if self.types[topic] not in LARGE | {'sensor_msgs/msg/LaserScan', 'nav_msgs/msg/OccupancyGrid', 'map_msgs/msg/OccupancyGridUpdate', 'nav_msgs/msg/Path'}:
                        item['data'] = ros_dict(msg)  # dumps sanitizes nonfinite values once
                    else:
                        item['data'] = None
                        if hasattr(msg, 'info') and hasattr(msg.info, 'width'):
                            item['grid'] = {'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution}
                except Exception as exc:
                    item['decode_error'] = repr(exc)
                self.handles['index'].write(dumps(item) + '\n')
                if time.monotonic() - last_flush >= 1:
                    for h in self.handles.values(): h.flush()
                    last_flush = time.monotonic()
        except BaseException as exc:
            self.failure = repr(exc)
            self.stop.set()
        finally:
            try:
                self.writer.close()
            except BaseException as exc:
                self.failure = (self.failure or '') + '; close: ' + repr(exc)
            for h in self.handles.values():
                try: h.close()
                except OSError as exc: self.failure = (self.failure or '') + '; index close: ' + repr(exc)

    def close(self):
        self.finished.set()
        self.thread.join()  # no callbacks/production locks held; only finite queued local I/O
        return {'written': dict(self.counts), 'writer_error': self.failure, 'http': dict(self.http_quality), **self.inbox.stats()}


class RosObserver:
    """One participant, ordinary subscriptions and bounded GET/LIST parameters only."""
    def __init__(self, stop):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        self.rclpy, self.stop = rclpy, stop
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node('nav_jerk_capture_' + str(os.getpid()), enable_rosout=False, start_parameter_services=False)
        self.executor = SingleThreadedExecutor(context=self.node.context)
        self.executor.add_node(self.node)
        self.subscriptions = []
        self.received = collections.Counter()
        self.last_receive = {}
        self.qos_events = collections.Counter()
        self.seq = 0
        self.sim_time = None
        self.clock_ros_ns = None
        self.sink = None

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while not self.stop.is_set() and time.monotonic() < end and self.rclpy.ok():
            # Humble restarts its ready-event iterator when spin kwargs change.
            # A shrinking deadline starves later subscriptions under sustained load.
            # Fixed idle wait is bounded (not a sleep); stop may overshoot by <=50 ms.
            self.executor.spin_once(timeout_sec=.05)

    def graph(self):
        def endpoint(e):
            q = e.qos_profile
            return {'node': e.node_namespace.rstrip('/') + '/' + e.node_name,
                    'type': e.topic_type, 'gid': bytes(e.endpoint_gid).hex(),
                    'reliability': str(q.reliability), 'durability': str(q.durability),
                    'history': str(q.history), 'depth': q.depth,
                    'deadline_ns': q.deadline.nanoseconds, 'lifespan_ns': q.lifespan.nanoseconds,
                    'liveliness': str(q.liveliness), 'liveliness_lease_duration_ns': q.liveliness_lease_duration.nanoseconds,
                    'avoid_ros_namespace_conventions': q.avoid_ros_namespace_conventions}
        return {topic: {'types': types, 'pubs': [endpoint(e) for e in self.node.get_publishers_info_by_topic(topic)],
                        'subs': [endpoint(e) for e in self.node.get_subscriptions_info_by_topic(topic)
                                 if e.node_name != self.node.get_name()]}
                for topic, types in self.node.get_topic_names_and_types()}

    def parameters(self, cfg):
        from rcl_interfaces.srv import ListParameters, GetParameters
        from rclpy.parameter import parameter_value_to_python
        deadline = time.monotonic() + cfg['param_total_sec']
        result, pending = {}, []
        try:
            for name in cfg['nodes']:
                name = '/' + name.strip('/')
                c = self.node.create_client(ListParameters, name + '/list_parameters')
                pending.append((name, c, c.call_async(ListParameters.Request()), time.monotonic()))
            requests = []
            while pending and time.monotonic() < deadline and not self.stop.is_set():
                self.spin(.02)
                for name, client, future, start in list(pending):
                    if future.done() or time.monotonic()-start >= cfg['param_timeout_sec']:
                        pending.remove((name, client, future, start))
                        self.node.destroy_client(client)
                        if not future.done() or future.exception():
                            result[name] = {'_error': 'list_parameters_timeout_or_error'}
                            continue
                        names = [n for n in future.result().result.names if not SECRET.search(n)]
                        # Bounded chunks, not a CLI participant per parameter.
                        result[name] = {}
                        for offset in range(0, len(names), 128):
                            req = GetParameters.Request(names=names[offset:offset+128])
                            c = self.node.create_client(GetParameters, name + '/get_parameters')
                            requests.append((name, req.names, c, c.call_async(req), time.monotonic()))
            while requests and time.monotonic() < deadline and not self.stop.is_set():
                self.spin(.02)
                for name, names, client, future, start in list(requests):
                    if future.done() or time.monotonic()-start >= cfg['param_timeout_sec']:
                        requests.remove((name, names, client, future, start))
                        self.node.destroy_client(client)
                        if not future.done() or future.exception():
                            result[name].setdefault('_missing_parameters', []).extend(names)
                        else:
                            result[name].update(zip(names, (parameter_value_to_python(v) for v in future.result().values)))
            for name, *_ in pending:
                result[name] = {'_error': 'snapshot_total_budget_or_stop'}
            for name, names, *_ in requests:
                result[name].setdefault('_missing_parameters', []).extend(names)
            return result
        finally:
            for _, c, *_ in pending:
                self.node.destroy_client(c)
            for _, _, c, *_ in locals().get('requests', []):
                self.node.destroy_client(c)

    def subscribe(self, topic_map, sink=None, critical_only=False):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.qos_event import SubscriptionEventCallbacks
        self.sink = sink
        for topic, item in topic_map['topics'].items():
            if item.get('excluded') or (critical_only and not item['critical']):
                continue
            if len(item['types']) != 1:
                item['excluded'] = 'ambiguous_topic_types'
                continue
            q = item['subscription_qos']
            qos = QoSProfile(depth=100,
                reliability=getattr(ReliabilityPolicy, q['reliability']),
                durability=getattr(DurabilityPolicy, q['durability']))
            def event(kind, e, topic=topic):
                self.qos_events[(topic, kind)] += max(1, getattr(e, 'total_count_change', 1))
                if self.sink:
                    self.sink.auxiliary(kind, {'topic': topic, 'event': str(e)})
            callbacks = SubscriptionEventCallbacks(
                incompatible_qos=lambda e, t=topic: event('incompatible_qos', e, t),
                message_lost=lambda e, t=topic: event('rmw_reported_message_lost', e, t), use_default_callbacks=False)
            try:
                typename = item['types'][0]
                sub = self.node.create_subscription(get_message_class(typename), topic,
                    self.callback(topic, typename), qos, raw=True, event_callbacks=callbacks)
                self.subscriptions.append((topic, sub))
            except Exception as exc:
                item['excluded'] = 'subscription_error: ' + repr(exc)

    def callback(self, topic, typename):
        def receive(raw):
            mono, wall = time.monotonic_ns(), time.time_ns()
            self.seq += 1
            self.received[topic] += 1
            self.last_receive[topic] = mono
            if topic == '/clock':
                from rclpy.serialization import deserialize_message
                c = deserialize_message(raw, get_message_class(typename)).clock
                self.clock_ros_ns = c.sec*1000000000+c.nanosec
            ros_ns = self.node.get_clock().now().nanoseconds if self.sim_time is False else self.clock_ros_ns
            if self.sink:
                self.sink.inbox.put({'seq': self.seq, 'topic': topic, 'raw': raw,
                    'monotonic_ns': mono, 'wall_ns': wall, 'ros_ns': ros_ns,
                    'publisher_gid': None, 'publisher_gid_reason': 'installed_Humble_rclpy_callback_does_not_expose_MessageInfo',
                    'rmw_source_timestamp_ns': None, 'rmw_received_timestamp_ns': None})
        return receive

    def coverage(self, topic_map):
        states = {}
        for topic, item in topic_map['topics'].items():
            states[topic] = {'received': self.received[topic], 'matched_publishers': None,
                            'excluded': item.get('excluded'), 'dds_or_source_loss': 'unknown'}
        for topic, sub in self.subscriptions:
            if hasattr(sub, 'get_publisher_count'):
                states[topic]['matched_publishers'] = sub.get_publisher_count()
            else:
                states[topic]['match_count_reason'] = 'installed_rclpy_does_not_expose_subscription_match_count'
            states[topic]['graph_publishers'] = self.node.count_publishers(topic)
            states[topic]['matching_evidence'] = 'messages_received' if self.received[topic] else 'unknown'
        missing = [t for t, i in topic_map['topics'].items() if i['critical'] and not self.received[t]]
        missing += list(topic_map['missing_roles'])
        if not topic_map.get('controller_odom_topic'):
            missing.append('controller_odom_parameter_unavailable')
        health = continuous_health(topic_map, self.received, self.last_receive, time.monotonic_ns(),
                                   getattr(self, 'continuous_stale_sec', 2.))
        missing += health['stale_topics']
        return {'state': 'INCOMPLETE' if missing else 'READY', 'missing': missing,
                'continuous_health': health, 'topics': states,
                'qos_events': [{'topic': t, 'kind': k, 'count': v} for (t, k), v in self.qos_events.items()]}

    def unsubscribe(self):
        self.sink = None
        for _, sub in self.subscriptions:
            self.node.destroy_subscription(sub)
        self.subscriptions.clear()

    def close(self):
        self.unsubscribe()
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=1)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def token_for_local_api(cfg):
    token = os.getenv(cfg['http_token_env'], '')
    # Match the established recorder behavior. Never output/store the environment.
    if not token and cfg['http_base'] == 'http://127.0.0.1:8080':
        for p in Path('/proc').glob('[0-9]*'):
            try:
                if Path(os.readlink(p / 'exe')).name != 'robot_api_server_node':
                    continue
                for entry in (p / 'environ').read_bytes().split(b'\0'):
                    if entry.startswith(b'ROBOT_API_TOKEN='):
                        return entry.split(b'=', 1)[1].decode()
            except OSError:
                pass
    return token


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None  # do not send credentials to a redirected host


def http_worker(cfg, stop, sink):
    token = token_for_local_api(cfg)
    opener = urllib.request.build_opener(NoRedirect)
    # Aggregate <= 1 request/s (three endpoints round-robin), never one worker per URL.
    allowed = {'/api/v1/navigation/state', '/api/v1/status', '/api/v1/docking/state'}
    paths = [p for p in cfg['http_paths'] if p in allowed]
    index = 0
    while paths and not stop.is_set():
        started = time.monotonic()
        path = paths[index % len(paths)]
        index += 1
        row = {'path': path, 'request_monotonic_ns': time.monotonic_ns(), 'request_wall_ns': time.time_ns(), 'source_stamp': None}
        try:
            request = urllib.request.Request(cfg['http_base'].rstrip('/') + path, method='GET',
                headers={'X-Robot-Token': token} if token else {})
            with opener.open(request, timeout=cfg['http_timeout_sec']) as response:
                raw = response.read(2*1024*1024+1)
                if len(raw) > 2*1024*1024:
                    raise ValueError('HTTP snapshot exceeds 2 MiB bound')
                row.update(status=response.status, data=redact(json.loads(raw)))
        except Exception as exc:
            row['error'] = str(exc)
        sink.auxiliary('http', row)
        stop.wait(max(0, 1 - (time.monotonic()-started)))


def create_mark(directory, label):
    directory = Path(directory)
    if not (directory / 'session.json').exists():
        raise ValueError('not a nav jerk capture directory')
    request = {'id': uuid.uuid4().hex, 'label': label[:512], 'monotonic_ns': time.monotonic_ns(),
               'wall_ns': time.time_ns(), 'host': os.uname().nodename if hasattr(os, 'uname') else os.environ.get('COMPUTERNAME'),
               'clock_domain': 'mark_writer_host; recorder receipt is authoritative if different'}
    folder = directory / 'marks'
    folder.mkdir(exist_ok=True)
    path = folder / (request['id'] + '.json')
    save(path, request)
    return request


def output_directory(value, prefix):
    if value:
        path = Path(value)
        path.mkdir(parents=True, exist_ok=False)
        return path
    root = Path('/tmp/njrh_reports')
    root.mkdir(parents=True, exist_ok=True)
    return Path(tempfile.mkdtemp(prefix=prefix + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '_', dir=root))


def directory_size(path):
    total = 0
    for f in Path(path).rglob('*'):
        try:
            if f.is_file(): total += f.stat().st_size
        except OSError: pass
    return total


def budget_reason(used, free, cfg):
    # Reserve space to drain an already accepted queue and close sqlite indexes.
    reserve = cfg['queue_bytes'] + 8*1024*1024
    if used + reserve >= cfg['disk_budget_mb']*1024*1024:
        return 'disk_budget'
    if free < cfg['min_free_mb']*1024*1024 + reserve:
        return 'disk_free'
    return None


def run_ros(args, cfg):
    stop = threading.Event()
    interrupted = []
    previous_signals = {}
    def halt(signum, frame):
        interrupted.append(signum)
        stop.set()
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_signals[sig] = signal.signal(sig, halt)
    out = output_directory(args.output_dir, 'nav_jerk_' + args.mode + '_')
    print('report_dir=' + str(out), flush=True)
    save(out / 'session.json', {'schema': 1, 'pid': os.getpid(), 'host': os.uname().nodename,
         'started_wall_ns': time.time_ns(), 'started_monotonic_ns': time.monotonic_ns(), 'mode': args.mode, 'cfg': cfg})
    observer = sink = http = None
    tails = []
    final = {'incomplete': True, 'reason': 'initialization_not_completed'}
    try:
        observer = RosObserver(stop)
        observer.continuous_stale_sec = cfg.get('continuous_stale_sec', 2.)
        observer.spin(cfg['discovery_sec'])
        params = observer.parameters(cfg)
        save(out / 'parameters_start.json', {'monotonic_ns': time.monotonic_ns(), 'wall_ns': time.time_ns(), 'nodes': params})
        graph = observer.graph()
        topic_map = build_topic_map(graph, params, cfg, args.deep)
        for topic, item in topic_map['topics'].items():
            try:
                if len(item['types']) != 1: raise ValueError('multiple types')
                get_message_class(item['types'][0])
            except Exception as exc:
                item['excluded'] = repr(exc)
        save(out / 'topic_map.json', topic_map)
        save(out / 'graph_start.json', graph)
        start = provenance(Path(args.workspace), cfg, params, out)
        save(out / 'provenance_start.json', start)
        sim_values = {p.get('use_sim_time') for p in params.values() if 'use_sim_time' in p}
        observer.sim_time = next(iter(sim_values)) if len(sim_values) == 1 else None
        if args.mode == 'record':
            import rosbag2_py
            if cfg['storage_id'] not in rosbag2_py.get_registered_writers():
                raise RuntimeError('storage plugin not installed: ' + cfg['storage_id'])
            reason = budget_reason(directory_size(out), shutil.disk_usage(out).free, cfg)
            if reason: raise RuntimeError(reason + ' before bag creation')
            sink = BagSink(out, topic_map, cfg, stop)
        record_start = time.monotonic()
        observer.subscribe(topic_map, sink, critical_only=args.mode == 'inspect')
        observer.spin(min(cfg['ready_timeout_sec'], cfg['duration_sec']))
        coverage = observer.coverage(topic_map)
        save(out / 'readiness.json', coverage)
        save(out / 'topic_map.json', topic_map)
        print(coverage['state'] + ' capture coverage; missing=' + ','.join(coverage['missing']) + '; this is not robot readiness', flush=True)
        if args.mode == 'inspect':
            final = {'incomplete': coverage['state'] != 'READY', 'reason': 'inspect_only', 'coverage': coverage}
            return 0
        for pattern in cfg['log_globs']:
            for path in Path(args.workspace).glob(pattern):
                if path not in [t.path for t in tails]: tails.append(RotatingLog(path, cfg['log_tail_bytes']))
        http_stop = threading.Event()
        if cfg['http_enabled']:
            http = threading.Thread(target=http_worker, args=(cfg, http_stop, sink), name='nav_jerk_http')
            http.start()
        deadline = record_start + cfg['duration_sec']
        next_sample, next_graph = 0., time.monotonic()+10
        prior_process = {}
        marks_seen = set()
        reason = 'duration'
        previous_clock = None
        previous_stale = None
        stale_observed = set()
        while not stop.is_set() and time.monotonic() < deadline:
            observer.spin(.04)
            if sys.stdin.isatty():
                import select
                if select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline()
                    if line: create_mark(out, line.strip() or 'operator_jerk')
            for path in (out / 'marks').glob('*.json'):
                if path.name not in marks_seen:
                    try:
                        mark = json.loads(path.read_text(encoding='utf-8'))
                        mark['recorder_monotonic_ns'] = time.monotonic_ns()
                        sink.auxiliary('manual_mark', mark)
                        marks_seen.add(path.name)
                    except (OSError, ValueError): pass
            now = time.monotonic()
            if now >= next_sample:
                next_sample = now+1
                used, free = directory_size(out), shutil.disk_usage(out).free
                samples = [process_sample(p['pid']) for p in start['processes']] + [process_sample(os.getpid())]
                for sample in samples:
                    sample['cpu_percent_one_core'] = None
                    if sample.get('available'):
                        key = (sample['pid'], sample['process_start_ticks'])
                        ticks = sample['utime_ticks']+sample['stime_ticks']
                        if key in prior_process:
                            old_time, old_ticks = prior_process[key]
                            sample['cpu_percent_one_core'] = 100*(ticks-old_ticks)/sample['clock_ticks_per_sec']/(now-old_time)
                        prior_process[key] = (now, ticks)
                sink.auxiliary('performance', {'processes': samples,
                    'recorder_pid': os.getpid(), 'disk_used_bytes': used, 'disk_free_bytes': free, 'queue': sink.inbox.stats()})
                health = continuous_health(topic_map, observer.received, observer.last_receive,
                                           time.monotonic_ns(), cfg.get('continuous_stale_sec', 2.))
                sink.auxiliary('continuous_health', health)
                stale_observed.update(health['stale_topics'])
                if health['stale_topics'] != previous_stale:
                    print(health['state']+' continuous capture; stale='+','.join(health['stale_topics'])+
                          '; no vehicle action', flush=True)
                    previous_stale = health['stale_topics']
                clock = (time.monotonic_ns(), time.time_ns())
                if previous_clock and abs((clock[1]-previous_clock[1])-(clock[0]-previous_clock[0])) > cfg['thresholds']['clock_jump_sec']*1e9:
                    sink.auxiliary('wall_clock_jump', {'before': previous_clock, 'after': clock})
                previous_clock = clock
                for tail in tails:
                    for row in tail.read(): sink.auxiliary('log', row)
                reason_now = budget_reason(used, free, cfg)
                if reason_now:
                    reason = reason_now
                    stop.set()
            if now >= next_graph:
                next_graph = now+10
                # Inventory/matching only: never auto-modify producer QoS or visualization.
                sink.auxiliary('graph', {'graph': observer.graph(), 'coverage': observer.coverage(topic_map)})
        record_stop = time.monotonic()
        if interrupted: reason = 'signal_' + str(interrupted[-1])
        elif sink.failure: reason = 'writer_error'
        http_stop.set()
        if http: http.join()
        coverage = observer.coverage(topic_map)
        observer.unsubscribe()
        for tail in tails:
            for row in tail.read(): sink.auxiliary('log', row)
        writer = sink.close()
        sink = None
        # Stop of recording does not skip bounded final read-only parameter snapshot.
        if not observer.rclpy.ok():
            end_params = {'_snapshot': {'_error': 'ROS context unavailable'}}
        else:
            observer.stop = threading.Event()
            end_params = observer.parameters(cfg)
        save(out / 'parameters_end.json', {'monotonic_ns': time.monotonic_ns(), 'wall_ns': time.time_ns(), 'nodes': end_params})
        end_dir = out / 'end_snapshot'
        end_dir.mkdir()
        save(out / 'provenance_end.json', provenance(Path(args.workspace), cfg, end_params, end_dir))
        save(out / 'graph_end.json', observer.graph())
        http_incomplete = cfg['http_enabled'] and (writer['http']['successes'] == 0 or writer['http']['failures'] > 0)
        final = {'reason': reason, 'incomplete': reason != 'duration' or bool(writer['writer_error']) or bool(writer['dropped']) or coverage['state'] != 'READY' or http_incomplete or bool(stale_observed),
                 'continuous_topics_observed_stale': sorted(stale_observed),
                 'coverage': coverage, 'writer': writer, 'duration_record_sec': record_stop-record_start,
                 'dds_source_loss': 'unknown', 'stopped_wall_ns': time.time_ns(), 'stopped_monotonic_ns': time.monotonic_ns()}
        return 1 if writer['writer_error'] else 0
    except BaseException as exc:
        final.update(reason='exception', error=repr(exc))
        print('INCOMPLETE ' + repr(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        stop.set()
        if 'http_stop' in locals(): http_stop.set()
        if http and http.is_alive(): http.join()
        if observer: observer.unsubscribe()
        if sink: final['writer'] = sink.close()
        for tail in tails: tail.close()
        try: save(out / 'capture_result.json', final)
        except OSError as exc: print('cannot save final metadata: ' + str(exc), file=sys.stderr)
        if observer: observer.close()
        for sig, handler in previous_signals.items(): signal.signal(sig, handler)
        print('complete=' + str(out) + ' incomplete=' + str(final['incomplete']), flush=True)


def json_rows(path):
    if not Path(path).exists():
        return
    with Path(path).open(encoding='utf-8') as f:
        for line_number, line in enumerate(f, 1):
            try:
                row = json.loads(line)
                row['_line'] = line_number
                yield row
            except ValueError:
                yield {'kind': 'truncated_or_invalid_json', '_line': line_number}


def velocity(data):
    if not isinstance(data, dict):
        return None
    for _ in range(2):
        if 'twist' in data: data = data['twist']
    try:
        v = (float(data['linear']['x']), float(data['linear']['y']), float(data['angular']['z']))
        return v if all(math.isfinite(x) for x in v) else None
    except (TypeError, ValueError, KeyError):
        return None


def derivative(a, b, max_gap):
    va, vb = velocity(a.get('data')), velocity(b.get('data'))
    dt = (b['monotonic_ns']-a['monotonic_ns'])/1e9
    headers = (a.get('source_headers', []), b.get('source_headers', []))
    source_invalid = all(headers) and headers[1][0]['stamp_ns'] <= headers[0][0]['stamp_ns']
    if not va or not vb or not 0 < dt <= max_gap or source_invalid:
        return {'acceleration': None, 'dt_sec': dt, 'reason': 'missing_velocity_gap_or_nonadvancing_source_stamp'}
    return {'acceleration': [(y-x)/dt for x, y in zip(va, vb)], 'dt_sec': dt,
            'time_basis': 'receive_monotonic_estimate_not_internal_acceleration'}


def event(kind, row, **detail):
    return {'kind': kind, 'monotonic_ns': row['monotonic_ns'], 'wall_ns': row.get('wall_ns'),
            'topic': row.get('topic'), 'ref': row.get('seq'), 'index_line': row.get('_line'),
            'level': 'direct_observation_candidate_not_fault', **detail}


def command_candidates(rows, thresholds, active_windows=()):
    events = []
    previous = None
    zero_start = None
    signs = [0, 0, 0]
    turns = collections.deque()
    dead = [thresholds['linear_deadzone']]*2 + [thresholds['angular_deadzone']]
    step = [thresholds['linear_step']]*2 + [thresholds['angular_step']]
    for row in rows:
        current = velocity(row.get('data'))
        if current is None:
            previous = None
            continue
        t = row['monotonic_ns']/1e9
        if previous:
            old = velocity(previous.get('data'))
            dt = t-previous['monotonic_ns']/1e9
            continuous = 0 < dt <= thresholds['gap_sec']
            if not continuous:
                # Idle/event-source silence is not a failed command stream.
                if any(begin <= previous['monotonic_ns']/1e9 and t <= end for begin, end in active_windows):
                    events.append(event('command_gap', row, previous_ref=previous['seq'], gap_sec=dt,
                        meaning='no observed command in interval; not an observed zero; source/DDS loss unknown'))
                zero_start = None
                signs = [0, 0, 0]
            else:
                zero = all(abs(v) <= d for v, d in zip(current, dead))
                old_zero = all(abs(v) <= d for v, d in zip(old, dead))
                if zero and not old_zero:
                    events.append(event('zero_command', row, previous_ref=previous['seq']))
                    zero_start = t
                if not zero and old_zero and zero_start is not None:
                    if t-zero_start <= thresholds['short_stop_sec']:
                        events.append(event('short_stop_restart', row, stop_sec=t-zero_start, previous_ref=previous['seq']))
                    zero_start = None
                if any(abs(x-y) >= s for x, y, s in zip(current, old, step)):
                    events.append(event('velocity_step', row, before=old, after=current, previous_ref=previous['seq']))
        for axis, (v, d) in enumerate(zip(current, dead)):
            sign = 1 if v > d else -1 if v < -d else 0
            if sign and signs[axis] and sign != signs[axis]:
                events.append(event('signed_reversal', row, axis=('vx', 'vy', 'wz')[axis]))
                if axis == 2:
                    turns.append(t)
                    while turns and t-turns[0] > thresholds['repeat_window_sec']: turns.popleft()
                    if len(turns) >= thresholds['repeat_count']:
                        events.append(event('repeated_angular_reversal', row, reversals=len(turns)))
            if sign: signs[axis] = sign
        previous = row
    return events


def decoded_state(row):
    data = row.get('data') or {}
    if isinstance(data, dict) and isinstance(data.get('data'), str):
        try:
            decoded = json.loads(data['data'])
            return decoded if isinstance(decoded, dict) else {'value': decoded}
        except ValueError: return data
    return data


def pose_xy_yaw(data):
    if not isinstance(data, dict): return None
    if 'pose' in data: data = data['pose']
    if 'pose' in data: data = data['pose']
    try:
        p, q = data['position'], data['orientation']
        return (float(p['x']), float(p['y']), math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2)))
    except (KeyError, TypeError, ValueError): return None


def percentiles(values):
    if not values: return {'p50': None, 'p95': None, 'p99': None, 'max': None}
    a = sorted(values)
    return {**{name: a[round((len(a)-1)*fraction)] for name, fraction in [('p50', .5), ('p95', .95), ('p99', .99)]}, 'max': a[-1]}


def active_action_windows(rows, end_ns):
    begin, windows = None, []
    for row in sorted(rows, key=lambda r: r['monotonic_ns']):
        active = any(s.get('status') in (1, 2, 3) for s in (row.get('data') or {}).get('status_list', []))
        if active and begin is None: begin = row['monotonic_ns']/1e9
        if not active and begin is not None:
            windows.append((begin, row['monotonic_ns']/1e9))
            begin = None
    if begin is not None: windows.append((begin, end_ns/1e9))
    return windows


def write_csv(path, fields, rows):
    with Path(path).open('x', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for row in rows:
            w.writerow({k: dumps(v) if isinstance(v, (dict, list, tuple)) else v for k, v in row.items()})


def raw_bag_csv(directory, output):
    """Lossless CSV export of serialized originals. No ROS participant or replay."""
    import base64
    dbs = sorted((directory / 'bag').glob('*.db3'))
    count = 0
    with (output / 'raw_bag.csv').open('x', encoding='utf-8', newline='') as f:
        w = csv.writer(f)
        w.writerow(['storage_file', 'message_id', 'topic', 'type', 'bag_timestamp_ns', 'cdr_base64'])
        if dbs:
            for path in dbs:
                db = sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True)
                try:
                    for mid, topic, typename, stamp, raw in db.execute('SELECT m.id,t.name,t.type,m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id ORDER BY m.id'):
                        w.writerow([path.name, mid, topic, typename, stamp, base64.b64encode(raw).decode('ascii')])
                        count += 1
                finally: db.close()
        elif (directory / 'bag').exists():
            import rosbag2_py
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(directory/'bag'), storage_id=''), rosbag2_py.ConverterOptions('', ''))
            types = {t.name: t.type for t in reader.get_all_topics_and_types()}
            while reader.has_next():
                topic, raw, stamp = reader.read_next()
                count += 1
                w.writerow(['rosbag2_reader', count, topic, types[topic], stamp, base64.b64encode(raw).decode('ascii')])
    return count


def velocity_svg(path, commands, odometry, marks, max_gap):
    """Offline, raw samples only. A new polyline starts after every long gap."""
    from html import escape
    series = dict(commands)
    for row in odometry: series.setdefault(row['topic'], []).append(row)
    times = [r['monotonic_ns'] for rows in series.values() for r in rows]
    if not times: return
    begin, end = min(times), max(times)
    duration = max((end-begin)/1e9, .001)
    width, row_height = 1200, 120
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{len(series)*3*row_height+80}" role="img">',
             '<title>Raw signed velocity by receipt monotonic time; gaps are disconnected</title>',
             '<rect width="100%" height="100%" fill="white"/>',
             '<text x="20" y="24" font-family="sans-serif" font-size="16">Raw velocity / receive time (not internal latency). Red ticks: operator marks.</text>']
    panel = 0
    for topic, rows in series.items():
        for axis, label in enumerate(('vx (m/s)', 'vy (m/s)', 'wz (rad/s)')):
            values = [(r, velocity(r.get('data'))) for r in rows]
            limit = max([abs(v[axis]) for _, v in values if v] + [.05])*1.1
            y = 50+panel*row_height
            center = y+58
            lines += [f'<text x="20" y="{y+12}" font-family="sans-serif" font-size="13">{escape(topic)} — {label}; ±{limit:.3f}</text>',
                      f'<path d="M100 {center} H1180" stroke="#cbd5e1" fill="none"/>',
                      f'<text x="10" y="{center+4}" font-size="11">0</text>']
            points, last = [], None
            def finish():
                if points: lines.append('<polyline points="'+' '.join(points)+'" stroke="#1769aa" stroke-width="1" fill="none"/>')
                points.clear()
            for r, v in values:
                t = r['monotonic_ns']
                if v is None or (last is not None and (t-last)/1e9 > max_gap): finish()
                if v:
                    points.append(f'{100+1080*((t-begin)/1e9)/duration:.2f},{center-40*v[axis]/limit:.2f}')
                last = t
            finish()
            for mark in marks:
                x = 100+1080*((mark['monotonic_ns']-begin)/1e9)/duration
                if 100 <= x <= 1180: lines.append(f'<path d="M{x:.2f} {y+18} V{y+98}" stroke="#a61b29" stroke-width="1"/>')
            for fraction in (0, .25, .5, .75, 1):
                lines.append(f'<text x="{100+1080*fraction-5}" y="{y+112}" font-size="10">{duration*fraction:.1f}s</text>')
            panel += 1
    lines.append('</svg>')
    Path(path).write_text('\n'.join(lines), encoding='utf-8')


def event_window(e, commands, contexts, topic_map, cfg):
    center = e['monotonic_ns']
    begin, end = center-int(cfg['window_sec']*1e9), center+int(cfg['window_sec']*1e9)
    transitions = []
    thresholds = cfg['thresholds']
    for topic, rows in commands.items():
        for candidate in command_candidates([r for r in rows if begin <= r['monotonic_ns'] <= end], thresholds):
            transitions.append(candidate)
    if e.get('kind') in ('command_gap', 'command_silence_until_capture_end'):
        transitions.append(e)
    transitions.sort(key=lambda r: r['monotonic_ns'])
    near = [r for r in contexts if begin <= r['monotonic_ns'] <= end]
    first = transitions[0] if transitions else None
    interval = None
    if first:
        topic = first['topic']
        interval = {'observed_topic': topic, 'roles': topic_map['topics'].get(topic, {}).get('roles', []),
                    'candidate_adjacent_edges': [edge for edge in topic_map['edges'] if topic in edge['to_topics']],
                    'ref': first['ref'], 'confidence': 'observed_boundary_only_not_internal_cause'}
        comparisons = []
        for edge in interval['candidate_adjacent_edges']:
            for upstream in edge['from_topics']:
                if upstream == topic: continue
                earlier = [r for r in commands.get(upstream, []) if r['monotonic_ns'] <= first['monotonic_ns']]
                prior = earlier[-1] if earlier else None
                if prior and first['monotonic_ns']-prior['monotonic_ns'] <= cfg['thresholds']['gap_sec']*1e9:
                    comparisons.append({'upstream': upstream, 'upstream_ref': prior['seq'],
                        'upstream_velocity': velocity(prior.get('data')), 'downstream': topic, 'downstream_ref': first['ref'],
                        'receive_delta_sec_not_internal_delay': (first['monotonic_ns']-prior['monotonic_ns'])/1e9,
                        'multi_publisher_ambiguity': topic_map['topics'].get(upstream, {}).get('multi_publisher', False) or topic_map['topics'].get(topic, {}).get('multi_publisher', False)})
        interval['adjacent_received_command_comparisons'] = comparisons
    actual = []
    for r in near:
        if r.get('type') == 'nav_msgs/msg/Odometry':
            actual.append({'ref': r['seq'], 'topic': r['topic'], 'monotonic_ns': r['monotonic_ns'], 'velocity': velocity(r.get('data'))})
    response = {}
    dead = [thresholds['linear_deadzone']]*2 + [thresholds['angular_deadzone']]
    for topic in {r['topic'] for r in actual}:
        rows = sorted((r for r in actual if r['topic']==topic and r['velocity'] is not None), key=lambda r: r['monotonic_ns'])
        before = [r for r in rows if r['monotonic_ns'] <= center]
        after = [r for r in rows if r['monotonic_ns'] > center]
        base = before[-1] if before else None
        result = {'status': 'insufficient_contiguous_samples', 'baseline_ref': base['ref'] if base else None,
                  'first_changed_ref': None, 'sample_refs': []}
        if base and center-base['monotonic_ns'] <= thresholds['gap_sec']*1e9:
            previous_time = base['monotonic_ns']
            for sample in after:
                if sample['monotonic_ns']-previous_time > thresholds['gap_sec']*1e9:
                    break
                previous_time = sample['monotonic_ns']
                result['sample_refs'].append(sample['ref'])
                result['status'] = 'no_velocity_change_in_observed_samples'
                if any(abs(v-b)>d for v, b, d in zip(sample['velocity'], base['velocity'], dead)):
                    result.update(status='observed_velocity_change', first_changed_ref=sample['ref'],
                                  delta=[v-b for v, b in zip(sample['velocity'], base['velocity'])])
                    break
        response[topic] = result
    phase_rows = [r for r in contexts if r.get('kind') == 'http' and r.get('path') == '/api/v1/navigation/state' and r['monotonic_ns'] <= center]
    phase = max(phase_rows, key=lambda r: r['monotonic_ns']) if phase_rows else None
    return {**e, 'window_monotonic_ns': [begin, end], 'first_observed_boundary': interval,
            'command_candidates': transitions, 'actual_motion_samples': actual,
            'actual_motion_response': {'by_topic': response, 'meaning': 'same-topic velocity change only; command response/causality not proven'},
            'navigation_phase_snapshot': phase,
            'navigation_phase_snapshot_age_sec': (center-phase['monotonic_ns'])/1e9 if phase else None,
            'adjacent_evidence': [
                {'ref': r.get('seq'), 'aux_line': r.get('_line') if r.get('kind') else None,
                 'topic': r.get('topic'), 'kind': r.get('kind'), 'monotonic_ns': r['monotonic_ns']}
                for r in near if r.get('type') != 'nav_msgs/msg/Odometry'],
            'conclusion_level': 'direct observations + correlated clues; cause pending verification',
            'missing_items': topic_map['observability_gaps'],
            'warning': 'Normal safety stops and goal completion stops are not automatically faults. Receive order is not causality.'}


def marker_window_coverage(topic_times, topic_map, marks, window_sec):
    """Offline receive-time evidence only; no interpolation or producer-loss claims."""
    rows=[]
    for mark in marks:
        center=mark['monotonic_ns']; begin=center-int(window_sec*1e9); end=center+int(window_sec*1e9)
        for topic in topic_map['topics']:
            samples=topic_times.get(topic,[])
            left=bisect.bisect_left(samples,(begin,-1)); mid=bisect.bisect_left(samples,(center,-1))
            right=bisect.bisect_right(samples,(end,float('inf')))
            part=samples[left:right]
            state='OBSERVED' if part else ('RETAINED_BEFORE_WINDOW' if topic=='/tf_static' and left else 'MISSING_IN_WINDOW')
            rows.append({'mark_id':mark.get('id'), 'center_monotonic_ns':center,'topic':topic,
                'before_count':mid-left,'after_count':right-mid,'window_count':right-left,'state':state,
                'first_seq':part[0][1] if part else None,'last_seq':part[-1][1] if part else None,
                'max_observed_interval_sec':max(((b[0]-a[0])/1e9 for a,b in zip(part,part[1:])),default=None),
                'window_start_unobserved_sec':(part[0][0]-begin)/1e9 if part else window_sec*2,
                'window_end_unobserved_sec':(end-part[-1][0])/1e9 if part else window_sec*2})
    return rows


def report(args, cfg):
    directory = Path(args.directory)
    session = json.loads((directory / 'session.json').read_text(encoding='utf-8'))
    if not args.config_explicit: cfg = session['cfg']
    topic_map = json.loads((directory / 'topic_map.json').read_text(encoding='utf-8'))
    output = output_directory(args.output_dir or str(directory / ('report_' + uuid.uuid4().hex[:8])), 'nav_jerk_report_')
    commands, contexts, action_rows = collections.defaultdict(list), [], []
    topic_times = collections.defaultdict(list)
    intervals, ages = collections.defaultdict(list), collections.defaultdict(list)
    counts, previous, source_previous, duplicates, regressions = collections.Counter(), {}, {}, collections.Counter(), collections.Counter()
    events, clocks = [], None
    last_state, mode_changes, handoffs = {}, collections.deque(), collections.deque()
    full_grids, updates_before_full = set(), collections.Counter()
    errors, motion_rows = [], []
    end_ns = session['started_monotonic_ns']
    timeline_fields = ['seq', 'monotonic_ns', 'wall_ns', 'ros_ns', 'source_headers', 'topic', 'type', 'publisher_gid', 'bag_timestamp_ns', 'topic_ordinal', 'vx', 'vy', 'wz', 'data_json', 'decode_error']
    with (output / 'timeline.csv').open('x', encoding='utf-8', newline='') as timeline:
        writer = csv.DictWriter(timeline, fieldnames=timeline_fields, extrasaction='ignore')
        writer.writeheader()
        for r in json_rows(directory / 'index.jsonl'):
            if 'seq' not in r:
                errors.append(r)
                continue
            topic = r['topic']
            topic_times[topic].append((r['monotonic_ns'],r['seq']))
            counts[topic] += 1
            end_ns = max(end_ns, r['monotonic_ns'])
            if topic in previous:
                gap = (r['monotonic_ns']-previous[topic]['monotonic_ns'])/1e9
                intervals[topic].append(gap)
                if gap > cfg['thresholds']['gap_sec'] and topic_map['topics'].get(topic, {}).get('cadence') == 'continuous':
                    events.append(event('observed_stream_gap', r, previous_ref=previous[topic]['seq'], gap_sec=gap))
            stamps = r.get('source_headers', [])
            if stamps:
                key = (topic, stamps[0].get('frame'))
                stamp = stamps[0]['stamp_ns']
                old = source_previous.get(key)
                if old is not None:
                    duplicates[topic] += stamp == old
                    regressions[topic] += stamp < old
                    if stamp < old and not topic_map['topics'].get(topic, {}).get('multi_publisher'):
                        events.append(event('source_stamp_regression', r, previous_stamp_ns=old))
                source_previous[key] = stamp
                if cfg['source_clock_verified'] and stamp > 0 and r.get('ros_ns') is not None:
                    ages[topic].append((r['ros_ns']-stamp)/1e9)
            if clocks:
                dm = r['monotonic_ns']-clocks['monotonic_ns']
                if abs(r['wall_ns']-clocks['wall_ns']-dm) > cfg['thresholds']['clock_jump_sec']*1e9:
                    events.append(event('wall_clock_jump', r))
                if r.get('ros_ns') is not None and clocks.get('ros_ns') is not None:
                    delta = r['ros_ns']-clocks['ros_ns']
                    if abs(delta-dm) > cfg['thresholds']['clock_jump_sec']*1e9:
                        events.append(event('ros_clock_jump', r, ros_delta_ns=delta, monotonic_delta_ns=dm,
                            meaning='ROS vs monotonic progression differs; simulation pause/rate changes are not automatically faults'))
                    if delta < 0:
                        events.append(event('ros_clock_regression', r))
            clocks = r
            v = velocity(r.get('data'))
            outrow = {k: r.get(k) for k in timeline_fields}
            outrow.update(vx=v[0] if v else None, vy=v[1] if v else None, wz=v[2] if v else None,
                          data_json=dumps(r.get('data')), source_headers=dumps(stamps))
            writer.writerow(outrow)
            if r['type'] in VELOCITIES:
                commands[topic].append(r)
            elif r['type'] == 'nav_msgs/msg/Odometry':
                contexts.append(r)
                motion_rows.append(r)
            elif r['type'] == 'nav_msgs/msg/OccupancyGrid':
                full_grids.add(topic)
            elif r['type'] == 'map_msgs/msg/OccupancyGridUpdate':
                if topic.removesuffix('_updates') not in full_grids:
                    updates_before_full[topic] += 1
            elif r['type'] not in LARGE | {'sensor_msgs/msg/Imu', 'sensor_msgs/msg/LaserScan'}:
                data = decoded_state(r)
                if r['type'] == 'action_msgs/msg/GoalStatusArray' and '/navigate_to_pose/' in topic:
                    action_rows.append(r)
                state = dumps(data)
                if state != last_state.get(topic):
                    contexts.append(r)
                    if r['type'] == 'nav2_msgs/msg/SpeedLimit':
                        events.append(event('speed_limit_changed', r, value=data, semantics='percentage' if data.get('percentage') else 'absolute; effective internal MPPI constraints unproven'))
                    if 'actual_motion_mode' in data:
                        code = data['actual_motion_mode'].get('code')
                        old_mode = last_state.get(topic + ':mode')
                        if old_mode is not None and code != old_mode:
                            mode_changes.append(r['monotonic_ns'])
                            while mode_changes and r['monotonic_ns']-mode_changes[0] > cfg['thresholds']['repeat_window_sec']*1e9: mode_changes.popleft()
                            events.append(event('mode_switch', r, before=old_mode, after=code))
                            if len(mode_changes) >= cfg['thresholds']['repeat_count']: events.append(event('frequent_mode_switch', r))
                        last_state[topic + ':mode'] = code
                    if r['type'] == 'geometry_msgs/msg/PoseWithCovarianceStamped' and topic in previous:
                        a, b = pose_xy_yaw(previous[topic].get('data')), pose_xy_yaw(r.get('data'))
                        if a and b and (math.hypot(b[0]-a[0], b[1]-a[1]) > cfg['thresholds']['position_jump_m'] or abs(math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))) > cfg['thresholds']['yaw_jump_rad']):
                            events.append(event('localization_pose_change', r, before=a, after=b, meaning='pose samples changed; motion/correction not distinguished'))
                    if r['type'] == 'rcl_interfaces/msg/Log' and re.search(r'handoff|terminal.*(start|enter|takeover)', str(data.get('msg', '')), re.I):
                        handoffs.append(r['monotonic_ns'])
                        while handoffs and r['monotonic_ns']-handoffs[0] > cfg['thresholds']['repeat_window_sec']*1e9: handoffs.popleft()
                        events.append(event('terminal_handoff_log', r, text=data.get('msg')))
                        if len(handoffs) >= cfg['thresholds']['repeat_count']: events.append(event('repeated_terminal_handoff_candidate', r))
                    last_state[topic] = state
            previous[topic] = r
    windows = active_action_windows(action_rows, end_ns)
    chain_topics = {t for key in ('controller_output', 'smoother_output', 'collision_output', 'safety_final_output') for t in topic_map['roles'].get(key, [])}
    for topic, rows in commands.items():
        events.extend(command_candidates(rows, cfg['thresholds'], windows if topic in chain_topics else ()))
        if topic in chain_topics and rows:
            last = rows[-1]
            gap = (end_ns-last['monotonic_ns'])/1e9
            if gap > cfg['thresholds']['gap_sec'] and any(a <= last['monotonic_ns']/1e9 and end_ns/1e9 <= b for a, b in windows):
                events.append(event('command_silence_until_capture_end', last, gap_sec=gap,
                    meaning='last observed active Action; no later command; not an observed zero'))
    for row in json_rows(directory / 'auxiliary.jsonl'):
        if 'monotonic_ns' not in row:
            errors.append(row)
            continue
        contexts.append(row)
        if row.get('kind') == 'manual_mark':
            # mark command can run in another namespace/host. Never subtract unrelated clocks.
            center = row['monotonic_ns'] if row.get('host') == session.get('host') else row['recorder_monotonic_ns']
            events.append({**row, 'monotonic_ns': center, 'kind': 'manual_mark', 'level': 'operator_observation'})
        elif row.get('kind') in ('wall_clock_jump', 'incompatible_qos', 'rmw_reported_message_lost'):
            events.append(row)
    ingested_marks = {e.get('id') for e in events if e.get('kind') == 'manual_mark'}
    for path in (directory / 'marks').glob('*.json'):
        try:
            mark = json.loads(path.read_text(encoding='utf-8'))
            if mark['id'] in ingested_marks:
                continue
            if mark.get('host') != session.get('host'):
                errors.append({'marker_file': path.name, 'gap': 'foreign-host marker not ingested; no comparable capture time'})
                continue
            events.append({**mark, 'kind': 'manual_mark', 'level': 'operator_observation',
                           'not_ingested_before_stop': True, 'marker_file': path.name})
        except (OSError, ValueError, KeyError) as exc:
            errors.append({'marker_file': path.name, 'error': repr(exc)})
    quality = {'topics': {}, 'costmap': {'initial_full_topics': sorted(full_grids), 'updates_before_initial_full': dict(updates_before_full),
               'complete_reconstruction_proven': False, 'reason': 'initial full + all increment sequence/delivery not yet proven; raw snapshots only'},
               'raw_index_errors': errors, 'source_age_basis': 'explicit operator-verified ROS source clock' if cfg['source_clock_verified'] else 'unknown: clock synchronization not verified'}
    for topic, item in topic_map['topics'].items():
        quality['topics'][topic] = {'count': counts[topic], 'observed_interval_sec': percentiles(intervals[topic]),
            'comparable_source_age_sec': percentiles(ages[topic]) if cfg['source_clock_verified'] else None,
            'duplicate_source_stamps': duplicates[topic], 'regressing_source_stamps': regressions[topic],
            'multi_publisher': item['multi_publisher'], 'source_stamp_counters_ambiguous': item['multi_publisher'], 'dds_or_source_loss': 'unknown'}
    if (directory/'capture_result.json').exists(): quality['capture_result'] = json.loads((directory/'capture_result.json').read_text(encoding='utf-8'))
    try: quality['raw_bag_exported_count'] = raw_bag_csv(directory, output)
    except Exception as exc: quality['raw_bag_export_error'] = repr(exc)
    estimate_rows = []
    for topic, rows in {**commands, **{t: [r for r in motion_rows if r['topic'] == t] for t in {r['topic'] for r in motion_rows}}}.items():
        acceleration = None
        for a, b in zip(rows, rows[1:]):
            estimate = derivative(a, b, cfg['thresholds']['gap_sec'])
            jerk = [(v-o)/estimate['dt_sec'] for v, o in zip(estimate['acceleration'], acceleration)] if estimate['acceleration'] and acceleration else None
            estimate_rows.append({'seq': b['seq'], 'topic': topic, 'monotonic_ns': b['monotonic_ns'], **estimate, 'jerk': jerk})
            acceleration = estimate['acceleration']
    write_csv(output/'derivative_estimates.csv', ['seq', 'topic', 'monotonic_ns', 'dt_sec', 'time_basis', 'acceleration', 'jerk', 'reason'], estimate_rows)
    events.sort(key=lambda e: e['monotonic_ns'])
    marker_rows = marker_window_coverage(topic_times, topic_map,
                                        [e for e in events if e['kind']=='manual_mark'], cfg['window_sec'])
    write_csv(output/'marker_coverage.csv', ['mark_id','center_monotonic_ns','topic','before_count',
        'after_count','window_count','state','first_seq','last_seq','max_observed_interval_sec',
        'window_start_unobserved_sec','window_end_unobserved_sec'], marker_rows)
    quality['manual_marker_coverage'] = marker_rows
    save(output/'quality.json', quality)
    write_csv(output/'events.csv', ['kind', 'monotonic_ns', 'wall_ns', 'topic', 'ref', 'previous_ref', 'level'], events)
    with (output/'events.jsonl').open('x', encoding='utf-8') as f:
        for e in events: f.write(dumps(event_window(e, commands, contexts, topic_map, cfg))+'\n')
    velocity_svg(output/'velocity.svg', commands, motion_rows, [e for e in events if e['kind'] == 'manual_mark'], cfg['thresholds']['gap_sec'])
    write_csv(output/'state_events.csv', ['seq', 'monotonic_ns', 'wall_ns', 'topic', 'kind', 'data', 'path', 'text'], contexts)
    summary = ['# Navigation jerk capture (offline)', '',
        '- Candidate events: '+str(len(events))+'; no candidate is automatically a navigation fault.',
        '- Original payload: bag/ and raw_bag.csv (lossless CDR base64); join by topic + bag_timestamp_ns to index.jsonl / timeline.csv.',
        '- The index bag timestamp is a unique storage key; original wall/monotonic/ROS/header times are separate.',
        '- CSV preserves signed vx/vy/wz. Derivatives use receive monotonic intervals, reject gaps/nonadvancing source stamps; no offline filter applied.',
        '- velocity.svg: raw signed chain/odometry traces on one receive-time axis; gaps are disconnected, no interpolation.',
        '- IMU: reported frame/covariances preserved; gravity unknown; corrected/raw stream names are not calibration proof.',
        '- Event windows: +/- '+str(cfg['window_sec'])+' s. See events.jsonl for phase, first observed boundary, evidence and gaps.',
        '- marker_coverage.csv: per-marker/per-topic counts, raw sequence references and edge gaps; cumulative session counts cannot prove window coverage.',
        '- No interpolation, no proof of internal latency/causality; velocity command interval is not controller compute time.',
        '- Capture complete: '+str(not quality.get('capture_result', {}).get('incomplete', True)), '', '## Observability gaps', '']
    summary.extend('- '+gap for gap in topic_map['observability_gaps'])
    summary.extend(['', '## Coverage', '', '|Topic|Messages|Interval P95 (s)|Multiple publishers|', '|---|---:|---:|---|'])
    for t, q in quality['topics'].items(): summary.append(f"|{t}|{q['count']}|{q['observed_interval_sec']['p95']}|{q['multi_publisher']}|")
    (output/'summary.md').write_text('\n'.join(summary)+'\n', encoding='utf-8')
    print('summary=' + str(output/'summary.md'))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('inspect', 'record', 'mark', 'report'))
    parser.add_argument('--config', default=str(Path(__file__).with_suffix('.yaml')))
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parents[2]))
    parser.add_argument('--output-dir', help='new directory; never overwrite an existing capture/report')
    parser.add_argument('--directory', help='existing capture for mark/report')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--disk-budget-mb', type=float)
    parser.add_argument('--min-free-mb', type=float)
    parser.add_argument('--label', default='operator_jerk')
    parser.add_argument('--deep', action='store_true', help='explicitly include observed relevant PointCloud2 (high cost)')
    parser.add_argument('--no-http', action='store_true')
    args = parser.parse_args(argv)
    args.config_explicit = '--config' in (argv if argv is not None else sys.argv[1:])
    cfg = load_config(args.config)
    for attr, key in [('duration', 'duration_sec'), ('disk_budget_mb', 'disk_budget_mb'), ('min_free_mb', 'min_free_mb')]:
        if getattr(args, attr) is not None: cfg[key] = getattr(args, attr)
    if args.no_http: cfg['http_enabled'] = False
    if args.mode in ('mark', 'report') and not args.directory:
        parser.error('--directory is required')
    if args.mode == 'mark':
        print(dumps(create_mark(args.directory, args.label)))
        return 0
    if args.mode == 'report': return report(args, cfg)
    if cfg['duration_sec'] <= 0 or cfg['disk_budget_mb'] <= 0 or cfg['min_free_mb'] < 0:
        parser.error('invalid duration/disk budget')
    return run_ros(args, cfg)


if __name__ == '__main__':
    sys.exit(main())
