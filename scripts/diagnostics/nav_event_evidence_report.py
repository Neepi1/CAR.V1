#!/usr/bin/env python3
"""Offline evidence inspection; stdlib only, no ROS entities or network access.

Join CDR payloads to receive metadata using (topic, bag_timestamp_ns), never header
time. A bag timestamp is a storage key, NOT a source acquisition timestamp.
Published maps are not MPPI's internal per-compute map. This module never invents
a trajectory or interprets callback arrival order as causal internal latency.
"""
import bisect
import collections
import csv
import json
import math
from pathlib import Path
import sqlite3
import struct


class Cdr:
    """Bounded ROS 2 XCDR1 reader, alignment relative to encapsulated payload."""
    def __init__(self, data):
        self.data = memoryview(data)
        if len(data) < 4 or bytes(data[:2]) not in (b'\0\0', b'\0\1'):
            raise ValueError('unsupported/truncated CDR encapsulation')
        self.order = '<' if data[1] == 1 else '>'
        self.pos = 4

    def number(self, kind):
        size = struct.calcsize(kind)
        self.pos += -(self.pos - 4) % size
        if self.pos + size > len(self.data):
            raise ValueError('truncated CDR scalar')
        value = struct.unpack_from(self.order + kind, self.data, self.pos)[0]
        self.pos += size
        return value

    def string(self):
        length = self.number('I')
        if length < 1 or length > len(self.data)-self.pos or self.data[self.pos+length-1] != 0:
            raise ValueError('invalid CDR string')
        value = bytes(self.data[self.pos:self.pos+length-1]).decode('utf-8', errors='strict')
        self.pos += length
        return value

    def count(self, minimum=1):
        count = self.number('I')
        if count > (len(self.data)-self.pos)//minimum:
            raise ValueError('CDR sequence exceeds payload')
        return count

    def array(self, kind, count=None):
        if count is None:
            count = self.count(struct.calcsize(kind))
        return [self.number(kind) for _ in range(count)]

    def stamp(self):
        sec, nano = self.number('i'), self.number('I')
        if nano >= 10**9:
            raise ValueError('invalid ROS nanoseconds')
        return sec * 10**9 + nano

    def header(self):
        return {'stamp_ns': self.stamp(), 'frame_id': self.string()}

    def vector(self, kind='d'):
        return dict(zip(('x', 'y', 'z'), self.array(kind, 3)))

    def quaternion(self):
        return dict(zip(('x', 'y', 'z', 'w'), self.array('d', 4)))

    def pose(self):
        return {'position': self.vector(), 'orientation': self.quaternion()}

    def twist(self):
        return {'linear': self.vector(), 'angular': self.vector()}


def decode_cdr(typename, payload):
    """Decode selected installed-standard ROS message layouts, fail closed otherwise.

    Unknown custom messages remain losslessly addressable in the original bag;
    they are explicitly unavailable to this decoder, not replaced with zeros.
    """
    r = Cdr(payload)
    if typename == 'geometry_msgs/msg/Twist':
        out = r.twist()
    elif typename == 'geometry_msgs/msg/TwistStamped':
        out = {'header': r.header(), 'twist': r.twist()}
    elif typename == 'nav_msgs/msg/Odometry':
        out = {'header': r.header(), 'child_frame_id': r.string(), 'pose': r.pose(),
               'pose_covariance': r.array('d', 36), 'twist': r.twist(),
               'twist_covariance': r.array('d', 36)}
    elif typename == 'sensor_msgs/msg/LaserScan':
        out = {'header': r.header()}
        for key in ('angle_min', 'angle_max', 'angle_increment', 'time_increment', 'scan_time', 'range_min', 'range_max'):
            out[key] = r.number('f')
        out.update(ranges=r.array('f'), intensities=r.array('f'))
    elif typename == 'tf2_msgs/msg/TFMessage':
        out = {'transforms': []}
        for _ in range(r.count(20)):
            out['transforms'].append({'header': r.header(), 'child_frame_id': r.string(),
                                      'translation': r.vector(), 'rotation': r.quaternion()})
    elif typename == 'nav_msgs/msg/Path':
        out = {'header': r.header(), 'poses': []}
        for _ in range(r.count(20)):
            out['poses'].append({'header': r.header(), 'pose': r.pose()})
    elif typename == 'geometry_msgs/msg/PolygonStamped':
        out = {'header': r.header(), 'points': [r.vector('f') for _ in range(r.count(12))]}
    elif typename == 'nav_msgs/msg/OccupancyGrid':
        out = {'header': r.header(), 'map_load_time_ns': r.stamp(), 'resolution': r.number('f'),
               'width': r.number('I'), 'height': r.number('I'), 'origin': r.pose()}
        out['data'] = r.array('b')
        if len(out['data']) != out['width']*out['height']:
            raise ValueError('occupancy grid dimensions/payload mismatch')
    elif typename == 'map_msgs/msg/OccupancyGridUpdate':
        out = {'header': r.header(), 'x': r.number('i'), 'y': r.number('i'),
               'width': r.number('I'), 'height': r.number('I'), 'data': r.array('b')}
        if len(out['data']) != out['width']*out['height']:
            raise ValueError('occupancy update dimensions/payload mismatch')
    elif typename == 'nav2_msgs/msg/Costmap':
        # Verified against installed Humble nav2_msgs/Costmap{,MetaData}.msg.
        out = {'header': r.header(), 'map_load_time_ns': r.stamp(), 'update_time_ns': r.stamp(),
               'layer': r.string(), 'resolution': r.number('f'), 'width': r.number('I'),
               'height': r.number('I'), 'origin': r.pose(), 'data': r.array('B')}
        if len(out['data']) != out['width']*out['height']:
            raise ValueError('Nav2 costmap dimensions/payload mismatch')
    elif typename == 'sensor_msgs/msg/Imu':
        out = {'header': r.header(), 'orientation': r.quaternion(), 'orientation_covariance': r.array('d', 9),
               'angular_velocity': r.vector(), 'angular_velocity_covariance': r.array('d', 9),
               'linear_acceleration': r.vector(), 'linear_acceleration_covariance': r.array('d', 9)}
    elif typename == 'geometry_msgs/msg/PoseWithCovarianceStamped':
        out = {'header': r.header(), 'pose': r.pose(), 'covariance': r.array('d', 36)}
    elif typename == 'nav2_msgs/msg/SpeedLimit':
        out = {'header': r.header(), 'percentage': bool(r.number('B')), 'speed_limit': r.number('d')}
    elif typename == 'std_msgs/msg/String':
        out = {'data': r.string()}
    elif typename == 'std_msgs/msg/Bool':
        out = {'data': bool(r.number('B'))}
    else:
        raise NotImplementedError('CDR decoder unavailable for ' + typename)
    # XCDR1 serializers may retain up to 7 bytes of final alignment padding.
    remaining = bytes(r.data[r.pos:])
    if len(remaining) > 7 or any(remaining):
        raise ValueError('unconsumed CDR payload; message layout not verified')
    return out


def _clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else ('Infinity' if value > 0 else '-Infinity')
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_clean(v) for v in value]
    return value


def _json(value):
    return json.dumps(_clean(value), ensure_ascii=False, separators=(',', ':'), allow_nan=False)


def _load(path, default=None, gaps=None, scope='motion'):
    if not path.exists():
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (ValueError,UnicodeError) as exc:
        if gaps is None:
            raise
        _gap(gaps,'metadata_json_invalid',scope=scope,path=str(path),error=str(exc))
        return default


def _lines(path, gaps=None, scope='motion'):
    if path.exists():
        with path.open(encoding='utf-8') as stream:
            for lineno, line in enumerate(stream, 1):
                if line.strip():
                    try:
                        yield lineno, json.loads(line)
                    except (ValueError,UnicodeError) as exc:
                        if gaps is None:
                            raise
                        _gap(gaps,'jsonl_record_invalid',scope=scope,path=str(path),line=lineno,error=str(exc))


def _writer(path, fields):
    stream = path.open('w', newline='', encoding='utf-8')
    writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
    writer.writeheader()
    return stream, writer


def _gap(gaps, code, scope='motion', blocking=True, **detail):
    gaps.append(dict(code=code, scope=scope, blocking=blocking, **detail))


def _percentile(values, percentile):
    if not values:
        return None
    values = sorted(values)
    return values[min(len(values)-1, int((len(values)-1)*percentile))]


def _velocity(data):
    v = data.get('twist', data)
    if not isinstance(v, dict) or 'linear' not in v or 'angular' not in v:
        return None
    return [v['linear']['x'], v['linear']['y'], v['angular']['z']]


def _pre_map_failure(frame):
    """Exact producer stages before its map copy, not a generic missing-map waiver.

    mppi_controller.cpp assigns these stages before reset/parameter lock/path
    transformation/map lock. Payload descriptors still undergo normal bounds and
    dtype validation below; a later-stage or partially copied map is not exempt.
    """
    if (frame.get('command_returned') is not False or frame.get('sequence_valid') is not False or
            frame.get('stage') not in ('idle_reset','parameters_lock','transformPath','costmap_lock') or
            not isinstance(frame.get('exception'),str) or not frame['exception'].strip() or
            not isinstance(frame.get('exception_type'),str) or not frame['exception_type'].strip()):
        return False
    if (frame.get('map_dimensions') != [0,0] or frame.get('map_copy_monotonic_ns') != 0 or
            frame.get('footprint') != [] or frame.get('resolution') != 0 or
            any(frame.get(key) != '' for key in ('map_frame','path_frame','base_frame'))):
        return False
    for key,shape in (('costmap',[0,0]),('path',[0,7]),('path_stamps',[0]),('sequence',[0,3])):
        block=frame.get(key)
        if not isinstance(block,dict) or block.get('shape') != shape or block.get('length') != 0:
            return False
    return frame.get('path_original_count') == 0 and frame.get('sequence_original_count') == 0


def _mppi_report(output, destination, marks, window, gaps):
    result = {'complete': False, 'instances': [], 'frames': 0,
              'marker_coverage': [], 'pre_map_failure_frames': 0, 'observations': [],
              'geometry_assessment': 'not_computed: no independent approximate model; use same-model offline replay',
              'observability_gaps': ['Internal odom has no source stamp/covariance.',
                  'Internal grid copy time is compute time, not sensor acquisition time.',
                  'Predicted control sequence is not actual wheel motion.']}
    roots = sorted(p for p in output.glob('mppi_*') if p.is_dir())
    if not roots:
        _gap(gaps, 'mppi_snapshot_missing', scope='mppi')
        return result
    stream, writer = _writer(destination/'mppi_frames.csv', ['instance', 'frame_line', 'compute_seq',
        'start_monotonic_ns', 'end_monotonic_ns', 'map_frame', 'pose_frame', 'path_frame',
        'stage', 'command_returned', 'command_json', 'exception', 'valid', 'evidence_class',
        'geometry_available', 'issues_json', 'raw_metadata_json'])
    all_times = []
    valid_times = []
    try:
        for root in roots:
            start_errors = len(gaps)
            schema = _load(root/'schema.json', {}, gaps, 'mppi')
            status = _load(root/'status.json', {}, gaps, 'mppi')
            payload_path = root/'payload.bin'
            size = payload_path.stat().st_size if payload_path.exists() else 0
            if schema.get('schema') != 1 or schema.get('byte_order') not in ('little', 'big'):
                _gap(gaps, 'mppi_schema_invalid', scope='mppi', instance=root.name)
            if not status.get('complete'):
                _gap(gaps, 'mppi_capture_not_confirmed_complete', scope='mppi', instance=root.name, status=status)
            for field in ('dropped', 'dropped_frames', 'write_errors', 'stale_generations'):
                if status.get(field, 0):
                    _gap(gaps, 'mppi_transport_loss', scope='mppi', instance=root.name, field=field, count=status[field])
            last_seq = None
            last_end = None
            count = 0
            for lineno, frame in _lines(root/'frames.jsonl', gaps, 'mppi'):
                issues = []
                pre_map_failure = _pre_map_failure(frame)
                count += 1
                seq = frame.get('compute_seq')
                start, end = frame.get('start_monotonic_ns'), frame.get('end_monotonic_ns')
                if not isinstance(seq, int) or (last_seq is not None and seq <= last_seq):
                    issues.append('compute_seq_missing_or_not_increasing')
                elif last_seq is not None and seq != last_seq+1:
                    issues.append('compute_seq_gap')
                last_seq = seq
                if not isinstance(start, int) or not isinstance(end, int) or end < start:
                    issues.append('compute_interval_invalid')
                else:
                    all_times.append(start)
                    if last_end is not None and start < last_end:
                        issues.append('compute_intervals_overlap_or_clock_mismatch')
                    last_end = end
                if frame.get('incomplete'):
                    issues.append('producer_incomplete')
                if schema.get('session') and frame.get('session') != schema['session']:
                    issues.append('session_mismatch')
                for field in (('pose_frame',) if pre_map_failure else ('map_frame', 'pose_frame', 'path_frame', 'base_frame')):
                    if not frame.get(field):
                        issues.append(field+'_missing')
                if not pre_map_failure and (frame.get('map_frame') != frame.get('pose_frame') or frame.get('map_frame') != frame.get('path_frame')):
                    issues.append('frame_transform_required_not_proven')
                if frame.get('pose_stamp_ns') is None or frame.get('path_stamp_ns') is None:
                    issues.append('source_stamp_field_missing')
                footprint = frame.get('footprint', [])
                if not pre_map_failure and (len(footprint)<3 or any(len(p)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in p) for p in footprint)):
                    issues.append('footprint_missing_or_invalid')
                if not pre_map_failure and (not isinstance(frame.get('resolution'),(int,float)) or frame['resolution']<=0):
                    issues.append('resolution_invalid')
                blocks = {}
                for key, dtype in (('costmap', 'u1'), ('path', 'f8'), ('path_stamps', 'i8'), ('sequence', 'f8')):
                    block = frame.get(key)
                    if not isinstance(block, dict):
                        if key != 'sequence' or frame.get('sequence_valid'):
                            issues.append(key+'_missing')
                        continue
                    shape = block.get('shape', [])
                    offset, length = block.get('offset'), block.get('length')
                    expected = math.prod(shape)*(1 if dtype=='u1' else 8) if shape and all(isinstance(x,int) and x>=0 for x in shape) else -1
                    if block.get('dtype') != dtype or not isinstance(offset,int) or not isinstance(length,int) or offset<0 or length<0 or length != expected or offset+length > size:
                        issues.append(key+'_payload_invalid')
                    else:
                        blocks[key] = (offset, offset+length)
                ordered = sorted((a,b,k) for k,(a,b) in blocks.items() if b>a)
                if any(ordered[i][1] > ordered[i+1][0] for i in range(len(ordered)-1)):
                    issues.append('overlapping_payload_blocks')
                grid_shape = frame.get('costmap', {}).get('shape')
                dimensions = frame.get('map_dimensions')
                if not dimensions or grid_shape != list(reversed(dimensions)):
                    issues.append('costmap_dimensions_mismatch')
                path_shape = frame.get('path', {}).get('shape', [])
                if path_shape and (len(path_shape)!=2 or path_shape[1]!=7 or frame.get('path_stamps',{}).get('shape') != [path_shape[0]]):
                    issues.append('path_layout_mismatch')
                if path_shape and frame.get('path_original_count', path_shape[0]) != path_shape[0]:
                    issues.append('path_truncated')
                if not frame.get('path_frames_all_equal_header', False) and path_shape and len(frame.get('path_frames', [])) != path_shape[0]:
                    issues.append('individual_path_frames_unproven')
                seq_shape = frame.get('sequence', {}).get('shape', [])
                if frame.get('sequence_valid') and (len(seq_shape)!=2 or seq_shape[1]!=3 or frame.get('sequence_original_count',seq_shape[0])!=seq_shape[0]):
                    issues.append('sequence_layout_or_truncation')
                if frame.get('command_returned') and not frame.get('sequence_valid'):
                    issues.append('returned_command_without_valid_sequence')
                if frame.get('sequence_valid') and 'sequence' in blocks and len(seq_shape)==2 and seq_shape[1]==3:
                    offset = frame.get('command_offset')
                    if not isinstance(offset,int) or not 0<=offset<seq_shape[0]:
                        issues.append('selected_command_offset_invalid')
                    elif schema.get('byte_order') in ('little','big'):
                        with payload_path.open('rb') as raw:
                            raw.seek(blocks['sequence'][0]+offset*24)
                            selected=struct.unpack(('<' if schema['byte_order']=='little' else '>')+'3d',raw.read(24))
                        command=frame.get('command', [])
                        if len(command)!=6 or not all(math.isfinite(v) for v in selected):
                            issues.append('command_values_invalid')
                        elif frame.get('command_returned') and any(abs(a-b)>1e-6 for a,b in zip(selected,(command[0],command[1],command[5]))):
                            issues.append('selected_sequence_command_mismatch')
                if issues:
                    _gap(gaps, 'mppi_payload_invalid', scope='mppi', instance=root.name, frame_line=lineno, compute_seq=seq, issues=issues)
                else:
                    valid_times.append(start)
                    if pre_map_failure:
                        result['pre_map_failure_frames'] += 1
                        result['observations'].append(dict(kind='pre_map_compute_failure_recorded',
                            instance=root.name,frame_line=lineno,compute_seq=seq,stage=frame.get('stage'),
                            exception_type=frame.get('exception_type'),exception=frame.get('exception'),
                            geometry_available=False,blocking=False,
                            explanation='Original failure recorded before map copy; no internal map or trajectory was generated.'))
                writer.writerow(dict(instance=root.name, frame_line=lineno, compute_seq=seq,
                    start_monotonic_ns=start, end_monotonic_ns=end, map_frame=frame.get('map_frame'),
                    pose_frame=frame.get('pose_frame'), path_frame=frame.get('path_frame'), stage=frame.get('stage'),
                    command_returned=frame.get('command_returned'), command_json=_json(frame.get('command')),
                    exception=frame.get('exception'), valid=not issues,
                    evidence_class='pre_map_compute_failure_recorded' if pre_map_failure and not issues else 'internal_compute_frame',
                    geometry_available=not pre_map_failure and not issues,
                    issues_json=_json(issues), raw_metadata_json=_json(frame)))
            if not count:
                _gap(gaps, 'mppi_frames_missing', scope='mppi', instance=root.name)
            result['instances'].append(dict(directory=str(root), frames=count, valid=len(gaps)==start_errors,
                                            schema=schema, status=status))
            result['frames'] += count
    finally:
        stream.close()
    all_times.sort()
    valid_times.sort()
    for mark in marks:
        center = mark.get('read_monotonic_ns')
        if isinstance(center,int):
            left,right = center-int(window*1e9),center+int(window*1e9)
            times = all_times[bisect.bisect_left(all_times,left):bisect.bisect_right(all_times,right)]
            valid = valid_times[bisect.bisect_left(valid_times,left):bisect.bisect_right(valid_times,right)]
            intervals=[(b-a)/1e9 for a,b in zip(times,times[1:])]
            complete=bool(valid) and len(valid)==len(times) and valid[0]-left<=500000000 and right-valid[-1]<=500000000 and max(intervals or [0])<=.5
            result['marker_coverage'].append(dict(marker=mark.get('label'),start_monotonic_ns=left,end_monotonic_ns=right,
                frames=len(times),structurally_valid_frames=len(valid),max_observed_interval_sec=max(intervals) if intervals else None,
                complete=complete,coverage_limit_sec=.5,
                note='Missing periods may be MPPI inactivity or missing capture; no stage inference from file presence.'))
            if not times:
                _gap(gaps, 'mppi_marker_window_missing', scope='mppi', marker=mark.get('label'), center_monotonic_ns=center,
                     explanation='No compute snapshot in this window; active controller/stage cannot be inferred.')
            elif not complete:
                _gap(gaps,'mppi_marker_window_partial',scope='mppi',marker=mark.get('label'),frames=len(times),valid_frames=len(valid),
                     explanation='Sparse/invalid internal frames; MPPI activity coverage is not proven for the complete marker window.')
    result['complete'] = result['frames'] > 0 and not any(g['blocking'] and g['scope']=='mppi' for g in gaps)
    return result


def build_report(output, destination, window=5):
    """Export passive evidence. Return dict with coverage/gaps and scoped completeness.

    ``motion_complete`` means indexed/decoded required streams cover the requested
    marker windows under the declared observation gap limits, NOT source loss-free
    capture, safety acceptance, or proof of causality. ``complete`` also requires
    a closed, structurally valid MPPI snapshot capture.
    """
    output, destination = Path(output), Path(destination)
    if not math.isfinite(window) or window <= 0:
        raise ValueError('window must be positive finite seconds')
    if destination.resolve() == output.resolve() or destination.resolve() == (output/'motion').resolve():
        raise ValueError('destination must not overwrite capture root/motion')
    destination.mkdir(parents=True, exist_ok=True)
    gaps, coverage = [], []
    motion = output/'motion'
    topic_map = _load(motion/'topic_map.json', {}, gaps)
    topics = topic_map.get('topics', [])
    if not topics:
        _gap(gaps, 'topic_map_missing')
    specs = {s.get('name', s.get('topic')): s for s in topics}
    manifest=_load(output/'topic_manifest.json',{},gaps)
    requested={s.get('name',s.get('topic')):s for s in manifest.get('topics',[])}
    for reason in manifest.get('chain_gaps',[]):
        _gap(gaps,'chain_discovery_unproven',reason=reason)
    for topic,spec in requested.items():
        if spec.get('critical') and topic not in specs:
            _gap(gaps,'required_topic_not_in_recorded_graph',topic=topic,role=spec.get('role'))
    for topic,spec in specs.items():
        expected=spec.get('endpoint_expected') or requested.get(topic,{}).get('endpoint_expected',{})
        for direction in ('publisher','subscriber'):
            # ROS namespace '/' + '/node' may be serialized as '//node'. This is
            # slash normalization only, not substring/short-name matching.
            canonical=lambda n: '/'+ '/'.join(part for part in str(n).split('/') if part)
            actual={canonical(row.get('node','')) for row in spec.get(direction+'s',[])}
            for node in expected.get(direction,[]):
                if canonical(node) not in actual:
                    _gap(gaps,'chain_endpoint_binding_unproven',topic=topic,direction=direction,
                         expected_node=node,observed_nodes=sorted(actual),
                         explanation='Expected running control-chain endpoint is not present in recorded graph evidence.')
    quality = _load(motion/'quality.json', {}, gaps)
    if not quality:
        _gap(gaps, 'collector_close_quality_missing')
    elif quality.get('incomplete') or quality.get('queue_drops',0):
        _gap(gaps, 'collector_incomplete', quality=quality)
    for topic, state in quality.get('topics', {}).items():
        if state.get('dropped',0):
            _gap(gaps, 'collector_known_drop', topic=topic, count=state['dropped'])
    marks = [dict(row, mark_line=line) for line,row in _lines(output/'marks.jsonl', gaps)]
    session=_load(output/'summary.json',{},gaps)
    for mark in marks:
        if session.get('boot_id') and mark.get('boot_id')!=session['boot_id']:
            _gap(gaps,'marker_boot_id_mismatch',marker=mark.get('label'),mark_line=mark.get('mark_line'))
    if not marks:
        _gap(gaps, 'manual_marker_missing', blocking=False)
    index = collections.defaultdict(list)
    sequence_seen = set()
    index_path = motion/'receive_index.csv'
    if not index_path.exists():
        _gap(gaps, 'receive_index_missing')
    else:
        with index_path.open(encoding='utf-8', newline='') as stream:
            for lineno,row in enumerate(csv.DictReader(stream),2):
                try:
                    for name in ('seq','topic_ordinal','bag_timestamp_ns','recv_monotonic_ns','recv_wall_ns','recv_ros_ns','serialized_bytes'):
                        row[name] = int(row[name])
                    row['index_line'] = lineno
                    if row['seq'] in sequence_seen:
                        _gap(gaps, 'duplicate_receive_sequence', index_line=lineno, seq=row['seq'])
                    sequence_seen.add(row['seq'])
                    index[(row['topic'], row['bag_timestamp_ns'])].append(row)
                except (KeyError,ValueError,TypeError) as exc:
                    _gap(gaps, 'receive_index_invalid', index_line=lineno, error=str(exc))
    for (topic,stamp),rows in index.items():
        if len(rows)!=1:
            _gap(gaps, 'duplicate_receive_index', topic=topic, bag_timestamp_ns=stamp, index_lines=[r['index_line'] for r in rows])
    ordered_meta = sorted((v[0] for v in index.values() if len(v)==1), key=lambda r:r['recv_monotonic_ns'])
    previous = None
    for row in ordered_meta:
        if previous:
            shift = (row['recv_wall_ns']-previous['recv_wall_ns'])-(row['recv_monotonic_ns']-previous['recv_monotonic_ns'])
            if abs(shift)>100000000:
                _gap(gaps, 'wall_clock_jump', index_line=row['index_line'], offset_change_ns=shift,
                     explanation='Marker windows still use monotonic time; wall/source alignment unverified.')
        previous = row
    times, observed, counts = collections.defaultdict(list), collections.defaultdict(list), collections.Counter()
    bag_seen = collections.Counter()
    velocity_previous = {}
    raw_fields = ['storage_file','message_id','topic','type','bag_timestamp_ns','seq','topic_ordinal','index_line',
        'recv_monotonic_ns','recv_wall_ns','recv_ros_ns','publisher_gid','rmw_source_timestamp_ns','rmw_received_timestamp_ns',
        'rmw_publication_sequence','rmw_reception_sequence','header_stamp_ns','frame_id','decode_status','data_json']
    raw_stream, raw_writer = _writer(destination/'raw_messages.csv', raw_fields)
    vel_stream, vel_writer = _writer(destination/'velocity.csv', ['topic','type','seq','bag_timestamp_ns','recv_monotonic_ns',
        'recv_wall_ns','header_stamp_ns','frame_id','vx','vy','wz','ax_estimate','ay_estimate','angular_accel_estimate',
        'jerk_x_estimate','estimate_time_basis'])
    event_stream, event_writer = _writer(destination/'velocity_events.csv', ['kind','topic','seq','recv_monotonic_ns','recv_wall_ns','detail_json'])
    evidence_stream, evidence_writer = _writer(destination/'evidence_index.csv', ['storage_file','message_id','topic','bag_timestamp_ns',
        'seq','receive_index_line','association','serialized_bytes','decode_status'])
    grids = collections.defaultdict(list)
    tf_edges = collections.defaultdict(list)
    try:
        dbs = sorted((motion/'bag').glob('*.db3'))
        if not dbs:
            _gap(gaps, 'sqlite_bag_missing')
        for path in dbs:
            db = sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True)
            try:
                for mid,topic,typename,stamp,payload,encoding in db.execute('SELECT m.id,t.name,t.type,m.timestamp,m.data,t.serialization_format FROM messages m JOIN topics t ON t.id=m.topic_id ORDER BY m.timestamp,m.id'):
                    key=(topic,stamp)
                    bag_seen[key]+=1
                    meta = index[key][0] if len(index.get(key,[]))==1 else None
                    row = dict(storage_file=path.name,message_id=mid,topic=topic,type=typename,bag_timestamp_ns=stamp)
                    expected_type=specs.get(topic,{}).get('type') or specs.get(topic,{}).get('expected_type')
                    if expected_type and expected_type!=typename and counts[topic]==0:
                        _gap(gaps,'bag_graph_type_mismatch',topic=topic,graph_type=expected_type,bag_type=typename)
                    if meta:
                        row.update(meta)
                        if meta['serialized_bytes']!=len(payload):
                            _gap(gaps, 'serialized_length_mismatch', topic=topic, message_id=mid)
                    else:
                        _gap(gaps, 'bag_message_without_unique_index', topic=topic, message_id=mid, bag_timestamp_ns=stamp)
                    decoded=None
                    try:
                        if encoding!='cdr':
                            raise ValueError('unsupported bag serialization format: '+str(encoding))
                        decoded=decode_cdr(typename,payload)
                        row['decode_status']='decoded'
                    except NotImplementedError as exc:
                        row['decode_status']='unsupported_type'
                        if counts[(topic,'unsupported')]==0:
                            _gap(gaps, 'cdr_type_unsupported', topic=topic, type=typename, error=str(exc), blocking=bool(specs.get(topic,{}).get('critical')))
                        counts[(topic,'unsupported')]+=1
                    except (ValueError,struct.error,UnicodeError) as exc:
                        row['decode_status']='decode_failed'
                        _gap(gaps, 'cdr_decode_failed', topic=topic, message_id=mid, error=str(exc))
                    evidence_writer.writerow(dict(storage_file=path.name,message_id=mid,topic=topic,bag_timestamp_ns=stamp,
                        seq=meta['seq'] if meta else None,receive_index_line=meta['index_line'] if meta else None,
                        association='unique_topic_bag_timestamp' if meta else 'unassociated',serialized_bytes=len(payload),decode_status=row['decode_status']))
                    if decoded is not None:
                        header=decoded.get('header',{})
                        row.update(header_stamp_ns=header.get('stamp_ns'),frame_id=header.get('frame_id'))
                        if meta:
                            times[topic].append(meta['recv_monotonic_ns'])
                            observed[topic].append((meta['recv_monotonic_ns'],meta['seq']))
                            for transform in decoded.get('transforms',[]):
                                tf_edges[(topic,transform['header']['frame_id'],transform['child_frame_id'])].append(
                                    (meta['recv_monotonic_ns'],transform['header']['stamp_ns'],meta['seq']))
                        if typename in ('nav_msgs/msg/OccupancyGrid','map_msgs/msg/OccupancyGridUpdate','nav2_msgs/msg/Costmap'):
                            decoded['data_count']=len(decoded.pop('data'))
                            decoded['data_storage']='Original CDR: storage_file/message_id in this row; no reconstruction performed'
                            if meta:
                                grids['update' if typename=='map_msgs/msg/OccupancyGridUpdate' else 'full'].append((meta['recv_monotonic_ns'],topic))
                        row['data_json']=_json(decoded)
                        v=_velocity(decoded)
                        if v is not None and meta:
                            previous=velocity_previous.get(topic)
                            dt=(meta['recv_monotonic_ns']-previous[0])/1e9 if previous else None
                            accel=None
                            limit=float(specs.get(topic,{}).get('max_gap_sec',2.0))
                            if previous and dt>limit:
                                event_writer.writerow(dict(kind='observed_receive_gap',topic=topic,seq=meta['seq'],recv_monotonic_ns=meta['recv_monotonic_ns'],recv_wall_ns=meta['recv_wall_ns'],detail_json=_json({'gap_sec':dt,'source_loss':'unknown','not_internal_compute_time':True})))
                            if previous and dt and 0<dt<=min(limit,.5):
                                accel=[(a-b)/dt for a,b in zip(v,previous[1])]
                            vel_writer.writerow(dict(topic=topic,type=typename,seq=meta['seq'],bag_timestamp_ns=stamp,
                                recv_monotonic_ns=meta['recv_monotonic_ns'],recv_wall_ns=meta['recv_wall_ns'],
                                header_stamp_ns=header.get('stamp_ns'),frame_id=header.get('frame_id'),vx=v[0],vy=v[1],wz=v[2],
                                ax_estimate=accel[0] if accel else None,ay_estimate=accel[1] if accel else None,
                                angular_accel_estimate=accel[2] if accel else None,
                                jerk_x_estimate=(accel[0]-previous[2][0])/dt if accel and previous and previous[2] else None,
                                estimate_time_basis='receiver_monotonic_not_source_time'))
                            if previous and dt and 0<dt<=limit:
                                for axis,a,b in zip(('vx','vy','wz'),v,previous[1]):
                                    kind=''
                                    if abs(a)<=.01<abs(b): kind=axis+'_zero'
                                    elif abs(b)<=.01<abs(a): kind=axis+'_nonzero'
                                    elif a*b<0 and min(abs(a),abs(b))>.01: kind=axis+'_reversal'
                                    elif abs(a-b)>(.15 if axis!='wz' else .25): kind=axis+'_step'
                                    if kind:
                                        event_writer.writerow(dict(kind=kind,topic=topic,seq=meta['seq'],recv_monotonic_ns=meta['recv_monotonic_ns'],recv_wall_ns=meta['recv_wall_ns'],detail_json=_json({'previous':b,'current':a,'candidate_not_fault':True})))
                            velocity_previous[topic]=(meta['recv_monotonic_ns'],v,accel)
                    raw_writer.writerow(row)
                    counts[topic]+=1
            finally:
                db.close()
    finally:
        for stream in (raw_stream,vel_stream,event_stream,evidence_stream):
            stream.close()
    for key,count in bag_seen.items():
        if count!=1:
            _gap(gaps, 'duplicate_bag_key',topic=key[0],bag_timestamp_ns=key[1],count=count)
    for key,rows in index.items():
        if key not in bag_seen:
            _gap(gaps, 'receive_index_without_bag',topic=key[0],bag_timestamp_ns=key[1],index_lines=[r['index_line'] for r in rows])
    for timestamp,topic in grids['update']:
        inferred_base=topic[:-len('_updates')] if topic.endswith('_updates') else None
        if not any(t<=timestamp and full_topic==inferred_base for t,full_topic in grids['full']):
            _gap(gaps, 'costmap_update_without_initial_full',topic=topic,recv_monotonic_ns=timestamp,
                 explanation='Published grid cannot be completely reconstructed from this update.',blocking=False)
            break
    tf_stream,tf_writer=_writer(destination/'tf_edges.csv',['topic','parent_frame','child_frame','recv_monotonic_ns','source_stamp_ns','receive_seq'])
    try:
        for (topic,parent,child),entries in sorted(tf_edges.items()):
            for mono,source,seq in entries:
                tf_writer.writerow(dict(topic=topic,parent_frame=parent,child_frame=child,recv_monotonic_ns=mono,source_stamp_ns=source,receive_seq=seq))
    finally:
        tf_stream.close()
    quality_rows=[]
    for topic,spec in specs.items():
        samples=sorted(times[topic])
        intervals=[(b-a)/1e9 for a,b in zip(samples,samples[1:])]
        quality_rows.append(dict(topic=topic,messages=counts[topic],decoded_indexed=len(samples),
            p50_sec=_percentile(intervals,.5),p95_sec=_percentile(intervals,.95),p99_sec=_percentile(intervals,.99),
            max_gap_sec=max(intervals) if intervals else None,dds_source_loss='unknown',source_age='unknown: clocks not synchronized/verified'))
        if spec.get('critical') and (not samples or spec.get('missing_reason')):
            _gap(gaps, 'required_topic_missing',topic=topic,role=spec.get('role'),missing_reason=spec.get('missing_reason'))
        for mark in marks:
            center=mark.get('read_monotonic_ns')
            if not isinstance(center,int):
                _gap(gaps,'marker_monotonic_time_missing',marker=mark.get('label'))
                continue
            left,right=center-int(window*1e9),center+int(window*1e9)
            lo,hi=bisect.bisect_left(samples,left),bisect.bisect_right(samples,right)
            inside=samples[lo:hi]
            task=spec.get('cadence')=='task'
            continuous=spec.get('cadence') in ('continuous','periodic','task')
            limit=float(spec.get('max_gap_sec',2.0))
            prior=samples[lo-1] if lo else None
            following=samples[hi] if hi<len(samples) else None
            supported=(samples[max(0,lo-1):min(len(samples),hi+1)] if samples else [])
            maxgap=max([(b-a)/1e9 for a,b in zip(supported,supported[1:])] or [0])
            if continuous:
                begin_ok=bool(samples) and samples[0]<=left and (prior is not None or (inside and inside[0]==left))
                end_ok=bool(samples) and samples[-1]>=right and (following is not None or (inside and inside[-1]==right))
                complete=bool(inside) and begin_ok and end_ok and maxgap<=limit
                covered=sum(max(0,min(b,right)-max(a,left)) for a,b in zip(supported,supported[1:]) if 0<b-a<=limit*1e9)
                fraction=min(1.,covered/(right-left))
            else:
                complete=prior is not None or bool(inside)
                fraction=None  # no event does not establish packet loss
            record=dict(marker=mark.get('label'),mark_line=mark.get('mark_line'),topic=topic,role=spec.get('role'),
                critical=bool(spec.get('critical')),window_start_monotonic_ns=left,window_end_monotonic_ns=right,
                messages_in_window=len(inside),previous_sample_monotonic_ns=prior,next_sample_monotonic_ns=following,
                max_observed_interval_sec=maxgap,allowed_gap_sec=limit,observed_time_coverage_fraction=fraction,
                coverage_kind='continuous_observation' if continuous else 'latest_observed_state_only',complete=complete,
                activity_note='activity unknown; task may have ended; evidence gap is not automatically a fault' if task else '',
                evidence_seq_json=_json([seq for t,seq in observed[topic] if left<=t<=right]))
            coverage.append(record)
            if spec.get('critical') and not complete:
                _gap(gaps,'marker_window_gap',topic=topic,marker=mark.get('label'),mark_line=mark.get('mark_line'),
                     max_observed_interval_sec=maxgap,allowed_gap_sec=limit,coverage_fraction=fraction,
                     activity_note='activity unknown; task may have ended; not automatically a fault' if task else '')
    for filename,rows in (('coverage.csv',coverage),('topic_quality.csv',quality_rows)):
        stream,writer=_writer(destination/filename,list(rows[0]) if rows else ['topic'])
        try: writer.writerows(rows)
        finally: stream.close()
    mppi=_mppi_report(output,destination,marks,window,gaps)
    motion_complete=not any(g['scope']=='motion' and g['blocking'] for g in gaps)
    result=dict(schema=1,source=str(output),destination=str(destination),window_sec=window,
        motion_complete=motion_complete,complete=motion_complete and mppi['complete'],coverage=coverage,gaps=gaps,mppi=mppi,
        topic_quality=quality_rows,markers=marks,
        source_binary_equivalence=manifest.get('source_binary_equivalence','not_verified'),interpretation_limits=[
            'No receive-order causal claims or source-age subtraction across unverified clocks.',
            'Coverage means observed intervals, not measured DDS/source packet-loss rate.',
            'Zero command and no-message interval are separate observations; neither is automatically a fault.',
            'Event topics retain latest observed state; no changes are not treated as message loss.',
            'Published costmap is asynchronous; only MPPI frames identify the internal grid for that compute.',
            'IMU remains in its reported frame; gravity/filter state is not inferred.',
            'Stop/recovery branch attribution still requires NAVLITE/log evidence from the companion report.'])
    (destination/'summary.json').write_text(_json(result)+'\n',encoding='utf-8')
    (destination/'summary.md').write_text('# Offline navigation evidence\n\n'
        +f'Motion windows complete: {motion_complete}. MPPI capture structurally complete: {mppi["complete"]}.\n\n'
        +f'{len(coverage)} topic/marker checks; {len(gaps)} explicit gaps. No navigation acceptance or causal claim.\n\n'
        +'Original samples: raw_messages.csv; signed commands/motion: velocity.csv; candidate events: velocity_events.csv; '
        +'bag/index links: evidence_index.csv; marker windows: coverage.csv; MPPI raw metadata: mppi_frames.csv (when available).\n\n'
        +'\n'.join('- '+g['code']+': '+_json({k:v for k,v in g.items() if k not in ('code','blocking')}) for g in gaps)+'\n',encoding='utf-8')
    return result
