#!/usr/bin/env python3
"""Lightweight navigation evidence. Default reads existing local logs only.

Linux, Python >= 3.8. Use `record`, `mark` or offline `report --help`.
Log read times are NOT event generation times. This does not record velocities
or laser data unless --motion explicitly starts the native raw-message helper.
--motion also requests bounded internal MPPI diagnostic files, never motion.
"""
import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import resource
import select
import signal
import stat
import sys
import tempfile
import time

DEFAULT_LOGS = (
    "resident_navigation_runtime.log", "robot_safety_common.log",
    "ranger_chassis_common.log", "robot_api_server.log",
)
READ_BYTES = 16 * 1024       # per file, per polling iteration
MAX_LINE = 8192
STAMP = re.compile(r"\[(\d{1,12})\.(\d{1,9})\]")
ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
GROUPS = (
    ("diagnostic", (b"navlite", b"nav_event")),
    ("bms", (b"bms_contact", b"bms_docking", b"docked_contact_block")),
    ("collision_log", (b"collision_monitor",)),
    ("control_period", (b"control loop missed", b"missed its desired rate")),
    ("mppi", (b"no valid control", b"no valid trajectories", b"mppi reset",
              b"resetting mppi", b"optimizer reset", b"failed to find a valid",
              b"ordinary mppi recovered a valid control")),
    ("recovery", (b"ordinary-recovery", b"ordinary_recovery", b"navigation_recovery")),
    ("path", (b"local path", b"reference path", b"path patch", b"path repair",
              b"no_rejoin", b"path replacement", b"local-repair", b"local_repair",
              b"path_patch", b"path_repair")),
    ("terminal", (b"terminal", b"handoff", b"final_verify", b"final verify",
                  b"ordinary_navigation_result", b"post_nav2", b"goal reached",
                  b"navigation succeeded", b"navigation failed")),
    ("mode_safety", (b"mode switch", b"mode_switch", b"spin_to_drive", b"mode_exit",
                     b"stop_priority", b"zero_cmd_priority", b"holding zero",
                     b"selected_source", b"selected_reason", b"safety reason")),
    ("warning_error", (b"[warn]", b"[warning]", b"[error]", b"[fatal]",
                       b"traceback", b"extrapolation", b"transform timeout")),
)
COLLISION_ACTIONS = (("stop", b"robot to stop"), ("slowdown", b"robot to slowdown"),
                     ("approach", b"robot to approach"), ("continue", b"robot to continue"))

# Used ONLY by offline report: the recording hot path still saves raw text.
NAVLITE_HEAD = re.compile(r"\bNAVLITE\s+(\w+)\b")
NAVLITE_FIELD = re.compile(r'(\w+)=("(?:\\.|[^"\\])*"|\([^()\r\n]*\)|[^\s"]+)')
EVIDENCE_KEYS = ('mppi_failure_detail', 'mppi_calculation_return', 'mppi_nonzero_return',
                 'controller_return', 'collision_publication', 'safety_publication',
                 'safety_selection', 'safety_state', 'chassis_input',
                 'chassis_sdk_cache', 'chassis_motion_cache')
NAVLITE_COLUMNS = [
    'event_line', 'read_monotonic_ns', 'read_wall_ns', 'source_stamp_ns', 'source',
    'module', 'event', 'branch', 'episode', 'compute_seq', 'consecutive_failures',
    'failure_elapsed_sec', 'stage', 'exception_type', 'original_error',
    'action', 'region', 'reason', 'selected_source', 'state', 'output_status',
    'in_vx', 'in_vy', 'in_wz', 'out_vx', 'out_vy', 'out_wz',
    'requested_out_vx', 'requested_out_vy', 'requested_out_wz',
    'actual_vx', 'actual_vy', 'actual_wz', 'actual_kind', 'motion_fresh',
    'input_seq', 'input_age_sec', 'input_gap', 'sdk_submit_seq', 'sdk_submit_age_sec',
    'sdk_linear', 'sdk_steer', 'sdk_angular', 'can_tx_confirmed',
    'desired_mode', 'actual_mode', 'mode_changing', 'mode_state',
    'parse_issues', 'fields_json', 'text',
]


def navlite_row(event, line_no):
    """Decode discrete producer evidence, never infer motion or cross-layer causality."""
    text = event.get('text', '')
    head = NAVLITE_HEAD.search(text)
    if not head:
        return None, ()
    module = head.group(1)
    payload = text[head.end():].strip()
    values, issues, offset = {}, [], 0
    for match in NAVLITE_FIELD.finditer(payload):
        if payload[offset:match.start()].strip():
            issues.append('unparsed_text')
        key, value = match.groups()
        if key in values:
            issues.append('duplicate_field:' + key)
        if value.startswith('"'):
            try:
                value = json.loads(value)
            except ValueError:
                issues.append('invalid_quoted_field:' + key)
        values[key] = value
        offset = match.end()
    if payload[offset:].strip():
        issues.append('unparsed_tail')

    def vector(key):
        raw = values.get(key, '')
        try:
            if not (raw.startswith('(') and raw.endswith(')')):
                return None
            result = tuple(float(v) for v in raw[1:-1].split(','))
            if len(result) == 3 and all(math.isfinite(v) for v in result):
                return result
        except (TypeError, ValueError):
            pass
        return None

    inp, out, requested = vector('in'), vector('out'), vector('requested_out')
    if values.get('input_known') == '0':
        inp = None
    status, actual, motion, observed = 'unknown', None, None, []
    name = values.get('event', '')
    if not issues:
        if name == 'diagnostics_ready' and module in (
                'mppi', 'controller', 'collision', 'safety', 'chassis'):
            status = 'diagnostics_ready_not_motion'
        elif module == 'mppi':
            if name in ('failure_start', 'failure_continues', 'failure_reason_changed') and all(
                    key in values for key in ('raw', 'exception_type', 'stage',
                                              'consecutive_failures', 'elapsed_sec')):
                observed.append('mppi_failure_detail')
            if values.get('command_returned') == '0':
                status = 'no_command_returned'  # Not a zero publication.
            elif values.get('command_returned') == '1' and out is not None:
                actual = out
                status = 'returned_nonzero' if any(out) else 'returned_zero'
                observed.append('mppi_calculation_return')
                if any(out):
                    observed.append('mppi_nonzero_return')
        elif module == 'controller' and out is not None:
            actual = out
            status = 'returned_nonzero' if any(out) else 'returned_zero'
            observed.append('controller_return')
        elif module == 'collision':
            published = values.get('published')
            publication = values.get('publication')
            if published == '0' and publication != 'sent':
                status = 'no_message'
            elif published == '1' and publication == 'sent' and requested is not None:
                actual = requested
                status = 'published_nonzero' if any(actual) else 'published_zero'
            if status != 'unknown' and inp is not None and requested is not None and all(
                    key in values for key in ('action', 'region', 'reason')):
                observed.append('collision_publication')
        elif module == 'safety':
            if values.get('publication') == 'sent' and out is not None:
                actual = out
                status = 'published_nonzero' if any(out) else 'published_zero'
                if all(key in values for key in ('input_known', 'source', 'final_reason', 'branch')):
                    observed.append('safety_publication')
        elif module == 'safety_state':
            if values.get('publication') == 'state_only':
                status = 'state_only'
                observed.append('safety_state')
        elif module == 'safety_arbitration':
            if values.get('publication') == 'no_message':
                status = 'no_message'  # Only this input ignored, not all safety outputs.
            if 'selected_source' in values:
                observed.append('safety_selection')
        elif module == 'chassis' and values.get('publication') == 'observation':
            status = 'driver_observation'
            if values.get('input_known') == '1' and inp is not None:
                observed.append('chassis_input')
            if values.get('sdk_known') == '1' and all(key in values for key in (
                    'sdk_submit_seq', 'sdk_linear', 'sdk_steer', 'sdk_angular')):
                observed.append('chassis_sdk_cache')
            if values.get('odom_known') == '1' and values.get('actual_kind') == 'wheel_odom_cache':
                motion = vector('actual')
                if motion is not None:
                    observed.append('chassis_motion_cache')
    row = {key: event.get(key) for key in NAVLITE_COLUMNS if key in event}
    row.update({key: values.get(key) for key in (
        'event', 'branch', 'episode', 'compute_seq', 'consecutive_failures', 'stage',
        'exception_type', 'action', 'region', 'state')})
    row.update({key: values.get(key) for key in (
        'actual_kind', 'motion_fresh', 'input_seq', 'input_age_sec', 'input_gap',
        'sdk_submit_seq', 'sdk_submit_age_sec', 'sdk_linear', 'sdk_steer', 'sdk_angular',
        'can_tx_confirmed', 'desired_mode', 'actual_mode', 'mode_changing', 'mode_state')})
    row.update(event_line=line_no, module=module, output_status=status,
               original_error=values.get('raw'),
               failure_elapsed_sec=values.get('elapsed_sec') if module == 'mppi' else None,
               reason=values.get('final_reason', values.get('reason')),
               selected_source=values.get('selected_source', values.get('source')),
               parse_issues=','.join(issues), fields_json=json.dumps(values, ensure_ascii=False))
    for prefix, sample in (('in', inp), ('out', actual), ('requested_out', requested),
                           ('actual', motion)):
        for axis, value in zip(('vx', 'vy', 'wz'), sample or (None, None, None)):
            row[prefix + '_' + axis] = value
    return row, observed


def new_coverage():
    return {key: {'count': 0, 'first_event_line': None, 'last_event_line': None,
                  'status': 'not_observed_not_proof_of_absence'} for key in EVIDENCE_KEYS}


def observe_coverage(coverage, observed, line_no):
    for key in observed:
        entry = coverage[key]
        entry['count'] += 1
        if entry['first_event_line'] is None:
            entry['first_event_line'] = line_no
        entry['last_event_line'] = line_no
        entry['status'] = 'discrete_log_evidence_only'


def collision_action(raw):
    lower = raw.lower()
    if b"collision_monitor" in lower:
        return next((action for action, phrase in COLLISION_ACTIONS if phrase in lower), None)
    return None


def classify(raw, extras=()):
    if collision_action(raw):
        return "collision_action"
    lower = raw.lower()
    for category, terms in GROUPS:
        if any(term in lower for term in terms):
            return category
    return "custom" if any(term in lower for term in extras) else None


def clocks():
    return {"read_wall_ns": time.time_ns(), "read_monotonic_ns": time.monotonic_ns()}


def boot_id():
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        return None


def source_stamp(text):
    match = STAMP.search(text)
    if not match:
        return None
    sec, frac = match.groups()
    return int(sec) * 1_000_000_000 + int(frac.ljust(9, "0"))


def save_json(path, obj):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(str(temp), str(path))


class BudgetReached(Exception):
    pass


class Sink:
    def __init__(self, directory, max_bytes):
        self.stream = (directory / "events.jsonl").open("wb", buffering=65536)
        self.limit, self.bytes = max_bytes, 0
        self.counts = Counter()

    def emit(self, category, **fields):
        obj = {"category": category, **clocks(), **fields}
        data = (json.dumps(obj, ensure_ascii=False, separators=(",", ":")) + "\n").encode()
        if self.bytes + len(data) > self.limit:
            raise BudgetReached("event_file_size_limit")
        self.stream.write(data)
        self.bytes += len(data)
        self.counts[category] += 1


class LogTail:
    """Bounded incremental reads; ordinary rename rotation and truncation handled."""
    def __init__(self, path, emit, process):
        self.path, self.emit, self.process = path, emit, process
        self.fd = None
        self.ever_opened = False
        self.existed_at_start = path.exists()
        self.buf = b""
        self.discard_to_newline = False
        self.generation = 0
        self.error = None
        self.identity = None
        self.bytes_read = self.lines_read = self.lines_matched = 0
        self.last_read_ns = self.last_event_ns = None
        self.backlog_bytes = self.max_backlog_bytes = 0
        self.issues = set()

    def event(self, action, **fields):
        self.emit("file_event", source=str(self.path), action=action, **fields)

    def open_file(self):
        fd = None
        try:
            fd = os.open(str(self.path), os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode):
                raise OSError("input must be a regular local log file")
        except OSError as exc:
            if fd is not None:
                os.close(fd)
            error = f"{type(exc).__name__}: {exc}"
            if error != self.error:
                self.event("unavailable", error=error)
            self.error = error
            self.issues.add("unavailable_observed")
            return False
        self.fd = fd
        self.identity = (info.st_dev, info.st_ino)
        # Ignore pre-existing content only on the first open. Newly created logs
        # and new rotation generations are read from their beginning.
        initial = not self.ever_opened and self.existed_at_start
        offset = info.st_size if initial else 0
        if offset:
            os.lseek(fd, offset - 1, os.SEEK_SET)
            self.discard_to_newline = os.read(fd, 1) != b"\n"
        os.lseek(fd, offset, os.SEEK_SET)
        self.ever_opened, self.error = True, None
        self.event("opened", generation=self.generation, inode=info.st_ino,
                   starting_offset=offset, preexisting_bytes_ignored=offset)
        return True

    def read_chunk(self):
        start = os.lseek(self.fd, 0, os.SEEK_CUR)
        data = os.read(self.fd, READ_BYTES)
        if not data:
            return
        self.bytes_read += len(data)
        self.last_read_ns = time.monotonic_ns()
        if self.discard_to_newline:
            end = data.find(b"\n")
            if end < 0:
                return
            start += end + 1
            data = data[end + 1:]
            self.discard_to_newline = False
        base = start - len(self.buf)
        whole = self.buf + data
        parts = whole.split(b"\n")
        self.buf = parts.pop()
        for raw in parts:
            self.lines_read += 1
            if len(raw) > MAX_LINE:
                self.issues.add("line_truncated")
                self.event("line_truncated", offset=base, original_bytes=len(raw))
            self.process(self, raw[:MAX_LINE], base)
            base += len(raw) + 1
        if len(self.buf) > MAX_LINE:
            self.issues.add("overlong_line_discarded")
            self.event("overlong_line_discarded", offset=base, buffered_bytes=len(self.buf))
            self.buf = b""
            self.discard_to_newline = True

    def poll(self):
        if self.fd is None and not self.open_file():
            return
        try:
            info = os.fstat(self.fd)
            pos = os.lseek(self.fd, 0, os.SEEK_CUR)
            if info.st_size < pos:
                self.issues.add("truncation_observed")
                self.event("truncated", old_offset=pos, new_size=info.st_size)
                os.lseek(self.fd, 0, os.SEEK_SET)
                self.buf = b""
                self.discard_to_newline = False
                self.generation += 1
            self.read_chunk()
            pos = os.lseek(self.fd, 0, os.SEEK_CUR)
            self.backlog_bytes = max(0, os.fstat(self.fd).st_size - pos)
            self.max_backlog_bytes = max(self.max_backlog_bytes, self.backlog_bytes)
            try:
                current = self.path.stat()
            except FileNotFoundError:
                # Keep draining the old inode until replacement appears.
                return
            if (current.st_dev, current.st_ino) != self.identity:
                # Drain an ordinary small rotation. Large unread tails are
                # explicitly reported rather than processed in an unbounded loop.
                if self.backlog_bytes or self.buf:
                    self.issues.add("rotation_gap_possible")
                self.event("rotated", unread_bytes=self.backlog_bytes,
                           partial_line_bytes=len(self.buf),
                           note="late writes to detached old inode are not guaranteed")
                os.close(self.fd)
                self.fd = None
                self.buf = b""
                self.discard_to_newline = False
                self.generation += 1
                self.open_file()
        except OSError as exc:
            self.issues.add("read_error")
            self.event("read_error", error=str(exc))
            if self.fd is not None:
                os.close(self.fd)
                self.fd = None

    def summary(self):
        return {"path": str(self.path), "ever_opened": self.ever_opened,
                "current_error": self.error, "bytes_read": self.bytes_read,
                "lines_read": self.lines_read, "lines_matched": self.lines_matched,
                "last_read_monotonic_ns": self.last_read_ns,
                "last_event_monotonic_ns": self.last_event_ns,
                "backlog_bytes": self.backlog_bytes,
                "max_backlog_bytes": self.max_backlog_bytes,
                "pending_partial_line_bytes": len(self.buf),
                "issues": sorted(self.issues)}


def add_mark(directory, label):
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    if summary.get("status") != "recording":
        raise ValueError("This session is not marked as recording")
    origin = boot_id()
    if not origin or summary.get('boot_id') != origin:
        raise ValueError('Mark must run on the recorder host during the same boot')
    path = directory / "marks.jsonl"
    if path.stat().st_size > 1024 * 1024:
        raise ValueError("mark file limit reached")
    data = (json.dumps({"category": "operator_mark", **clocks(),
                       "boot_id": origin,
                       "label": label[:256]}, ensure_ascii=False) + "\n").encode()
    fd = os.open(str(path), os.O_WRONLY | os.O_APPEND | os.O_CLOEXEC)
    try:
        if os.write(fd, data) != len(data):
            raise OSError("incomplete marker write")
    finally:
        os.close(fd)


def record(args):
    paths = []
    if args.log_dir:
        paths.extend(Path(args.log_dir).expanduser() / name for name in DEFAULT_LOGS)
    paths.extend(Path(p).expanduser() for p in args.log)
    paths = list(dict.fromkeys(p.resolve() for p in paths))
    if not 1 <= len(paths) <= 8:
        raise ValueError("Provide --log-dir or --log; allow 1 to 8 unique files")
    if not all(math.isfinite(x) for x in (args.interval, args.duration, args.max_mb, args.min_free_mb)):
        raise ValueError("Time and budget values must be finite numbers")
    if args.interval < 0.1 or args.duration <= 0 or args.max_mb < 0.05 or args.min_free_mb < 0:
        raise ValueError("Require interval >= 0.1s, duration > 0 and max-mb >= 0.05")
    if not any(p.is_file() for p in paths):
        raise ValueError("No regular input log is visible. Check path and container; no session started")
    if args.output:
        output = Path(args.output).expanduser().resolve()
    else:
        Path('/tmp/njrh_reports').mkdir(parents=True, exist_ok=True)
        output = Path(tempfile.mkdtemp(prefix=time.strftime('nav_event_lite_%Y%m%dT%H%M%SZ_',time.gmtime()),
                                       dir='/tmp/njrh_reports'))
    if output.exists() and any(output.iterdir()):
        raise ValueError("Output directory must be new or empty; existing evidence is not overwritten")
    output.mkdir(parents=True, exist_ok=True)
    (output / "marks.jsonl").touch(exist_ok=False)
    started = clocks()
    begin, cpu_begin = time.monotonic(), time.process_time()
    manifest = {"version": "1.4", "status": "recording", "pid": os.getpid(),
                "recorder_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "production_binary_source_match": "not_verified_by_log_only_recorder",
                "boot_id": boot_id(),
                "started": started, "mode": "local_log_only_no_ros",
                "poll_interval_s": args.interval, "duration_limit_s": args.duration,
                "logs": [str(p) for p in paths],
                "extra_keywords": args.keyword,
                "source_clock_verified": False,
                "limitations": [
                    "Only log text actually written by the application can be collected.",
                    "No ROS graph, topics, services, network, bag, odometry or scan capture.",
                    "Reader timestamps are not producer times; source clock domain is unverified.",
                    "No collision log does not mean no collision action.",
                    "INFO events suppressed by the producer log level cannot be recovered (last audited Nav2 level: WARN).",
                    "Ordinary rotation is handled; late old-inode writes / fast copytruncate may escape detection.",
                    "No byte at launch from the old log history is replayed as a new event."]}
    save_json(output / "summary.json", manifest)
    sink = Sink(output, int(args.max_mb * 1024 * 1024))
    period = {}  # At most one bounded control-period aggregate per input file.
    categories = Counter()
    collision_actions = Counter()
    extras = tuple(k.lower().encode() for k in args.keyword)

    def process(tail, raw, offset):
        category = classify(raw, extras)
        if category is None:
            return
        text = ANSI.sub("", raw.decode("utf-8", errors="replace")).rstrip("\r")
        tail.lines_matched += 1
        tail.last_event_ns = time.monotonic_ns()
        categories[category] += 1
        fields = {"source": str(tail.path), "inode": tail.identity[1],
                  "generation": tail.generation, "offset": offset,
                  "source_stamp_ns": source_stamp(text), "text": text}
        if category.startswith("collision_"):
            action = collision_action(raw)
            fields["collision_action"] = action
            if action:
                collision_actions[action] += 1
        if category != "control_period":
            sink.emit(category, **fields)
            return
        key = str(tail.path)
        if key not in period:
            sink.emit("control_period_first", **fields)
            period[key] = {"source": key, "count": 1, "first": fields,
                           "last": fields, "first_read_monotonic_ns": time.monotonic_ns()}
        else:
            period[key]["count"] += 1
            period[key]["last"] = fields

    def flush_period():
        for group in period.values():
            if group["count"] > 1:
                sink.emit("control_period_summary", total_including_saved_first=group["count"],
                          source=group["source"], first=group["first"], last=group["last"],
                          first_read_monotonic_ns=group["first_read_monotonic_ns"])
        period.clear()

    tails = [LogTail(path, sink.emit, process) for path in paths]
    interrupted = [None]
    def handle_signal(signum, _frame):
        interrupted[0] = "signal_" + str(signum)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, handle_signal)
    stdin_active = not args.no_stdin and sys.stdin.isatty()
    last_flush = last_health = last_disk = begin
    last_cpu, last_cpu_wall = cpu_begin, begin
    worst_cpu, worst_backlog = 0.0, 0
    previous_clock = started
    reason, error = "duration", None
    evidence = None
    evidence_result = None
    print("LOG + RAW EVIDENCE recording:" if args.motion else "LOG-ONLY recording:", output, flush=True)
    print("Enter = mark; Ctrl+C stops this recorder only. No robot commands.", flush=True)
    print("INFO suppressed by production logging cannot be captured; no log != no action.", flush=True)
    try:
        if args.motion:
            from nav_event_evidence import EvidenceSession
            evidence = EvidenceSession(output, args)
            evidence.start()
            manifest['mode'] = 'logs_and_native_raw_and_internal_mppi'
            manifest['production_binary_source_match'] = 'unverified_see_runtime_start_provenance'
            manifest['limitations'] = [item for item in manifest['limitations']
                if not item.startswith('No ROS graph')]
            manifest['limitations'].append('Raw/internal coverage is in evidence_quality.json, not log completeness.')
            begin = time.monotonic()
            save_json(output/'summary.json', manifest)
        while time.monotonic() - begin < args.duration and not interrupted[0]:
            tick = time.monotonic()
            for tail in tails:
                tail.poll()
            if evidence:
                evidence.poll()
                if evidence.process and evidence.process.poll() is not None:
                    reason = 'native_capture_exit_' + str(evidence.process.returncode)
                    break  # End this capture only; don't spend the session pretending raw data continues.
            if stdin_active:
                readable, _, _ = select.select([sys.stdin], [], [], 0)
                if readable:
                    label = sys.stdin.readline(512)
                    if label:
                        add_mark(output, label.strip() or "operator_jerk")
                        print("MARK saved", flush=True)
                    else:
                        stdin_active = False
            now = time.monotonic()
            current_clock = clocks()
            clock_shift = ((current_clock['read_wall_ns']-previous_clock['read_wall_ns']) -
                           (current_clock['read_monotonic_ns']-previous_clock['read_monotonic_ns']))
            if abs(clock_shift) > 1_000_000_000:
                sink.emit('clock_jump', wall_minus_monotonic_change_ns=clock_shift)
            previous_clock = current_clock
            if now - last_flush >= 1.0:
                flush_period()
                sink.stream.flush()
                last_flush = now
            if now - last_health >= 2.0:
                cpu = time.process_time()
                pct = 100 * (cpu - last_cpu) / (now - last_cpu_wall)
                worst_cpu = max(worst_cpu, pct)
                backlog = sum(t.backlog_bytes for t in tails)
                worst_backlog = max(worst_backlog, backlog)
                sink.emit("recorder_health", cpu_one_core_pct=round(pct, 3),
                          max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                          backlog_bytes=backlog,
                          source_ages_s={str(t.path): None if t.last_read_ns is None else
                                         round((time.monotonic_ns()-t.last_read_ns)/1e9, 3)
                                         for t in tails},
                          note="log silence is not sensor/node health")
                last_cpu, last_cpu_wall, last_health = cpu, now, now
            if now - last_disk >= 2.0:
                disk = os.statvfs(str(output))
                if disk.f_bavail * disk.f_frsize < args.min_free_mb * 1024 * 1024:
                    reason = "disk_low"
                    break
                last_disk = now
            # Never busy-loop to catch up under load.
            time.sleep(max(0.01, args.interval - (time.monotonic() - tick)))
        if interrupted[0]:
            reason = interrupted[0]
        flush_period()
        sink.emit("record_end", reason=reason)
        sink.stream.flush()
    except Exception as exc:
        reason = str(exc) if isinstance(exc, BudgetReached) else (
            "io_error" if isinstance(exc, OSError) else "recorder_error")
        error = repr(exc)
    finally:
        if evidence:
            try:
                evidence_result = evidence.close()
            except Exception as exc:
                evidence_result = {'ready':False,'gaps':['evidence finalization: '+repr(exc)]}
        for tail in tails:
            if tail.fd is not None:
                try:
                    tail.backlog_bytes = max(0, os.fstat(tail.fd).st_size - os.lseek(tail.fd,0,os.SEEK_CUR))
                    tail.max_backlog_bytes = max(tail.max_backlog_bytes, tail.backlog_bytes)
                except OSError:
                    tail.issues.add("final_stat_failed")
                os.close(tail.fd)
        try:
            sink.stream.close()
        except OSError as exc:
            reason, error = "io_error", repr(exc)
        duration = max(0.001, time.monotonic()-begin)
        gaps = []
        if error or reason not in ('duration','signal_2','signal_15'):
            gaps.append(reason)
        for tail in tails:
            if not tail.ever_opened or not tail.path.is_file():
                gaps.append(str(tail.path)+': unavailable')
            if tail.backlog_bytes or tail.buf:
                gaps.append(str(tail.path)+': unread_or_partial_tail')
            gaps.extend(str(tail.path)+': '+issue for issue in sorted(tail.issues))
        manifest.update(status="finished", ended=clocks(), elapsed_s=round(duration, 3),
                        incomplete=bool(gaps), incomplete_reasons=gaps,
                        end_reason=reason, error=error, event_file_bytes=sink.bytes,
                        cpu_one_core_average_pct=round(100*(time.process_time()-cpu_begin)/duration, 3),
                        cpu_one_core_peak_2s_pct=None if last_health == begin else round(worst_cpu, 3),
                        max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                        matched_line_counts=dict(categories), saved_records=dict(sink.counts),
                        files=[t.summary() for t in tails],
                        max_observed_backlog_bytes=worst_backlog,
                        full_motion_reconstruction_available=False,
                        collision_log_seen=bool(categories["collision_log"] or collision_actions),
                        collision_action_counts=dict(collision_actions),
                        collision_evidence_status="action_text_observed_review_content" if collision_actions
                        else "node_log_only_not_action_proof" if categories["collision_log"]
                        else "not_observed_not_proof_of_no_action")
        if evidence_result is not None:
            manifest['evidence'] = evidence_result
            manifest['evidence_incomplete'] = not evidence_result.get('ready', False)
        try:
            save_json(output / "summary.json", manifest)
        except OSError as exc:
            print("Cannot save final summary:", exc, file=sys.stderr)
            return 2
    print("Stopped:", reason, "| CPU average (one core):",
          manifest["cpu_one_core_average_pct"], "% | max RSS:", manifest["max_rss_kib"], "KiB")
    print("Output:", output)
    print("INCOMPLETE" if gaps else "LOG_CAPTURE_COMPLETE (not full motion evidence)",
          '; '.join(gaps), flush=True)
    if args.motion:
        print('Use report for per-marker raw/internal coverage; LOG_CAPTURE_COMPLETE is not evidence completeness.',flush=True)
        if not (evidence_result or {}).get('ready'):
            return 3
    return 0 if reason in ("duration", "signal_2", "signal_15") else 2


def report(args):
    """Streaming CSV export; no recording process, ROS, plots or interpolation."""
    directory = Path(args.output).expanduser().resolve()
    if not math.isfinite(args.window) or not 0 < args.window <= 60:
        raise ValueError('Require 0 < --window <= 60 seconds')
    session = json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    if session.get('status') != 'finished':
        raise ValueError('Stop the recorder before offline report')
    gaps = list(session.get('incomplete_reasons', []))

    def records(name):
        with (directory/name).open(encoding='utf-8') as stream:
            line_no = 0
            while True:
                raw = stream.readline(128*1024)
                if not raw: break
                line_no += 1
                if not raw.endswith('\n'):
                    raise ValueError(f'{name}:{line_no}: partial or oversized JSON record')
                obj = json.loads(raw)
                if not isinstance(obj, dict):
                    raise ValueError(f'{name}:{line_no}: expected JSON object')
                yield line_no, obj

    marks = []
    for line_no, mark in records('marks.jsonl'):
        if len(marks) >= 2048:
            raise ValueError('Too many markers; offline limit is 2048')
        stamp = mark.get('read_monotonic_ns')
        if (not isinstance(stamp, int) or not session.get('boot_id') or
                mark.get('boot_id') != session['boot_id']):
            gaps.append(f'marks.jsonl:{line_no}: unverified marker clock; excluded')
            continue
        marks.append({'id':line_no, 'label':mark.get('label',''), 'stamp':stamp,
                      'event_count':0, 'evidence_coverage':new_coverage()})
    marks.sort(key=lambda x:x['stamp'])
    stamps = [m['stamp'] for m in marks]
    window = int(args.window*1e9)
    start = session['started']['read_monotonic_ns']
    end = session['ended']['read_monotonic_ns']
    for mark in marks:
        mark['window_clipped'] = mark['stamp']-window < start or mark['stamp']+window > end
    destination = Path(tempfile.mkdtemp(prefix='report_',dir=str(directory)))
    fields = ['event_line','read_monotonic_ns','read_wall_ns','source_stamp_ns','category',
              'source','generation','inode','offset','collision_action','text','details_json']
    total = window_rows = 0
    coverage = new_coverage()
    navlite_count = parse_issue_count = 0
    with (destination/'timeline.csv').open('w',newline='',encoding='utf-8') as all_file, \
            (destination/'marker_windows.csv').open('w',newline='',encoding='utf-8') as window_file, \
            (destination/'navlite.csv').open('w',newline='',encoding='utf-8') as detail_file:
        timeline = csv.DictWriter(all_file,fieldnames=fields)
        windows = csv.DictWriter(window_file,fieldnames=['marker_id','label','delta_read_s']+fields)
        detail = csv.DictWriter(detail_file, fieldnames=NAVLITE_COLUMNS)
        timeline.writeheader()
        windows.writeheader()
        detail.writeheader()
        for line_no,event in records('events.jsonl'):
            stamp=event.get('read_monotonic_ns')
            if not isinstance(stamp,int):
                raise ValueError(f'events.jsonl:{line_no}: missing reader monotonic stamp')
            row={key:event.get(key) for key in fields}
            row['event_line']=line_no
            row['details_json']=json.dumps({k:v for k,v in event.items() if k not in fields},ensure_ascii=False)
            timeline.writerow(row)
            total+=1
            parsed, observed = navlite_row(event, line_no)
            if parsed is not None:
                detail.writerow(parsed)
                navlite_count += 1
                parse_issue_count += bool(parsed['parse_issues'])
                observe_coverage(coverage, observed, line_no)
            for index in range(bisect.bisect_left(stamps,stamp-window),
                               bisect.bisect_right(stamps,stamp+window)):
                mark=marks[index]
                if window_rows >= 200000:
                    raise ValueError('Window export exceeds 200000 rows; use a smaller --window')
                windows.writerow({'marker_id':mark['id'],'label':mark['label'],
                    'delta_read_s':round((stamp-mark['stamp'])/1e9,6),**row})
                mark['event_count']+=1
                observe_coverage(mark['evidence_coverage'], observed, line_no)
                window_rows+=1
    missing = [key for key, value in coverage.items() if not value['count']]
    result={'version':'1.4','time_basis':'recorder_read_monotonic_not_source_time',
            'incomplete':bool(gaps),'incomplete_reasons':gaps,'markers':marks,
            'window_sec':args.window,'events':total,'window_rows':window_rows,
            'source_clock_verified':False,'full_motion_reconstruction_available':False,
            'evidence_coverage':coverage, 'unobserved_evidence':missing,
            'navlite_rows':navlite_count, 'navlite_parse_issue_rows':parse_issue_count,
            'observability_gaps':[
                'Continuous command/odom/scan/TF/costmap data are not recorded.',
                'No controller stage timings: elapsed_sec is failure duration, not compute time.',
                'No log may mean no event, suppressed logging or unavailable instrumentation.',
                'Nonzero return/publication does not prove chassis movement or downstream release.',
                'safety_arbitration no_message applies only to that input, not all final output.',
                'Coverage is evidence presence, not proof of a complete stop/recovery episode.'],
            'collision_action_counts':session.get('collision_action_counts',{}),
            'collision_action_counts_scope':'legacy Robot-to-action text only; NAVLITE in evidence_coverage',
            'cpu_one_core_average_pct':session.get('cpu_one_core_average_pct'),
            'limitations':session.get('limitations',[])}
    if (directory/'evidence_session.json').exists() or (directory/'motion').exists():
        replay_results = []
        if args.replay_helper:
            from nav_event_evidence import bounded_command
            helper = Path(args.replay_helper).expanduser().resolve()
            if not helper.is_file():
                raise ValueError('Offline replay helper missing: '+str(helper))
            for instance in sorted(directory.glob('mppi_*')):
                if not instance.is_dir():
                    continue
                target = destination/('replay_'+instance.name)
                replay_results.append({'instance':str(instance),'output':str(target),
                    **bounded_command([str(helper),'--input',str(instance),'--output',str(target)],60)})
            save_json(destination/'replay_runs.json',replay_results)
        from nav_event_evidence_report import build_report
        result['raw_internal_evidence'] = build_report(directory, destination/'motion', args.window)
        result['replay_runs'] = replay_results
        result['observability_gaps'][0] = ('Raw messages/internal snapshot coverage is reported separately in motion/; '
                                          'missing data must not be inferred from log presence.')
    save_json(destination/'summary.json',result)
    lines=['# Navigation event log report','',
           ('LOG STREAM SUMMARY; raw/internal evidence is reported separately in motion/.'
            if 'raw_internal_evidence' in result else
            'LOG ONLY. Discrete logged velocities may exist; no continuous motion/odom/scan evidence.'),
           'Windows use reader monotonic time, NOT source time or causal ordering.',
           'Filtered/suppressed logs and producer buffering remain unobservable.',
           'Control-period summaries retain first/last samples, not every original line.',
           f'Recorder end: {session.get("end_reason")}; incomplete={bool(gaps)}',
           f'Events={total}; markers={len(marks)}; window=+/-{args.window}s','',
           'Evidence: events.jsonl:<event_line>; timeline.csv; marker_windows.csv; navlite.csv.',
           f'NAVLITE rows={navlite_count}; parse-issue rows={parse_issue_count}', '',
           '## Evidence coverage (not a navigation gate)', '',
           'Missing means unobserved, NOT no stop/no failure. Log capture completeness is separate.']
    for key, value in coverage.items():
        lines.append(f'- {key}: {value["status"]}; count={value["count"]}; '
                     f'first/last event_line={value["first_event_line"]}/{value["last_event_line"]}')
    lines.extend(['', '## Interpretation limits', ''])
    lines.extend('- ' + gap for gap in result['observability_gaps'])
    lines.extend(['', '## Capture gaps and markers', ''])
    lines.extend('- '+gap for gap in gaps)
    for mark in marks:
        label=str(mark['label']).replace('\n',' ').replace('\r',' ')
        lines.append(f'- Mark {mark["id"]}: {label}; rows={mark["event_count"]}; '
                     f'window_clipped={mark["window_clipped"]}')
        absent = [key for key, value in mark['evidence_coverage'].items() if not value['count']]
        lines.append('  Unobserved in this window: ' + (', '.join(absent) or
                     'none of the listed categories; still discrete log evidence only'))
    (destination/'summary.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    if 'raw_internal_evidence' in result:
        with (destination/'summary.md').open('a',encoding='utf-8') as stream:
            stream.write('\n## Raw/internal evidence\n\nSee [motion report](motion/summary.md). '
                         'Log-only warnings above describe the text stream, not this additional capture.\n')
    print('report='+str(destination))
    print('summary='+str(destination/'summary.md'))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    rec = sub.add_parser("record", help="Tail selected existing local log files")
    rec.add_argument("--log-dir", help="Directory containing the four known CAR.V1 log names")
    rec.add_argument("--log", action="append", default=[], help="Additional/exact log path; repeatable")
    rec.add_argument("--output", help="New or empty directory; default unique /tmp/njrh_reports/nav_event_lite_...")
    rec.add_argument("--duration", type=float, default=300)
    rec.add_argument("--interval", type=float, default=0.2, help="Poll seconds (minimum 0.1)")
    rec.add_argument("--max-mb", type=float, default=16, help="Event file size cap; stop recorder at cap")
    rec.add_argument("--min-free-mb", type=float, default=128)
    rec.add_argument("--keyword", action="append", default=[], help="Additional literal keyword")
    rec.add_argument("--no-stdin", action="store_true")
    rec.add_argument('--motion',action='store_true',help='Native raw bag + bounded internal MPPI snapshots; no HTTP/Python ROS')
    rec.add_argument('--native-helper',default=str(Path(__file__).parent/'native/build/navlite_raw_capture'))
    rec.add_argument('--motion-max-mb',type=float,default=512)
    rec.add_argument('--mppi-max-mb',type=float,default=256)
    rec.add_argument('--live-params',action='store_true',help='Optional read-only param dump, <=4s/node before recording')
    rec.add_argument('--workspace',default=str(Path(__file__).resolve().parents[2]))
    inspect = sub.add_parser('inspect',help='Read process remaps/config; optional bounded live parameters. No motion.')
    inspect.add_argument('--output',required=True,help='New/empty inspection directory')
    inspect.add_argument('--live-params',action='store_true')
    inspect.add_argument('--workspace',default=str(Path(__file__).resolve().parents[2]))
    mark = sub.add_parser("mark", help="Append a local marker; no ROS/network/signals")
    mark.add_argument("--output", required=True)
    mark.add_argument("--label", default="operator_jerk")
    rep = sub.add_parser('report', help='Offline CSV and marker +/- window report; never contacts ROS')
    rep.add_argument('--output',required=True,help='Finished recording directory')
    rep.add_argument('--window',type=float,default=5,help='Seconds before/after each marker')
    rep.add_argument('--replay-helper',help='Optional offline native same-model final-sequence replay executable')
    args = parser.parse_args()
    try:
        if args.command == 'inspect':
            from nav_event_evidence import discover_runtime
            destination = Path(args.output).resolve()
            if destination.exists() and any(destination.iterdir()):
                raise ValueError('Inspection output must be new or empty')
            snapshot = discover_runtime(destination,args.workspace,args.live_params)
            print(json.dumps(snapshot,ensure_ascii=False,indent=2))
            print('BINDINGS_ONLY: live graph, messages and internal frames are checked by record --motion.')
            return 0 if not snapshot['chain_gaps'] else 3
        if args.command == "mark":
            add_mark(Path(args.output).expanduser().resolve(), args.label)
            print("MARK saved")
            return 0
        if args.command == 'report':
            return report(args)
        return record(args)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print("ERROR:", exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
