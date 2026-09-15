#!/usr/bin/env python3
"""Private, startup-only budget/ownership check. No ROS imports or recovery."""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] not in ('Z', 'X') else None
    except (OSError, IndexError):
        return None


def ekf_process(pid):
    """Verify the actual ELF/executable and complete remap arguments, not text."""
    try:
        proc = Path(f'/proc/{pid}')
        started = identity(pid)
        executable = os.readlink(proc / 'exe')
        if not started or not executable.endswith('/lib/robot_localization/ekf_node'):
            return False
        if not os.path.samefile(proc / 'exe', executable):
            return False
        args = (proc / 'cmdline').read_bytes().split(b'\0')
        return (b'__node:=robot_local_state' in args and
                b'/odometry/filtered:=/local_state/odometry' in args and
                identity(pid) == started)
    except OSError:
        return False


def descendants(pid):
    # Jetson's kernel does not expose /proc/PID/task/PID/children. One process
    # snapshot also avoids spawning pgrep once per level of the launcher tree.
    children_by_parent = {}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] not in ('Z', 'X'):
                children_by_parent.setdefault(fields[1], []).append(proc.name)
        except (OSError, IndexError):
            continue
    pending = [pid]
    seen = {pid}
    while pending:
        parent = pending.pop()
        for child in children_by_parent.get(parent, []):
            if child not in seen:
                seen.add(child)
                pending.append(child)
                yield child


def remaining(deadline):
    if not math.isfinite(deadline):
        raise ValueError('startup deadline must be finite')
    return max(0.0, deadline - time.monotonic())


def stop_probe(process):
    # Only the readiness process created here, never the producer/robot nodes.
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def run_probe(command, marker, deadline, owner_ok):
    with tempfile.TemporaryFile() as output:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
        try:
            while remaining(deadline) > 0:
                if not owner_ok():
                    return False
                # Do not seek the child's shared file description while it
                # writes: pread leaves the writer offset intact.
                size = os.fstat(output.fileno()).st_size
                text = os.pread(output.fileno(), 65536, max(0, size - 65536)).decode(errors='replace')
                if marker in text:
                    # Preserve the existing acceptance of an explicit success
                    # receipt even if this disposable DDS client hangs on exit.
                    print(text, end='', flush=True)
                    return True
                if process.poll() is not None:
                    print(text, end='', flush=True)
                    return False
                time.sleep(min(0.1, remaining(deadline)))
            return False
        finally:
            stop_probe(process)


def wait_owned(pid, deadline, probe, ready_mode, helper_log):
    started = identity(pid)
    stage = 'WAIT_EKF_PROCESS'
    ekf_pid = None
    ekf_started = None

    def owner_ok():
        return bool(started and identity(pid) == started and
                    (ekf_pid is None or identity(ekf_pid) == ekf_started))

    try:
        while remaining(deadline) > 0 and owner_ok():
            matches = [child for child in descendants(pid) if ekf_process(child)]
            if len(matches) > 1:
                stage = 'DUPLICATE_OWNED_EKF'
                break
            if matches:
                ekf_pid = matches[0]
                ekf_started = identity(ekf_pid)
                print(f'[runtime-overlay] LOCAL_STATE_START_READY_STAGE stage=processes_ready pid={ekf_pid}', flush=True)
                stage = 'WAIT_EKF_ENDPOINT'
                checks = [('local-state-endpoint', 'robot_local_state endpoint ready')]
                if ready_mode == 'fresh_tf':
                    checks.append(('fresh-tf', 'fresh TF ready:'))
                for command, marker in checks:
                    budget = remaining(deadline)
                    if budget <= 0:
                        break
                    args = ([probe, command, f'{budget:.6f}', 'ekf'] if command == 'local-state-endpoint'
                            else [probe, command, 'odom', 'base_link', f'{budget:.6f}',
                                  os.environ.get('LOCAL_STATE_TF_READY_MAX_AGE_SEC', '0.75')])
                    if not run_probe(args, marker, deadline, owner_ok):
                        break
                    stage = 'WAIT_EKF_TF'
                else:
                    if owner_ok() and ekf_process(ekf_pid) and remaining(deadline) > 0:
                        print('[runtime-overlay] LOCAL_STATE_START_READY_STAGE stage=owned_ready', flush=True)
                        return 0
                break
            time.sleep(min(0.2, remaining(deadline)))
    finally:
        # Signal handlers raise rather than orphaning the disposable probe.
        pass
    reason = 'producer_exited' if not owner_ok() else 'deadline_or_probe_failure'
    print(f'[runtime-overlay] LOCAL_STATE_START_FAILED stage={stage} reason={reason} log={helper_log}', file=sys.stderr)
    try:
        with open(helper_log, 'rb') as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - 2048))
            print(stream.read().decode(errors='replace'), file=sys.stderr)
    except OSError:
        pass
    return 1


def main():
    command, *args = sys.argv[1:]
    if command == 'deadline':
        seconds = float(args[0])
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError('startup timeout must be positive and finite')
        print(f'{time.monotonic() + seconds:.6f}')
        return 0
    if command == 'remaining':
        seconds = remaining(float(args[0]))
        print(f'{seconds:.6f}')
        return 0 if seconds > 0 else 1
    if command == 'ekf-running':
        return 0 if any(ekf_process(p.name) for p in Path('/proc').iterdir() if p.name.isdigit()) else 1
    if command == 'wait' and args[3] in ('endpoint', 'fresh_tf'):
        return wait_owned(args[0], float(args[1]), *args[2:])
    raise ValueError('invalid local-state startup command')


if __name__ == '__main__':
    def interrupted(signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (ValueError, OSError) as error:
        print(f'[runtime-overlay] local-state startup observer error: {error}', file=sys.stderr)
        raise SystemExit(2)
