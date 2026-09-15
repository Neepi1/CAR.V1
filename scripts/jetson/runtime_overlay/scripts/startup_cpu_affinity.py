#!/usr/bin/env python3
"""Token-scoped startup placement; no ROS, signals, cgroups or environment edits.

The OS adapter is an intentional test seam. Callers use main(argv); production
uses Linux /proc identities and sched affinity, tests can supply an isolated OS.
"""
import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import time


STARTUP_CPUS = set(range(8))
MAX_ROUNDS = 8
LOCK_TIMEOUT = 5.0


class LinuxSystem:
    def task_start(self, pid, tid):
        try:
            fields = (Path('/proc') / str(pid) / 'task' / str(tid) / 'stat').read_text().rpartition(') ')[2].split()
            return int(fields[19]) if fields[0] not in ('Z', 'X') else None
        except (OSError, ValueError, IndexError):
            return None

    def process(self, pid):
        try:
            fields = (Path('/proc') / str(pid) / 'stat').read_text().rpartition(') ')[2].split()
            if fields[0] in ('Z', 'X'):
                return None
            return {'pid': int(pid), 'start': int(fields[19]), 'ppid': int(fields[1])}
        except (OSError, ValueError, IndexError):
            return None

    def processes(self):
        return {int(p.name): value for p in Path('/proc').iterdir()
                if p.name.isdigit() and (value := self.process(int(p.name))) is not None}

    def tasks(self, pid):
        result = {}
        try:
            for task in (Path('/proc') / str(pid) / 'task').iterdir():
                try:
                    fields = (task / 'stat').read_text().rpartition(') ')[2].split()
                    if fields[0] not in ('Z', 'X'):
                        result[int(task.name)] = int(fields[19])
                except (OSError, ValueError, IndexError):
                    continue
        except OSError:
            pass
        return result

    def affinity(self, tid):
        return os.sched_getaffinity(tid)

    def set_affinity(self, tid, cpus):
        os.sched_setaffinity(tid, cpus)


def cpus(value):
    result = set()
    try:
        for part in value.split(','):
            ends = part.split('-')
            if not 1 <= len(ends) <= 2 or any(not n.isdecimal() for n in ends):
                raise ValueError()
            low, high = int(ends[0]), int(ends[-1])
            if low > high or high > 7:
                raise ValueError()
            result.update(range(low, high + 1))
    except (ValueError, AttributeError):
        raise ValueError('CPU mask must be a nonempty subset of 0-7') from None
    if not result:
        raise ValueError('empty CPU mask')
    return result


def same(process, record):
    return process is not None and process['pid'] == record['pid'] and process['start'] == record['start']


def format_cpus(values):
    groups = []
    for value in sorted(values):
        if groups and groups[-1][1] + 1 == value:
            groups[-1][1] = value
        else:
            groups.append([value, value])
    return ','.join(str(low) if low == high else f'{low}-{high}' for low, high in groups)


def record_key(record):
    return f"{record['pid']}:{record['start']}"


def save(session, state):
    temporary = session / 'state.tmp'
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(state, stream, sort_keys=True)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, session / 'state.json')


@contextmanager
def locked(session):
    info = session.lstat()
    if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
        raise ValueError('session must be a real private directory')
    if os.name != 'nt' and (info.st_uid != os.geteuid() or info.st_mode & 0o077):
        raise ValueError('session must be owned by this uid and mode 0700')
    flags = os.O_RDWR | os.O_CREAT | getattr(os, 'O_NOFOLLOW', 0)
    descriptor = os.open(session / 'lock', flags, 0o600)
    if os.fstat(descriptor).st_size == 0:
        os.write(descriptor, b'\0')
    deadline = time.monotonic() + LOCK_TIMEOUT
    try:
        if os.name == 'nt':  # Allows OS-adapter tests on Windows; not a robot backend.
            import msvcrt
            def acquire():
                os.lseek(descriptor, 0, os.SEEK_SET)
                msvcrt.locking(descriptor, msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            def acquire():
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        while True:
            try:
                acquire()
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise TimeoutError('session lock timed out') from None
                time.sleep(0.01)
        state = json.loads((session / 'state.json').read_text(encoding='utf-8'))
        if state.get('schema') != 1 or state.get('phase') not in ('startup', 'steady'):
            raise ValueError('invalid session schema/phase')
        yield state
    finally:
        os.close(descriptor)


def effective(state, steady, system):
    return STARTUP_CPUS if state['phase'] == 'startup' and same(system.process(state['owner']['pid']), state['owner']) else steady


def apply_threads(system, record, target):
    """Identity-check each task immediately before the syscall and verify after.

Linux sched_setaffinity takes a numeric TID, not a pidfd: without stopping a
process the final stat/syscall race cannot be made kernel-atomic. Never follow
an observed replacement identity; disappearance is harmless, other errors fail.
"""
    pid = record['pid']
    observed = []
    failures = []
    if not same(system.process(pid), record):
        return observed
    for tid, start in system.tasks(pid).items():
        if not same(system.process(pid), record) or system.task_start(pid, tid) != start:
            continue
        try:
            if set(system.affinity(tid)) != target:
                if not same(system.process(pid), record) or system.task_start(pid, tid) != start:
                    continue
                system.set_affinity(tid, target)
            if same(system.process(pid), record) and system.task_start(pid, tid) == start:
                if set(system.affinity(tid)) != target:
                    raise RuntimeError(f'affinity verification failed pid={pid} tid={tid}')
                observed.append((pid, record['start'], tid, start))
        except (OSError, RuntimeError) as error:
            if same(system.process(pid), record) and system.task_start(pid, tid) == start:
                failures.append(f'pid={pid} tid={tid}: {error}')
    if failures:
        raise RuntimeError('; '.join(failures))
    return observed


def converge_process(system, record, target):
    previous = None
    for _ in range(MAX_ROUNDS):
        signature = sorted(apply_threads(system, record, target))
        if signature == previous:
            return
        previous = signature
    raise RuntimeError(f"tasks did not converge pid={record['pid']}")


def target_for(process, records, system):
    current, seen = process, set()
    while current and current['pid'] not in seen:
        seen.add(current['pid'])
        record = records.get(record_key(current))
        if record:
            if current['pid'] != process['pid'] and record.get('no_descendants', False):
                return None
            return record
        if any(old['pid'] == current['pid'] for old in records.values()):
            return None  # A reused PID is a boundary, not a route to an older root.
        parent = system.process(current['ppid'])
        if parent and parent['start'] > current['start']:
            return None
        current = parent
    return None


def belongs_to_owner(process, owner, system):
    current, seen = process, set()
    while current and current['pid'] not in seen:
        seen.add(current['pid'])
        if same(current, owner):
            return True
        parent = system.process(current['ppid'])
        if parent and parent['start'] > current['start']:
            return False
        current = parent
    return False


def registration_is_blocked(process, records, role, system):
    if role == 'navigation_runtime_owner':
        return False  # Explicit creation of a new navigation subtree by the caller.
    own = records.get(record_key(process))
    if own and own.get('scope_blocked', False):
        return True
    current, seen = system.process(process['ppid']), set()
    while current and current['pid'] not in seen:
        seen.add(current['pid'])
        ancestor = records.get(record_key(current))
        if ancestor:
            return ancestor.get('no_descendants', False)
        current = system.process(current['ppid'])
    return False


def begin(args, system):
    owner = system.process(args.owner_pid)
    if owner is None:
        raise ValueError('owner PID is not alive')
    base = Path(args.session_dir)
    base.mkdir(parents=True, exist_ok=True)
    session = Path(tempfile.mkdtemp(prefix='startup_cpu_', dir=base))
    os.chmod(session, 0o700)
    save(session, {'schema': 1, 'phase': 'startup', 'owner': owner, 'records': {}})
    print(session.resolve())
    return 0


def register(args, system):
    session = Path(args.session)
    steady = cpus(args.steady_cpus)
    process = system.process(args.pid)
    if process is None:
        return 0
    if not session.exists():
        converge_process(system, process, steady)
        return 0
    with locked(session) as state:
        if not same(system.process(process['pid']), process):
            return 0
        if (record_key(process) not in state['records'] and
                not belongs_to_owner(process, state['owner'], system)):
            if process['pid'] == os.getpid() and not same(system.process(state['owner']['pid']), state['owner']):
                converge_process(system, process, steady)
                return 0  # An orphan exec may launch steady, but cannot reopen/join old scope.
            raise ValueError('registration target is outside this owner generation')
        if state['phase'] == 'steady' and state.get('restored', False):
            # Daily short-lived probes still inherit the session token. A fully
            # closed session is immutable: apply steady without growing its log.
            converge_process(system, process, steady)
            return 0
        blocked = registration_is_blocked(process, state['records'], args.role, system)
        record = {**process, 'steady': sorted(steady), 'role': args.role,
                  'scope_blocked': blocked,
                  'no_descendants': args.no_descendants or blocked}
        state['records'][record_key(record)] = record
        save(session, state)  # Register BEFORE borrowing CPUs, so failures remain recoverable.
        converge_process(system, record, steady if blocked else effective(state, steady, system))
    return 0


def mask(args, system):
    steady = cpus(args.steady_cpus)
    session = Path(args.session)
    if session.exists():
        with locked(session) as state:
            steady = effective(state, steady, system)
    print(format_cpus(steady))
    return 0


def execute(args, system):
    command = args.argv[1:] if args.argv[:1] == ['--'] else args.argv
    if not command:
        raise ValueError('exec requires a command after --')
    args.pid = os.getpid()
    try:
        register(args, system)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
        print(f'[startup-cpu] WARN: boost failed; falling back to steady: {error}', file=sys.stderr)
        process = system.process(args.pid)
        if process is None:
            raise RuntimeError('cannot verify own identity for steady exec')
        # Scope rejection must never authorize closing somebody else's session.
        # Only an identity already recorded there could need boost rollback.
        registered_here = False
        try:
            with locked(Path(args.session)) as state:
                registered_here = record_key(process) in state['records']
        except (OSError, ValueError, KeyError, TypeError, RuntimeError):
            pass
        if registered_here:
            try:
                finish(argparse.Namespace(session=args.session, reason='boost_failed'), system)
            except (OSError, ValueError, KeyError, TypeError, RuntimeError) as recovery_error:
                print(f'[startup-cpu] WARN: session recovery incomplete: {recovery_error}', file=sys.stderr)
        converge_process(system, process, cpus(args.steady_cpus))
    os.execvp(command[0], command)
    return 0


def finish(args, system):
    session = Path(args.session)
    if not session.exists():
        return 0
    with locked(session) as state:
        if state['phase'] == 'steady' and state.get('restored', False):
            print('[startup-cpu] already steady; no recovery scan needed', file=sys.stderr)
            return 0
        state['phase'], state['reason'] = 'steady', args.reason
        save(session, state)  # Close admission before changing any process.
        previous = None
        deadline = time.monotonic() + 5.0
        for _ in range(MAX_ROUNDS):
            seen = []
            failures = []
            for process in system.processes().values():
                target = target_for(process, state['records'], system)
                if target is None or not same(system.process(process['pid']), process):
                    continue
                # Remember observed descendants even if their parent exits later.
                key = record_key(process)
                if key not in state['records']:
                    state['records'][key] = {**process, 'steady': target['steady'],
                                             'role': 'inherited', 'no_descendants': False}
                try:
                    seen.extend(apply_threads(system, process, set(target['steady'])))
                except (OSError, RuntimeError) as error:
                    failures.append(str(error))
            save(session, state)
            signature = sorted(seen)
            if signature == previous:
                if failures:
                    raise RuntimeError('steady restore incomplete: ' + '; '.join(failures))
                state['restored'] = True
                save(session, state)
                print(f"[startup-cpu] steady reason={args.reason} tasks={len(signature)}", file=sys.stderr)
                return 0
            previous = signature
            if time.monotonic() >= deadline:
                break
            time.sleep(0.01)
    raise RuntimeError('affinity did not converge within bounded finish')


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest='command', required=True)
    start = commands.add_parser('begin')
    start.add_argument('--owner-pid', type=int, required=True)
    start.add_argument('--session-dir', required=True)
    entry = commands.add_parser('register')
    entry.add_argument('--session', required=True)
    entry.add_argument('--pid', type=int, required=True)
    entry.add_argument('--steady-cpus', required=True)
    entry.add_argument('--role', default='')
    entry.add_argument('--no-descendants', action='store_true')
    execution = commands.add_parser('exec')
    execution.add_argument('--session', required=True)
    execution.add_argument('--steady-cpus', required=True)
    execution.add_argument('--role', default='')
    execution.add_argument('--no-descendants', action='store_true')
    execution.add_argument('argv', nargs=argparse.REMAINDER)
    lookup = commands.add_parser('mask')
    lookup.add_argument('--session', required=True)
    lookup.add_argument('--steady-cpus', required=True)
    close = commands.add_parser('finish')
    close.add_argument('--session', required=True)
    close.add_argument('--reason', required=True)
    return result


def main(argv=None, *, system=None):
    args = parser().parse_args(argv)
    system = system or LinuxSystem()
    try:
        return {'begin': begin, 'register': register, 'finish': finish,
                'exec': execute, 'mask': mask}[args.command](args, system)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
        print(f'[startup-cpu] ERROR: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
