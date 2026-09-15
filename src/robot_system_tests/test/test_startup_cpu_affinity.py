"""Public command behavior with an isolated OS adapter; never pins a robot."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import threading

import pytest


SCRIPT = Path(os.environ.get('NJRH_STARTUP_CPU_AFFINITY_TEST_SCRIPT',
    str(Path(__file__).resolve().parents[3] / 'scripts/jetson/runtime_overlay/scripts/startup_cpu_affinity.py')))


def load_module():
    spec = importlib.util.spec_from_file_location('startup_affinity_under_test', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeSystem:
    def __init__(self):
        self.nodes = {100: (10, 1, {100: 10, 101: 11}),
                      200: (20, 100, {200: 20, 201: 21}),
                      300: (30, 200, {300: 30})}
        self.masks = {tid: {0, 1, 4} for _, _, tasks in self.nodes.values() for tid in tasks}
        self.writes = []

    def process(self, pid):
        node = self.nodes.get(pid)
        return {'pid': pid, 'start': node[0], 'ppid': node[1]} if node else None

    def processes(self):
        return {pid: self.process(pid) for pid in self.nodes}

    def tasks(self, pid):
        return dict(self.nodes[pid][2]) if pid in self.nodes else {}

    def task_start(self, pid, tid):
        return self.nodes.get(pid, (None, None, {}))[2].get(tid)

    def affinity(self, tid):
        return set(self.masks[tid])

    def set_affinity(self, tid, cpus):
        self.writes.append((tid, set(cpus)))
        self.masks[tid] = set(cpus)


def test_finish_restores_all_threads_and_closest_registered_ancestor(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    assert module.main(['begin', '--owner-pid', '100', '--session-dir', str(tmp_path)], system=system) == 0
    session = capsys.readouterr().out.strip()
    assert module.main(['register', '--session', session, '--pid', '100', '--steady-cpus', '0-1,4'], system=system) == 0
    assert module.main(['register', '--session', session, '--pid', '200', '--steady-cpus', '2'], system=system) == 0
    assert system.masks[100] == system.masks[101] == set(range(8))
    assert system.masks[200] == system.masks[201] == set(range(8))
    system.masks[300] = set(range(8))  # Unregistered child inherited startup mask.
    assert module.main(['finish', '--session', session, '--reason', 'startup_complete'], system=system) == 0
    assert system.masks[100] == system.masks[101] == {0, 1, 4}
    assert system.masks[200] == system.masks[201] == system.masks[300] == {2}


def test_exec_and_mask_use_live_phase_without_changing_environment(tmp_path, capsys, monkeypatch):
    module, system = load_module(), FakeSystem()
    assert module.main(['begin', '--owner-pid', '100', '--session-dir', str(tmp_path)], system=system) == 0
    session = capsys.readouterr().out.strip()
    assert module.main(['mask', '--session', session, '--steady-cpus', '1,4'], system=system) == 0
    assert capsys.readouterr().out.strip() == '0-7'
    monkeypatch.setenv('NJRH_CPUSET_CONTROLLER_SERVER', '0-1,4')
    monkeypatch.setattr(module.os, 'getpid', lambda: 200)
    executed = []
    monkeypatch.setattr(module.os, 'execvp', lambda command, args: executed.append((command, args)))
    assert module.main(['exec', '--session', session, '--steady-cpus', '2', '--role', 'nav', '--', 'program', '--flag'], system=system) == 0
    assert executed == [('program', ['program', '--flag'])]
    assert system.masks[200] == set(range(8))
    assert module.main(['finish', '--session', session, '--reason', 'done'], system=system) == 0
    assert module.main(['mask', '--session', session, '--steady-cpus', '2'], system=system) == 0
    assert capsys.readouterr().out.strip() == '2'
    assert module.main(['exec', '--session', session, '--steady-cpus', '2', '--', 'program'], system=system) == 0
    assert system.masks[200] == {2}
    assert os.environ['NJRH_CPUSET_CONTROLLER_SERVER'] == '0-1,4'


def new_session(module, system, tmp_path, capsys):
    assert module.main(['begin', '--owner-pid', '100', '--session-dir', str(tmp_path)], system=system) == 0
    return capsys.readouterr().out.strip()


def register_pid(module, system, session, pid, mask, *flags):
    return module.main(['register', '--session', session, '--pid', str(pid), '--steady-cpus', mask, *flags], system=system)


def test_no_descendants_barrier_excludes_mapping_but_explicit_nav_subtree_restores(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    system.nodes[400] = (40, 200, {400: 40})
    system.nodes[500] = (50, 400, {500: 50})
    system.masks.update({400: {1}, 500: set(range(8))})
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 100, '0-1,4') == 0
    assert register_pid(module, system, session, 200, '0-1,4', '--no-descendants') == 0
    assert register_pid(module, system, session, 400, '2', '--role', 'navigation_runtime_owner') == 0
    system.masks[300] = {6, 7}  # Mapping launched by API, deliberately unregistered.
    assert module.main(['finish', '--session', session, '--reason', 'done'], system=system) == 0
    assert system.masks[300] == {6, 7}
    assert system.masks[400] == system.masks[500] == {2}


def test_pid_replacement_and_its_children_are_not_modified(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 100, '0-1,4') == 0
    assert register_pid(module, system, session, 200, '2') == 0
    system.nodes[200] = (99, 100, {200: 99})
    system.nodes[300] = (100, 200, {300: 100})
    system.masks[200] = system.masks[300] = {7}
    system.writes.clear()
    assert module.main(['finish', '--session', session, '--reason', 'cancel'], system=system) == 0
    assert system.masks[200] == system.masks[300] == {7}
    assert not any(tid in (200, 300) for tid, _ in system.writes)


def test_unrelated_registration_cannot_borrow_or_join_session(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    system.nodes[900] = (90, 1, {900: 90})
    system.masks[900] = {7}
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 900, '2') != 0
    assert system.masks[900] == {7}


def test_register_catches_thread_born_while_applying(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    original = system.set_affinity
    def spawn_thread(tid, target):
        original(tid, target)
        if tid == 200 and 202 not in system.masks:
            system.nodes[200][2][202] = 22
            system.masks[202] = {2}
    system.set_affinity = spawn_thread
    assert register_pid(module, system, session, 200, '2') == 0
    assert system.masks[202] == set(range(8))


def test_one_task_failure_does_not_prevent_other_threads_and_subtrees_restoring(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 100, '0-1,4') == 0
    assert register_pid(module, system, session, 200, '2') == 0
    system.masks[300] = set(range(8))
    original = system.set_affinity
    def deny_one(tid, target):
        if tid == 200:
            raise PermissionError('isolated denial')
        original(tid, target)
    system.set_affinity = deny_one
    assert module.main(['finish', '--session', session, '--reason', 'cancel'], system=system) != 0
    assert system.masks[100] == {0, 1, 4}
    assert system.masks[201] == system.masks[300] == {2}
    assert module.main(['mask', '--session', session, '--steady-cpus', '0-1,4'], system=system) == 0
    assert capsys.readouterr().out.strip() == '0-1,4'


def test_owner_exit_disables_borrowing_and_finish_is_idempotent(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 200, '2') == 0
    del system.nodes[100]
    assert module.main(['mask', '--session', session, '--steady-cpus', '2'], system=system) == 0
    assert capsys.readouterr().out.strip() == '2'
    assert register_pid(module, system, session, 200, '2') == 0
    assert system.masks[200] == system.masks[201] == {2}
    for _ in range(2):
        assert module.main(['finish', '--session', session, '--reason', 'owner_exit'], system=system) == 0
    assert module.main(['finish', '--session', str(tmp_path / 'absent'), '--reason', 'absent'], system=system) == 0


def test_steady_registration_never_borrows_again(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    assert module.main(['finish', '--session', session, '--reason', 'failed'], system=system) == 0
    system.writes.clear()
    assert register_pid(module, system, session, 200, '2') == 0
    assert all(mask == {2} for _, mask in system.writes)


def test_registration_and_finish_are_serialized(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    entered, release, finishing = threading.Event(), threading.Event(), threading.Event()
    original = system.set_affinity
    def delayed_set(tid, target):
        if tid == 200 and target == set(range(8)):
            entered.set()
            assert release.wait(2), 'test release not delivered'
        original(tid, target)
    system.set_affinity = delayed_set
    results = []
    registration = threading.Thread(target=lambda: results.append(register_pid(module, system, session, 200, '2')))
    def close():
        finishing.set()
        results.append(module.main(['finish', '--session', session, '--reason', 'concurrent'], system=system))
    completion = threading.Thread(target=close)
    try:
        registration.start()
        assert entered.wait(2)
        completion.start()
        assert finishing.wait(2)
        release.set()
        registration.join(3)
        completion.join(3)
        assert not registration.is_alive() and not completion.is_alive()
        assert results == [0, 0]
        assert system.masks[200] == system.masks[201] == {2}
    finally:
        release.set()
        registration.join(3)
        if completion.ident:
            completion.join(3)


@pytest.mark.parametrize('mask', ['', '8', '-1', '3-1', '0;7', '0-1000000', '1,,4'])
def test_invalid_mask_never_changes_affinity(tmp_path, capsys, mask):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 200, mask) != 0
    assert system.writes == []


def test_thread_identity_replacement_during_enumeration_is_skipped(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    original = system.task_start
    replaced = False
    def identity(pid, tid):
        nonlocal replaced
        if tid == 201 and not replaced:
            replaced = True
            system.nodes[200][2][201] = 99
        return original(pid, tid)
    system.task_start = identity
    assert register_pid(module, system, session, 200, '2') == 0
    # Subsequent stable pass may legitimately cover the newly created task in
    # this same process; the stale identity is never used for the syscall.
    assert system.masks[201] == set(range(8))


def test_pid_reuse_between_affinity_read_and_write_is_not_touched(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    original = system.affinity
    def replace_on_read(tid):
        result = original(tid)
        if tid == 200:
            system.nodes[200] = (999, 1, {200: 999})
            system.masks[200] = {7}
        return result
    system.affinity = replace_on_read
    assert register_pid(module, system, session, 200, '2') == 0
    assert system.masks[200] == {7}
    assert not any(tid == 200 for tid, _ in system.writes)


def test_exec_boost_failure_closes_session_restores_steady_and_still_executes(tmp_path, capsys, monkeypatch):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 100, '0-1,4') == 0
    original = system.set_affinity
    def deny_boost(tid, target):
        if tid in (200, 201) and target == set(range(8)):
            raise PermissionError('boost denied only')
        original(tid, target)
    system.set_affinity = deny_boost
    monkeypatch.setattr(module.os, 'getpid', lambda: 200)
    executed = []
    monkeypatch.setattr(module.os, 'execvp', lambda command, args: executed.append(args))
    assert module.main(['exec', '--session', session, '--steady-cpus', '2', '--', 'program', 'arg'], system=system) == 0
    assert executed == [['program', 'arg']]
    assert system.masks[100] == {0, 1, 4}
    assert system.masks[200] == system.masks[201] == {2}
    assert module.main(['mask', '--session', session, '--steady-cpus', '2'], system=system) == 0
    captured = capsys.readouterr()
    assert captured.out.strip() == '2'
    assert 'boost failed' in captured.err and 'steady' in captured.err


def test_finished_session_does_not_reopen_scope_for_later_unknown_children(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 100, '0-1,4') == 0
    assert module.main(['finish', '--session', session, '--reason', 'done'], system=system) == 0
    system.nodes[800] = (80, 100, {800: 80})
    system.masks[800] = {7}
    system.writes.clear()
    assert module.main(['finish', '--session', session, '--reason', 'cleanup'], system=system) == 0
    assert system.writes == []
    assert system.masks[800] == {7}


def test_shared_navigation_role_below_api_barrier_stays_steady(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    system.nodes[400] = (40, 300, {400: 40})
    system.masks[400] = {6, 7}
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 100, '0-1,4') == 0
    assert register_pid(module, system, session, 200, '0-1,4', '--no-descendants', '--role', 'robot_api_server') == 0
    system.writes.clear()
    assert register_pid(module, system, session, 300, '3', '--role', 'hesai_ros_driver') == 0
    assert system.masks[300] == {3}
    assert all(target != set(range(8)) for _, target in system.writes)
    assert module.main(['finish', '--session', session, '--reason', 'done'], system=system) == 0
    assert system.masks[300] == {3}
    assert system.masks[400] == {6, 7}


def test_foreign_exec_fallback_cannot_finish_another_owners_session(tmp_path, capsys, monkeypatch):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 100, '0-1,4') == 0
    system.nodes[900] = (90, 1, {900: 90})
    system.masks[900] = {7}
    monkeypatch.setattr(module.os, 'getpid', lambda: 900)
    executed = []
    monkeypatch.setattr(module.os, 'execvp', lambda command, args: executed.append(args))
    assert module.main(['exec', '--session', session, '--steady-cpus', '2', '--', 'foreign_program'], system=system) == 0
    assert executed == [['foreign_program']]
    assert system.masks[900] == {2}
    assert system.masks[100] == system.masks[101] == set(range(8))
    assert module.main(['mask', '--session', session, '--steady-cpus', '0-1,4'], system=system) == 0
    assert capsys.readouterr().out.strip() == '0-7'


def test_completed_session_late_exec_loop_keeps_state_bytes_unchanged(tmp_path, capsys, monkeypatch):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 100, '0-1,4') == 0
    assert module.main(['finish', '--session', session, '--reason', 'done'], system=system) == 0
    state_path = Path(session) / 'state.json'
    original_bytes = state_path.read_bytes()
    original_mtime = state_path.stat().st_mtime_ns
    executed = []
    monkeypatch.setattr(module.os, 'execvp', lambda command, args: executed.append(args))
    for index in range(20):
        pid, start = 1000 + index, 100 + index
        system.nodes[pid] = (start, 100, {pid: start})
        system.masks[pid] = {7}
        monkeypatch.setattr(module.os, 'getpid', lambda pid=pid: pid)
        assert module.main(['exec', '--session', session, '--steady-cpus', '0-1,4',
                            '--role', 'runtime_probe', '--', 'probe', str(index)], system=system) == 0
        assert system.masks[pid] == {0, 1, 4}
        del system.nodes[pid]
    assert executed == [['probe', str(index)] for index in range(20)]
    assert state_path.read_bytes() == original_bytes
    assert state_path.stat().st_mtime_ns == original_mtime


def test_incomplete_steady_recovery_still_records_late_subtree_for_retry(tmp_path, capsys):
    module, system = load_module(), FakeSystem()
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 100, '0-1,4') == 0
    assert register_pid(module, system, session, 200, '2') == 0
    original = system.set_affinity
    def temporary_denial(tid, target):
        if tid == 200:
            raise PermissionError('temporary fixture denial')
        original(tid, target)
    system.set_affinity = temporary_denial
    assert module.main(['finish', '--session', session, '--reason', 'failed_restore'], system=system) != 0
    state_path = Path(session) / 'state.json'
    partial_bytes = state_path.read_bytes()
    system.nodes[400] = (40, 100, {400: 40})
    system.masks[400] = {7}
    assert register_pid(module, system, session, 400, '1') == 0
    assert system.masks[400] == {1}
    assert state_path.read_bytes() != partial_bytes
    system.nodes[500] = (50, 400, {500: 50})
    system.masks[500] = {7}
    system.set_affinity = original
    assert module.main(['finish', '--session', session, '--reason', 'retry'], system=system) == 0
    assert system.masks[200] == {2}
    assert system.masks[500] == {1}


@pytest.mark.skipif(sys.platform != 'linux', reason='real Linux affinity test uses only its own sleep child')
def test_linux_kernel_all_threads_of_owned_sleep_child(tmp_path, capsys):
    if not set(range(8)).issubset(os.sched_getaffinity(0)):
        pytest.skip('test caller must be allowed all Jetson CPUs 0-7')
    module = load_module()
    child = subprocess.Popen([sys.executable, '-u', '-c',
        'import threading,time; '
        '[threading.Thread(target=time.sleep,args=(20,),daemon=True).start() for _ in range(6)]; '
        'print("ready",flush=True); time.sleep(20)'], stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True)
    try:
        assert child.stdout.readline().strip() == 'ready'
        assert module.main(['begin', '--owner-pid', str(child.pid), '--session-dir', str(tmp_path)]) == 0
        session = capsys.readouterr().out.strip()
        assert module.main(['register', '--session', session, '--pid', str(child.pid), '--steady-cpus', '0-1,4']) == 0
        task_ids = list(module.LinuxSystem().tasks(child.pid))
        assert len(task_ids) >= 7
        assert all(os.sched_getaffinity(tid) == set(range(8)) for tid in task_ids)
        assert module.main(['finish', '--session', session, '--reason', 'isolated_test']) == 0
        assert all(os.sched_getaffinity(tid) == {0, 1, 4} for tid in task_ids)
    finally:
        child.terminate()  # Only this fixture's own, never-ROS process.
        child.communicate(timeout=5)


@pytest.mark.skipif(sys.platform != 'linux', reason='real exec fixture affects only its own sleep child')
def test_linux_exec_wrapper_preserves_pid_then_restores_spawned_threads(tmp_path, capsys):
    original_parent_mask = os.sched_getaffinity(0)
    if not set(range(8)).issubset(original_parent_mask):
        pytest.skip('test caller must be allowed all Jetson CPUs 0-7')
    module = load_module()
    # The pytest owner is NOT registered, so no affinity operation targets it.
    assert module.main(['begin', '--owner-pid', str(os.getpid()), '--session-dir', str(tmp_path)]) == 0
    session = capsys.readouterr().out.strip()
    child = subprocess.Popen([sys.executable, str(SCRIPT), 'exec', '--session', session,
        '--steady-cpus', '0-1,4', '--role', 'isolated_fixture', '--',
        sys.executable, '-u', '-c', 'import os,threading,time; '
        '[threading.Thread(target=time.sleep,args=(20,),daemon=True).start() for _ in range(6)]; '
        'print(os.getpid(),flush=True); time.sleep(20)'], stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True)
    try:
        assert child.stdout.readline().strip() == str(child.pid)
        tasks = list(module.LinuxSystem().tasks(child.pid))
        assert len(tasks) >= 7
        assert all(os.sched_getaffinity(tid) == set(range(8)) for tid in tasks)
        assert module.main(['finish', '--session', session, '--reason', 'isolated_exec']) == 0
        assert all(os.sched_getaffinity(tid) == {0, 1, 4} for tid in tasks)
        assert os.sched_getaffinity(0) == original_parent_mask
    finally:
        child.terminate()
        child.communicate(timeout=5)
