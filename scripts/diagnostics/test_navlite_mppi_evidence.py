"""Real plugin -> file -> lightweight recorder regression; private ROS namespace only."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mppi-test', type=Path, required=True)
    parser.add_argument('--runtime-test', type=Path, required=True)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('NAVLITE_ISOLATED') != '1' or os.environ.get('ROS_DOMAIN_ID') != '181':
        parser.error('requires private network/IPC/mount namespace and domain 181')
    interfaces = {line.split(':', 1)[0].strip()
                  for line in Path('/proc/net/dev').read_text().splitlines() if ':' in line}
    if interfaces - {'lo'}:
        parser.error('refusing test: network namespace contains non-loopback interfaces')
    args.output.mkdir(parents=True, exist_ok=False)
    log = args.output / 'resident_navigation_runtime.log'
    log.touch()
    capture = args.output / 'capture'
    recorder_path = Path(__file__).with_name('nav_event_lite.py')
    recorder = subprocess.Popen([
        'python3', '-B', str(recorder_path), 'record', '--log', str(log),
        '--output', str(capture), '--duration', '110', '--interval', '0.1',
        '--no-stdin', '--min-free-mb', '0'], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)
    results = {}
    try:
        deadline = time.monotonic() + 5
        while True:
            events = capture / 'events.jsonl'
            if events.exists() and 'opened' in events.read_text():
                break
            if recorder.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('recorder did not open fixture log')
            time.sleep(0.05)
        env = dict(os.environ)
        env['AMENT_PREFIX_PATH'] = str(args.prefix) + ':' + env.get('AMENT_PREFIX_PATH', '')
        with log.open('ab') as output:
            for name, binary in [('mppi', args.mppi_test), ('runtime', args.runtime_test)]:
                result = subprocess.run([str(binary), '--gtest_output=xml:' +
                    str(args.output / (name + '.xml'))], stdout=output,
                    stderr=subprocess.STDOUT, env=env, timeout=45)
                results[name] = result.returncode
        time.sleep(0.4)  # test-only: let the file tail drain; not a production delay
    finally:
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
        recout = recorder.communicate(timeout=8)[0].decode(errors='replace')
        (args.output / 'recorder.log').write_text(recout)
    assert all(code == 0 for code in results.values()), (results, log.read_text()[-6000:])
    assert recorder.returncode == 0, recout
    events = [json.loads(line) for line in (capture / 'events.jsonl').read_text().splitlines()]
    diagnostics = [row for row in events if 'NAVLITE' in row.get('text', '')]
    assert any('NAVLITE mppi event=diagnostics_ready schema=1' in row['text']
               for row in diagnostics), 'missing real-plugin startup evidence'
    failure = next(row for row in diagnostics if 'NAVLITE mppi event=failure_start' in row['text'])
    assert 'raw="Optimizer fail to compute path"' in failure['text'], failure
    assert 'stage=fallback' in failure['text'] and 'fail_flag=1' in failure['text'], failure
    assert 'optimize_passes=4 passes_entered_failed=3' in failure['text'], failure
    assert all(row['category'] == 'diagnostic' and row['source_stamp_ns'] is not None
               for row in diagnostics)
    result = subprocess.run(['python3', '-B', str(recorder_path), 'report',
                             '--output', str(capture)], capture_output=True, text=True)
    assert result.returncode == 0, (result.stdout, result.stderr)
    reports = sorted(capture.glob('report*'))
    # Offline export reads captured text; it does not subscribe to ROS.
    csv_paths = list(capture.rglob('navlite.csv'))
    assert csv_paths, ('no structured report', reports, recout)
    with csv_paths[-1].open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    actual_failures = [row for row in rows if row['module'] == 'mppi' and
                       row['event'] == 'failure_start']
    recovered = [row for row in rows if row['module'] == 'mppi' and
                 row['output_status'] == 'returned_nonzero']
    assert actual_failures and recovered, 'real failure/recovery missing from structured output'
    assert actual_failures[0]['original_error'] == 'Optimizer fail to compute path'
    result = dict(tests=results, event_count=len(diagnostics), output=str(capture),
                  failure_row=actual_failures[0], recovery_row=recovered[0],
                  scope='real installed optimizer + synthetic costmap; not hardware acceptance')
    (args.output / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
