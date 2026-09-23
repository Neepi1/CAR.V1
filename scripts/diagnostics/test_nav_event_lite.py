"""Linux CLI tests, fake temporary logs only; no ROS or real robot inputs."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPT = Path(__file__).with_name('nav_event_lite.py')


class CaptureTests(unittest.TestCase):
    def offline_report(self, lines):
        import csv
        self.out.mkdir()
        session = {'status': 'finished', 'boot_id': 'fixture',
                   'started': {'read_monotonic_ns': 0},
                   'ended': {'read_monotonic_ns': 30000000000}}
        (self.out/'summary.json').write_text(json.dumps(session))
        (self.out/'marks.jsonl').write_text(json.dumps({
            'boot_id': 'fixture', 'read_monotonic_ns': 15000000000,
            'label': 'sidepass'})+'\n')
        (self.out/'events.jsonl').write_text(''.join(json.dumps({
            'category': 'diagnostic', 'read_monotonic_ns': 15000000000+i,
            'read_wall_ns': 1800000000000000000+i,
            'source_stamp_ns': 1789725000000000000+i,
            'source': 'fake.log', 'text': text})+'\n' for i, text in enumerate(lines)))
        raw = (self.out/'events.jsonl').read_bytes()
        result = subprocess.run([sys.executable, '-B', str(SCRIPT), 'report',
            '--output', str(self.out)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, (result.stdout, result.stderr))
        destination = next(self.out.glob('report_*'))
        self.assertEqual(raw, (self.out/'events.jsonl').read_bytes())
        with (destination/'navlite.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        return rows, json.loads((destination/'summary.json').read_text())

    def test_offline_navlite_separates_calculation_publication_and_state(self):
        lines = [
            'NAVLITE mppi event=failure_start episode=1 compute_seq=4 '
            'consecutive_failures=1 elapsed_sec=0 stage=transformPath '
            'exception_type=runtime_error raw="TF lookup failed: not a collision" '
            'fail_flag=0 command_returned=0',
            'NAVLITE mppi event=failure_continues episode=1 compute_seq=19 '
            'consecutive_failures=16 elapsed_sec=1.04 stage=transformPath '
            'exception_type=runtime_error raw="TF lookup failed: not a collision" '
            'fail_flag=0 command_returned=0',
            'NAVLITE mppi event=calculation_recovered_zero command_returned=1 out=(0,0,0)',
            'NAVLITE mppi event=first_nonzero_after_calculation_recovery '
            'command_returned=1 out=(0.2,-0.01,-0.05)',
            'NAVLITE controller event=computed_return layer=outer_rotation_shim out=(0.2,0,0)',
            'NAVLITE collision action=STOP region=StopZone published=1 zero=1 '
            'in=(0.2,0,0) requested_out=(0,0,0) publication=sent reason=polygon_stop',
            'NAVLITE collision action=STOP region=StopZone published=0 zero=1 '
            'in=(0.2,0,0) requested_out=(0,0,0) publication=no_message reason=stop_pub_timeout',
            'NAVLITE safety_state previous=COMMAND_STALE state=OK allowed=1 publication=state_only',
            'NAVLITE safety_arbitration input_source=0 selected_source=2 '
            'in=(0.2,0,0) publication=no_message reason=other_source_selected',
            'NAVLITE collision action=DO_NOTHING region=none published=1 '
            'in=(0.2,0,0) requested_out=(0.2,0,0) publication=sent reason=pass',
            'NAVLITE safety branch=pass state=OK allowed=1 source=0 input_known=1 '
            'in=(0.2,0,0) out=(0.2,0,0) final_reason=pass publication=sent',
        ]
        rows, summary = self.offline_report(lines)
        self.assertEqual(rows[0]['original_error'], 'TF lookup failed: not a collision')
        self.assertEqual(rows[1]['consecutive_failures'], '16')
        self.assertEqual(rows[1]['failure_elapsed_sec'], '1.04')
        self.assertEqual(rows[2]['output_status'], 'returned_zero')
        self.assertEqual(rows[3]['output_status'], 'returned_nonzero')
        self.assertEqual(rows[3]['out_vy'], '-0.01')
        self.assertEqual(rows[4]['module'], 'controller')
        self.assertEqual(rows[5]['output_status'], 'published_zero')
        self.assertEqual(rows[6]['output_status'], 'no_message')
        self.assertEqual(rows[6]['out_vx'], '')
        self.assertEqual(rows[6]['requested_out_vx'], '0.0')
        self.assertEqual(rows[7]['output_status'], 'state_only')
        self.assertEqual(rows[8]['output_status'], 'no_message')
        self.assertEqual(rows[-1]['output_status'], 'published_nonzero')
        self.assertEqual([int(row['event_line']) for row in rows], list(range(1, 12)))
        coverage = summary['evidence_coverage']
        self.assertEqual(coverage['mppi_failure_detail']['count'], 2)
        self.assertEqual(coverage['mppi_nonzero_return']['count'], 1)
        self.assertEqual(coverage['collision_publication']['count'], 3)
        self.assertEqual(coverage['safety_state']['count'], 1)
        self.assertEqual(summary['markers'][0]['evidence_coverage'], coverage)
        self.assertFalse(summary['full_motion_reconstruction_available'])

    def test_legacy_mppi_recovery_is_saved_without_inventing_velocity(self):
        self.start()
        text = ('[INFO] [1789725000.123] [controller_server]: '
                'Ordinary MPPI recovered a valid control in the same FollowPath')
        self.append(text + '\n')
        events, _ = self.stop()
        saved = [event for event in events if event.get('text') == text]
        self.assertEqual(len(saved), 1)
        self.assertEqual(saved[0]['category'], 'mppi')
        self.assertNotIn('out', saved[0])

    def test_offline_missing_events_are_not_normal_or_capture_loss(self):
        rows, summary = self.offline_report([
            '[INFO] [controller]: Ordinary MPPI recovered a valid control in the same FollowPath',
            '[runtime-overlay] lifecycle configure node=/collision_monitor',
        ])
        self.assertEqual(rows, [])
        self.assertFalse(summary['incomplete'])
        self.assertEqual(len(summary['unobserved_evidence']), 11)
        self.assertTrue(all(v['status'] == 'not_observed_not_proof_of_absence'
                            for v in summary['evidence_coverage'].values()))

    def test_chassis_cache_is_not_can_confirmation_or_fresh_motion(self):
        rows, summary = self.offline_report([
            'NAVLITE chassis event=diagnostics_ready schema=1',
            'NAVLITE chassis event=sample publication=observation input_known=1 '
            'in=(-0.2,0,0.1) input_seq=42 input_age_sec=0.03 input_gap=0 '
            'sdk_known=1 sdk_submit_seq=42 sdk_linear=-0.2 sdk_steer=0.3 sdk_angular=0.1 '
            'callback_submitted=0 reason=motion_command can_tx_confirmed=unknown '
            'desired_mode=0 actual_mode=2 mode_changing=1 mode_state=WAITING '
            'odom_known=1 actual=(-0.08,0,0.05) actual_kind=wheel_odom_cache '
            'motion_fresh=unknown wheel_can_age_sec=unknown',
            'NAVLITE chassis event=sample publication=observation input_known=0 '
            'in=(0,0,0) sdk_known=0 odom_known=0 actual=(0,0,0)',
        ])
        self.assertEqual(rows[0]['output_status'], 'diagnostics_ready_not_motion')
        self.assertEqual(rows[1]['output_status'], 'driver_observation')
        self.assertEqual(rows[1]['in_vx'], '-0.2')
        self.assertEqual(rows[1]['out_vx'], '')
        self.assertEqual(rows[1]['actual_vx'], '-0.08')
        self.assertEqual(rows[1]['actual_kind'], 'wheel_odom_cache')
        self.assertEqual(rows[1]['motion_fresh'], 'unknown')
        self.assertEqual(rows[1]['sdk_steer'], '0.3')
        self.assertEqual(rows[1]['actual_mode'], '2')
        self.assertEqual(rows[2]['in_vx'], '')
        self.assertEqual(rows[2]['actual_vx'], '')
        self.assertEqual(summary['evidence_coverage']['chassis_input']['count'], 1)
        self.assertEqual(summary['evidence_coverage']['chassis_sdk_cache']['count'], 1)
        self.assertEqual(summary['evidence_coverage']['chassis_motion_cache']['count'], 1)

    def test_offline_zero_pass_and_unknown_input_do_not_invent_motion(self):
        rows, _ = self.offline_report([
            'NAVLITE collision action=DO_NOTHING region=none published=1 '
            'in=(0,0,0) requested_out=(0,0,0) publication=sent reason=pass',
            'NAVLITE safety branch=watchdog source=-1 input_known=0 in=(nan,nan,nan) '
            'out=(0,0,0) state=COMMAND_STALE final_reason=COMMAND_STALE publication=sent',
            'NAVLITE safety_arbitration input_source=0 selected_source=0 event=selected '
            'final_output=see_safety_event',
            'NAVLITE mppi event=calculation_recovered_nonzero command_returned=1 out=(nan,0,0)',
            'NAVLITE mppi event=calculation_recovered_nonzero command_returned=1 out=(inf,0,0)',
        ])
        self.assertEqual(rows[0]['output_status'], 'published_zero')
        self.assertEqual(rows[1]['in_vx'], '')
        self.assertEqual(rows[1]['selected_source'], '-1')
        self.assertEqual(rows[1]['output_status'], 'published_zero')
        self.assertTrue(all(row['output_status'] == 'unknown' for row in rows[2:]))

    def test_offline_bad_fields_and_escaped_exception_preserve_evidence(self):
        error = 'TF "not found"; detail=not_collision\nsecond line\\tail'
        rows, summary = self.offline_report([
            'NAVLITE mppi event=failure_start command_returned=0 stage=transformPath '
            'exception_type=runtime_error consecutive_failures=1 elapsed_sec=0 raw='+json.dumps(error),
            'NAVLITE collision published=0 published=1 publication=sent requested_out=(0.2,0,0)',
            'NAVLITE mppi raw="broken quote command_returned=1 out=(0.2,0,0)',
            'NAVLITE future_format sample=one',
        ])
        self.assertEqual(rows[0]['original_error'], error)
        self.assertEqual(summary['navlite_parse_issue_rows'], 2)
        self.assertEqual(rows[1]['output_status'], 'unknown')
        self.assertEqual(rows[2]['output_status'], 'unknown')
        self.assertEqual(rows[3]['output_status'], 'unknown')
        self.assertEqual(summary['evidence_coverage']['collision_publication']['count'], 0)

    def test_navlite_stop_resume_and_no_publication_are_preserved(self):
        self.start()
        lines = [
            'NAVLITE controller branch=mppi_no_valid_control zero=1 out=(0,0,0)',
            'NAVLITE collision action=STOP region=StopZone published=1 zero=1',
            'NAVLITE collision action=STOP region=StopZone published=0 zero=1 '
            'publication=stop_pub_timeout_no_message',
            'NAVLITE safety branch=snapshot_event state=COMMAND_STALE allowed=0 zero=1',
            'NAVLITE collision action=DO_NOTHING region=none published=1 zero=0',
            'NAVLITE safety branch=pass state=OK allowed=1 out=(0.2,0,0) zero=0',
        ]
        self.append(''.join('[WARN] [1789725000.123] [fixture]: '+x+'\n' for x in lines))
        records, _ = self.stop()
        saved = [x['text'] for x in records if x['category'] == 'diagnostic']
        self.assertEqual(len(saved), len(lines))
        for expected, actual in zip(lines, saved):
            self.assertIn(expected, actual)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='navlite_fake_')
        self.root = Path(self.tmp.name)
        self.log = self.root / 'fake.log'
        self.log.write_text('old history\n')
        self.out = self.root / 'capture'
        self.proc = None

    def tearDown(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.communicate()
        self.tmp.cleanup()

    def start(self, *args):
        self.proc = subprocess.Popen([sys.executable, '-B', str(SCRIPT), 'record',
            '--log', str(self.log), '--output', str(self.out), '--duration', '20',
            '--interval', '0.1', '--min-free-mb', '0', '--no-stdin', *args],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        until = time.monotonic()+4
        while time.monotonic()<until:
            if self.proc.poll() is not None:
                self.fail(self.proc.communicate())
            events=self.out/'events.jsonl'
            if events.exists() and '"opened"' in events.read_text():
                return
            time.sleep(0.03)
        self.fail('recorder did not open fake log')

    def append(self, text):
        with self.log.open('a') as f:
            f.write(text)
        time.sleep(0.25)

    def stop(self):
        self.proc.send_signal(signal.SIGINT)
        stdout,stderr=self.proc.communicate(timeout=5)
        self.assertEqual(self.proc.returncode,0,(stdout,stderr))
        records=[json.loads(x) for x in (self.out/'events.jsonl').read_text().splitlines()]
        summary=json.loads((self.out/'summary.json').read_text())
        return records,summary

    def test_lifecycle_is_not_collision_action_and_project_tags_are_saved(self):
        self.start()
        self.append('[runtime-overlay] lifecycle configure node=/collision_monitor\n'
                    '[INFO] [1789725000.123] [ranger_base]: mode_switch requested\n'
                    '[INFO] [1789725000.124] [controller]: [ordinary-recovery] phase=waiting\n'
                    '[INFO] [1789725000.125] [controller]: [local-repair] route replaced\n'
                    '[INFO] [1789725000.126] [collision_monitor]: Robot to stop due to StopZone\n'
                    '[INFO] [1789725000.127] [collision_monitor]: Robot to continue normal operation\n')
        events,summary=self.stop()
        by_text={x.get('text'):x for x in events if 'text' in x}
        config=next(x for text,x in by_text.items() if 'lifecycle' in text)
        self.assertEqual(config['category'],'collision_log')
        self.assertIsNone(config.get('collision_action'))
        self.assertTrue(any(x['category']=='recovery' for x in events))
        self.assertTrue(any(x['category']=='path' for x in events))
        self.assertTrue(any(x['category']=='mode_safety' for x in events))
        self.assertEqual(summary['collision_action_counts'],{'stop':1,'continue':1})

    def test_missing_log_is_incomplete_without_blocking_valid_log(self):
        self.start('--log',str(self.root/'missing.log'))
        self.append('[WARN] [1789725000.123] diagnostic message\n')
        events,summary=self.stop()
        self.assertTrue(summary['incomplete'])
        self.assertTrue(any('missing.log' in x for x in summary['incomplete_reasons']))
        self.assertTrue(any(x['category']=='warning_error' for x in events))

    def test_mark_and_offline_window_report_preserve_raw_evidence(self):
        import csv
        self.start()
        self.append('[WARN] [1789725000.123] marker-near-event\n')
        result=subprocess.run([sys.executable,'-B',str(SCRIPT),'mark',
            '--output',str(self.out),'--label','sidepass'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.append('[WARN] [1789725000.124] marker-after-event\n')
        self.stop()
        raw=(self.out/'events.jsonl').read_bytes()
        result=subprocess.run([sys.executable,'-B',str(SCRIPT),'report',
            '--output',str(self.out),'--window','5'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,(result.stdout,result.stderr))
        folders=list(self.out.glob('report_*'))
        self.assertEqual(len(folders),1)
        with (folders[0]/'marker_windows.csv').open() as f:
            rows=list(csv.DictReader(f))
        texts='\n'.join(row['text'] for row in rows)
        self.assertIn('marker-near-event',texts)
        self.assertIn('marker-after-event',texts)
        self.assertNotIn('old history',texts)
        self.assertEqual((self.out/'events.jsonl').read_bytes(),raw)
        quality=json.loads((folders[0]/'summary.json').read_text())
        self.assertTrue(quality['markers'][0]['window_clipped'])
        self.assertEqual(quality['time_basis'],'recorder_read_monotonic_not_source_time')

    def test_idle_sigint_is_clean_and_preserves_history(self):
        self.start()
        events,summary=self.stop()
        self.assertFalse(summary['incomplete'])
        self.assertEqual(summary['end_reason'],'signal_2')
        self.assertEqual(self.log.read_text(),'old history\n')
        self.assertFalse(any(x.get('text')=='old history' for x in events))

    def test_rename_rotation_reads_new_generation(self):
        self.start()
        self.append('[WARN] before_rotation\n')
        self.log.rename(self.root/'old.log')
        self.log.write_text('[WARN] after_rotation\n')
        time.sleep(0.35)
        events,summary=self.stop()
        lines={x['text']:x for x in events if 'text' in x}
        self.assertIn('[WARN] before_rotation',lines)
        self.assertIn('[WARN] after_rotation',lines)
        self.assertNotEqual(lines['[WARN] before_rotation']['generation'],
                            lines['[WARN] after_rotation']['generation'])
        self.assertFalse(summary['incomplete'])

    def test_truncation_and_long_lines_are_visible_gaps(self):
        self.start()
        self.log.write_text('')
        time.sleep(0.25)
        self.append('[WARN] '+('x'*9000)+'\n')
        _,summary=self.stop()
        self.assertTrue(summary['incomplete'])
        self.assertIn('truncation_observed',' '.join(summary['incomplete_reasons']))
        self.assertIn('line_truncated',' '.join(summary['incomplete_reasons']))

    def test_disk_budget_stops_only_recorder_with_partial_evidence(self):
        self.start('--max-mb','0.05')
        self.append(('[WARN] '+('x'*200)+'\n')*500)
        stdout,stderr=self.proc.communicate(timeout=5)
        self.assertEqual(self.proc.returncode,2,(stdout,stderr))
        summary=json.loads((self.out/'summary.json').read_text())
        self.assertTrue(summary['incomplete'])
        self.assertEqual(summary['end_reason'],'event_file_size_limit')
        self.assertLessEqual((self.out/'events.jsonl').stat().st_size,int(0.05*1024*1024))
        self.assertGreater(self.log.stat().st_size,100000)

    def test_low_free_disk_returns_incomplete(self):
        self.start('--min-free-mb','1000000000000')
        stdout,stderr=self.proc.communicate(timeout=6)
        self.assertEqual(self.proc.returncode,2,(stdout,stderr))
        summary=json.loads((self.out/'summary.json').read_text())
        self.assertEqual(summary['end_reason'],'disk_low')
        self.assertTrue(summary['incomplete'])

    def test_existing_output_is_never_overwritten(self):
        self.out.mkdir()
        evidence=self.out/'summary.json'
        evidence.write_text('original evidence')
        result=subprocess.run([sys.executable,'-B',str(SCRIPT),'record',
            '--log',str(self.log),'--output',str(self.out)],capture_output=True,text=True)
        self.assertEqual(result.returncode,2)
        self.assertEqual(evidence.read_text(),'original evidence')

    def test_no_visible_log_rejected_without_session(self):
        result=subprocess.run([sys.executable,'-B',str(SCRIPT),'record',
            '--log',str(self.root/'missing'),'--output',str(self.out)],capture_output=True,text=True)
        self.assertEqual(result.returncode,2)
        self.assertFalse(self.out.exists())

    def test_repeated_and_reversed_source_stamps_are_not_rewritten(self):
        self.start()
        self.append('[WARN] [1789725000.123] one\n[WARN] [1789725000.123] two\n'
                    '[WARN] [1789724000.123] three\n')
        events,_=self.stop()
        messages=[x for x in events if x.get('text')]
        self.assertEqual([x['source_stamp_ns'] for x in messages],
            [1789725000123000000,1789725000123000000,1789724000123000000])
        self.assertEqual([x['read_monotonic_ns'] for x in messages],
                         sorted(x['read_monotonic_ns'] for x in messages))

    def test_marker_wrong_boot_refused_and_expired_session_refused(self):
        self.start()
        manifest=self.out/'summary.json'
        data=json.loads(manifest.read_text())
        data['boot_id']='other-host'
        manifest.write_text(json.dumps(data))
        command=[sys.executable,'-B',str(SCRIPT),'mark','--output',str(self.out)]
        self.assertEqual(subprocess.run(command,capture_output=True).returncode,2)
        self.stop()
        self.assertEqual(subprocess.run(command,capture_output=True).returncode,2)
        self.assertEqual((self.out/'marks.jsonl').read_text(),'')

    def test_wall_clock_jump_is_recorded_without_changing_system_clock(self):
        import nav_event_lite
        from unittest.mock import patch
        real_clock=time.time_ns
        calls=[0]
        def shifted_clock():
            calls[0]+=1
            return real_clock()+(2000000000 if calls[0]>=3 else 0)
        argv=[str(SCRIPT),'record','--log',str(self.log),'--output',str(self.out),
              '--duration','0.35','--interval','0.1','--min-free-mb','0','--no-stdin']
        old_handlers={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
        try:
            with patch.object(sys,'argv',argv), patch.object(time,'time_ns',shifted_clock):
                self.assertEqual(nav_event_lite.main(),0)
        finally:
            for s,handler in old_handlers.items(): signal.signal(s,handler)
        events=[json.loads(x) for x in (self.out/'events.jsonl').read_text().splitlines()]
        self.assertTrue(any(x['category']=='clock_jump' for x in events))

    def test_synthetic_four_log_load_reports_recorder_cpu(self):
        others=[self.root/f'fake_{i}.log' for i in range(3)]
        for p in others:p.write_text('')
        args=[item for p in others for item in ('--log',str(p))]
        self.start(*args)
        deadline=time.monotonic()+5
        lines=('[WARN] synthetic events only '+('x'*100)+'\n')*5
        while time.monotonic()<deadline:
            for p in [self.log]+others:
                with p.open('a') as f:f.write(lines)
            time.sleep(0.1)
        time.sleep(0.25)
        events,summary=self.stop()
        self.assertFalse(summary['incomplete'])
        self.assertGreater(summary['event_file_bytes'],100000)
        print('PERF_FAKE_4_LOGS '+json.dumps({k:summary[k] for k in
            ('elapsed_s','cpu_one_core_average_pct','cpu_one_core_peak_2s_pct',
             'max_rss_kib','event_file_bytes','max_observed_backlog_bytes')}),flush=True)


if __name__ == '__main__':
    unittest.main(verbosity=2)
