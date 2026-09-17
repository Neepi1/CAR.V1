# BMS contact context / memory latch — 2026-09-17

## Scope and commercial behavior

A live BMS contact still stops propulsion immediately. Confirmed docking memory
must survive subsequent no-contact or stale samples until the existing release
conditions are met. However, the safety node's own stop must not manufacture the
external docking context used to establish that memory.

Only the contact classifier's command-context input and memory-establishment
input change. A receive timestamp is written only by `/cmd_vel_docking`'s input
callback and cleared alongside the existing command caches. The existing command
priority window remains 0.25 s. Output-cache timestamps, immediate zero output,
zero priority, contact thresholds, status/persistent evidence, reverse permits,
release predicates, service contracts and docking stages are unchanged.

There is no new state machine, service, lock, business gate or recovery motion.
`BMS_CONTACT_EVIDENCE` logs raw BMS fields and pre/post memory values only on a
contact rising edge or memory establishment; ordinary no-change samples add no log.

## Actual baseline, not workspace assumptions

Running and incident safety executable SHA256:
`0dd3437bd6d1d04f0893bf9099a76fcf0a7eba23206f37102cc6d7e326417052`.
Its retained main source SHA256:
`9546bb80cfac6ed97e52a4652adb2fff46ae57c7c1d47d8ba33a2a465694450a`.
This source matched local and Jetson source before editing. The deployed manifest
is `/tmp/njrh_reports/elevator_bypass_relink_20260910/candidate_manifest.json`.
The default build-directory executable is different and was not used as baseline.

Before candidate construction the recorded objects/archives were checked and
relinked to reproduce the installed executable byte for byte. Only the modified
`robot_safety_node.cpp` was compiled and then linked against these verified inputs;
no API, docking manager, sensor, ROS library or system package was rebuilt.

## Incident evidence and limits

All event times below are UTC+8 on 2026-09-17:

- 01:42:28.697: API reports ordinary navigation, `bms_contact=false`,
  `latch_docked=false`, `docking_state=stopped`, `docking_status=idle`.
- 01:42:51.629: safety logs `bms_contact_rising_edge`, epoch
  `1789580571.629064448`. This is inside the ordinary navigation task window,
  01:42:28–01:43:20, not the later fine-docking window.
- 02:15:39.270–02:16:10.115: the later fine-docking command was repeatedly
  forced to zero by `blocked_nonzero_docking_command`.
- The later state snapshot has `memory_latched=true`, fresh BMS no-contact,
  and `reverse_session_seen=false`. This is in-memory evidence, not the API's
  persistent `latched_docked` file value.

Sources: `/tmp/njrh_reports/docking_latest_20260917T0930` and
`/tmp/njrh_reports/navigation_latest_20260916T1742_ktvLiH`.

The historical rising-edge log has no raw current/status/present values and no
pre/post memory snapshot. It does **not** establish which raw branch was taken,
whether the contact was physically real, or when memory was first established.
The Ranger source currently publishes status UNKNOWN and present=false; if that
was the only source at the event, the applicable branch would be current >0.10 A.
That conditional inference is not a recovered historical BMS sample. No claim
of a proven current spike or false physical contact is made.

## Reproduced defect

Using a copy of the actual installed binary, with no external docking command,
idle status and no persistent dock evidence:

1. First contact sample: live=true, memory=false. The rising-edge stop writes
   the docking output cache and its timestamp.
2. Second contact sample within 0.25 s: the same cache now qualifies as a fresh
   docking command, causing memory=true despite no external docking context.
3. Later fresh no-contact cannot clear this memory through the ordinary release
   path because no completed controlled-undock session exists.

This proves a self-generated-context defect, not the complete historical physical
cause of the 01:42:51 sample. Both CHARGING and positive-current/voltage-contact
inputs reproduce it. The stop can also incorrectly supply context to FULL.

## Existing memory clearance (unchanged)

- Standard path: memory set; explicit reverse session seen; current undock
  succeeded; reverse permission disabled; fresh no-contact BMS. Callback order
  between success, permit and BMS remains handled by the existing code.
- Existing no-motion reconciliation: explicit outside-dock proof, fresh stable
  no-contact for 3 s, no docked status, no fresh reverse permit or docking command,
  plus a valid request. No reconciliation was invoked or added in this change.
- No-contact alone, stale BMS, cancellation, failed undock or old success does
  not clear confirmed memory. Restart is not used as a memory-release mechanism.

## Isolated validation / activation boundary

`test/test_bms_contact_history_isolated.py` uses the existing real ROS fixture,
a copied executable, private network/IPC/PID/mount namespaces, domain 183,
loopback only, private shared memory and remapped outputs. No chassis/real task
is involved. Run only with the guarded isolated fixture environment; ordinary
pytest collection outside that environment skips the file.

Current evidence and runner:
`/tmp/njrh_reports/bms_docking_interlock_20260917_vjCzXh`.
`run_isolated.sh` accepts a new result-directory name, `NJRH_TEST_SAFETY_BIN`
selects a copied candidate, and `NJRH_TEST_FILE` optionally selects the prior
undock regression suite. Do not reuse a pytest result directory.

- Baseline: 8 expected defect failures / 11 passes.
- Candidate: 19/19 contact/context/memory tests passed.
- Same candidate and fixture repeated twice: 38/38 passed.
- Prior undock clearance/failure/cancellation/current-contact regressions: 7/7.
- Tests linked against retained contact policy archive: 6/6.
- Tests linked against retained elevator bypass archive: 5/5.

An early fixture used a fixed 0.3 s before assuming docked status was received.
The corrected fixture waits for status discovery and observes memory confirmation.
Both baseline and candidate were rerun against that same corrected fixture;
production timeouts were not changed. Original failed runs remain in the report.

Candidate SHA256:
`c5fccbb5146f95f3d405a7686ade81b53142c1821a3fedd708cfc1fa7e075e50`.
This candidate was installed and verified running after the explicitly authorized
whole-service restart on 2026-09-17. Source sync, build and isolated tests are not
physical docking acceptance. Actual contact/undock testing still needs separate
user authorization and the next raw-edge evidence. No movement task or memory
reconciliation request was sent.

Deployment evidence: `/tmp/njrh_reports/bms_interlock_deploy_HKBO8jSl`.
The only restart command was the user-prescribed `njrh-runtime.service` restart.
It was requested at 11:51:08 UTC; systemctl returned at 11:51:18. The new navigation
context became ready at 11:52:28 (about 80 s including old-chain stop). At 11:53:17
API reported healthy navigation, AMCL_READY, safe_for_goal_start=true and no docked
contact block. New safety PID 84017 loaded the exact tested hash above. API PID
84490 retained SHA256 `1e02bead73ab728577d26ee2d2f45f71c66cb3588d2e98b948e43b1198ddcbaa`.
No build was run during this deployment: only the previously tested safety binary
and corresponding source were installed. Old safety binary/source are backed up
in the deployment report's `backup` directory.

The old-chain stop logged cleanup residuals and an ExecStop error; the existing
startup cleanup subsequently removed them. No startup/systemd strategy was changed.
Fresh process checks and health snapshots, not the systemctl return alone, were
used to verify this activation.
