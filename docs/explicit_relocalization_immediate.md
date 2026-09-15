# Explicit Isaac correction application

Explicit business relocalization is performed with the robot stationary. Once
an Isaac candidate passes the existing admission checks, the bridge atomically
sets current `map -> odom` to the accepted target. Both small and large explicit
corrections use `smoothing_policy=explicit_relocalization_immediate`, total and
remaining duration zero, progress one, and equal current/target sequences.

The existing dedicated publisher remains the only `sendTransform()` caller.
Its status snapshot is updated after publication, under the publication-stats
mutex. Consumers still confirm the new explicit sequence and settled/published
target; setting an internal target is not itself completion. No extra wait,
navigation-readiness requirement, stationary gate, or lock is introduced.

Ordinary AMCL gated corrections, including post-Isaac AMCL refinement, retain
the existing smoothing and admission logic. Configured translation/yaw rates
remain 0.20 m/s and 0.25 rad/s. Neither odom nor lidar configuration changes.
Legacy `explicit_relocalization_fast_*` parameters are accepted/reported for
compatibility but no longer select the explicit correction policy. Setting the
old fast flag false does not restore smoothing. Initial lock remains immediate.

## Validation and activation

`isolated_correction_application` runs the real bridge with synthetic odom,
Isaac, and AMCL in localhost-only ROS domain 219. It verifies initial lock,
large and small explicit applications without intermediate TF, published target
sequence, and ordinary AMCL intermediate TF/rates. It terminates its bridge on
every exit path. Do not run it against the live robot domain.

Candidate binaries are staged separately from active executables. Activation
requires an explicitly authorized full `njrh-runtime.service` restart; do not
rename, replace, or restart individual live nodes. Hardware acceptance still
requires an authorized stationary relocalization measuring acceptance-to-TF
completion and confirming ordinary AMCL behavior. The previous measured 3.095 s
settling interval is a potential saving, not a measured new total duration.
