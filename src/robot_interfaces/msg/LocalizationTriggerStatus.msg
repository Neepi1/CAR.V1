# Per-request outcome. UNKNOWN is not a terminal failure and must not unlock admission.
uint8 RUNNING=0
uint8 UNKNOWN=1
uint8 SUCCEEDED=2
uint8 FAILED=3
string request_id
uint8 state
string code
string detail
bool outcome_known
string building_id
string floor_id
string map_id
uint64 asset_epoch
string asset_digest
uint64 localizer_generation
uint64 baseline_explicit_sequence
uint64 accepted_explicit_sequence
