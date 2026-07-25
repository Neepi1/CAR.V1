builtin_interfaces/Time stamp
uint64 generation

bool motion_blocked
bool hold_active
string[] hold_keys

bool execution_session_engaged
bool execution_lease_active
string execution_owner
string execution_mission_id
string execution_transaction_id
string execution_lease_id
builtin_interfaces/Duration execution_lease_remaining
bool execution_mode_contract_valid
bool normal_source_only

# Effective block contributed by this interlock contract for the normal Nav2
# source. `/safety/motion_allowed` remains authoritative for all safety gates.
bool interlock_effective_motion_blocked
string interlock_effective_block_reason

string transition_reason
