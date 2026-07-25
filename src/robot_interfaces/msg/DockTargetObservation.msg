std_msgs/Header header

# Adapter identity, for example "gs2_scan" or "orbbec_336l_depth".
string source

# sensor_healthy reports that the adapter processed a fresh sensor sample.
# valid reports that the charging-dock geometry passed all fit gates.
bool sensor_healthy
bool valid

# All geometric values use header.frame_id, normally base_link.
# forward_gap_m is the +X gap from charge_contact_link to the dock face.
# lateral_error_m is positive when the dock center is toward vehicle +Y.
# yaw_error_rad is positive when a counter-clockwise vehicle correction is required.
float64 forward_gap_m
float64 lateral_error_m
float64 yaw_error_rad

float64 lateral_span_m
float64 confidence
uint32 inlier_count
float64 rms_error_m
string reason
