# Must stay field-for-field compatible with PX4-Autopilot/msg/TargetRelativePose.msg.

uint64 timestamp
uint64 timestamp_sample

float32[3] position
float32[4] q

uint8 target_id
bool position_valid
bool orientation_valid
