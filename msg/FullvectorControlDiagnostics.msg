# Structured diagnostics for the fullvector controller.
# This topic is for ULog analysis only and is not consumed by the control path.

uint64 timestamp
uint64 timestamp_sample

# Final output selected by the controller in this cycle.
uint8 OUTPUT_STATE_NONE = 0
uint8 OUTPUT_STATE_NORMAL = 1
uint8 OUTPUT_STATE_HOLD_LAST = 2
uint8 OUTPUT_STATE_SAFE_FALLBACK = 3
uint8 OUTPUT_STATE_NEUTRAL_TILT = 4
uint8 OUTPUT_STATE_ATTITUDE_ONLY = 5
uint8 output_state

# Reason for a non-normal output state.
uint8 FALLBACK_REASON_NONE = 0
uint8 FALLBACK_REASON_DISABLED = 1
uint8 FALLBACK_REASON_DISARMED = 2
uint8 FALLBACK_REASON_TERMINATION = 3
uint8 FALLBACK_REASON_UNSUPPORTED_MODE = 4
uint8 FALLBACK_REASON_NATIVE_REQUEST = 5
uint8 FALLBACK_REASON_INVALID_DT = 6
uint8 FALLBACK_REASON_STATE_UNAVAILABLE = 7
uint8 FALLBACK_REASON_ATTITUDE_STALE = 8
uint8 FALLBACK_REASON_POSITION_STALE = 9
uint8 FALLBACK_REASON_MANUAL_THROTTLE_IDLE = 10
uint8 FALLBACK_REASON_RELATIVE_POSE_ABORTED = 11
uint8 fallback_reason

# Whether the continuous setpoint fields were updated in this cycle.
bool control_setpoint_valid
bool position_control_active

# Input freshness levels: 0 normal, 1 warning, 2 hold, 3 failed.
uint8 state_age_level
uint8 position_state_age_level
uint8 attitude_state_age_level

# Current input ages. UINT64_MAX means that no sample has been received.
uint64 position_age
uint64 velocity_age
uint64 attitude_age
uint64 angular_velocity_age

# Position-controller acceleration at each limiting stage, NED [m/s^2].
float32[3] acceleration_sp_raw
float32[3] acceleration_sp_limited
float32[3] acceleration_sp_final
bool acceleration_limited
bool jerk_limited

# Attitude outer-loop and angular-rate inner-loop diagnostics.
uint8 ATTITUDE_ERROR_SOURCE_ABSOLUTE = 0
uint8 ATTITUDE_ERROR_SOURCE_RELATIVE_YAW = 1
uint8 ATTITUDE_ERROR_SOURCE_FULL_RELATIVE = 2
uint8 attitude_error_source
float32[3] attitude_sp
float32[3] attitude_error
float32[3] angular_velocity_sp
float32[3] angular_velocity_error

# Saturation of the four fullvector motors and four tilt servos.
bool actuator_saturated
uint8 motor_upper_saturation_mask
uint8 motor_lower_saturation_mask
uint8 servo_saturation_mask
