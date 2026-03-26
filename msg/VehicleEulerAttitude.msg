# Euler attitude message for onboard modules

uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32 roll                    # roll angle [rad]
float32 pitch                   # pitch angle [rad]
float32 yaw                     # yaw angle [rad]

# TOPICS vehicle_euler_attitude