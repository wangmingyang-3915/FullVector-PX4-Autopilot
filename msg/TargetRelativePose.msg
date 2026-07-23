# Six-degree-of-freedom pose of this (camera/observer) vehicle relative to the tracked target vehicle.
# The parent/reference frame is the target vehicle's body FRD frame (X forward, Y right, Z down).
# The child frame is this vehicle's body FRD frame.

uint64 timestamp                 # Time when the message was published [microseconds]
uint64 timestamp_sample          # Time when the camera pose sample was captured [microseconds]

float32[3] position              # This vehicle body origin expressed in target body FRD [meters]
float32[4] q                     # Quaternion from this vehicle body FRD to target body FRD (w, x, y, z)

uint8 target_id                  # Tracked fiducial/target identifier
bool position_valid              # True when position contains a fresh, finite measurement
bool orientation_valid           # True when q contains a fresh, finite, normalized orientation
