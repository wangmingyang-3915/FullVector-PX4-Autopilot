import math
from typing import Optional

import rclpy
from px4_msgs.msg import TargetRelativePose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TargetRelativePoseBridge(Node):
    """Publish body_frd pose expressed in target_body_frd to PX4."""

    def __init__(self) -> None:
        super().__init__('target_relative_pose_bridge')

        self.declare_parameter('parent_frame', 'body_frd')
        self.declare_parameter('target_frame', 'target_body_frd')
        self.declare_parameter('target_id', 1)
        self.declare_parameter('output_topic', '/fmu/in/target_relative_pose')
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('max_pose_age_s', 0.2)

        self._parent_frame = str(self.get_parameter('parent_frame').value)
        self._target_frame = str(self.get_parameter('target_frame').value)
        self._target_id = int(self.get_parameter('target_id').value)
        output_topic = str(self.get_parameter('output_topic').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._max_pose_age_ns = int(float(self.get_parameter('max_pose_age_s').value) * 1e9)

        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = self.create_publisher(TargetRelativePose, output_topic, qos)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_valid: Optional[bool] = None
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish_pose)

        self.get_logger().info(
            f'Publishing pose of {self._parent_frame} expressed in '
            f'{self._target_frame} on {output_topic}'
        )

    def _new_message(self, now_us: int) -> TargetRelativePose:
        msg = TargetRelativePose()
        msg.timestamp = now_us
        msg.timestamp_sample = 0
        msg.position = [0.0, 0.0, 0.0]
        msg.q = [1.0, 0.0, 0.0, 0.0]
        msg.target_id = self._target_id
        msg.position_valid = False
        msg.orientation_valid = False
        return msg

    def _set_valid_state(self, valid: bool, reason: str = '') -> None:
        if self._last_valid == valid:
            return

        self._last_valid = valid

        if valid:
            self.get_logger().info('Target relative pose is valid')
        else:
            self.get_logger().warning(f'Target relative pose is invalid: {reason}')

    def _publish_pose(self) -> None:
        now = self.get_clock().now()
        msg = self._new_message(now.nanoseconds // 1000)

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._parent_frame,
                Time(),
            )
        except TransformException as error:
            self._set_valid_state(False, str(error))
            self._publisher.publish(msg)
            return

        sample_time = Time.from_msg(transform.header.stamp)
        pose_age_ns = now.nanoseconds - sample_time.nanoseconds

        if pose_age_ns < 0 or pose_age_ns > self._max_pose_age_ns:
            self._set_valid_state(False, f'stale TF age={pose_age_ns / 1e6:.1f} ms')
            self._publisher.publish(msg)
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        position = [translation.x, translation.y, translation.z]
        quaternion_xyzw = [rotation.x, rotation.y, rotation.z, rotation.w]

        if not all(math.isfinite(value) for value in position + quaternion_xyzw):
            self._set_valid_state(False, 'non-finite transform')
            self._publisher.publish(msg)
            return

        quaternion_norm = math.sqrt(sum(value * value for value in quaternion_xyzw))

        if quaternion_norm < 1e-6:
            self._set_valid_state(False, 'zero-norm quaternion')
            self._publisher.publish(msg)
            return

        msg.timestamp_sample = sample_time.nanoseconds // 1000
        msg.position = [float(value) for value in position]
        # geometry_msgs uses x,y,z,w; PX4 messages use w,x,y,z.
        msg.q = [
            float(rotation.w / quaternion_norm),
            float(rotation.x / quaternion_norm),
            float(rotation.y / quaternion_norm),
            float(rotation.z / quaternion_norm),
        ]
        msg.position_valid = True
        msg.orientation_valid = True
        self._set_valid_state(True)
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetRelativePoseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
