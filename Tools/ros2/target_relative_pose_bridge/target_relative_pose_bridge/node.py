import math
from typing import List, Optional, Tuple

import rclpy
from px4_msgs.msg import OffboardControlMode, TargetRelativePose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
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
        self.declare_parameter('target_frames', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('target_ids', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('output_topic', '/fmu/in/target_relative_pose')
        self.declare_parameter(
            'offboard_control_mode_topic',
            '',
        )
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('offboard_heartbeat_rate_hz', 20.0)
        self.declare_parameter('max_pose_age_s', 0.2)
        # Do not republish stale orientation into the fast attitude loop by default.
        self.declare_parameter('dropout_grace_s', 0.0)
        # 对接失联和位姿异常值门控参数。
        self.declare_parameter('offboard_ready_timeout_s', 1.5)
        self.declare_parameter('position_jump_m', 0.2)
        self.declare_parameter('position_rate_limit_m_s', 3.0)
        self.declare_parameter('orientation_jump_rad', 0.2)
        self.declare_parameter('orientation_rate_limit_rad_s', 2.0)

        self._parent_frame = str(self.get_parameter('parent_frame').value)
        target_frame = str(self.get_parameter('target_frame').value)
        target_id = int(self.get_parameter('target_id').value)
        target_frames = [
            str(value) for value in self.get_parameter('target_frames').value
        ]
        target_ids = [
            int(value) for value in self.get_parameter('target_ids').value
        ]
        output_topic = str(self.get_parameter('output_topic').value)
        offboard_control_mode_topic = str(
            self.get_parameter('offboard_control_mode_topic').value
        )
        if not offboard_control_mode_topic:
            output_topic_prefix, separator, output_topic_name = (
                output_topic.rpartition('/')
            )

            if not separator or output_topic_name != 'target_relative_pose':
                raise ValueError(
                    'offboard_control_mode_topic must be set when output_topic '
                    'does not end in /target_relative_pose'
                )

            offboard_control_mode_topic = (
                f'{output_topic_prefix}/offboard_control_mode'
            )
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        offboard_heartbeat_rate_hz = float(
            self.get_parameter('offboard_heartbeat_rate_hz').value
        )
        self._max_pose_age_ns = int(float(self.get_parameter('max_pose_age_s').value) * 1e9)
        self._dropout_grace_ns = int(
            float(self.get_parameter('dropout_grace_s').value) * 1e9
        )
        # 将时限转换为纳秒，并缓存运动学门控阈值。
        self._offboard_ready_timeout_ns = int(
            float(self.get_parameter('offboard_ready_timeout_s').value) * 1e9
        )
        self._position_jump_m = float(self.get_parameter('position_jump_m').value)
        self._position_rate_limit_m_s = float(
            self.get_parameter('position_rate_limit_m_s').value
        )
        self._orientation_jump_rad = float(
            self.get_parameter('orientation_jump_rad').value
        )
        self._orientation_rate_limit_rad_s = float(
            self.get_parameter('orientation_rate_limit_rad_s').value
        )

        if publish_rate_hz <= 0.0 or offboard_heartbeat_rate_hz <= 0.0:
            raise ValueError(
                'publish_rate_hz and offboard_heartbeat_rate_hz must be positive'
            )

        # 检查各时间参数范围。
        if (
            self._max_pose_age_ns < 0
            or self._dropout_grace_ns < 0
            or self._offboard_ready_timeout_ns <= 0
        ):
            raise ValueError(
                'max_pose_age_s and dropout_grace_s must be non-negative; '
                'offboard_ready_timeout_s must be positive'
            )

        # 异常值门限必须为非负数。
        if (
            self._position_jump_m < 0.0
            or self._position_rate_limit_m_s < 0.0
            or self._orientation_jump_rad < 0.0
            or self._orientation_rate_limit_rad_s < 0.0
        ):
            raise ValueError('pose anomaly gate thresholds must be non-negative')

        if target_frames or target_ids:
            if not target_frames or len(target_frames) != len(target_ids):
                raise ValueError(
                    'target_frames and target_ids must be non-empty and have equal lengths'
                )

            if len(set(target_ids)) != len(target_ids):
                raise ValueError('target_ids must not contain duplicates')

            if any(target_id < 0 or target_id > 255 for target_id in target_ids):
                raise ValueError('target_ids must be in the uint8 range 0..255')

            self._targets: List[Tuple[int, str]] = list(zip(target_ids, target_frames))
        else:
            # Preserve the original single-target parameter interface.
            self._targets = [(target_id, target_frame)]

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = self.create_publisher(TargetRelativePose, output_topic, qos)
        self._offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            offboard_control_mode_topic,
            qos,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_valid: Optional[bool] = None
        self._active_target_id: Optional[int] = None
        self._last_pose = None
        self._holding_last_pose = False
        self._offboard_ready = False
        # 记录最近一次通过门控的本地接收时刻。
        self._last_valid_receive_ns: Optional[int] = None
        self._pose_timer = self.create_timer(1.0 / publish_rate_hz, self._publish_pose)
        self._heartbeat_timer = self.create_timer(
            1.0 / offboard_heartbeat_rate_hz,
            self._publish_offboard_heartbeat,
        )

        target_description = ', '.join(
            f'{target_id}:{target_frame}' for target_id, target_frame in self._targets
        )
        self.get_logger().info(
            f'Publishing pose of {self._parent_frame} expressed in '
            f'the freshest target frame on {output_topic}; targets=[{target_description}], '
            f'dropout_grace={self._dropout_grace_ns / 1e9:.3f} s; '
            f'offboard_ready_timeout={self._offboard_ready_timeout_ns / 1e9:.3f} s; '
            f'Offboard heartbeat={offboard_heartbeat_rate_hz:.1f} Hz '
            f'on {offboard_control_mode_topic}'
        )

    def _new_message(self, now_us: int, target_id: int) -> TargetRelativePose:
        msg = TargetRelativePose()
        msg.timestamp = now_us
        msg.timestamp_sample = 0
        msg.position = [0.0, 0.0, 0.0]
        msg.q = [1.0, 0.0, 0.0, 0.0]
        msg.target_id = target_id
        msg.position_valid = False
        msg.orientation_valid = False
        return msg

    def _publish_output(self, msg: TargetRelativePose) -> None:
        self._publisher.publish(msg)

    def _publish_offboard_heartbeat(self) -> None:
        # Commander only accepts Offboard when it receives a recent
        # OffboardControlMode with at least one control level enabled.
        # 短时视觉失效交给飞控保持；超过上限后撤销位置控制声明，避免就绪状态永久锁存。
        now_ns = self.get_clock().now().nanoseconds

        if (
            self._offboard_ready
            and self._last_valid_receive_ns is not None
            and now_ns - self._last_valid_receive_ns
            > self._offboard_ready_timeout_ns
        ):
            self._offboard_ready = False
            self._last_pose = None
            self._active_target_id = None
            self._set_valid_state(
                False,
                'no accepted pose before offboard readiness timeout',
            )

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = now_ns // 1000
        offboard_mode.position = self._offboard_ready
        offboard_mode.velocity = False
        offboard_mode.acceleration = False
        offboard_mode.attitude = False
        offboard_mode.body_rate = False
        offboard_mode.actuator = False
        self._offboard_control_mode_publisher.publish(offboard_mode)

    def _set_valid_state(
        self,
        valid: bool,
        reason: str = '',
        target_id: Optional[int] = None,
    ) -> None:
        target_changed = valid and target_id != self._active_target_id

        if self._last_valid == valid and not target_changed:
            return

        self._last_valid = valid

        if valid:
            self._active_target_id = target_id
            self.get_logger().info(
                f'Target relative pose is valid using target ID {target_id}'
            )
        else:
            self.get_logger().warning(f'Target relative pose is invalid: {reason}')

    def _publish_pose(self) -> None:
        now = self.get_clock().now()
        best_pose = None
        invalid_reasons = []

        for target_id, target_frame in self._targets:
            # 活动会话只跟踪已选目标，避免多目标之间跳变。
            if (
                self._offboard_ready
                and self._active_target_id is not None
                and target_id != self._active_target_id
            ):
                invalid_reasons.append(
                    f'ID {target_id}: target switching locked during active session'
                )
                continue

            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    self._parent_frame,
                    Time(),
                )
            except TransformException as error:
                invalid_reasons.append(f'ID {target_id}: {error}')
                continue

            sample_time = Time.from_msg(transform.header.stamp)
            pose_age_ns = now.nanoseconds - sample_time.nanoseconds

            if pose_age_ns < 0 or pose_age_ns > self._max_pose_age_ns:
                invalid_reasons.append(
                    f'ID {target_id}: stale TF age={pose_age_ns / 1e6:.1f} ms'
                )
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            position = [translation.x, translation.y, translation.z]
            quaternion_xyzw = [rotation.x, rotation.y, rotation.z, rotation.w]

            if not all(math.isfinite(value) for value in position + quaternion_xyzw):
                invalid_reasons.append(f'ID {target_id}: non-finite transform')
                continue

            quaternion_norm = math.sqrt(sum(value * value for value in quaternion_xyzw))

            if quaternion_norm < 1e-6:
                invalid_reasons.append(f'ID {target_id}: zero-norm quaternion')
                continue

            candidate = (
                sample_time.nanoseconds,
                target_id,
                position,
                [
                    float(rotation.w / quaternion_norm),
                    float(rotation.x / quaternion_norm),
                    float(rotation.y / quaternion_norm),
                    float(rotation.z / quaternion_norm),
                ],
            )

            # 与上一份有效位姿比较，执行位置和姿态运动学门控。
            if self._last_pose is not None and target_id == self._last_pose[1]:
                (
                    previous_sample_ns,
                    _,
                    previous_position,
                    previous_quaternion_wxyz,
                ) = self._last_pose
                sample_dt_s = max(
                    0.0,
                    min(1.0, (sample_time.nanoseconds - previous_sample_ns) / 1e9),
                )
                # 位置门限随有效样本间隔线性放宽。
                position_delta = math.sqrt(
                    sum(
                        (position[index] - previous_position[index]) ** 2
                        for index in range(3)
                    )
                )
                position_limit = (
                    self._position_jump_m
                    + self._position_rate_limit_m_s * sample_dt_s
                )

                if position_delta > position_limit:
                    invalid_reasons.append(
                        f'ID {target_id}: position jump {position_delta:.3f} m '
                        f'exceeds {position_limit:.3f} m'
                    )
                    continue

                # 用四元数夹角检查姿态跳变，并兼容 q 与 -q。
                quaternion_dot = abs(
                    sum(
                        candidate[3][index] * previous_quaternion_wxyz[index]
                        for index in range(4)
                    )
                )
                orientation_delta = 2.0 * math.acos(
                    max(0.0, min(1.0, quaternion_dot))
                )
                orientation_limit = (
                    self._orientation_jump_rad
                    + self._orientation_rate_limit_rad_s * sample_dt_s
                )

                if orientation_delta > orientation_limit:
                    invalid_reasons.append(
                        f'ID {target_id}: orientation jump '
                        f'{orientation_delta:.3f} rad exceeds '
                        f'{orientation_limit:.3f} rad'
                    )
                    continue

            if best_pose is None or candidate[0] > best_pose[0]:
                best_pose = candidate

        if best_pose is None:
            if self._last_pose is not None and self._dropout_grace_ns > 0:
                sample_time_ns, target_id, position, quaternion_wxyz = self._last_pose
                held_pose_age_ns = now.nanoseconds - sample_time_ns

                if 0 <= held_pose_age_ns <= self._dropout_grace_ns:
                    msg = self._new_message(now.nanoseconds // 1000, target_id)
                    msg.timestamp_sample = sample_time_ns // 1000
                    msg.position = [float(value) for value in position]
                    msg.q = list(quaternion_wxyz)
                    msg.position_valid = True
                    msg.orientation_valid = True

                    if not self._holding_last_pose:
                        self._holding_last_pose = True
                        self.get_logger().warning(
                            'Temporarily holding the last valid pose during a '
                            f'detection dropout; target ID {target_id}'
                        )

                    self._publish_output(msg)
                    return

            message_target_id = (
                self._active_target_id
                if self._active_target_id is not None
                else self._targets[0][0]
            )
            msg = self._new_message(now.nanoseconds // 1000, message_target_id)
            self._holding_last_pose = False
            self._set_valid_state(False, '; '.join(invalid_reasons))
            self._publish_output(msg)
            return

        sample_time_ns, target_id, position, quaternion_wxyz = best_pose
        if self._holding_last_pose:
            self.get_logger().info(
                f'Fresh target ID {target_id} pose recovered after a short dropout'
            )

        self._holding_last_pose = False
        self._last_pose = best_pose
        # 只有通过全部门控的位姿才刷新 Offboard 就绪时限。
        self._last_valid_receive_ns = now.nanoseconds
        msg = self._new_message(now.nanoseconds // 1000, target_id)
        msg.timestamp_sample = sample_time_ns // 1000
        msg.position = [float(value) for value in position]
        msg.q = quaternion_wxyz
        msg.position_valid = True
        msg.orientation_valid = True
        self._offboard_ready = True
        self._set_valid_state(True, target_id=target_id)
        self._publish_output(msg)


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
