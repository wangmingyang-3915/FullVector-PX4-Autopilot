#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <utility>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/target_relative_pose.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace std::chrono_literals;

class ImageRelay final : public rclcpp::Node
{
public:
	ImageRelay() : Node("vision_dashboard_image_relay"), rate_started_(Clock::now())
	{
		const auto now = Clock::now();
		last_image_ = last_camera_info_ = last_detections_ = last_pose_ = now - 1s;
		last_heartbeat_ = last_vehicle_status_ = last_tf_ = now - 1s;

		auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
		auto tf_qos = rclcpp::QoS(100);

		image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
			"/vision_dashboard/image_preview", sensor_qos);
		camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
			"/vision_dashboard/camera_info", sensor_qos);
		detections_publisher_ = create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
			"/vision_dashboard/detections", sensor_qos);
		pose_publisher_ = create_publisher<px4_msgs::msg::TargetRelativePose>(
			"/vision_dashboard/target_relative_pose", sensor_qos);
		heartbeat_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
			"/vision_dashboard/offboard_control_mode", sensor_qos);
		vehicle_status_publisher_ = create_publisher<px4_msgs::msg::VehicleStatus>(
			"/vision_dashboard/vehicle_status", sensor_qos);
		tf_publisher_ = create_publisher<tf2_msgs::msg::TFMessage>(
			"/vision_dashboard/tf", tf_qos);
		rate_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(
			"/vision_dashboard/source_rates", sensor_qos);

		image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
			"/image_raw", sensor_qos, [this](sensor_msgs::msg::Image::UniquePtr message) {
				++counts_[0];
				forward_if_due<sensor_msgs::msg::Image>(std::move(message), image_publisher_, last_image_, 1s);
			});
		camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
			"/camera_info", sensor_qos, [this](sensor_msgs::msg::CameraInfo::UniquePtr message) {
				++counts_[1];
				forward_if_due<sensor_msgs::msg::CameraInfo>(
					std::move(message), camera_info_publisher_, last_camera_info_, 500ms);
			});
		detections_subscription_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
			"/detections", sensor_qos,
			[this](apriltag_msgs::msg::AprilTagDetectionArray::UniquePtr message) {
				++counts_[2];
				forward_if_due<apriltag_msgs::msg::AprilTagDetectionArray>(
					std::move(message), detections_publisher_, last_detections_, 100ms);
			});
		pose_subscription_ = create_subscription<px4_msgs::msg::TargetRelativePose>(
			"/uav1/fmu/in/target_relative_pose", sensor_qos,
			[this](px4_msgs::msg::TargetRelativePose::UniquePtr message) {
				++counts_[3];
				forward_if_due<px4_msgs::msg::TargetRelativePose>(
					std::move(message), pose_publisher_, last_pose_, 100ms);
			});
		heartbeat_subscription_ = create_subscription<px4_msgs::msg::OffboardControlMode>(
			"/uav1/fmu/in/offboard_control_mode", sensor_qos,
			[this](px4_msgs::msg::OffboardControlMode::UniquePtr message) {
				++counts_[4];
				forward_if_due<px4_msgs::msg::OffboardControlMode>(
					std::move(message), heartbeat_publisher_, last_heartbeat_, 200ms);
			});
		vehicle_status_subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
			"/uav1/fmu/out/vehicle_status", sensor_qos,
			[this](px4_msgs::msg::VehicleStatus::UniquePtr message) {
				++counts_[5];
				forward_if_due<px4_msgs::msg::VehicleStatus>(
					std::move(message), vehicle_status_publisher_, last_vehicle_status_, 200ms);
			});
		tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
			"/tf", tf_qos, [this](tf2_msgs::msg::TFMessage::UniquePtr message) {
				++counts_[6];
				forward_if_due<tf2_msgs::msg::TFMessage>(
					std::move(message), tf_publisher_, last_tf_, 100ms);
			});

		rate_timer_ = create_wall_timer(1s, [this]() { publish_source_rates(); });
		RCLCPP_INFO(
			get_logger(),
			"Continuously monitoring production vision topics; dashboard copies are rate limited");
	}

private:
	using Clock = std::chrono::steady_clock;

	template<typename Message>
	void forward_if_due(
		typename Message::UniquePtr message,
		const typename rclcpp::Publisher<Message>::SharedPtr &publisher,
		Clock::time_point &last_publish,
		Clock::duration interval)
	{
		const auto now = Clock::now();
		if (now - last_publish >= interval) {
			last_publish = now;
			publisher->publish(std::move(message));
		}
	}

	void publish_source_rates()
	{
		const auto now = Clock::now();
		const float seconds = std::chrono::duration<float>(now - rate_started_).count();
		std_msgs::msg::Float32MultiArray rates;
		rates.data.reserve(counts_.size());
		for (const auto count : counts_) {
			rates.data.push_back(seconds > 0.0F ? static_cast<float>(count) / seconds : 0.0F);
		}
		rate_publisher_->publish(rates);
		counts_.fill(0);
		rate_started_ = now;
	}

	std::array<std::uint64_t, 7> counts_{};
	Clock::time_point rate_started_;
	Clock::time_point last_image_;
	Clock::time_point last_camera_info_;
	Clock::time_point last_detections_;
	Clock::time_point last_pose_;
	Clock::time_point last_heartbeat_;
	Clock::time_point last_vehicle_status_;
	Clock::time_point last_tf_;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
	rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TargetRelativePose>::SharedPtr pose_publisher_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr heartbeat_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_publisher_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rate_publisher_;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
	rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_subscription_;
	rclcpp::Subscription<px4_msgs::msg::TargetRelativePose>::SharedPtr pose_subscription_;
	rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr heartbeat_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
	rclcpp::TimerBase::SharedPtr rate_timer_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageRelay>());
	rclcpp::shutdown();
	return 0;
}
