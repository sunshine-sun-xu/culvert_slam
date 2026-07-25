#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class LivoxPreprocessNode : public rclcpp::Node
{
public:
  LivoxPreprocessNode()
  : Node("livox_preprocess_node")
  {
    input_imu_topic_ = this->declare_parameter<std::string>("input_imu_topic", "/livox/imu");
    input_lidar_topic_ = this->declare_parameter<std::string>("input_lidar_topic", "/livox/lidar");
    output_imu_topic_ = this->declare_parameter<std::string>("output_imu_topic", "/livox/imu_rotated");
    output_lidar_topic_ = this->declare_parameter<std::string>("output_lidar_topic", "/livox/lidar_rotated");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "");
    rotation_y_deg_ = this->declare_parameter<double>("rotation_y_deg", 60.0);

    const double rotation_y_rad = rotation_y_deg_ * M_PI / 180.0;
    rotation_matrix_ = Eigen::AngleAxisd(rotation_y_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    rotation_quaternion_ = Eigen::Quaterniond(rotation_matrix_);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LivoxPreprocessNode::imu_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_lidar_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LivoxPreprocessNode::lidar_callback, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_imu_topic_, 10);
    lidar_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(output_lidar_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "livox_preprocess_node started: imu %s -> %s, lidar %s -> %s, Ry=%.1f deg",
      input_imu_topic_.c_str(),
      output_imu_topic_.c_str(),
      input_lidar_topic_.c_str(),
      output_lidar_topic_.c_str(),
      rotation_y_deg_);
  }

private:
  Eigen::Vector3d rotate_vector(double x, double y, double z) const
  {
    return rotation_matrix_ * Eigen::Vector3d(x, y, z);
  }

  std::array<double, 9> rotate_covariance(const std::array<double, 9> & cov) const
  {
    Eigen::Matrix3d cov_matrix;
    cov_matrix <<
      cov[0], cov[1], cov[2],
      cov[3], cov[4], cov[5],
      cov[6], cov[7], cov[8];
    const Eigen::Matrix3d rotated = rotation_matrix_ * cov_matrix * rotation_matrix_.transpose();
    return {
      rotated(0, 0), rotated(0, 1), rotated(0, 2),
      rotated(1, 0), rotated(1, 1), rotated(1, 2),
      rotated(2, 0), rotated(2, 1), rotated(2, 2)
    };
  }

  void maybe_override_frame(std::string & frame_id) const
  {
    if (!output_frame_id_.empty()) {
      frame_id = output_frame_id_;
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out = *msg;
    maybe_override_frame(out.header.frame_id);

    const auto accel = rotate_vector(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
    out.linear_acceleration.x = accel.x();
    out.linear_acceleration.y = accel.y();
    out.linear_acceleration.z = accel.z();

    const auto gyro = rotate_vector(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);
    out.angular_velocity.x = gyro.x();
    out.angular_velocity.y = gyro.y();
    out.angular_velocity.z = gyro.z();

    Eigen::Quaterniond q_in(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z);
    const Eigen::Quaterniond q_out = rotation_quaternion_ * q_in;
    out.orientation.x = q_out.x();
    out.orientation.y = q_out.y();
    out.orientation.z = q_out.z();
    out.orientation.w = q_out.w();

    out.angular_velocity_covariance = rotate_covariance(msg->angular_velocity_covariance);
    out.linear_acceleration_covariance = rotate_covariance(msg->linear_acceleration_covariance);
    if (msg->orientation_covariance[0] >= 0.0) {
      out.orientation_covariance = rotate_covariance(msg->orientation_covariance);
    }

    imu_pub_->publish(out);
  }

  void lidar_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    livox_ros_driver2::msg::CustomMsg out = *msg;
    maybe_override_frame(out.header.frame_id);

    for (auto & point : out.points) {
      const auto rotated = rotate_vector(point.x, point.y, point.z);
      point.x = static_cast<float>(rotated.x());
      point.y = static_cast<float>(rotated.y());
      point.z = static_cast<float>(rotated.z());
    }

    lidar_pub_->publish(out);
  }

  double rotation_y_deg_{60.0};
  std::string input_imu_topic_;
  std::string input_lidar_topic_;
  std::string output_imu_topic_;
  std::string output_lidar_topic_;
  std::string output_frame_id_;
  Eigen::Matrix3d rotation_matrix_{Eigen::Matrix3d::Identity()};
  Eigen::Quaterniond rotation_quaternion_{Eigen::Quaterniond::Identity()};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxPreprocessNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
