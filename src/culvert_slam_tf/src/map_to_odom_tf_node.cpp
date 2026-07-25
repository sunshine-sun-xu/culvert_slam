#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class MapToOdomTfNode : public rclcpp::Node
{
public:
  MapToOdomTfNode()
  : Node("map_to_odom_tf_node")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    optimized_pose_topic_ =
      declare_parameter<std::string>("optimized_pose_topic", "/optimized_pose");
    publish_rate_ = declare_parameter<double>("publish_rate", 30.0);
    lookup_timeout_sec_ = declare_parameter<double>("lookup_timeout_sec", 0.05);
    identity_until_optimized_ = declare_parameter<bool>("identity_until_optimized", true);

    if (publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be > 0");
    }
    if (map_frame_.empty() || odom_frame_.empty() || base_frame_.empty()) {
      throw std::invalid_argument("map_frame, odom_frame and base_frame must be non-empty");
    }
    if (map_frame_ == odom_frame_ || odom_frame_ == base_frame_ || map_frame_ == base_frame_) {
      throw std::invalid_argument("map_frame, odom_frame and base_frame must be distinct");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    optimized_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      optimized_pose_topic_,
      rclcpp::QoS(10),
      std::bind(&MapToOdomTfNode::optimizedPoseCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));
    publish_timer_ = create_wall_timer(period, std::bind(&MapToOdomTfNode::publishMapToOdom, this));

    RCLCPP_INFO(
      get_logger(),
      "map_to_odom_tf_node started: %s -> %s -> %s, optimized_pose_topic=%s",
      map_frame_.c_str(),
      odom_frame_.c_str(),
      base_frame_.c_str(),
      optimized_pose_topic_.c_str());
  }

private:
  void optimizedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    if (pose->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Ignoring optimized pose in frame '%s'; expected '%s'",
        pose->header.frame_id.c_str(),
        map_frame_.c_str());
      return;
    }

    latest_map_to_base_.header = pose->header;
    latest_map_to_base_.child_frame_id = base_frame_;
    latest_map_to_base_.transform.translation.x = pose->pose.position.x;
    latest_map_to_base_.transform.translation.y = pose->pose.position.y;
    latest_map_to_base_.transform.translation.z = pose->pose.position.z;
    latest_map_to_base_.transform.rotation = pose->pose.orientation;
    have_optimized_pose_ = true;
  }

  void publishMapToOdom()
  {
    if (!have_optimized_pose_) {
      if (identity_until_optimized_) {
        publishIdentityMapToOdom();
      }
      return;
    }

    geometry_msgs::msg::TransformStamped odom_to_base_msg;
    try {
      odom_to_base_msg = tf_buffer_->lookupTransform(
        odom_frame_,
        base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(lookup_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Cannot compute %s->%s yet; missing %s->%s: %s",
        map_frame_.c_str(),
        odom_frame_.c_str(),
        odom_frame_.c_str(),
        base_frame_.c_str(),
        ex.what());
      return;
    }

    tf2::Transform map_to_base;
    tf2::Transform odom_to_base;
    tf2::fromMsg(latest_map_to_base_.transform, map_to_base);
    tf2::fromMsg(odom_to_base_msg.transform, odom_to_base);

    const tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = get_clock()->now();
    map_to_odom_msg.header.frame_id = map_frame_;
    map_to_odom_msg.child_frame_id = odom_frame_;
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom);
    tf_broadcaster_->sendTransform(map_to_odom_msg);
  }

  void publishIdentityMapToOdom()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = get_clock()->now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr optimized_pose_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped latest_map_to_base_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string optimized_pose_topic_;
  double publish_rate_;
  double lookup_timeout_sec_;
  bool identity_until_optimized_;
  bool have_optimized_pose_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapToOdomTfNode>());
  rclcpp::shutdown();
  return 0;
}
