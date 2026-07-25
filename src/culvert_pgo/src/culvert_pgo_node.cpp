#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "culvert_mapping_interfaces/msg/dirty_submap_list.hpp"
#include "culvert_mapping_interfaces/msg/optimized_submap_pose.hpp"
#include "culvert_mapping_interfaces/msg/optimized_submap_pose_array.hpp"
#include "culvert_mapping_interfaces/msg/submap_grid_array.hpp"
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "culvert_pgo/scan_context_manager.hpp"

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

struct Keyframe
{
  rclcpp::Time stamp;
  Eigen::Vector3d local_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond local_orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d global_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond global_orientation = Eigen::Quaterniond::Identity();
  pcl::PointCloud<culvert_pgo::SCPointType>::Ptr body_cloud;
  pcl::PointCloud<culvert_pgo::SCPointType>::Ptr descriptor_cloud;
};

struct LoopCandidate
{
  size_t source_id = 0;
  size_t target_id = 0;
  double descriptor_score = 0.0;
  double yaw_hint_rad = 0.0;
  double degeneration_score = 1.0;
};

struct VerifiedLoop
{
  size_t source_id = 0;
  size_t target_id = 0;
  Eigen::Quaterniond relative_orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d relative_position = Eigen::Vector3d::Zero();
  double fitness = 0.0;
  double inlier_rmse = 0.0;
  int inliers = 0;
};

struct DegenerationMetrics
{
  double score = 1.0;
  int valid_points = 0;
  double eigen_ratio = 1.0;
};

struct SubmapDefinition
{
  int submap_id = -1;
  uint32_t version = 0;
  bool is_frozen = false;
  int anchor_keyframe_id = -1;
  geometry_msgs::msg::Pose initial_pose;
  std::vector<int> keyframe_indices;
};

struct PublishedSubmapPose
{
  uint32_t version = 0;
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

class CulvertPGONode : public rclcpp::Node
{
public:
  CulvertPGONode()
  : Node("culvert_pgo_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/fastlio2/lio_odom");
    body_cloud_topic_ = declare_parameter<std::string>("body_cloud_topic", "/fastlio2/body_cloud");
    lidar_cloud_topic_ = declare_parameter<std::string>("lidar_cloud_topic", "/fastlio2/body_cloud");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    keyframe_meter_gap_ = declare_parameter<double>("keyframe_meter_gap", 0.5);
    keyframe_deg_gap_ = declare_parameter<double>("keyframe_deg_gap", 10.0);
    scan_context_enable_ = declare_parameter<bool>("scan_context_enable", true);
    scan_context_recent_exclusion_ = declare_parameter<int>("scan_context_recent_exclusion", 30);
    scan_context_loop_threshold_ = declare_parameter<double>("scan_context_loop_threshold", 0.15);
    scan_context_max_radius_ = declare_parameter<double>("scan_context_max_radius", 50.0);
    scan_context_num_rings_ = declare_parameter<int>("scan_context_num_rings", 30);
    scan_context_num_sectors_ = declare_parameter<int>("scan_context_num_sectors", 80);
    scan_context_num_candidates_ = declare_parameter<int>("scan_context_num_candidates", 10);
    scan_context_min_observed_ratio_ = declare_parameter<double>("scan_context_min_observed_ratio", 0.03);
    scan_context_max_yaw_diff_deg_ = declare_parameter<double>("scan_context_max_yaw_diff_deg", 35.0);
    dynamic_keyframe_enable_ = declare_parameter<bool>("dynamic_keyframe_enable", true);
    keyframe_meter_gap_min_ = declare_parameter<double>("keyframe_meter_gap_min", 0.5);
    keyframe_meter_gap_max_ = declare_parameter<double>("keyframe_meter_gap_max", 2.0);
    keyframe_deg_gap_min_ = declare_parameter<double>("keyframe_deg_gap_min", 10.0);
    keyframe_deg_gap_max_ = declare_parameter<double>("keyframe_deg_gap_max", 30.0);
    degeneration_score_enable_ = declare_parameter<bool>("degeneration_score_enable", true);
    degeneration_score_min_points_ = declare_parameter<int>("degeneration_score_min_points", 200);
    degeneration_eigen_ratio_threshold_ = declare_parameter<double>("degeneration_eigen_ratio_threshold", 0.08);
    degeneration_score_loop_gate_ = declare_parameter<double>("degeneration_score_loop_gate", 0.55);
    degeneration_score_optimize_gate_ = declare_parameter<double>("degeneration_score_optimize_gate", 0.40);
    loop_verification_method_ = declare_parameter<std::string>("loop_verification_method", "gicp");
    loop_fitness_threshold_ = declare_parameter<double>("loop_fitness_threshold", 0.35);
    loop_inlier_rmse_threshold_ = declare_parameter<double>("loop_inlier_rmse_threshold", 0.40);
    loop_min_inliers_ = declare_parameter<int>("loop_min_inliers", 50);
    loop_submap_half_range_ = declare_parameter<int>("loop_submap_half_range", 5);
    loop_submap_resolution_ = declare_parameter<double>("loop_submap_resolution", 0.15);
    optimized_map_publish_resolution_ = declare_parameter<double>("optimized_map_publish_resolution", 0.20);
    registration_max_iterations_ = declare_parameter<int>("registration_max_iterations", 50);
    registration_max_correspondence_distance_ =
      declare_parameter<double>("registration_max_correspondence_distance", 5.0);
    pose_graph_enable_ = declare_parameter<bool>("pose_graph_enable", true);
    optimize_every_n_keyframes_ = declare_parameter<int>("optimize_every_n_keyframes", 10);
    loop_search_radius_ = declare_parameter<double>("loop_search_radius", 10.0);
    submap_grid_topic_ =
      declare_parameter<std::string>("submap_grid_topic", "/submap_manager/submap_grids");
    optimized_submap_pose_topic_ =
      declare_parameter<std::string>("optimized_submap_pose_topic", "~/optimized_submap_poses");
    dirty_submap_topic_ =
      declare_parameter<std::string>("dirty_submap_topic", "~/dirty_submap_list");

    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/optimized_path", 10);
    optimized_submap_pose_pub_ =
      create_publisher<culvert_mapping_interfaces::msg::OptimizedSubmapPoseArray>(
        optimized_submap_pose_topic_, 10);
    dirty_submap_pub_ =
      create_publisher<culvert_mapping_interfaces::msg::DirtySubmapList>(
        dirty_submap_topic_, 10);
    optimized_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/optimized_cloud", 1);
    loop_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/loop_markers", 10);
    candidate_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/candidate_markers", 10);
    degeneration_score_pub_ = create_publisher<std_msgs::msg::Float32>("~/degeneration_score", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    scan_context_manager_.setGeometry(
      scan_context_num_rings_,
      scan_context_num_sectors_,
      scan_context_max_radius_);
    scan_context_manager_.setDistanceThreshold(scan_context_loop_threshold_);
    scan_context_manager_.setQualityThreshold(scan_context_min_observed_ratio_);

    initializeOptimizer();
    configureRegistrations();

    cloud_sub_.subscribe(this, body_cloud_topic_);
    odom_sub_.subscribe(this, odom_topic_);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(20), cloud_sub_, odom_sub_);
    sync_->registerCallback(
      std::bind(&CulvertPGONode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    submap_grid_sub_ =
      create_subscription<culvert_mapping_interfaces::msg::SubmapGridArray>(
        submap_grid_topic_,
        rclcpp::QoS(10),
        std::bind(&CulvertPGONode::submapGridCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CulvertPGONode::processBackend, this));

    optimized_path_.header.frame_id = map_frame_;

    RCLCPP_INFO(
      get_logger(),
      "culvert_pgo_node started: odom=%s body_cloud=%s, backend flow = keyframe -> scancontext -> %s -> gtsam -> map_to_odom",
      odom_topic_.c_str(),
      body_cloud_topic_.c_str(),
      loop_verification_method_.c_str());
  }

private:
  void initializeOptimizer()
  {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(params);
    odom_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    prior_noise_ = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
  }

  void configureRegistrations()
  {
    icp_.setMaximumIterations(registration_max_iterations_);
    icp_.setMaxCorrespondenceDistance(registration_max_correspondence_distance_);
    icp_.setTransformationEpsilon(1e-6);
    icp_.setEuclideanFitnessEpsilon(1e-6);
    icp_.setRANSACIterations(0);

    gicp_.setMaximumIterations(registration_max_iterations_);
    gicp_.setMaxCorrespondenceDistance(registration_max_correspondence_distance_);
    gicp_.setTransformationEpsilon(1e-6);
    gicp_.setEuclideanFitnessEpsilon(1e-6);
    gicp_.setRANSACIterations(0);
  }

  void submapGridCallback(
    const culvert_mapping_interfaces::msg::SubmapGridArray::SharedPtr msg)
  {
    for (const auto & submap : msg->submaps) {
      SubmapDefinition definition;
      definition.submap_id = submap.submap_id;
      definition.version = submap.version;
      definition.is_frozen = submap.is_frozen;
      definition.anchor_keyframe_id = submap.anchor_keyframe_id;
      definition.initial_pose = submap.initial_pose;
      definition.keyframe_indices.assign(
        submap.keyframe_indices.begin(), submap.keyframe_indices.end());
      submap_definitions_[definition.submap_id] = definition;
    }
  }

  void syncCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
  {
    pending_keyframe_.stamp = odom_msg->header.stamp;
    pending_keyframe_.local_position = Eigen::Vector3d(
      odom_msg->pose.pose.position.x,
      odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z);
    pending_keyframe_.local_orientation = Eigen::Quaterniond(
      odom_msg->pose.pose.orientation.w,
      odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z);
    pending_keyframe_.body_cloud = makeCloud(*cloud_msg);
    pending_keyframe_.descriptor_cloud = pending_keyframe_.body_cloud;
    latest_degeneration_metrics_ = computeDegenerationMetrics(*pending_keyframe_.body_cloud);
    latest_local_position_ = pending_keyframe_.local_position;
    latest_local_orientation_ = pending_keyframe_.local_orientation;
    latest_stamp_ = pending_keyframe_.stamp;
    have_pending_ = true;
  }

  void processBackend()
  {
    if (!have_pending_) {
      publishMapToOdom();
      publishDegenerationScore();
      return;
    }

    if (shouldCreateKeyframe(pending_keyframe_)) {
      addKeyframe(pending_keyframe_);
      if (scan_context_enable_ && pending_keyframe_.descriptor_cloud) {
        const auto descriptor_index =
          scan_context_manager_.saveDescriptor(*pending_keyframe_.descriptor_cloud);
        if (!descriptor_index.has_value()) {
          RCLCPP_INFO(
            get_logger(),
            "Skip ScanContext registration for low-information keyframe %zu",
            keyframes_.size() - 1);
        }
      }
      if (scan_context_enable_) {
        runScanContextCandidateSearch();
      }
      verifyLoopCandidates();
      if (pose_graph_enable_ && optimize_every_n_keyframes_ > 0 &&
        static_cast<int>(keyframes_.size()) % optimize_every_n_keyframes_ == 0 &&
        latest_degeneration_metrics_.score <= degeneration_score_optimize_gate_)
      {
        optimizePoseGraph();
      }
      rebuildOptimizedPath();
      publishOptimizedCloud();
      publishLoopMarkers();
      publishCandidateMarkers();
    }

    publishMapToOdom();
    path_pub_->publish(optimized_path_);
    publishOptimizedSubmapPoses();
    publishDegenerationScore();
    have_pending_ = false;
  }

  bool shouldCreateKeyframe(const Keyframe & keyframe) const
  {
    if (keyframes_.empty()) {
      return true;
    }
    const auto & last = keyframes_.back();
    const double distance = (keyframe.local_position - last.local_position).norm();
    const double angular =
      keyframe.local_orientation.angularDistance(last.local_orientation) * 180.0 / M_PI;
    if (!dynamic_keyframe_enable_) {
      return distance >= keyframe_meter_gap_ || angular >= keyframe_deg_gap_;
    }
    const double degeneration = std::clamp(latest_degeneration_metrics_.score, 0.0, 1.0);
    const double dynamic_meter_gap =
      keyframe_meter_gap_min_ + (keyframe_meter_gap_max_ - keyframe_meter_gap_min_) * degeneration;
    const double dynamic_deg_gap =
      keyframe_deg_gap_min_ + (keyframe_deg_gap_max_ - keyframe_deg_gap_min_) * degeneration;
    return distance >= dynamic_meter_gap || angular >= dynamic_deg_gap;
  }

  DegenerationMetrics computeDegenerationMetrics(
    const pcl::PointCloud<culvert_pgo::SCPointType> & cloud) const
  {
    DegenerationMetrics metrics;
    metrics.valid_points = static_cast<int>(cloud.size());
    if (!degeneration_score_enable_ || cloud.empty()) {
      return metrics;
    }

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto & point : cloud.points) {
      mean.x() += point.x;
      mean.y() += point.y;
      mean.z() += point.z;
    }
    mean /= static_cast<double>(cloud.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto & point : cloud.points) {
      const Eigen::Vector3d centered(point.x - mean.x(), point.y - mean.y(), point.z - mean.z());
      covariance += centered * centered.transpose();
    }
    covariance /= std::max<size_t>(1, cloud.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    const auto eigenvalues = solver.eigenvalues();
    const double max_eig = std::max(1e-6, eigenvalues(2));
    const double min_eig = std::max(0.0, eigenvalues(0));
    metrics.eigen_ratio = min_eig / max_eig;

    const double point_factor = std::clamp(
      static_cast<double>(cloud.size()) / static_cast<double>(std::max(1, degeneration_score_min_points_)),
      0.0, 1.0);
    const double structure_factor = std::clamp(
      metrics.eigen_ratio / std::max(1e-6, degeneration_eigen_ratio_threshold_),
      0.0, 1.0);
    metrics.score = 1.0 - (0.5 * point_factor + 0.5 * structure_factor);
    metrics.score = std::clamp(metrics.score, 0.0, 1.0);
    return metrics;
  }

  void addKeyframe(Keyframe keyframe)
  {
    const size_t idx = keyframes_.size();
    if (idx == 0) {
      keyframe.global_position = keyframe.local_position;
      keyframe.global_orientation = keyframe.local_orientation;
      initial_values_.insert(symbol(idx), toPose3(keyframe.global_orientation, keyframe.global_position));
      graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        symbol(idx), toPose3(keyframe.global_orientation, keyframe.global_position), prior_noise_));
    } else {
      keyframe.global_orientation = map_to_odom_rotation_ * keyframe.local_orientation;
      keyframe.global_position = map_to_odom_rotation_ * keyframe.local_position + map_to_odom_translation_;
      initial_values_.insert(symbol(idx), toPose3(keyframe.global_orientation, keyframe.global_position));

      const auto & last = keyframes_.back();
      const Eigen::Quaterniond relative_orientation = last.local_orientation.inverse() * keyframe.local_orientation;
      const Eigen::Vector3d relative_position =
        last.local_orientation.inverse() * (keyframe.local_position - last.local_position);
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        symbol(idx - 1), symbol(idx),
        toPose3(relative_orientation, relative_position),
        odom_noise_));
    }
    keyframes_.push_back(std::move(keyframe));
  }

  pcl::PointCloud<culvert_pgo::SCPointType>::Ptr makeCloud(
    const sensor_msgs::msg::PointCloud2 & cloud_msg) const
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(cloud_msg, pcl_cloud);
    auto cloud = std::make_shared<pcl::PointCloud<culvert_pgo::SCPointType>>();
    cloud->points.reserve(pcl_cloud.points.size());
    for (const auto & point : pcl_cloud.points) {
      culvert_pgo::SCPointType p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      p.intensity = point.intensity;
      cloud->points.push_back(p);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
  }

  void runScanContextCandidateSearch()
  {
    if (keyframes_.size() <= static_cast<size_t>(scan_context_recent_exclusion_)) {
      return;
    }
    const size_t source_id = keyframes_.size() - 1;
    const auto match = scan_context_manager_.detectLoopClosure(
      scan_context_recent_exclusion_, scan_context_num_candidates_);
    if (!match.has_value()) {
      return;
    }

    const size_t target_id = static_cast<size_t>(match->target_index);
    if (hasCandidateOrLoop(source_id, target_id)) {
      return;
    }
    if (latest_degeneration_metrics_.score > degeneration_score_loop_gate_) {
      return;
    }

    const double source_yaw = yawOfQuaternion(keyframes_[source_id].local_orientation);
    const double target_yaw = yawOfQuaternion(keyframes_[target_id].local_orientation);
    const double odom_yaw_delta = normalizeAngle(source_yaw - target_yaw);
    const double sc_yaw_delta = normalizeAngle(match->yaw_diff_rad);
    const double yaw_error = std::abs(normalizeAngle(sc_yaw_delta - odom_yaw_delta));
    if (yaw_error * 180.0 / M_PI > scan_context_max_yaw_diff_deg_) {
      RCLCPP_INFO(
        get_logger(),
        "ScanContext candidate rejected by yaw gate: source=%zu target=%zu sc=%.2fdeg odom=%.2fdeg err=%.2fdeg",
        source_id,
        target_id,
        sc_yaw_delta * 180.0 / M_PI,
        odom_yaw_delta * 180.0 / M_PI,
        yaw_error * 180.0 / M_PI);
      return;
    }

    const double spatial_distance =
      (keyframes_[source_id].global_position - keyframes_[target_id].global_position).norm();
    if (spatial_distance > loop_search_radius_) {
      return;
    }

    LoopCandidate candidate;
    candidate.source_id = source_id;
    candidate.target_id = target_id;
    candidate.descriptor_score = match->distance;
    candidate.yaw_hint_rad = sc_yaw_delta;
    candidate.degeneration_score = latest_degeneration_metrics_.score;
    loop_candidates_.push_back(candidate);

    RCLCPP_INFO(
      get_logger(),
      "ScanContext candidate queued: source=%zu target=%zu desc_score=%.3f spatial_dist=%.2f yaw_hint=%.2f deg",
      source_id, target_id, candidate.descriptor_score, spatial_distance,
      candidate.yaw_hint_rad * 180.0 / M_PI);
  }

  void verifyLoopCandidates()
  {
    if (loop_candidates_.empty()) {
      return;
    }
    const LoopCandidate candidate = loop_candidates_.front();
    loop_candidates_.pop_front();

    auto source_cloud = keyframes_[candidate.source_id].body_cloud;
    auto target_submap = buildSubmap(candidate.target_id, loop_submap_half_range_, loop_submap_resolution_);
    if (!source_cloud || source_cloud->empty() || !target_submap || target_submap->empty()) {
      return;
    }

    pcl::PointCloud<culvert_pgo::SCPointType>::Ptr aligned(new pcl::PointCloud<culvert_pgo::SCPointType>());
    const Eigen::Matrix4f initial_guess = makeInitialGuess(candidate);

    double fitness = std::numeric_limits<double>::infinity();
    bool converged = false;
    if (loop_verification_method_ == "icp") {
      icp_.setInputSource(source_cloud);
      icp_.setInputTarget(target_submap);
      icp_.align(*aligned, initial_guess);
      converged = icp_.hasConverged();
      fitness = icp_.getFitnessScore();
    } else {
      gicp_.setInputSource(source_cloud);
      gicp_.setInputTarget(target_submap);
      gicp_.align(*aligned, initial_guess);
      converged = gicp_.hasConverged();
      fitness = gicp_.getFitnessScore();
    }

    if (!converged) {
      RCLCPP_INFO(
        get_logger(),
        "Loop candidate rejected (%s not converged): source=%zu target=%zu",
        loop_verification_method_.c_str(), candidate.source_id, candidate.target_id);
      return;
    }

    const Eigen::Matrix4f final_transform =
      loop_verification_method_ == "icp" ? icp_.getFinalTransformation() : gicp_.getFinalTransformation();
    const Eigen::Matrix3d refined_rotation = final_transform.block<3, 3>(0, 0).cast<double>();
    const Eigen::Vector3d refined_translation = final_transform.block<3, 1>(0, 3).cast<double>();

    const auto & target_keyframe = keyframes_[candidate.target_id];
    const Eigen::Quaterniond source_global_orientation = Eigen::Quaterniond(refined_rotation);
    const Eigen::Vector3d source_global_position = refined_translation;
    const Eigen::Quaterniond relative_orientation =
      target_keyframe.global_orientation.inverse() * source_global_orientation;
    const Eigen::Vector3d relative_position =
      target_keyframe.global_orientation.inverse() *
      (source_global_position - target_keyframe.global_position);

    VerifiedLoop loop;
    loop.source_id = candidate.source_id;
    loop.target_id = candidate.target_id;
    loop.relative_orientation = relative_orientation;
    loop.relative_position = relative_position;
    loop.fitness = fitness;
    loop.inlier_rmse = std::sqrt(std::max(0.0, fitness));
    loop.inliers = static_cast<int>(aligned->size());

    const bool accepted =
      loop.fitness <= loop_fitness_threshold_ &&
      loop.inlier_rmse <= loop_inlier_rmse_threshold_ &&
      loop.inliers >= loop_min_inliers_;

    if (!accepted) {
      RCLCPP_INFO(
        get_logger(),
        "Loop candidate rejected by %s verifier: source=%zu target=%zu fitness=%.3f rmse=%.3f inliers=%d",
        loop_verification_method_.c_str(),
        loop.source_id,
        loop.target_id,
        loop.fitness,
        loop.inlier_rmse,
        loop.inliers);
      return;
    }

    const auto loop_key = encodeLoopKey(loop.source_id, loop.target_id);
    if (inserted_loop_keys_.insert(loop_key).second) {
      verified_loops_.push_back(loop);
      pending_loop_factors_.push_back(loop);
    }

    RCLCPP_INFO(
      get_logger(),
      "Loop candidate accepted by %s verifier: source=%zu target=%zu fitness=%.3f rmse=%.3f inliers=%d",
      loop_verification_method_.c_str(),
      loop.source_id,
      loop.target_id,
      loop.fitness,
      loop.inlier_rmse,
      loop.inliers);
  }

  pcl::PointCloud<culvert_pgo::SCPointType>::Ptr buildSubmap(
    const size_t center_idx, const int half_range, const double resolution) const
  {
    const int min_idx = std::max(0, static_cast<int>(center_idx) - half_range);
    const int max_idx = std::min(static_cast<int>(keyframes_.size()) - 1, static_cast<int>(center_idx) + half_range);
    auto merged = std::make_shared<pcl::PointCloud<culvert_pgo::SCPointType>>();

    for (int idx = min_idx; idx <= max_idx; ++idx) {
      const auto & keyframe = keyframes_[idx];
      pcl::PointCloud<culvert_pgo::SCPointType> transformed;
      const Eigen::Affine3f transform = toAffine(keyframe.global_orientation, keyframe.global_position);
      pcl::transformPointCloud(*keyframe.body_cloud, transformed, transform);
      *merged += transformed;
    }

    if (resolution > 0.0 && !merged->empty()) {
      pcl::VoxelGrid<culvert_pgo::SCPointType> voxel;
      voxel.setLeafSize(resolution, resolution, resolution);
      voxel.setInputCloud(merged);
      auto filtered = std::make_shared<pcl::PointCloud<culvert_pgo::SCPointType>>();
      voxel.filter(*filtered);
      return filtered;
    }
    return merged;
  }

  Eigen::Matrix4f makeInitialGuess(const LoopCandidate & candidate) const
  {
    const auto & source = keyframes_[candidate.source_id];
    Eigen::Quaterniond yaw_hint(Eigen::AngleAxisd(candidate.yaw_hint_rad, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond guess_orientation = source.global_orientation * yaw_hint;
    return toAffine(guess_orientation, source.global_position).matrix();
  }

  void optimizePoseGraph()
  {
    if (!pose_graph_enable_) {
      return;
    }
    if (pending_loop_factors_.empty() && initial_values_.empty()) {
      return;
    }

    for (const auto & loop : pending_loop_factors_) {
      const double loop_sigma = std::max(1e-4, loop.fitness);
      const auto loop_noise = gtsam::noiseModel::Diagonal::Variances(
        gtsam::Vector6::Ones() * loop_sigma);
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        symbol(loop.target_id), symbol(loop.source_id),
        toPose3(loop.relative_orientation, loop.relative_position),
        loop_noise));
    }
    pending_loop_factors_.clear();

    if (graph_.empty() && initial_values_.empty()) {
      return;
    }

    isam2_->update(graph_, initial_values_);
    isam2_->update();
    graph_.resize(0);
    initial_values_.clear();

    const auto estimate = isam2_->calculateBestEstimate();
    for (size_t idx = 0; idx < keyframes_.size(); ++idx) {
      const auto pose = estimate.at<gtsam::Pose3>(symbol(idx));
      const auto rotation = pose.rotation().matrix();
      const auto translation = pose.translation();
      keyframes_[idx].global_orientation = Eigen::Quaterniond(rotation);
      keyframes_[idx].global_position = Eigen::Vector3d(
        translation.x(), translation.y(), translation.z());
    }
    updateMapToOdomOffset();
  }

  void publishOptimizedCloud()
  {
    if (!optimized_cloud_pub_ || optimized_cloud_pub_->get_subscription_count() == 0 || keyframes_.empty()) {
      return;
    }

    auto merged = std::make_shared<pcl::PointCloud<culvert_pgo::SCPointType>>();
    for (const auto & keyframe : keyframes_) {
      pcl::PointCloud<culvert_pgo::SCPointType> transformed;
      const Eigen::Affine3f transform = toAffine(keyframe.global_orientation, keyframe.global_position);
      pcl::transformPointCloud(*keyframe.body_cloud, transformed, transform);
      *merged += transformed;
    }

    if (optimized_map_publish_resolution_ > 0.0 && !merged->empty()) {
      pcl::VoxelGrid<culvert_pgo::SCPointType> voxel;
      voxel.setLeafSize(
        optimized_map_publish_resolution_,
        optimized_map_publish_resolution_,
        optimized_map_publish_resolution_);
      voxel.setInputCloud(merged);
      pcl::PointCloud<culvert_pgo::SCPointType> filtered;
      voxel.filter(filtered);
      *merged = filtered;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*merged, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = latest_stamp_;
    optimized_cloud_pub_->publish(msg);
  }

  void updateMapToOdomOffset()
  {
    if (keyframes_.empty()) {
      map_to_odom_rotation_ = Eigen::Quaterniond::Identity();
      map_to_odom_translation_ = Eigen::Vector3d::Zero();
      return;
    }
    const auto & last = keyframes_.back();
    map_to_odom_rotation_ = last.global_orientation * last.local_orientation.inverse();
    map_to_odom_translation_ =
      last.global_position - map_to_odom_rotation_ * last.local_position;
  }

  void rebuildOptimizedPath()
  {
    optimized_path_.poses.clear();
    optimized_path_.header.frame_id = map_frame_;
    optimized_path_.header.stamp = latest_stamp_;
    for (const auto & keyframe : keyframes_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.header.stamp = keyframe.stamp;
      pose.pose.position.x = keyframe.global_position.x();
      pose.pose.position.y = keyframe.global_position.y();
      pose.pose.position.z = keyframe.global_position.z();
      pose.pose.orientation.x = keyframe.global_orientation.x();
      pose.pose.orientation.y = keyframe.global_orientation.y();
      pose.pose.orientation.z = keyframe.global_orientation.z();
      pose.pose.orientation.w = keyframe.global_orientation.w();
      optimized_path_.poses.push_back(pose);
    }
  }

  void publishOptimizedSubmapPoses()
  {
    culvert_mapping_interfaces::msg::OptimizedSubmapPoseArray array;
    array.header.stamp.sec = static_cast<int32_t>(latest_stamp_.seconds());
    array.header.stamp.nanosec = static_cast<uint32_t>(
      latest_stamp_.nanoseconds() - static_cast<int64_t>(array.header.stamp.sec) * 1000000000LL);
    array.header.frame_id = map_frame_;
    array.optimization_epoch = ++optimization_epoch_;

    std::vector<int32_t> dirty_ids;
    std::vector<uint32_t> dirty_versions;

    for (const auto & [submap_id, submap] : submap_definitions_) {
      if (submap.anchor_keyframe_id < 0 ||
        submap.anchor_keyframe_id >= static_cast<int>(keyframes_.size()))
      {
        continue;
      }

      const auto & anchor = keyframes_[static_cast<size_t>(submap.anchor_keyframe_id)];
      const Eigen::Quaterniond optimized_orientation = anchor.global_orientation;
      const Eigen::Vector3d optimized_position = anchor.global_position;
      const Eigen::Quaterniond initial_orientation(
        submap.initial_pose.orientation.w,
        submap.initial_pose.orientation.x,
        submap.initial_pose.orientation.y,
        submap.initial_pose.orientation.z);
      const Eigen::Vector3d initial_position(
        submap.initial_pose.position.x,
        submap.initial_pose.position.y,
        submap.initial_pose.position.z);

      const auto [dirty, previous] =
        updatePublishedSubmapPose(submap_id, submap.version, optimized_orientation, optimized_position);

      culvert_mapping_interfaces::msg::OptimizedSubmapPose pose_msg;
      pose_msg.header = array.header;
      pose_msg.submap_id = submap_id;
      pose_msg.version = submap.version;
      pose_msg.dirty = dirty;
      pose_msg.is_frozen = submap.is_frozen;
      pose_msg.anchor_keyframe_id = submap.anchor_keyframe_id;
      pose_msg.initial_pose = submap.initial_pose;
      pose_msg.optimized_pose = makePose(optimized_position, optimized_orientation);

      const Eigen::Quaterniond delta_orientation =
        optimized_orientation * initial_orientation.inverse();
      const Eigen::Vector3d delta_position =
        optimized_position - initial_position;
      pose_msg.delta_pose = makePose(delta_position, delta_orientation);

      array.submaps.push_back(pose_msg);
      if (dirty) {
        dirty_ids.push_back(submap_id);
        dirty_versions.push_back(submap.version);
      }
      (void)previous;
    }

    if (!array.submaps.empty()) {
      optimized_submap_pose_pub_->publish(array);
    }

    if (!dirty_ids.empty()) {
      culvert_mapping_interfaces::msg::DirtySubmapList dirty_msg;
      dirty_msg.header = array.header;
      dirty_msg.optimization_epoch = array.optimization_epoch;
      dirty_msg.submap_ids = std::move(dirty_ids);
      dirty_msg.versions = std::move(dirty_versions);
      dirty_submap_pub_->publish(dirty_msg);
    }
  }

  std::pair<bool, PublishedSubmapPose> updatePublishedSubmapPose(
    const int submap_id,
    const uint32_t version,
    const Eigen::Quaterniond & orientation,
    const Eigen::Vector3d & position)
  {
    const auto it = published_submap_poses_.find(submap_id);
    const PublishedSubmapPose next{version, orientation, position};
    if (it == published_submap_poses_.end()) {
      published_submap_poses_[submap_id] = next;
      return {true, next};
    }

    const PublishedSubmapPose & prev = it->second;
    const bool version_changed = prev.version != version;
    const bool pose_changed =
      (prev.position - position).norm() > 1e-4 ||
      prev.orientation.angularDistance(orientation) > 1e-4;
    published_submap_poses_[submap_id] = next;
    return {version_changed || pose_changed, next};
  }

  static geometry_msgs::msg::Pose makePose(
    const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    return pose;
  }

  bool hasCandidateOrLoop(const size_t source_id, const size_t target_id) const
  {
    for (const auto & candidate : loop_candidates_) {
      if (candidate.source_id == source_id && candidate.target_id == target_id) {
        return true;
      }
    }
    const auto key = encodeLoopKey(source_id, target_id);
    return inserted_loop_keys_.find(key) != inserted_loop_keys_.end();
  }

  std::uint64_t encodeLoopKey(size_t source_id, size_t target_id) const
  {
    const auto min_id = std::min(source_id, target_id);
    const auto max_id = std::max(source_id, target_id);
    return (static_cast<std::uint64_t>(min_id) << 32) | static_cast<std::uint64_t>(max_id);
  }

  void publishLoopMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = now();
    marker.ns = "culvert_pgo_keyframes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.20;
    marker.scale.y = 0.20;
    marker.scale.z = 0.20;
    marker.color.r = 0.2;
    marker.color.g = 0.9;
    marker.color.b = 0.2;
    marker.color.a = 1.0;

    for (const auto & keyframe : keyframes_) {
      geometry_msgs::msg::Point point;
      point.x = keyframe.global_position.x();
      point.y = keyframe.global_position.y();
      point.z = keyframe.global_position.z();
      marker.points.push_back(point);
    }
    marker_array.markers.push_back(marker);
    loop_markers_pub_->publish(marker_array);
  }

  void publishCandidateMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker queued_marker;
    queued_marker.header.frame_id = map_frame_;
    queued_marker.header.stamp = now();
    queued_marker.ns = "culvert_pgo_candidates";
    queued_marker.id = 0;
    queued_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    queued_marker.action = visualization_msgs::msg::Marker::ADD;
    queued_marker.pose.orientation.w = 1.0;
    queued_marker.scale.x = 0.06;
    queued_marker.color.r = 1.0;
    queued_marker.color.g = 0.8;
    queued_marker.color.b = 0.1;
    queued_marker.color.a = 1.0;

    for (const auto & candidate : loop_candidates_) {
      appendLine(candidate.source_id, candidate.target_id, queued_marker, false);
    }

    visualization_msgs::msg::Marker verified_marker = queued_marker;
    verified_marker.ns = "culvert_pgo_verified_loops";
    verified_marker.id = 1;
    verified_marker.color.r = 0.1;
    verified_marker.color.g = 1.0;
    verified_marker.color.b = 0.2;
    verified_marker.points.clear();
    for (const auto & loop : verified_loops_) {
      appendLine(loop.source_id, loop.target_id, verified_marker, true);
    }

    marker_array.markers.push_back(queued_marker);
    marker_array.markers.push_back(verified_marker);
    candidate_markers_pub_->publish(marker_array);
  }

  void appendLine(
    const size_t source_id,
    const size_t target_id,
    visualization_msgs::msg::Marker & marker,
    const bool use_global) const
  {
    geometry_msgs::msg::Point p1;
    geometry_msgs::msg::Point p2;
    const auto & source = keyframes_[source_id];
    const auto & target = keyframes_[target_id];
    const auto & source_pos = use_global ? source.global_position : source.local_position;
    const auto & target_pos = use_global ? target.global_position : target.local_position;
    p1.x = source_pos.x();
    p1.y = source_pos.y();
    p1.z = source_pos.z();
    p2.x = target_pos.x();
    p2.y = target_pos.y();
    p2.z = target_pos.z();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  void publishMapToOdom()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = map_to_odom_translation_.x();
    transform.transform.translation.y = map_to_odom_translation_.y();
    transform.transform.translation.z = map_to_odom_translation_.z();
    transform.transform.rotation.x = map_to_odom_rotation_.x();
    transform.transform.rotation.y = map_to_odom_rotation_.y();
    transform.transform.rotation.z = map_to_odom_rotation_.z();
    transform.transform.rotation.w = map_to_odom_rotation_.w();
    tf_broadcaster_->sendTransform(transform);
  }

  void publishDegenerationScore()
  {
    if (!degeneration_score_pub_) {
      return;
    }
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(latest_degeneration_metrics_.score);
    degeneration_score_pub_->publish(msg);
  }

  static gtsam::Symbol symbol(const size_t idx)
  {
    return gtsam::Symbol('x', idx);
  }

  static gtsam::Pose3 toPose3(
    const Eigen::Quaterniond & orientation,
    const Eigen::Vector3d & position)
  {
    return gtsam::Pose3(
      gtsam::Rot3(orientation.toRotationMatrix()),
      gtsam::Point3(position.x(), position.y(), position.z()));
  }

  static Eigen::Affine3f toAffine(
    const Eigen::Quaterniond & orientation,
    const Eigen::Vector3d & position)
  {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = orientation.toRotationMatrix().cast<float>();
    transform.translation() = position.cast<float>();
    return transform;
  }

  static double yawOfQuaternion(const Eigen::Quaterniond & quaternion)
  {
    return std::atan2(
      2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
      1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  std::string odom_topic_;
  std::string body_cloud_topic_;
  std::string lidar_cloud_topic_;
  std::string submap_grid_topic_;
  std::string optimized_submap_pose_topic_;
  std::string dirty_submap_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double keyframe_meter_gap_;
  double keyframe_deg_gap_;
  bool scan_context_enable_;
  int scan_context_recent_exclusion_;
  double scan_context_loop_threshold_;
  double scan_context_max_radius_;
  int scan_context_num_rings_;
  int scan_context_num_sectors_;
  int scan_context_num_candidates_;
  double scan_context_min_observed_ratio_;
  double scan_context_max_yaw_diff_deg_;
  bool dynamic_keyframe_enable_;
  double keyframe_meter_gap_min_;
  double keyframe_meter_gap_max_;
  double keyframe_deg_gap_min_;
  double keyframe_deg_gap_max_;
  bool degeneration_score_enable_;
  int degeneration_score_min_points_;
  double degeneration_eigen_ratio_threshold_;
  double degeneration_score_loop_gate_;
  double degeneration_score_optimize_gate_;
  std::string loop_verification_method_;
  double loop_fitness_threshold_;
  double loop_inlier_rmse_threshold_;
  int loop_min_inliers_;
  int loop_submap_half_range_;
  double loop_submap_resolution_;
  double optimized_map_publish_resolution_;
  int registration_max_iterations_;
  double registration_max_correspondence_distance_;
  bool pose_graph_enable_;
  int optimize_every_n_keyframes_;
  double loop_search_radius_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<culvert_mapping_interfaces::msg::OptimizedSubmapPoseArray>::SharedPtr
    optimized_submap_pose_pub_;
  rclcpp::Publisher<culvert_mapping_interfaces::msg::DirtySubmapList>::SharedPtr
    dirty_submap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr optimized_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr degeneration_score_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<culvert_mapping_interfaces::msg::SubmapGridArray>::SharedPtr
    submap_grid_sub_;

  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_values_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;

  pcl::IterativeClosestPoint<culvert_pgo::SCPointType, culvert_pgo::SCPointType> icp_;
  pcl::GeneralizedIterativeClosestPoint<culvert_pgo::SCPointType, culvert_pgo::SCPointType> gicp_;

  nav_msgs::msg::Path optimized_path_;
  Keyframe pending_keyframe_;
  bool have_pending_ = false;
  std::vector<Keyframe> keyframes_;
  std::deque<LoopCandidate> loop_candidates_;
  std::vector<VerifiedLoop> verified_loops_;
  std::deque<VerifiedLoop> pending_loop_factors_;
  std::unordered_set<std::uint64_t> inserted_loop_keys_;
  std::unordered_map<int, SubmapDefinition> submap_definitions_;
  std::unordered_map<int, PublishedSubmapPose> published_submap_poses_;
  culvert_pgo::ScanContextManager scan_context_manager_;
  DegenerationMetrics latest_degeneration_metrics_;

  Eigen::Quaterniond map_to_odom_rotation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d map_to_odom_translation_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond latest_local_orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d latest_local_position_ = Eigen::Vector3d::Zero();
  rclcpp::Time latest_stamp_ {0, 0, RCL_ROS_TIME};
  uint32_t optimization_epoch_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CulvertPGONode>());
  rclcpp::shutdown();
  return 0;
}
