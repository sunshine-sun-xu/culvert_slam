#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace culvert_pgo
{

using SCPointType = pcl::PointXYZI;
using SCCloudType = pcl::PointCloud<SCPointType>;

struct ScanContextMatch
{
  int target_index = -1;
  double distance = 1.0;
  double yaw_diff_rad = 0.0;
};

struct DescriptorRecord
{
  Eigen::MatrixXf descriptor;
  Eigen::ArrayXXf observed;
  Eigen::VectorXf ring_key;
  Eigen::VectorXf sector_key;
  double quality = 0.0;
};

class ScanContextManager
{
public:
  ScanContextManager() = default;

  void setGeometry(int num_rings, int num_sectors, double max_radius);
  void setMaximumRadius(double max_radius);
  void setDistanceThreshold(double distance_threshold);
  void setQualityThreshold(double min_observed_ratio);
  std::optional<int> saveDescriptor(const SCCloudType & cloud);
  std::optional<ScanContextMatch> detectLoopClosure(
    int recent_exclusion,
    int num_candidates) const;

  size_t size() const;
  const DescriptorRecord & recordAt(size_t idx) const;

private:
  DescriptorRecord makeDescriptorRecord(const SCCloudType & cloud) const;
  Eigen::VectorXf makeRingKey(
    const Eigen::MatrixXf & descriptor,
    const Eigen::ArrayXXf & observed) const;
  Eigen::VectorXf makeSectorKey(
    const Eigen::MatrixXf & descriptor,
    const Eigen::ArrayXXf & observed) const;
  int fastAlignUsingSectorKey(
    const Eigen::VectorXf & key1,
    const Eigen::VectorXf & key2) const;
  Eigen::MatrixXf circularShift(const Eigen::MatrixXf & matrix, int shift) const;
  double directDistance(
    const DescriptorRecord & lhs,
    const DescriptorRecord & rhs) const;
  std::pair<double, int> distanceBetweenScanContexts(
    const DescriptorRecord & lhs,
    const DescriptorRecord & rhs) const;

  int num_rings_ = 30;
  int num_sectors_ = 80;
  double max_radius_ = 50.0;
  double unit_sector_angle_ = 360.0 / 80.0;
  double distance_threshold_ = 0.15;
  int num_candidates_from_tree_ = 10;
  double search_ratio_ = 0.1;
  double min_observed_ratio_ = 0.03;

  std::vector<DescriptorRecord> records_;
};

}  // namespace culvert_pgo
