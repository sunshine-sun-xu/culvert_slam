#include "culvert_pgo/scan_context_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <flann/flann.hpp>

namespace culvert_pgo
{

namespace
{

float xyToTheta(const float x, const float y)
{
  float angle = std::atan2(y, x) * 180.0F / static_cast<float>(M_PI);
  if (angle < 0.0F) {
    angle += 360.0F;
  }
  return angle;
}

float meanObserved(const Eigen::VectorXf & values, const Eigen::ArrayXf & observed)
{
  float sum = 0.0F;
  int count = 0;
  for (int i = 0; i < values.size(); ++i) {
    if (observed(i) > 0.5F) {
      sum += values(i);
      ++count;
    }
  }
  return count > 0 ? sum / static_cast<float>(count) : 0.0F;
}

float maxObserved(const Eigen::VectorXf & values, const Eigen::ArrayXf & observed)
{
  float max_value = 0.0F;
  bool initialized = false;
  for (int i = 0; i < values.size(); ++i) {
    if (observed(i) > 0.5F) {
      max_value = initialized ? std::max(max_value, values(i)) : values(i);
      initialized = true;
    }
  }
  return initialized ? max_value : 0.0F;
}

float stdObserved(const Eigen::VectorXf & values, const Eigen::ArrayXf & observed, const float mean)
{
  float sum_sq = 0.0F;
  int count = 0;
  for (int i = 0; i < values.size(); ++i) {
    if (observed(i) > 0.5F) {
      const float diff = values(i) - mean;
      sum_sq += diff * diff;
      ++count;
    }
  }
  return count > 1 ? std::sqrt(sum_sq / static_cast<float>(count)) : 0.0F;
}

}  // namespace

void ScanContextManager::setGeometry(const int num_rings, const int num_sectors, const double max_radius)
{
  num_rings_ = std::max(5, num_rings);
  num_sectors_ = std::max(12, num_sectors);
  max_radius_ = std::max(1.0, max_radius);
  unit_sector_angle_ = 360.0 / static_cast<double>(num_sectors_);
}

void ScanContextManager::setMaximumRadius(const double max_radius)
{
  max_radius_ = std::max(1.0, max_radius);
}

void ScanContextManager::setDistanceThreshold(const double distance_threshold)
{
  distance_threshold_ = distance_threshold;
}

void ScanContextManager::setQualityThreshold(const double min_observed_ratio)
{
  min_observed_ratio_ = std::clamp(min_observed_ratio, 0.0, 1.0);
}

std::optional<int> ScanContextManager::saveDescriptor(const SCCloudType & cloud)
{
  DescriptorRecord record = makeDescriptorRecord(cloud);
  if (record.quality < min_observed_ratio_) {
    return std::nullopt;
  }
  records_.push_back(std::move(record));
  return static_cast<int>(records_.size()) - 1;
}

std::optional<ScanContextMatch> ScanContextManager::detectLoopClosure(
  const int recent_exclusion,
  const int num_candidates) const
{
  if (records_.size() <= static_cast<size_t>(recent_exclusion + 1)) {
    return std::nullopt;
  }

  const int source_index = static_cast<int>(records_.size()) - 1;
  const int valid_target_count = source_index - recent_exclusion;
  if (valid_target_count <= 0) {
    return std::nullopt;
  }

  const auto & source_record = records_.back();
  const int candidate_count = std::max(1, std::min(num_candidates_from_tree_, num_candidates));

  std::vector<float> dataset_storage(
    static_cast<size_t>(valid_target_count) * static_cast<size_t>(source_record.ring_key.size()));
  for (int row = 0; row < valid_target_count; ++row) {
    for (int col = 0; col < source_record.ring_key.size(); ++col) {
      dataset_storage[static_cast<size_t>(row) * static_cast<size_t>(source_record.ring_key.size()) +
        static_cast<size_t>(col)] = records_[row].ring_key(col);
    }
  }
  flann::Matrix<float> dataset(
    dataset_storage.data(),
    valid_target_count,
    source_record.ring_key.size());
  flann::Index<flann::L2<float>> kdtree(dataset, flann::KDTreeIndexParams(4));
  kdtree.buildIndex();

  std::vector<float> query_storage(static_cast<size_t>(source_record.ring_key.size()));
  for (int col = 0; col < source_record.ring_key.size(); ++col) {
    query_storage[static_cast<size_t>(col)] = source_record.ring_key(col);
  }
  flann::Matrix<float> query(query_storage.data(), 1, source_record.ring_key.size());
  std::vector<int> indices_storage(static_cast<size_t>(candidate_count));
  std::vector<float> dists_storage(static_cast<size_t>(candidate_count));
  flann::Matrix<int> indices(indices_storage.data(), 1, candidate_count);
  flann::Matrix<float> dists(dists_storage.data(), 1, candidate_count);
  kdtree.knnSearch(query, indices, dists, candidate_count, flann::SearchParams(32));

  ScanContextMatch best_match;
  best_match.distance = std::numeric_limits<double>::max();
  for (int i = 0; i < candidate_count; ++i) {
    const int target_index = indices_storage[static_cast<size_t>(i)];
    if (target_index < 0 || target_index >= valid_target_count) {
      continue;
    }
    const auto [distance, shift] = distanceBetweenScanContexts(source_record, records_[target_index]);
    if (distance < best_match.distance) {
      best_match.target_index = target_index;
      best_match.distance = distance;
      best_match.yaw_diff_rad = static_cast<double>(shift) * unit_sector_angle_ * M_PI / 180.0;
    }
  }

  if (best_match.target_index < 0 || best_match.distance > distance_threshold_) {
    return std::nullopt;
  }
  return best_match;
}

size_t ScanContextManager::size() const
{
  return records_.size();
}

const DescriptorRecord & ScanContextManager::recordAt(const size_t idx) const
{
  return records_.at(idx);
}

DescriptorRecord ScanContextManager::makeDescriptorRecord(const SCCloudType & cloud) const
{
  constexpr float no_point = -1000.0F;
  DescriptorRecord record;
  record.descriptor = no_point * Eigen::MatrixXf::Ones(num_rings_, num_sectors_);
  record.observed = Eigen::ArrayXXf::Zero(num_rings_, num_sectors_);

  const float unit_ring_gap = static_cast<float>(max_radius_ / static_cast<double>(num_rings_));

  for (const auto & point : cloud.points) {
    const float range = std::sqrt(point.x * point.x + point.y * point.y);
    if (range > static_cast<float>(max_radius_) || range < 1e-3F) {
      continue;
    }

    const float angle = xyToTheta(point.x, point.y);
    const int ring_index = std::clamp(
      static_cast<int>(std::ceil(range / unit_ring_gap)),
      1, num_rings_) - 1;
    const int sector_index = std::clamp(
      static_cast<int>(std::ceil(angle / unit_sector_angle_)),
      1, num_sectors_) - 1;

    record.observed(ring_index, sector_index) = 1.0F;
    record.descriptor(ring_index, sector_index) = std::max(
      record.descriptor(ring_index, sector_index),
      point.z + 2.0F);
  }

  int observed_bins = 0;
  for (int row = 0; row < record.descriptor.rows(); ++row) {
    for (int col = 0; col < record.descriptor.cols(); ++col) {
      if (record.observed(row, col) < 0.5F) {
        record.descriptor(row, col) = 0.0F;
      } else {
        ++observed_bins;
      }
    }
  }

  record.ring_key = makeRingKey(record.descriptor, record.observed);
  record.sector_key = makeSectorKey(record.descriptor, record.observed);
  record.quality = static_cast<double>(observed_bins) /
    static_cast<double>(num_rings_ * num_sectors_);
  return record;
}

Eigen::VectorXf ScanContextManager::makeRingKey(
  const Eigen::MatrixXf & descriptor,
  const Eigen::ArrayXXf & observed) const
{
  Eigen::VectorXf ring_key(descriptor.rows() * 3);
  for (int row = 0; row < descriptor.rows(); ++row) {
    const Eigen::VectorXf row_values = descriptor.row(row).transpose();
    const Eigen::ArrayXf row_observed = observed.row(row).transpose();
    const float mean = meanObserved(row_values, row_observed);
    const float max_value = maxObserved(row_values, row_observed);
    const float std = stdObserved(row_values, row_observed, mean);
    ring_key(row * 3 + 0) = mean;
    ring_key(row * 3 + 1) = max_value;
    ring_key(row * 3 + 2) = std;
  }
  return ring_key;
}

Eigen::VectorXf ScanContextManager::makeSectorKey(
  const Eigen::MatrixXf & descriptor,
  const Eigen::ArrayXXf & observed) const
{
  Eigen::VectorXf sector_key(descriptor.cols());
  for (int col = 0; col < descriptor.cols(); ++col) {
    sector_key(col) = meanObserved(descriptor.col(col), observed.col(col));
  }
  return sector_key;
}

int ScanContextManager::fastAlignUsingSectorKey(
  const Eigen::VectorXf & key1,
  const Eigen::VectorXf & key2) const
{
  int best_shift = 0;
  double best_norm = std::numeric_limits<double>::max();
  for (int shift = 0; shift < key1.size(); ++shift) {
    Eigen::VectorXf shifted(key2.size());
    for (int idx = 0; idx < key2.size(); ++idx) {
      shifted((idx + shift) % key2.size()) = key2(idx);
    }
    const double norm = (key1 - shifted).norm();
    if (norm < best_norm) {
      best_norm = norm;
      best_shift = shift;
    }
  }
  return best_shift;
}

Eigen::MatrixXf ScanContextManager::circularShift(const Eigen::MatrixXf & matrix, const int shift) const
{
  if (shift == 0) {
    return matrix;
  }
  Eigen::MatrixXf shifted = Eigen::MatrixXf::Zero(matrix.rows(), matrix.cols());
  for (int col = 0; col < matrix.cols(); ++col) {
    shifted.col((col + shift) % matrix.cols()) = matrix.col(col);
  }
  return shifted;
}

double ScanContextManager::directDistance(
  const DescriptorRecord & lhs,
  const DescriptorRecord & rhs) const
{
  int effective_columns = 0;
  double sum_similarity = 0.0;
  for (int col = 0; col < lhs.descriptor.cols(); ++col) {
    const Eigen::ArrayXf overlap = (lhs.observed.col(col) > 0.5F).cast<float>() *
      (rhs.observed.col(col) > 0.5F).cast<float>();
    if (overlap.sum() < 1.0F) {
      continue;
    }
    const auto lhs_col = lhs.descriptor.col(col);
    const auto rhs_col = rhs.descriptor.col(col);
    const double lhs_norm = lhs_col.norm();
    const double rhs_norm = rhs_col.norm();
    if (lhs_norm == 0.0 || rhs_norm == 0.0) {
      continue;
    }
    const double similarity = lhs_col.dot(rhs_col) / (lhs_norm * rhs_norm);
    sum_similarity += similarity;
    ++effective_columns;
  }
  if (effective_columns == 0) {
    return 1.0;
  }
  return 1.0 - sum_similarity / static_cast<double>(effective_columns);
}

std::pair<double, int> ScanContextManager::distanceBetweenScanContexts(
  const DescriptorRecord & lhs,
  const DescriptorRecord & rhs) const
{
  const int align_shift = fastAlignUsingSectorKey(lhs.sector_key, rhs.sector_key);
  const int search_radius =
    static_cast<int>(std::round(0.5 * search_ratio_ * lhs.descriptor.cols()));
  std::vector<int> search_space {align_shift};
  for (int offset = 1; offset <= search_radius; ++offset) {
    search_space.push_back((align_shift + offset + lhs.descriptor.cols()) % lhs.descriptor.cols());
    search_space.push_back((align_shift - offset + lhs.descriptor.cols()) % lhs.descriptor.cols());
  }
  std::sort(search_space.begin(), search_space.end());
  search_space.erase(std::unique(search_space.begin(), search_space.end()), search_space.end());

  int best_shift = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (const int shift : search_space) {
    DescriptorRecord shifted = rhs;
    shifted.descriptor = circularShift(rhs.descriptor, shift);
    shifted.observed = rhs.observed.matrix().array();
    shifted.observed = rhs.observed;
    const double distance = directDistance(lhs, shifted);
    if (distance < best_distance) {
      best_distance = distance;
      best_shift = shift;
    }
  }
  return {best_distance, best_shift};
}

}  // namespace culvert_pgo
