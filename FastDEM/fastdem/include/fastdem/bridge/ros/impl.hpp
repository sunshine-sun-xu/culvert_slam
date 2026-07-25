// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_BRIDGE_ROS_IMPL_HPP
#define FASTDEM_BRIDGE_ROS_IMPL_HPP

#include <fastdem/elevation_map.hpp>


#include <cstring>
#include <string>
#include <vector>

namespace fastdem::detail {

// ── toPointCloud2Impl ────────────────────────────────────────────────────────
//
// Converts ElevationMap to PointCloud2 with ALL non-internal layers as fields.
//   elevation → z coordinate
//   color     → rgb (PCL packed float convention)
//   others    → named FLOAT32 fields
//
// Accepts optional submap region (sub_start, sub_size) to serialize only a
// portion of the map without intermediate GridMap allocation.  When sub_size
// equals the full buffer size, the function degenerates to the full-map path.
//
template <typename PointCloud2T, typename PointFieldT, typename TimeT>
PointCloud2T toPointCloud2Impl(const ElevationMap& map, const TimeT& stamp,
                               const char* elevation_layer,
                               const nanogrid::Index& sub_start,
                               const nanogrid::Size& sub_size) {
  const auto& elev = map.get(elevation_layer);
  const auto size = map.getSize();
  const Eigen::Index rows = size(0);
  const Eigen::Index cols = size(1);
  const Eigen::Index sub_rows = sub_size(0);
  const Eigen::Index sub_cols = sub_size(1);
  const auto startIdx = map.getStartIndex();
  const double res = map.getResolution();

  // Precompute world coordinates for each buffer row/col in the submap region
  const double origin_x =
      map.getPosition().x() + map.getLength().x() / 2.0 - res / 2.0;
  const double origin_y =
      map.getPosition().y() + map.getLength().y() / 2.0 - res / 2.0;

  std::vector<float> row_x(sub_rows);
  std::vector<Eigen::Index> buf_row(sub_rows);
  for (Eigen::Index i = 0; i < sub_rows; ++i) {
    Eigen::Index r = (sub_start(0) + i) % rows;
    buf_row[i] = r;
    int unwrapped = (r - startIdx(0) + rows) % rows;
    row_x[i] = static_cast<float>(origin_x - unwrapped * res);
  }

  std::vector<float> col_y(sub_cols);
  std::vector<Eigen::Index> buf_col(sub_cols);
  for (Eigen::Index j = 0; j < sub_cols; ++j) {
    Eigen::Index c = (sub_start(1) + j) % cols;
    buf_col[j] = c;
    int unwrapped = (c - startIdx(1) + cols) % cols;
    col_y[j] = static_cast<float>(origin_y - unwrapped * res);
  }

  // Collect layers: elevation → z, color → rgb (packed), rest → float fields
  std::vector<std::string> float_layers;
  bool has_color = false;
  for (const auto& l : map.getLayers()) {
    if (layer::isInternal(l)) continue;
    if (l == elevation_layer) continue;
    if (l == layer::color) {
      has_color = true;
      continue;
    }
    float_layers.push_back(l);
  }

  // Build PointCloud2 field descriptors (all fields are 4 bytes)
  PointCloud2T msg;
  uint32_t offset = 0;
  auto addField = [&](const std::string& name, uint8_t datatype) {
    PointFieldT f;
    f.name = name;
    f.offset = offset;
    f.datatype = datatype;
    f.count = 1;
    msg.fields.push_back(f);
    offset += 4;
  };

  addField("x", PointFieldT::FLOAT32);
  addField("y", PointFieldT::FLOAT32);
  addField("z", PointFieldT::FLOAT32);
  for (const auto& l : float_layers) {
    addField(l, PointFieldT::FLOAT32);
  }
  if (has_color) {
    addField("rgb", PointFieldT::FLOAT32);
  }

  const uint32_t point_step = offset;

  // Prepare layer data pointers
  const float* elev_data = elev.data();
  std::vector<const float*> float_ptrs;
  float_ptrs.reserve(float_layers.size());
  for (const auto& l : float_layers) {
    float_ptrs.push_back(map.get(l).data());
  }
  const float* color_data =
      has_color ? map.get(layer::color).data() : nullptr;

  // Count valid points in submap region
  size_t valid_count = 0;
  for (Eigen::Index j = 0; j < sub_cols; ++j) {
    const Eigen::Index base = buf_col[j] * rows;
    for (Eigen::Index i = 0; i < sub_rows; ++i) {
      if (std::isfinite(elev_data[base + buf_row[i]])) ++valid_count;
    }
  }

  // Fill message metadata
  msg.header.stamp = stamp;
  msg.header.frame_id = map.getFrameId();
  msg.height = 1;
  msg.width = static_cast<uint32_t>(valid_count);
  msg.point_step = point_step;
  msg.row_step = static_cast<uint32_t>(valid_count) * point_step;
  msg.is_dense = true;
  msg.is_bigendian = false;
  msg.data.resize(valid_count * point_step);

  // Fill point data (column-major for cache efficiency)
  uint8_t* out = msg.data.data();
  for (Eigen::Index j = 0; j < sub_cols; ++j) {
    const float y = col_y[j];
    const Eigen::Index base = buf_col[j] * rows;
    for (Eigen::Index i = 0; i < sub_rows; ++i) {
      const Eigen::Index idx = base + buf_row[i];
      const float z = elev_data[idx];
      if (!std::isfinite(z)) continue;

      const float x = row_x[i];
      std::memcpy(out, &x, 4);
      out += 4;
      std::memcpy(out, &y, 4);
      out += 4;
      std::memcpy(out, &z, 4);
      out += 4;

      for (const float* ptr : float_ptrs) {
        const float val = ptr[idx];
        std::memcpy(out, &val, 4);
        out += 4;
      }

      if (color_data) {
        std::memcpy(out, &color_data[idx], 4);
        out += 4;
      }
    }
  }

  return msg;
}

/// Full-map convenience overload.
template <typename PointCloud2T, typename PointFieldT, typename TimeT>
PointCloud2T toPointCloud2Impl(const ElevationMap& map, const TimeT& stamp,
                               const char* elevation_layer) {
  return toPointCloud2Impl<PointCloud2T, PointFieldT>(
      map, stamp, elevation_layer, map.getStartIndex(), map.getSize());
}

/// Collect non-internal layer names from an ElevationMap.
inline std::vector<std::string> visibleLayers(const ElevationMap& map) {
  std::vector<std::string> layers;
  for (const auto& l : map.getLayers()) {
    if (!layer::isInternal(l)) layers.push_back(l);
  }
  return layers;
}

}  // namespace fastdem::detail

#endif  // FASTDEM_BRIDGE_ROS_IMPL_HPP
