// SPDX-License-Identifier: BSD-3-Clause
//
// Offline submap reconstruction glue:
//   submaps/index.yaml + submap_*/meta.yaml + patches/*.pcd
//     -> merged optimized cloud
//     -> FastDEM buildDEM()
//     -> global elevation DEM point cloud

#include <Eigen/Geometry>

#include <fastdem/io/pcd_convert.hpp>
#include <nanopcl/core/transform.hpp>
#include <nanopcl/io/pcd_io.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct PatchAsset {
  int keyframe_id = -1;
  fs::path path;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

fastdem::RasterMethod parseRasterMethod(const std::string& method) {
  if (method == "max") return fastdem::RasterMethod::Max;
  if (method == "min") return fastdem::RasterMethod::Min;
  if (method == "mean") return fastdem::RasterMethod::Mean;
  if (method == "minmax") return fastdem::RasterMethod::MinMax;
  throw std::runtime_error("Unknown raster method '" + method +
                           "'. Expected: max, min, mean, minmax.");
}

Eigen::Isometry3d poseFromYaml(const YAML::Node& patch) {
  const auto position = patch["position"];
  const auto orientation = patch["orientation_xyzw"];
  if (!position || position.size() != 3) {
    throw std::runtime_error("Patch metadata is missing position[3].");
  }
  if (!orientation || orientation.size() != 4) {
    throw std::runtime_error("Patch metadata is missing orientation_xyzw[4].");
  }

  const double x = position[0].as<double>();
  const double y = position[1].as<double>();
  const double z = position[2].as<double>();
  const double qx = orientation[0].as<double>();
  const double qy = orientation[1].as<double>();
  const double qz = orientation[2].as<double>();
  const double qw = orientation[3].as<double>();

  Eigen::Quaterniond q(qw, qx, qy, qz);
  q.normalize();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = q.toRotationMatrix();
  return pose;
}

fs::path resolvePatchPath(const fs::path& submap_dir, const YAML::Node& patch) {
  if (patch["local_path"]) {
    const fs::path local = submap_dir / patch["local_path"].as<std::string>();
    if (!patch["local_path"].as<std::string>().empty() && fs::exists(local)) {
      return local;
    }
  }
  if (patch["source_path"]) {
    const fs::path source = patch["source_path"].as<std::string>();
    if (fs::exists(source)) {
      return source;
    }
  }
  throw std::runtime_error("Patch metadata has no valid local_path/source_path.");
}

std::vector<PatchAsset> loadPatchAssets(const fs::path& submaps_dir) {
  const fs::path index_path = submaps_dir / "index.yaml";
  if (!fs::exists(index_path)) {
    throw std::runtime_error("Missing submap index: " + index_path.string());
  }

  const YAML::Node index = YAML::LoadFile(index_path.string());
  const YAML::Node submaps = index["submaps"];
  if (!submaps || !submaps.IsSequence()) {
    throw std::runtime_error("index.yaml has no sequence field 'submaps'.");
  }

  std::vector<PatchAsset> patches;
  for (const auto& submap : submaps) {
    if (!submap["meta_path"]) {
      throw std::runtime_error("Submap index entry is missing meta_path.");
    }

    const fs::path meta_path = submaps_dir / submap["meta_path"].as<std::string>();
    const fs::path submap_dir = meta_path.parent_path();
    const YAML::Node meta = YAML::LoadFile(meta_path.string());
    const YAML::Node patch_nodes = meta["patches"];
    if (!patch_nodes || !patch_nodes.IsSequence()) {
      throw std::runtime_error("Submap meta has no sequence field 'patches': " +
                               meta_path.string());
    }

    for (const auto& patch : patch_nodes) {
      PatchAsset asset;
      if (patch["keyframe_id"]) {
        asset.keyframe_id = patch["keyframe_id"].as<int>();
      }
      asset.path = resolvePatchPath(submap_dir, patch);
      asset.pose = poseFromYaml(patch);
      patches.push_back(asset);
    }
  }
  return patches;
}

nanopcl::PointCloud mergeOptimizedPatches(
    const std::vector<PatchAsset>& patches,
    const std::string& global_frame) {
  nanopcl::PointCloud merged;
  merged.setFrameId(global_frame);

  for (const auto& asset : patches) {
    auto cloud = nanopcl::io::loadPCD(asset.path.string());
    if (cloud.empty()) {
      std::cerr << "Warning: empty patch skipped: " << asset.path << "\n";
      continue;
    }

    auto transformed =
        nanopcl::transformCloud(std::move(cloud), asset.pose, global_frame);
    merged += transformed;
  }
  return merged;
}

void writeSummary(
    const fs::path& path,
    const fs::path& submaps_dir,
    const fs::path& output_dir,
    const std::vector<PatchAsset>& patches,
    const nanopcl::PointCloud& merged,
    const fastdem::ElevationMap& dem,
    float resolution,
    const std::string& method) {
  std::ofstream out(path);
  out << "schema_version: 1\n";
  out << "source_submaps_dir: " << submaps_dir.string() << "\n";
  out << "output_dir: " << output_dir.string() << "\n";
  out << "patch_count: " << patches.size() << "\n";
  out << "merged_points: " << merged.size() << "\n";
  out << "dem_cells_x: " << dem.getSize()(0) << "\n";
  out << "dem_cells_y: " << dem.getSize()(1) << "\n";
  out << "resolution: " << resolution << "\n";
  out << "raster_method: " << method << "\n";
  out << "outputs:\n";
  out << "  merged_cloud: global_merged_cloud.pcd\n";
  out << "  elevation_dem: global_elevation_dem.pcd\n";
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr
        << "Usage: submap_rebuild <submaps_dir> <output_dir> [resolution] [method]\n"
        << "  resolution: DEM cell size in meters, default 0.1\n"
        << "  method: max|min|mean|minmax, default max\n";
    return 1;
  }

  try {
    const fs::path submaps_dir = fs::path(argv[1]);
    const fs::path output_dir = fs::path(argv[2]);
    const float resolution = argc >= 4 ? std::stof(argv[3]) : 0.1f;
    const std::string method_name = argc >= 5 ? argv[4] : "max";
    const auto method = parseRasterMethod(method_name);

    fs::create_directories(output_dir);

    std::cout << "Loading submap index from " << submaps_dir << " ...\n";
    const auto patch_assets = loadPatchAssets(submaps_dir);
    std::cout << "  patches: " << patch_assets.size() << "\n";

    std::cout << "Merging optimized patch clouds ...\n";
    auto merged = mergeOptimizedPatches(patch_assets, "map");
    std::cout << "  merged points: " << merged.size() << "\n";
    if (merged.empty()) {
      throw std::runtime_error("No points after merging patch clouds.");
    }

    const fs::path merged_path = output_dir / "global_merged_cloud.pcd";
    nanopcl::io::savePCD(merged_path.string(), merged);
    std::cout << "Saved merged cloud: " << merged_path << "\n";

    fastdem::DEMConfig dem_config;
    dem_config.resolution = resolution;
    dem_config.method = method;

    std::cout << "Building DEM with FastDEM buildDEM() ...\n";
    auto dem = fastdem::buildDEM(merged, dem_config);
    dem.setFrameId("map");

    const auto dem_cloud = fastdem::toPointCloud(dem);
    const fs::path dem_path = output_dir / "global_elevation_dem.pcd";
    nanopcl::io::savePCD(dem_path.string(), dem_cloud);
    std::cout << "Saved elevation DEM cloud: " << dem_path << "\n";

    writeSummary(output_dir / "rebuild_summary.yaml", submaps_dir, output_dir,
                 patch_assets, merged, dem, resolution, method_name);
    std::cout << "Saved summary: " << (output_dir / "rebuild_summary.yaml")
              << "\n";
  } catch (const std::exception& e) {
    std::cerr << "submap_rebuild failed: " << e.what() << "\n";
    return 2;
  }

  return 0;
}
