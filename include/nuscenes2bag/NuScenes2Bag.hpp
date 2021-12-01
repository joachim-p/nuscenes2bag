#pragma once

#include <rosbag/bag.h>

#include "nuscenes2bag/DatasetTypes.hpp"
#include <optional>
#include <filesystem>
namespace fs = std::filesystem;

namespace nuscenes2bag {

struct NuScenes2Bag {

public:
  NuScenes2Bag();

  void convertDirectory(const fs::path &inDatasetPath,
                        const std::string& version,
                        const fs::path &outputRosbagPath,
                        int32_t threadNumber,
                        std::optional<int32_t> sceneNumberOpt
                        );

private:
  std::string inDatasetPathString;
};

}