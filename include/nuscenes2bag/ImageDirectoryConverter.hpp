#pragma once

#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <filesystem>
namespace fs = std::filesystem;

namespace nuscenes2bag {

std::optional<sensor_msgs::Image> readImageFile(const fs::path& filePath) noexcept;

}
