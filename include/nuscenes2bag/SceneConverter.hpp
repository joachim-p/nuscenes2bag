#pragma once

#if CMAKE_CXX_STANDARD >= 17
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

#include "nuscenes2bag/MetaDataReader.hpp"
#include "nuscenes2bag/FileProgress.hpp"
#include "nuscenes2bag/Boxes.h"

#include "rosbag/bag.h"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

namespace nuscenes2bag {

class SceneConverter {
    public:
    SceneConverter(const MetaDataProvider& metaDataProvider);

    void submit(const Token& sceneToken, FileProgress& fileProgress);

    void run(const fs::path& inPath, const fs::path& outDirectoryPath, FileProgress& fileProgress);

    private:
    void convertSampleDatas(rosbag::Bag& outBag, const fs::path &inPath, FileProgress& fileProgress);
    void convertEgoPoseInfos(rosbag::Bag& outBag, const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfo);
    void convertAnnotations(rosbag::Bag& outBag);
    void getBoxes(const SampleDataInfo& sampleData, std::vector<Box>& boxes);

    private:
    const MetaDataProvider& metaDataProvider;
    std::vector<SampleDataInfo> sampleDatas;
    std::vector<EgoPoseInfo> egoPoseInfos;
    std::map<Token, SampleInfo> sceneSamples;
    std::map<Token, std::vector<SampleAnnotationInfo>> sceneAnnotations;
    SceneId sceneId;
    Token sceneToken;
};

Eigen::Vector3d lerp(const double t, const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);

Box makeBox(const SampleAnnotationInfo& annotation);
Box makeBox(const SampleAnnotationInfo& annotation, const Eigen::Vector3d& center, const Eigen::Quaterniond& rotation);

geometry_msgs::Point makePointMsg(const Eigen::Vector3d& point);

// Provides the default colors based on the category names
std_msgs::ColorRGBA getColor(const std::string& categoryName);

visualization_msgs::Marker makeMarkerMsg(const Box& box, const int32_t id, const ros::Time& timestamp, const ros::Duration& lifetime);
visualization_msgs::MarkerArray makeMarkerArrayMsg(const std::vector<Box>& boxes, const ros::Time& timestamp, const ros::Duration& lifetime);
}