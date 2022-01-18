#pragma once

#include <filesystem>
namespace fs = std::filesystem;

#include "nuscenes2bag/MetaDataReader.hpp"
#include "nuscenes2bag/FileProgress.hpp"
#include "nuscenes2bag/SampleAnnotations.h"
#include "nuscenes2bag/SampleToken.h"

#include "rosbag/bag.h"
#include "std_msgs/Header.h"
#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

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

    void getAnnotations(const SampleDataInfo& sampleData, const std_msgs::Header& header, std::vector<SampleAnnotation>& annotations);
    SampleAnnotation getAnnotation(const SampleAnnotationInfo& annotation, const std_msgs::Header& header);
    Category getCategory(const std::string& instanceToken);
    std::vector<Attribute> getAttributes(const std::vector<std::string>& attribute_tokens);

    private:
    const MetaDataProvider& metaDataProvider;
    std::vector<SampleDataInfo> sampleDatas;
    std::vector<EgoPoseInfo> egoPoseInfos;
    std::map<Token, SampleInfo> sceneSamples;
    std::map<Token, std::vector<SampleAnnotationInfo>> sceneAnnotations;
    SceneId sceneId;
    Token sceneToken;
    std::map<Token, InstanceInfo> instances;
    std::map<Token, CategoryInfo> categories;
    std::map<Token, AttributeInfo> attributes;
};

Eigen::Vector3d lerp(const double t, const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);

geometry_msgs::Point makePointMsg(const Eigen::Vector3d& point);
jsk_recognition_msgs::BoundingBox makeVisualizationMsg(const SampleAnnotation& annotation);
jsk_recognition_msgs::BoundingBoxArray makeVisualizationMsg(const SampleAnnotations& annotations);
}