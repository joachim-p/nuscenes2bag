#include "nuscenes2bag/SceneConverter.hpp"
#include "nuscenes2bag/DatasetTypes.hpp"
#include "nuscenes2bag/utils.hpp"

#include "nuscenes2bag/EgoPoseConverter.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverterXYZIR.hpp"
#include "nuscenes2bag/RadarDirectoryConverter.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <regex>
#include <string>

using namespace std;

namespace nuscenes2bag {

SceneConverter::SceneConverter(const MetaDataProvider& metaDataProvider)
  : metaDataProvider(metaDataProvider)
{}


#if CMAKE_CXX_STANDARD >= 17

std::optional<SampleType>
getSampleType(const std::string_view filename)
{
  std::array<std::pair<const char*, SampleType>, 3> pairs = {
    { { "CAM", SampleType::CAMERA },
      { "RADAR", SampleType::RADAR },
      { "LIDAR", SampleType::LIDAR } }
  };
  for (const auto& [str, SampleType] : pairs) {
    if (filename.find(str) != string::npos) {
      return std::optional(SampleType);
    }
  }
  cout << "Unknown file " << filename << endl;
  return std::nullopt;
}

#else

SampleType getSampleType(const std::string &filename)
{
  std::array<std::pair<const char*, SampleType>, 3> pairs = {
    { { "CAM", SampleType::CAMERA },
      { "RADAR", SampleType::RADAR },
      { "LIDAR", SampleType::LIDAR } }
  };
  for (const auto& keyvalue : pairs) {
    const char* str = keyvalue.first;
    const SampleType& type = keyvalue.second;
    if (filename.find(str) != string::npos) {
      return type;
    }
  }
  cout << "Unknown file " << filename << endl;

  return SampleType::NONE;
}

#endif

#if CMAKE_CXX_STANDARD >= 17

template<typename T>
void
writeMsg(const std::string_view topicName,
         const std::string &frameID,
         const TimeStamp timeStamp,
         rosbag::Bag& outBag,
         std::optional<T> msgOpt)
{
  if (msgOpt.has_value()) {
    auto& msg = msgOpt.value();
    msg.header.frame_id = frameID;
    msg.header.stamp = stampUs2RosTime(timeStamp);
    outBag.write(std::string(topicName).c_str(), msg.header.stamp, msg);
  }
}

#else

template<typename T> void writeMsg(const std::string &topicName,
                                   const std::string &frameID,
                                   const TimeStamp timeStamp,
                                   rosbag::Bag& outBag,
                                   T msg)
{
  if (msg) {
    msg->header.frame_id = frameID;
    msg->header.stamp = stampUs2RosTime(timeStamp);
    outBag.write(std::string(topicName).c_str(), msg->header.stamp, msg);
  }
}

#endif

static const std::regex TOPIC_REGEX = std::regex(".*__([A-Z_]+)__.*");

void
SceneConverter::submit(const Token& sceneToken, FileProgress& fileProgress)
{

#if CMAKE_CXX_STANDARD >= 17
  std::optional sceneInfoOpt = metaDataProvider.getSceneInfo(sceneToken);
  // if(!sceneInfoOpt.has_value()) {
  //     // cout << "SceneInfo for " << sceneToken << " not found!" << endl;
  //     return;
  // }
  assert(sceneInfoOpt.has_value());
  SceneInfo& sceneInfo = sceneInfoOpt.value();
#else
  boost::shared_ptr<SceneInfo> sceneInfoPtr = metaDataProvider.getSceneInfo(sceneToken);
  assert(sceneInfoPtr != nullptr);
  SceneInfo& sceneInfo = *sceneInfoPtr;
#endif

  sceneId = sceneInfo.sceneId;
  this->sceneToken = sceneToken;
  sceneSamples = metaDataProvider.getSceneSamples(sceneToken);
  sceneAnnotations = metaDataProvider.getSceneSampleAnnotations(sceneToken);
  sampleDatas = metaDataProvider.getSceneSampleData(sceneToken);
  egoPoseInfos = metaDataProvider.getEgoPoseInfo(sceneToken);

  fileProgress.addToProcess(sampleDatas.size());
}

void
SceneConverter::run(const fs::path& inPath,
                    const fs::path& outDirectoryPath,
                    FileProgress& fileProgress)
{

  std::string bagName =
    outDirectoryPath.string() + "/" + std::to_string(sceneId) + ".bag";

  rosbag::Bag outBag;
  outBag.open(bagName, rosbag::bagmode::Write);

  auto sensorInfos = metaDataProvider.getSceneCalibratedSensorInfo(sceneToken);
  convertEgoPoseInfos(outBag, sensorInfos);
  convertAnnotations(outBag);
  convertSampleDatas(outBag, inPath, fileProgress);

  outBag.close();
}

void
SceneConverter::convertSampleDatas(rosbag::Bag& outBag,
                                   const fs::path& inPath,
                                   FileProgress& fileProgress)
{
  for (const auto& sampleData : sampleDatas) {
    fs::path sampleFilePath = inPath / sampleData.fileName;

#if CMAKE_CXX_STANDARD >= 17
    std::optional<SampleType> sampleTypeOpt = getSampleType(sampleFilePath.string());
    if (!sampleTypeOpt.has_value()) {
      continue;
    }
    SampleType& sampleType = sampleTypeOpt.value();
#else
    SampleType sampleType = getSampleType(sampleFilePath.string());
#endif

    CalibratedSensorInfo calibratedSensorInfo =
      metaDataProvider.getCalibratedSensorInfo(
        sampleData.calibratedSensorToken);
    CalibratedSensorName calibratedSensorName =
      metaDataProvider.getSensorName(calibratedSensorInfo.sensorToken);
    std::string sensorName = toLower(calibratedSensorName.name);

    if (sampleType == SampleType::CAMERA) {
      auto topicName = sensorName + "/raw";
      auto msg = readImageFile(sampleFilePath);
      writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

    } else if (sampleType == SampleType::LIDAR) {
      auto topicName = sensorName;

      // PointCloud format:
      auto msg = readLidarFile(sampleFilePath); // x,y,z,intensity
      //auto msg = readLidarFileXYZIR(sampleFilePath); // x,y,z,intensity,ring

      writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

    } else if (sampleType == SampleType::RADAR) {
      auto topicName = sensorName;
      auto msg = readRadarFile(sampleFilePath);
      writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

    } else {
      cout << "Unknown sample type" << endl;
    }

    fileProgress.addToProcessed(1);
  }
}

geometry_msgs::TransformStamped
makeTransform(const char* frame_id,
              const char* child_frame_id,
              const double* translation,
              const double* rotation,
              ros::Time stamp = ros::Time(0))
{
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = std::string(frame_id);
  msg.header.stamp = stamp;
  msg.child_frame_id = std::string(child_frame_id);
  assignArray2Vector3(msg.transform.translation, translation);
  assignArray2Quaternion(msg.transform.rotation, rotation);
  return msg;
}

geometry_msgs::TransformStamped
makeIdentityTransform(const char* frame_id,
                      const char* child_frame_id,
                      ros::Time stamp = ros::Time(0))
{
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = std::string(frame_id);
  msg.header.stamp = stamp;
  msg.child_frame_id = std::string(child_frame_id);
  msg.transform.rotation.w = 1;
  return msg;
}

void
SceneConverter::convertEgoPoseInfos(
  rosbag::Bag& outBag,
  const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfos)
{

  std::vector<geometry_msgs::TransformStamped> constantTransforms;
  for (const auto& calibratedSensorInfo : calibratedSensorInfos) {
    auto sensorTransform =
      makeTransform("base_link",
                    toLower(calibratedSensorInfo.name.name).c_str(),
                    calibratedSensorInfo.info.translation,
                    calibratedSensorInfo.info.rotation);
    constantTransforms.push_back(sensorTransform);
  }
  geometry_msgs::TransformStamped tfMap2Odom =
    makeIdentityTransform("map", "odom");
  constantTransforms.push_back(tfMap2Odom);

  const std::string odomTopic = "/odom";
  for (const auto& egoPose : egoPoseInfos) {
    // write odom
    nav_msgs::Odometry odomMsg = egoPoseInfo2OdometryMsg(egoPose);
    outBag.write(odomTopic.c_str(), odomMsg.header.stamp, odomMsg);

    // write TFs
    geometry_msgs::TransformStamped tfOdom2Base =
      egoPoseInfo2TransformStamped(egoPose);
    tf::tfMessage tfMsg;
    tfMsg.transforms.push_back(tfOdom2Base);
    for (const auto& constantTransform : constantTransforms) {
      auto constantTransformWithNewStamp = constantTransform;
      constantTransformWithNewStamp.header.stamp = odomMsg.header.stamp;
      tfMsg.transforms.push_back(constantTransformWithNewStamp);
    }
    outBag.write("/tf", odomMsg.header.stamp, tfMsg);
  }
}

bool compareByTimestamp(const SampleDataInfo& a, const SampleDataInfo& b)
{
    return a.timeStamp < b.timeStamp;
}

void
SceneConverter::convertAnnotations(rosbag::Bag& outBag) // SampleType& sensorType = SampleType::LIDAR
{
  // In sampleDatas the keyframes are ordered by timestamp but the intermediate frames are not.
  // Sorting them can help with debugging.
  //std::sort(sampleDatas.begin(), sampleDatas.end(), compareByTimestamp);

  for (const auto& sampleData : sampleDatas) {

#if CMAKE_CXX_STANDARD >= 17
    std::optional<SampleType> sampleTypeOpt = getSampleType(sampleData.fileName);
    if (!sampleTypeOpt.has_value()) {
      continue;
    }
    SampleType& sampleType = sampleTypeOpt.value();
#else
    SampleType sampleType = getSampleType(sampleData.fileName);
#endif

    CalibratedSensorInfo calibratedSensorInfo =
      metaDataProvider.getCalibratedSensorInfo(
        sampleData.calibratedSensorToken);
    CalibratedSensorName calibratedSensorName =
      metaDataProvider.getSensorName(calibratedSensorInfo.sensorToken);
    std::string sensorName = toLower(calibratedSensorName.name);

    if (sampleType == SampleType::LIDAR) {
      std::vector<Box> boxes;
      getBoxes(sampleData, boxes);

      const ros::Time& timestamp = stampUs2RosTime(sampleData.timeStamp);

      Boxes boxesMsg;
      boxesMsg.header.stamp = timestamp;
      boxesMsg.header.frame_id = "map";
      boxesMsg.boxes = boxes;
      outBag.write("boxes", timestamp, boxesMsg);

      jsk_recognition_msgs::BoundingBoxArray boxesVizMsg = makeVisualizationMsg(boxes, timestamp);
      outBag.write("boxes_viz", timestamp, boxesVizMsg);
    }
  }
}

Eigen::Quaterniond makeQuaterniond(const float* rotation)
{
  return Eigen::Quaterniond(rotation[0], rotation[1], rotation[2], rotation[3]);
}

void
SceneConverter::getBoxes(const SampleDataInfo& sampleData, std::vector<Box>& boxes)
{
  const Token& currSampleToken = sampleData.sampleToken;

  SampleInfo currSample;
  {
    auto it = sceneSamples.find(currSampleToken);
    if (it == sceneSamples.end()) {
      std::cout << "can't find current sample token in sceneSamples" << std::endl;
      return;
    }
    currSample = it->second;
  }

  if ((sampleData.isKeyFrame) || (currSample.prev.empty())) {
    // If sample data is a key frame, or no previous annotations are available,
    // return the annotations for the current sample.

    std::vector<SampleAnnotationInfo> annotations;
    {
      auto it = sceneAnnotations.find(currSampleToken);
      if (it == sceneAnnotations.end()) {
        std::cout << "can't find current sample token in sceneAnnotations" << std::endl;
        return;
      }
      annotations = it->second;
    }

    for (const auto& annotation: annotations)
    {
      boxes.push_back(makeBox(annotation));
    }

    return;
  }
  else {
    // Sample data is intermediate, use linear interpolation to estimate position of boxes

    SampleInfo prevSample;
    {
      auto it = sceneSamples.find(currSample.prev);
      if (it == sceneSamples.end()) {
        std::cout << "can't find prev sample token in sceneSamples" << std::endl;
        return;
      }
      prevSample = it->second;
    }

    std::vector<SampleAnnotationInfo> currAnnotations;
    {
      auto it = sceneAnnotations.find(currSampleToken);
      if (it == sceneAnnotations.end()) {
        std::cout << "can't find current sample token in sceneAnnotations" << std::endl;
        return;
      }
      currAnnotations = it->second;
    }

    std::vector<SampleAnnotationInfo> prevAnnotations;
    {
      auto it = sceneAnnotations.find(prevSample.token);
      if (it == sceneAnnotations.end()) {
        std::cout << "can't find previous sample token in sceneAnnotations" << std::endl;
        return;
      }
      prevAnnotations = it->second;
    }

    // Map instance tokens to prev_ann records
    std::map<std::string, SampleAnnotationInfo> prevInstanceMap;
    for (const auto& annotation : prevAnnotations)
    {
      prevInstanceMap.insert({annotation.instanceToken, annotation});
    }

    const TimeStamp t0 = prevSample.timeStamp;
    const TimeStamp t1 = currSample.timeStamp;
    TimeStamp t = sampleData.timeStamp;

    // There are rare situations where the timestamps in the DB are off so ensure that t0 < t < t1.
    t = std::max(t0, std::min(t1, t));

    for (const auto& annotation : currAnnotations)
    {
      auto it = prevInstanceMap.find(annotation.instanceToken);
      if (it == prevInstanceMap.end()) {
        // The instance does not exist in the previous frame so get the current annotation.
        boxes.push_back(makeBox(annotation));
      }
      else {
        // The annotated instance existed in the previous frame, therefore interpolate center & orientation.
        const SampleAnnotationInfo prevAnnotation = it->second;

        const uint64_t numerator = (t - t0); // unsigned long long
        const uint64_t denominator = (t1 - t0);

        const double amount = static_cast<double>(numerator) / static_cast<double>(denominator);

        // Interpolate center
        const Eigen::Vector3d c0(prevAnnotation.translation[0], prevAnnotation.translation[1], prevAnnotation.translation[2]);
        const Eigen::Vector3d c1(annotation.translation[0], annotation.translation[1], annotation.translation[2]);
        const Eigen::Vector3d center = lerp(amount, c1, c0);

        // Interpolate orientation (w,x,y,z)
        const Eigen::Quaterniond q0 = makeQuaterniond(prevAnnotation.rotation);
        const Eigen::Quaterniond q1 = makeQuaterniond(annotation.rotation);
        const Eigen::Quaterniond rotation = q0.slerp(amount, q1);

        auto box = makeBox(annotation);
        box.center.x = center.x();
        box.center.y = center.y();
        box.center.z = center.z();
        box.orientation.x = rotation.x();
        box.orientation.y = rotation.y();
        box.orientation.z = rotation.z();
        box.orientation.w = rotation.w();

        boxes.push_back(box);
      }
    }
  }
}

Eigen::Vector3d lerp(const double t, const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) {
  return t*p0 + (1.0 - t)*p1;
}

Box makeBox(const SampleAnnotationInfo& annotation)
{
  Box boxMsg;

  // NuScenes data is stored as float but ROS uses double
  boxMsg.center.x = static_cast<double>(annotation.translation[0]);
  boxMsg.center.y = static_cast<double>(annotation.translation[1]);
  boxMsg.center.z = static_cast<double>(annotation.translation[2]);

  boxMsg.size.x = static_cast<double>(annotation.size[1]);
  boxMsg.size.y = static_cast<double>(annotation.size[0]);
  boxMsg.size.z = static_cast<double>(annotation.size[2]);

  boxMsg.orientation.w = static_cast<double>(annotation.rotation[0]);
  boxMsg.orientation.x = static_cast<double>(annotation.rotation[1]);
  boxMsg.orientation.y = static_cast<double>(annotation.rotation[2]);
  boxMsg.orientation.z = static_cast<double>(annotation.rotation[3]);

  boxMsg.label = 0;

  boxMsg.token = annotation.token;

  boxMsg.category_name = annotation.categoryName;

  return boxMsg;
}

geometry_msgs::Point makePointMsg(const Eigen::Vector3d& point)
{
  geometry_msgs::Point pointMsg;
  pointMsg.x = point.x();
  pointMsg.y = point.y();
  pointMsg.z = point.z();
  return pointMsg;
}

inline bool stringContains(const std::string& str, const std::string& substr)
{
  if (str.find(substr) != std::string::npos) {
      return true;
  }
  return false;
}

jsk_recognition_msgs::BoundingBox makeVisualizationMsg(const Box& box, const ros::Time& timestamp)
{
  jsk_recognition_msgs::BoundingBox msg;

  msg.header.frame_id = "map";
  msg.header.stamp = timestamp;

  msg.pose.position.x = box.center.x;
  msg.pose.position.y = box.center.y;
  msg.pose.position.z = box.center.z;
  
  msg.pose.orientation.x = box.orientation.x;
  msg.pose.orientation.y = box.orientation.y;
  msg.pose.orientation.z = box.orientation.z;
  msg.pose.orientation.w = box.orientation.w;

  msg.dimensions.x = box.size.x;
  msg.dimensions.y = box.size.y;
  msg.dimensions.z = box.size.z;

  msg.value = 1.0; // labels are always correct
  msg.label = box.label;

  return msg;
}

jsk_recognition_msgs::BoundingBoxArray makeVisualizationMsg(const std::vector<Box>& boxes, const ros::Time& timestamp)
{
  jsk_recognition_msgs::BoundingBoxArray msg;
  msg.header.frame_id = "map";
  msg.header.stamp = timestamp;

  for (const auto& box : boxes)
  {
    msg.boxes.push_back(makeVisualizationMsg(box, timestamp));
  }

  return msg;
}

}