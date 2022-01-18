#include <iostream>
#include <map>
#include <set>
#include <filesystem>
namespace fs = std::filesystem;

#include <nlohmann/json.hpp>

#include "nuscenes2bag/MetaDataTypes.hpp"
#include "nuscenes2bag/MetaDataProvider.hpp"
#include "nuscenes2bag/ToDebugString.hpp"

namespace nuscenes2bag {

class InvalidMetaDataException : public std::exception
{
private:
  std::string msg;

public:
  InvalidMetaDataException(const std::string& msg)
    : msg(msg)
  {}
  ~InvalidMetaDataException() throw(){}
  const char* what() const throw() { return this->msg.c_str(); }
};

class MetaDataReader : public MetaDataProvider {
public:
  void loadFromDirectory(const fs::path &directoryPath);

  std::vector<Token> getAllSceneTokens() const override;

  // scene related
  std::optional<SceneInfo>
  getSceneInfo(const Token &sceneToken) const override;
  std::optional<SceneInfo>
  getSceneInfoByNumber(const uint32_t sceneNumber) const override;
  std::vector<SampleDataInfo>
  getSceneSampleData(const Token &sceneToken) const override;
  std::vector<EgoPoseInfo>
  getEgoPoseInfo(const Token &sceneToken) const override;
  std::map<Token, SampleInfo>
  getSceneSamples(const Token& sceneToken) const override;
  std::map<Token, std::vector<SampleAnnotationInfo>>
  getSceneSampleAnnotations(const Token& sceneToken) const override;
  std::vector<CalibratedSensorInfoAndName>
  getSceneCalibratedSensorInfo(const Token &sceneToken) const override;

  // general
  const std::map<Token, InstanceInfo>& getInstanceInfo() const override;
  const std::map<Token, AttributeInfo>& getAttributeInfo() const override;
  const std::map<Token, CategoryInfo>& getCategoryInfo() const override;
  CalibratedSensorInfo getCalibratedSensorInfo(
    const Token& calibratedSensorToken) const override;
  CalibratedSensorName
  getSensorName(const Token &sensorToken) const override;

private:
  static nlohmann::json slurpJsonFile(const fs::path &filePath);
  static std::vector<SceneInfo>
  loadScenesFromFile(const fs::path &filePath);
  static std::map<Token, std::vector<SampleInfo>>
  loadSampleInfos(const fs::path &filePath);
  static std::map<Token, std::vector<SampleDataInfo>>
  loadSampleDataInfos(const fs::path &filePath);
  static std::map<Token, std::vector<EgoPoseInfo>> loadEgoPoseInfos(
      const fs::path &filePath,
      std::map<Token, Token> sample2SampleData);
  static std::map<Token, CalibratedSensorInfo>
  loadCalibratedSensorInfo(const fs::path &filePath);
  static std::map<Token, CalibratedSensorName>
  loadCalibratedSensorNames(const fs::path &filePath);
  static std::map<Token, AttributeInfo>
  loadAttributeInfo(const fs::path& filePath);
  static std::map<Token, CategoryInfo>
  loadCategoryInfo(const fs::path& filePath);
  static std::map<Token, InstanceInfo>
  loadInstanceInfo(const fs::path& filePath);
  static std::map<Token, std::vector<SampleAnnotationInfo>>
  loadSampleAnnotations(const fs::path& filePath);

  std::vector<SceneInfo> scenes;
  std::map<Token, std::vector<SampleInfo>> scene2Samples;
  std::map<Token, std::vector<SampleDataInfo>> sample2SampleData;
  std::map<Token, std::vector<EgoPoseInfo>> scene2EgoPose;
  std::map<Token, CalibratedSensorInfo> calibratedSensorToken2CalibratedSensorInfo;
  std::map<Token, std::set<CalibratedSensorInfoAndName>> scene2CalibratedSensorInfo;
  std::map<Token, CalibratedSensorName> sensorToken2CalibratedSensorName;
  std::map<Token, AttributeInfo> attributes;
  std::map<Token, CategoryInfo> categories;
  std::map<Token, InstanceInfo> instances;
  std::map<Token, std::vector<SampleAnnotationInfo>> sample2SampleAnnotations;
  bool loadFromDirectoryCalled = false;
};

}