#pragma once 

#include <string>
#include <array>
#include <vector>
#include <optional>

#include "nuscenes2bag/ToDebugString.hpp"
#include "nuscenes2bag/DatasetTypes.hpp"

namespace nuscenes2bag {

struct SceneInfo {
    Token token; 
    //Token logToken;
    uint32_t sampleNumber;
    SceneId sceneId;
    std::string name;
    std::string description;
    Token firstSampleToken; 
    //Token lastSampleToken;
};

struct SampleInfo {
    Token scene_token;
    Token token;
    TimeStamp timeStamp;
    Token prev;
    //Token next;
    //std::vector<Token> anns;
};

struct SampleDataInfo {
    // Token scene_token;
    Token token;
    Token sampleToken;
    TimeStamp timeStamp;
    Token egoPoseToken;
    Token calibratedSensorToken;
    std::string fileFormat;
    bool isKeyFrame;
    std::string fileName;
    //uint32_t height;
    //uint32_t width;
    //Token prev;
    //Token next;
};

struct AttributeInfo {
    Token token;
    std::string name;
};

struct CategoryInfo {
    Token token;
    std::string name;
    int index;
};

struct InstanceInfo {
    //Token token;
    Token categoryToken;
    uint32_t nbrAnnotations;
    //Token firstAnnotationToken;
    //Token lastAnnotationToken;
};

struct SampleAnnotationInfo {
    Token token;
    Token sampleToken;
    Token instanceToken;
    Token visibilityToken;
    std::vector<Token> attributeTokens;
    float translation[3]; // x, y, z
    float rotation[4]; // w, x, y, z
    float size[3]; // width, length, height
    int num_lidar_pts;
    int num_radar_pts;
};

struct CalibratedSensorInfo {
    Token token;
    Token sensorToken;
    double translation[3];
    double rotation[4];
    std::optional<IntrinsicsMatrix> cameraIntrinsics;
};

struct CalibratedSensorName {
    Token token;
    std::string name;
    std::string modality;
};

struct CalibratedSensorInfoAndName {
    CalibratedSensorInfo info;
    CalibratedSensorName name;

    inline friend bool operator<(const CalibratedSensorInfoAndName& l, const CalibratedSensorInfoAndName& r)
    {
        return l.info.token < r.info.token;
    }
};

struct EgoPoseInfo {
    Token token;
    TimeStamp timeStamp;
    double translation[3];
    double rotation[4];
};

template <> std::string to_debug_string(const SceneInfo& t);
template <> std::string to_debug_string(const SampleInfo& t);
template <> std::string to_debug_string(const SampleDataInfo& t);

}