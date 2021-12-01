#include "nuscenes2bag/NuScenes2Bag.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/RunEvery.hpp"
#include "nuscenes2bag/SceneConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include <memory> // std::unique_ptr

#include <boost/asio/thread_pool.hpp>
#include <boost/asio//defer.hpp>

#include <iostream>

#include <array>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <thread>

using namespace std;

namespace nuscenes2bag {

NuScenes2Bag::NuScenes2Bag() {}

void
NuScenes2Bag::convertDirectory(const fs::path& inDatasetPath,
                               const std::string& version,
                               const fs::path& outputRosbagPath,
                               int threadNumber,
                               std::optional<int32_t> sceneNumberOpt
                               )
{
  if ((threadNumber < 1) || (threadNumber > 64)) {
    std::cout << "Forcing at least one job number (-j1)" << std::endl;
    threadNumber = 1;
  }

  MetaDataReader metaDataReader;

  fs::path metadataPath = inDatasetPath;
  metadataPath /= fs::path(version); // Append sub-directory
  std::cout << "Loading metadata from " + metadataPath.string() + " ..." << std::endl;

  try {
    // If file is not found, a runtime_error is thrown
  metaDataReader.loadFromDirectory(metadataPath);
  } catch (const runtime_error& e) {
      std::cerr << "Error: " << e.what() << '\n';
      std::exit(-1);
  }

  cout << "Initializing " << threadNumber << " threads..." << endl;

  boost::asio::thread_pool pool(threadNumber);
  std::vector<std::unique_ptr<SceneConverter>> sceneConverters;

  FileProgress fileProgress;

  fs::create_directories(outputRosbagPath);

  std::vector<Token> chosenSceneTokens;

  if(sceneNumberOpt.has_value()) {
    auto sceneInfoOpt = metaDataReader.getSceneInfoByNumber(sceneNumberOpt.value());
    if(sceneInfoOpt.has_value()) {
      chosenSceneTokens.push_back(sceneInfoOpt->token);
    } else {
      std::cout << "Scene with ID=" << sceneNumberOpt.value() << " not found!" << std::endl;
    }
  } else {
    chosenSceneTokens = metaDataReader.getAllSceneTokens();;
  }

  for (const auto& sceneToken : chosenSceneTokens) {
    std::unique_ptr<SceneConverter> sceneConverter =
      std::make_unique<SceneConverter>(metaDataReader);
    sceneConverter->submit(sceneToken, fileProgress);
    SceneConverter* sceneConverterPtr = sceneConverter.get();
    sceneConverters.push_back(std::move(sceneConverter));
    boost::asio::defer(pool, [&, sceneConverterPtr]() {
      sceneConverterPtr->run(inDatasetPath, outputRosbagPath, fileProgress);
    });
  }

  RunEvery showProgress(std::chrono::milliseconds(1000), [&fileProgress]() {
    std::cout << "Progress: "
              << static_cast<int>(fileProgress.getProgressPercentage() * 100)
              << "% [" << fileProgress.processedFiles << "/"
              << fileProgress.toProcessFiles << "]" << std::endl;
  });

  // TODO: replace check with futures
  while (fileProgress.processedFiles != fileProgress.toProcessFiles) {
    showProgress.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  pool.join();
}

}

