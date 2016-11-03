//
// Created by Masayuki IZUMI on 7/14/16.
//

#ifndef AICPHELPER_CLOUDS_H
#define AICPHELPER_CLOUDS_H

#include "Cloud.h"
#include "Events.h"

#include <map>

class Clouds {
public:
  Clouds();

  std::map<boost::filesystem::path, std::shared_ptr<Cloud>> clouds() {
    return clouds_;
  };

  std::shared_ptr<Cloud> cloud(Cloud::Key key) {
    return clouds_[key];
  }


private:
  std::map<boost::filesystem::path, std::shared_ptr<Cloud>> clouds_;

  void initializeConnections();
  void onDirectoryOpen(const OpenDirectoryEvent &event);
  void onBoxUpdate(const UpdateBoxEvent &event);
  void onBoxOffsetUpdate(const UpdateBoxOffsetEvent &event);
  void onCloudTransform(const TransformCloudsEvent &event);
};

#endif //AICPHELPER_CLOUDS_H
