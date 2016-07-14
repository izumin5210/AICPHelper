//
// Created by Masayuki IZUMI on 7/14/16.
//

#ifndef AICPHELPER_EVENTS_H
#define AICPHELPER_EVENTS_H

#include "Box.h"

namespace bfs       = boost::filesystem;
using bpath         = bfs::path;

struct OpenDirectoryEvent {
  bpath path;
};

struct UpdateCloudEvent {
  std::string key;
};

struct UpdateBoxEvent {
  Box box;
};

struct UpdateBoxOffsetEvent {
  glm::vec3 offset;
};

struct EnableCroppingEvent {
  bool enable;
};

#endif //AICPHELPER_EVENTS_H
