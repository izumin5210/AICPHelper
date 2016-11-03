//
// Created by Masayuki IZUMI on 7/14/16.
//

#ifndef AICPHELPER_EVENTS_H
#define AICPHELPER_EVENTS_H

#include "Box.h"

#include <boost/filesystem.hpp>

struct OpenDirectoryEvent {
  boost::filesystem::path path;
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

struct SaveCroppingParamsEvent {
  boost::filesystem::path path;
};

struct UpdateAppearanceEvent {
  float point_size;
  float slerp_t;
  bool visible_current_cloud;
  bool visible_next_cloud;
  bool visible_transformed_cloud;
  bool coloring_limbs;
};

struct TransformCloudsEvent {
  boost::filesystem::path path;
  int max_iterations;
};

#endif //AICPHELPER_EVENTS_H
