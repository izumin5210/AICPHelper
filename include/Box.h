//
// Created by Masayuki IZUMI on 7/14/16.
//

#ifndef AICPHELPER_BOX_H
#define AICPHELPER_BOX_H

#include "glm/glm.hpp"
#include "glm/gtx/quaternion.hpp"

#include "Singleton.h"

struct Box {
  enum struct Kind {
    HEAD,
    TRUNK,
    LEFT_UP_ARM,
    RIGHT_UP_ARM,
    LEFT_DOWN_ARM,
    RIGHT_DOWN_ARM,
    LEFT_UP_LEG,
    RIGHT_UP_LEG,
    LEFT_DOWN_LEG,
    RIGHT_DOWN_LEG,
    NUM_OF_LIMBS
  };

  Kind kind;
  glm::vec3 size;
  glm::vec3 rot;
  glm::vec3 trans;
};

struct Limbs {
  std::map<Box::Kind, Box> boxes;
  glm::vec3 offset;

  static Limbs& get() {
    return Singleton<Limbs>::get();
  }
};

#endif //AICPHELPER_BOX_H
