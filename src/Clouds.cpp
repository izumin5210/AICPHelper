//
// Created by Masayuki IZUMI on 7/14/16.
//

#include "Clouds.h"
#include "Signal.h"

Clouds::Clouds() {
  initializeConnections();
}

void Clouds::initializeConnections() {
  Signal<OpenDirectoryEvent>::connect(this, &Clouds::onDirectoryOpen);
  Signal<UpdateBoxEvent>::connect(this, &Clouds::onBoxUpdate);
  Signal<UpdateBoxOffsetEvent>::connect(this, &Clouds::onBoxOffsetUpdate);
  Signal<TransformCloudsEvent>::connect(this, &Clouds::onCloudTransform);
}

void Clouds::onDirectoryOpen(const OpenDirectoryEvent &event) {
  clouds_[event.path] = std::make_shared<Cloud>(event.path);
  clouds_[event.path]->emitUpdateEvent();
}

void Clouds::onBoxUpdate(const UpdateBoxEvent &event) {
  for (auto pair : clouds_) {
    pair.second->updateBox(event.box);
  }
}

void Clouds::onBoxOffsetUpdate(const UpdateBoxOffsetEvent &event) {
  for (auto pair : clouds_) {
    pair.second->updateBoxOffset(event.offset);
  }
}

void Clouds::onCloudTransform(const TransformCloudsEvent &event) {
  for (auto pair : clouds_) {
    pair.second->transform(event.max_iterations);
  }
}
