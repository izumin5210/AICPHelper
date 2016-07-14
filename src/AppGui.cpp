//
// Created by Masayuki IZUMI on 7/13/16.
//

#include "AppGui.h"
#include "Events.h"
#include "Signal.h"

#include <pcl/io/pcd_io.h>

AppGui::AppGui()
  : boxes_({
      { Box::Kind::HEAD,            glm::vec3(0.3, 0.3, 0.3), glm::vec3(),  glm::vec3(0.0, 1.55, 0.0) },
      { Box::Kind::TRUNK,           glm::vec3(0.4, 0.7, 0.3), glm::vec3(),  glm::vec3(0.0, 1.05, 0.0) },
      { Box::Kind::LEFT_UP_ARM,     glm::vec3(0.2, 0.3, 0.2), glm::vec3(),  glm::vec3(-0.3, 1.25, 0.0) },
      { Box::Kind::RIGHT_UP_ARM,    glm::vec3(0.2, 0.3, 0.2), glm::vec3(),  glm::vec3(+0.3, 1.25, 0.0) },
      { Box::Kind::LEFT_DOWN_ARM,   glm::vec3(0.2, 0.4, 0.2), glm::vec3(),  glm::vec3(-0.3, 0.9, 0.0) },
      { Box::Kind::RIGHT_DOWN_ARM,  glm::vec3(0.2, 0.4, 0.2), glm::vec3(),  glm::vec3(+0.3, 0.9, 0.0) },
      { Box::Kind::LEFT_UP_LEG,     glm::vec3(0.2, 0.3, 0.2), glm::vec3(),  glm::vec3(-0.1, 0.55, 0.0) },
      { Box::Kind::RIGHT_UP_LEG,    glm::vec3(0.2, 0.3, 0.2), glm::vec3(),  glm::vec3(+0.1, 0.55, 0.0) },
      { Box::Kind::LEFT_DOWN_LEG,   glm::vec3(0.2, 0.4, 0.2), glm::vec3(),  glm::vec3(-0.1, 0.2, 0.0) },
      { Box::Kind::RIGHT_DOWN_LEG,  glm::vec3(0.2, 0.4, 0.2), glm::vec3(),  glm::vec3(+0.1, 0.2, 0.0) },
    }),
    enable_cropping_(false)
{}

void AppGui::initialize(cinder::app::AppBase *app) {
  app_ = app;
  ui::initialize(options);
  for (auto box : boxes_) {
    Signal<UpdateBoxEvent>::emit({box});
  }
}

void AppGui::update() {
  auto left_window_pos = glm::vec2(kWindowSpacing, kWindowSpacing);
  auto right_window_pos = glm::vec2(app_->getWindowWidth() - (kWindowWidth + kWindowSpacing), kWindowSpacing);
  drawMenuBar(left_window_pos, right_window_pos);
  drawInfoWindow(left_window_pos);
  drawCroppingWindow(right_window_pos);
}

void AppGui::drawMenuBar(glm::vec2 &left_window_pos, glm::vec2 &right_window_pos) {
  ui::ScopedMainMenuBar menu_bar;

  if (ui::BeginMenu("File")) {
    if (ui::MenuItem("Open directory")) {
      auto dir = app_->getFolderPath();
      if (bfs::is_directory(dir)) {
        Signal<OpenDirectoryEvent>::emit({dir});
      }
    }
    ui::EndMenu();
  }

  left_window_pos.y += ui::GetItemRectSize().y;
  right_window_pos.y += ui::GetItemRectSize().y;
}

void AppGui::drawInfoWindow(glm::vec2 &window_pos) {
  ui::ScopedWindow window("Information", kWindowFlags);
  ui::LabelText("FPS", "%f", app_->getAverageFps());
  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}

void AppGui::drawCroppingWindow(glm::vec2 &window_pos) {
  ui::ScopedWindow window("Crop", kWindowFlags);

  int id = 0;

  ui::Columns(2);
  ui::PushID(id++);
  ui::Bullet();
  ui::TextUnformatted("Enable");
  ui::NextColumn();
  ui::PushItemWidth(-1);
  if (ui::Checkbox("##value", &enable_cropping_)) {
    Signal<EnableCroppingEvent>::emit({enable_cropping_});
  }
  ui::PopItemWidth();
  ui::NextColumn();
  ui::PopID();
  ui::Columns();

  ui::Text("Offset");
  ui::Columns(2);
  ui::PushID(id++);
  ui::Bullet();
  ui::TextUnformatted("translate");
  ui::NextColumn();
  ui::PushItemWidth(-1);
  if (ui::DragFloat3("##value", &boxes_offset_[0], 0.02f)) {
    Signal<UpdateBoxOffsetEvent>::emit({boxes_offset_});
  }
  ui::PopItemWidth();
  ui::NextColumn();
  ui::PopID();
  ui::Columns();

  ui::Separator();

  for (size_t i = 0; i < boxes_.size(); i++) {
    bool updated = false;

    ui::Text("%s", kCroppingWindowNames[boxes_[i].kind].c_str());
    ui::Columns(2);

    ui::PushID(id++);
    ui::Bullet();
    ui::TextUnformatted("size");
    ui::NextColumn();
    ui::PushItemWidth(-1);
    updated = updated || ui::DragFloat3("##value", &boxes_[i].size[0], 0.02f);
    ui::PopItemWidth();
    ui::NextColumn();
    ui::PopID();

    ui::PushID(id++);
    ui::Bullet();
    ui::TextUnformatted("translate");
    ui::NextColumn();
    ui::PushItemWidth(-1);
    updated = updated || ui::DragFloat3("##value", &boxes_[i].trans[0], 0.02f);
    ui::PopItemWidth();
    ui::NextColumn();
    ui::PopID();

    ui::PushID(id++);
    ui::Bullet();
    ui::TextUnformatted("rotate");
    ui::NextColumn();
    ui::PushItemWidth(-1);
    updated = updated || ui::DragFloat3("##value", &boxes_[i].rot[0], 0.02f);
    ui::PopItemWidth();
    ui::NextColumn();
    ui::PopID();

    ui::Columns(1);

    if (updated) {
      Signal<UpdateBoxEvent>::emit({boxes_[i]});
    }
  }

  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}
