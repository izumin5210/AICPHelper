//
// Created by Masayuki IZUMI on 7/13/16.
//

#include "AppGui.h"
#include "Events.h"
#include "Signal.h"

#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>

AppGui::AppGui(std::shared_ptr<Clouds> clouds)
  : clouds_(clouds)
  , point_size_                 (1.0f)
  , slerp_t_                    (1.0f)
  , visible_current_cloud_      (true)
  , visible_next_cloud_         (false)
  , visible_transformed_cloud_  (false)
  , coloring_limbs_             (false)
  , enable_cropping_            (false)
{}

void AppGui::initialize(cinder::app::AppBase *app) {
  app_ = app;
  ui::initialize(kUiOptions);
  emitAppearanceUpdate();
}

void AppGui::update() {
  auto left_window_pos = glm::vec2(kWindowSpacing, kWindowSpacing);
  auto right_window_pos = glm::vec2(app_->getWindowWidth() - (kWindowWidth + kWindowSpacing), kWindowSpacing);
  drawMenuBar(left_window_pos, right_window_pos);
  drawCloudsWindow(left_window_pos);
  drawTransformationWindow(left_window_pos);
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

void AppGui::drawCloudsWindow(glm::vec2 &window_pos) {
  ui::ScopedWindow window("Clouds", kWindowFlags);

  for (auto pair : clouds_->clouds()) {
    ui::Text("%s", pair.first.filename().c_str());
    ui::PushItemWidth(-1);
    if (ui::ListBoxHeader("##value")) {
      for (auto path : pair.second->paths()) {
        ui::Selectable(path.filename().c_str(), pair.second->is_current(path));
      }
      ui::ListBoxFooter();
    }
    ui::PopItemWidth();
  }

  if (!clouds_->clouds().empty()) {
    bool updated = false;
    updated = updated || ui::InputFloat("Point size", &point_size_, 0.1f);
    updated = updated || ui::Checkbox("Show current cloud", &visible_current_cloud_);
    updated = updated || ui::Checkbox("Show next cloud", &visible_next_cloud_);
    updated = updated || ui::Checkbox("Color limbs", &coloring_limbs_);
    if (updated) {
      emitAppearanceUpdate();
    }
  } else {
    ui::TextUnformatted("NO CLOUDS LOADED");
  }

  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}

void AppGui::drawTransformationWindow(glm::vec2 &window_pos) {
  ui::ScopedWindow window("Transformation", kWindowFlags);

  for (auto pair : clouds_->clouds()) {
    ui::Text("%s", pair.first.filename().c_str());
    ui::Indent();
    if (ui::Button("Start ICP")) {
      Signal<TransformCloudsEvent>::emit({pair.second->dir(), 30});
    }
    ui::Unindent();
  }

  if (clouds_->clouds().empty()) {
    ui::TextUnformatted("NO CLOUDS LOADED");
  } else {
    bool updated = false;
    updated = updated || ui::Checkbox("Show transformed cloud", &visible_transformed_cloud_);
    updated = updated || ui::SliderFloat("Slerp params", &slerp_t_, 0.0f, 1.0f);
    if (updated) {
      emitAppearanceUpdate();
    }
  }

  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
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

  ui::Columns(2);
  ui::PushID(id++);
  ui::Bullet();
  ui::TextUnformatted("Save as yaml file");
  ui::NextColumn();
  ui::PushItemWidth(-1);
  if (ui::Button("save")) {
    saveCroppingParams();
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
  if (ui::DragFloat3("##value", &Limbs::get().offset[0], 0.02f)) {
    Signal<UpdateBoxOffsetEvent>::emit({Limbs::get().offset});
  }
  ui::PopItemWidth();
  ui::NextColumn();
  ui::PopID();
  ui::Columns();

  ui::Separator();

  for (auto pair : Limbs::get().boxes) {
    bool updated = false;

    ui::Text("%s", kCroppingWindowNames[pair.first].c_str());
    ui::Columns(2);

    ui::PushID(id++);
    ui::Bullet();
    ui::TextUnformatted("size");
    ui::NextColumn();
    ui::PushItemWidth(-1);
    updated = updated || ui::DragFloat3("##value", &Limbs::get().boxes[pair.first].size[0], 0.02f);
    ui::PopItemWidth();
    ui::NextColumn();
    ui::PopID();

    ui::PushID(id++);
    ui::Bullet();
    ui::TextUnformatted("translate");
    ui::NextColumn();
    ui::PushItemWidth(-1);
    updated = updated || ui::DragFloat3("##value", &Limbs::get().boxes[pair.first].trans[0], 0.02f);
    ui::PopItemWidth();
    ui::NextColumn();
    ui::PopID();

    ui::PushID(id++);
    ui::Bullet();
    ui::TextUnformatted("rotate");
    ui::NextColumn();
    ui::PushItemWidth(-1);
    updated = updated || ui::DragFloat3("##value", &Limbs::get().boxes[pair.first].rot[0], 0.02f);
    ui::PopItemWidth();
    ui::NextColumn();
    ui::PopID();

    ui::Columns(1);

    if (updated) {
      Signal<UpdateBoxEvent>::emit({Limbs::get().boxes[pair.first]});
    }
  }

  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}

void AppGui::saveCroppingParams() {
  auto path = app_->getSaveFilePath(bpath(), {".yml", ".yaml"});
  Signal<SaveCroppingParamsEvent>::emit({path});
}

void AppGui::emitAppearanceUpdate() {
  Signal<UpdateAppearanceEvent>::emit({
    point_size_,
    slerp_t_,
    visible_current_cloud_,
    visible_next_cloud_,
    visible_transformed_cloud_,
    coloring_limbs_
  });
}
