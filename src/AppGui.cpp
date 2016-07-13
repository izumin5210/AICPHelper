//
// Created by Masayuki IZUMI on 7/13/16.
//

#include "AppGui.h"
#include "Signal.h"

#include <pcl/io/pcd_io.h>

void AppGui::initialize(cinder::app::AppBase *app) {
  app_ = app;
  ui::initialize(options);
}

void AppGui::update() {
  auto windowPos = glm::vec2(kWindowSpacing, kWindowSpacing);
  drawMenuBar(windowPos);
  drawInfoWindow(windowPos);
}

void AppGui::drawMenuBar(glm::vec2 &windowPos) {
  ui::ScopedMainMenuBar menuBar;

  if (ui::BeginMenu("File")) {
    if (ui::MenuItem("Open *.pcd file")) {
      auto pcdfile = app_->getOpenFilePath(bpath(), {"pcd"});
      if (bfs::exists(pcdfile)) {
        const PointCloudPtr cloud(new PointCloud);
        pcl::io::loadPCDFile(pcdfile.string(), *cloud);
        Signal<PointCloudPtr>::emit(cloud);
      }
    }
    ui::EndMenu();
  }

  windowPos.y += ui::GetItemRectSize().y;
}

void AppGui::drawInfoWindow(glm::vec2 &windowPos) {
  ui::ScopedWindow window("Information", kWindowFlags);
  ui::LabelText("FPS", "%f", app_->getAverageFps());
  ui::SetWindowPos(windowPos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  windowPos.y += ui::GetWindowHeight() + kWindowSpacing;
}
