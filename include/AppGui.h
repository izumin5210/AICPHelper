//
// Created by Masayuki IZUMI on 7/13/16.
//

#ifndef AICPHELPER_APPGUI_H
#define AICPHELPER_APPGUI_H

#include "cinder/app/App.h"

#include "CinderImGui.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Box.h"
#include "Clouds.h"

namespace bfs = boost::filesystem;

class AppGui {
public:
  using bpath         = bfs::path;
  using PointT        = pcl::PointXYZRGBA;
  using PointCloud    = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;

  AppGui(std::shared_ptr<Clouds> clouds);
  void initialize(cinder::app::AppBase *app);
  void update();

private:
  const ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_ShowBorders;
  const int kWindowSpacing  = 8;
  const int kWindowWidth    = 320;

  std::map<Box::Kind, std::string> kCroppingWindowNames = {
    { Box::Kind::HEAD,            "Head" },
    { Box::Kind::TRUNK,           "Trunk" },
    { Box::Kind::LEFT_UP_ARM,     "Left-up arm" },
    { Box::Kind::RIGHT_UP_ARM,    "Right-up arm" },
    { Box::Kind::LEFT_DOWN_ARM,   "Left-down arm" },
    { Box::Kind::RIGHT_DOWN_ARM,  "Right-down arm" },
    { Box::Kind::LEFT_UP_LEG,     "Left-up leg" },
    { Box::Kind::RIGHT_UP_LEG,    "Right-up leg" },
    { Box::Kind::LEFT_DOWN_LEG,   "Left-down leg" },
    { Box::Kind::RIGHT_DOWN_LEG,  "Right-down leg" },
  };

  // bleu de provence
  const ci::ColorA8u kColorPrimary    = ci::ColorA8u(0x00, 0x9a, 0xc5, 0xcc);
  const ci::ColorA8u kColorPrimaryA99 = ci::ColorA8u(0x00, 0x9a, 0xc5, 0x99);
  const ci::ColorA8u kColorPrimaryA33 = ci::ColorA8u(0x00, 0x9a, 0xc5, 0x33);
  const ci::ColorA8u kColorPrimaryDark = ci::ColorA8u(0x00, 0x27, 0x33, 0xbb);
  // rosso di toscana
  const ci::ColorA8u kColorAccent     = ci::ColorA8u(0xf1, 0x67, 0x3f, 0xee);
  const ci::ColorA8u kColorAccentAcc  = ci::ColorA8u(0xf1, 0x67, 0x3f, 0xcc);
  const ci::ColorA8u kColorAccentA99  = ci::ColorA8u(0xf1, 0x67, 0x3f, 0x99);

  const ci::ColorA8u kColorWhite      = ci::ColorA8u(0xdd, 0xdd, 0xdd, 0xcc);
  const ci::ColorA8u kColorBlackA55   = ci::ColorA8u(0x11, 0x11, 0x11, 0x55);

  const ui::Options kUiOptions = ui::Options()
    .darkTheme()
    .color(ImGuiCol_MenuBarBg,              kColorPrimaryA33)
    .color(ImGuiCol_TitleBg,                kColorPrimaryDark)
    .color(ImGuiCol_TitleBgCollapsed,       kColorPrimaryDark)
    .color(ImGuiCol_TitleBgActive,          kColorPrimaryA99)
    .color(ImGuiCol_WindowBg,               kColorPrimaryDark)
    .color(ImGuiCol_Border,                 kColorPrimaryA99)
    .color(ImGuiCol_FrameBg,                kColorPrimaryA33)
    .color(ImGuiCol_FrameBgHovered,         kColorAccentAcc)
    .color(ImGuiCol_FrameBgActive,          kColorAccent)
    .color(ImGuiCol_ScrollbarBg,            kColorPrimaryA33)
    .color(ImGuiCol_ScrollbarGrab,          kColorPrimaryA99)
    .color(ImGuiCol_ScrollbarGrabHovered,   kColorPrimaryA99)
    .color(ImGuiCol_ScrollbarGrabActive,    kColorPrimary)
    .color(ImGuiCol_CheckMark,              kColorAccent)
    .color(ImGuiCol_SliderGrab,             kColorPrimaryA99)
    .color(ImGuiCol_SliderGrabActive,       kColorPrimary)
    .color(ImGuiCol_Button,                 kColorPrimaryA33)
    .color(ImGuiCol_ButtonHovered,          kColorAccentAcc)
    .color(ImGuiCol_ButtonActive,           kColorAccent)
    .color(ImGuiCol_Header,                 kColorAccentA99)
    .color(ImGuiCol_HeaderHovered,          kColorAccentAcc)
    .color(ImGuiCol_HeaderActive,           kColorAccent)
    .color(ImGuiCol_Column,                 kColorBlackA55)
    .color(ImGuiCol_ColumnHovered,          kColorAccentAcc)
    .color(ImGuiCol_ColumnActive,           kColorAccent)
    .color(ImGuiCol_PlotLines,              kColorPrimaryA99)
    .color(ImGuiCol_PlotLinesHovered,       kColorPrimary)
    .color(ImGuiCol_PlotHistogram,          kColorPrimaryA99)
    .color(ImGuiCol_PlotHistogramHovered,   kColorPrimary)
    .color(ImGuiCol_Text,                   kColorPrimary)
    .color(ImGuiCol_TextDisabled,           kColorPrimaryA99)
    .color(ImGuiCol_TextSelectedBg,         kColorAccent)
    .color(ImGuiCol_PopupBg,                kColorPrimaryDark)
    .antiAliasedLines(true)
    .antiAliasedShapes(true)
    .windowRounding(0.0f)
    .frameRounding(0.0f);

  std::shared_ptr<Clouds> clouds_;
  cinder::app::AppBase* app_;

  float point_size_;
  float slerp_t_;
  bool visible_current_cloud_;
  bool visible_next_cloud_;
  bool visible_transformed_cloud_;
  bool coloring_limbs_;

  bool enable_cropping_;

  void drawMenuBar(glm::vec2 &left_window_pos, glm::vec2 &right_window_pos);
  void drawCloudsWindow(glm::vec2 &window_pos);
  void drawTransformationWindow(glm::vec2 &window_pos);
  void drawInfoWindow(glm::vec2 &window_pos);
  void drawCroppingWindow(glm::vec2 &window_pos);
  void saveCroppingParams();

  void emitAppearanceUpdate();
};

#endif //AICPHELPER_APPGUI_H
