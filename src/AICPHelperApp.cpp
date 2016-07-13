#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/CameraUi.h"
#include "cinder/gl/gl.h"

#include "CinderImGui.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace ci;
using namespace ci::app;
using namespace std;

class AICPHelperApp : public App {
public:
  using PointT        = pcl::PointXYZRGBA;
  using PointCloud    = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;

  AICPHelperApp();

  void setup() override;
  void mouseDown(MouseEvent event) override;
  void mouseDrag(MouseEvent event) override;
  void mouseWheel(MouseEvent event) override;
  void update() override;
  void draw() override;


private:
  const ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_ShowBorders;
  const int kWindowSpacing = 8;
  const int kWindowWidth = 320;

  const ColorA8u kColorBlackA55   = ColorA8u(0x22, 0x22, 0x22, 0x55);
  const ColorA8u kColorBlackAcc   = ColorA8u(0x22, 0x22, 0x22, 0xcc);
  // bleu de provence
  const ColorA8u kColorPrimary    = ColorA8u(0x00, 0x9a, 0xc5, 0xcc);
  const ColorA8u kColorPrimaryA99 = ColorA8u(0x00, 0x9a, 0xc5, 0x99);
  const ColorA8u kColorPrimaryA22 = ColorA8u(0x00, 0x9a, 0xc5, 0x22);
  // rosso di toscana
  const ColorA8u kColorAccent     = ColorA8u(0xf1, 0x67, 0x3f, 0xee);
  const ColorA8u kColorAccentAcc  = ColorA8u(0xf1, 0x67, 0x3f, 0xcc);
  const ColorA8u kColorAccentA99  = ColorA8u(0xf1, 0x67, 0x3f, 0x99);

  CameraPersp camera_;
  CameraUi camera_ui_;

  gl::VertBatchRef grid_batch_;
};

AICPHelperApp::AICPHelperApp()
  : camera_ui_  (&camera_)
  , grid_batch_ (gl::VertBatch::create(GL_LINES))
{}

void AICPHelperApp::setup() {
  grid_batch_->color(1, 1, 1, 0.3);
  for (float i = -5; i <= 5.0; i += 0.5) {
    for (float j = -5; j <= 5.0; j += 0.5) {
      grid_batch_->vertex(vec3( i, 0, -j));
      grid_batch_->vertex(vec3( i, 0,  j));
      grid_batch_->vertex(vec3(-i, 0,  j));
      grid_batch_->vertex(vec3( i, 0,  j));
    }
  }

  camera_.setEyePoint(vec3(4, 2, -4));
  camera_.lookAt(vec3(0, 0.5, 0));

  auto options = ui::Options()
    .darkTheme()
    .color(ImGuiCol_MenuBarBg,              kColorPrimaryA22)
    .color(ImGuiCol_TitleBg,                kColorPrimaryA22)
    .color(ImGuiCol_TitleBgCollapsed,       kColorPrimaryA22)
    .color(ImGuiCol_TitleBgActive,          kColorPrimaryA99)
    .color(ImGuiCol_WindowBg,               kColorPrimaryA22)
    .color(ImGuiCol_Border,                 kColorPrimaryA99)
    .color(ImGuiCol_FrameBg,                kColorPrimaryA22)
    .color(ImGuiCol_FrameBgHovered,         kColorAccentAcc)
    .color(ImGuiCol_FrameBgActive,          kColorAccent)
    .color(ImGuiCol_ScrollbarBg,            kColorPrimaryA22)
    .color(ImGuiCol_ScrollbarGrab,          kColorPrimaryA99)
    .color(ImGuiCol_ScrollbarGrabHovered,   kColorPrimaryA99)
    .color(ImGuiCol_ScrollbarGrabActive,    kColorPrimary)
    .color(ImGuiCol_CheckMark,              kColorAccent)
    .color(ImGuiCol_SliderGrab,             kColorPrimaryA99)
    .color(ImGuiCol_SliderGrabActive,       kColorPrimary)
    .color(ImGuiCol_Button,                 kColorPrimaryA22)
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
    .color(ImGuiCol_TextDisabled,           kColorBlackA55)
    .color(ImGuiCol_TextSelectedBg,         kColorAccent)
    .color(ImGuiCol_PopupBg,                kColorBlackAcc)
    .antiAliasedLines(true)
    .antiAliasedShapes(true)
    .windowRounding(0.0f)
    .frameRounding(0.0f);
  ui::initialize(options);

  gl::enableFaceCulling(true);
  gl::enableVerticalSync(false);
  gl::enableDepthRead();
  gl::enableDepthWrite();
}


void AICPHelperApp::mouseDown(MouseEvent event) {
  camera_ui_.mouseDown(event);
}

void AICPHelperApp::mouseDrag(MouseEvent event) {
  camera_ui_.mouseDrag(event);
}

void AICPHelperApp::mouseWheel(MouseEvent event) {
  camera_ui_.mouseWheel(event);
}

void AICPHelperApp::update() {
  auto windowPos = vec2(kWindowSpacing, kWindowSpacing);
  {
    ui::ScopedWindow window("Information", kWindowFlags);
    ui::LabelText("FPS", "%f", getAverageFps());
    ui::SetWindowPos(windowPos);
    ui::SetWindowSize(vec2(kWindowWidth, 0));
    windowPos.y += ui::GetWindowHeight() + kWindowSpacing;
  }
}

void AICPHelperApp::draw() {
  gl::clear(Color( 0, 0, 0 ));
  gl::setMatrices(camera_);
  grid_batch_->draw();
}

CINDER_APP(AICPHelperApp, RendererGl, [](App::Settings *settings) {
  settings->setHighDensityDisplayEnabled();
  settings->setWindowSize(1280, 960);
  settings->disableFrameRate();
})
