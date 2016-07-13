#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/CameraUi.h"
#include "cinder/gl/gl.h"

#include "AppGui.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class AICPHelperApp : public App {
public:
  AICPHelperApp();

  void setup() override;
  void mouseDown(MouseEvent event) override;
  void mouseDrag(MouseEvent event) override;
  void mouseWheel(MouseEvent event) override;
  void update() override;
  void draw() override;


private:
  CameraPersp camera_;
  CameraUi camera_ui_;

  gl::VertBatchRef grid_batch_;

  AppGui gui_;
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

  gui_.initialize(get());

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
  gui_.update();
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
