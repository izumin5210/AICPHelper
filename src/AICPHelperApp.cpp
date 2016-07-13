#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class AICPHelperApp : public App {
public:
  void setup() override;
  void mouseDown(MouseEvent event) override;
  void update() override;
  void draw() override;
};

void AICPHelperApp::setup() {
}

void AICPHelperApp::mouseDown(MouseEvent event) {
}

void AICPHelperApp::update() {
}

void AICPHelperApp::draw() {
  gl::clear(Color( 0, 0, 0 ));
}

CINDER_APP( AICPHelperApp, RendererGl )
