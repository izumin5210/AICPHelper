#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/CameraUi.h"
#include "cinder/gl/gl.h"

#include <pcl/filters/crop_box.h>

#include "AppGui.h"
#include "Cloud.h"
#include "Events.h"
#include "Signal.h"

using namespace ci;
using namespace ci::app;
using namespace std;

namespace bfs = boost::filesystem;

class AICPHelperApp : public App {
public:
  using bpath         = bfs::path;
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
  map<Box::Kind, ColorA8u> kColorMap = {
    { Box::Kind::HEAD,            ColorA8u(0xcc, 0x66, 0x66, 0xcc) },
    { Box::Kind::TRUNK,           ColorA8u(0x33, 0x33, 0xcc, 0xcc) },
    { Box::Kind::LEFT_UP_ARM,     ColorA8u(0xcc, 0xcc, 0x99, 0xcc) },
    { Box::Kind::RIGHT_UP_ARM,    ColorA8u(0xff, 0x66, 0xcc, 0xcc) },
    { Box::Kind::LEFT_DOWN_ARM,   ColorA8u(0x33, 0x99, 0xff, 0xcc) },
    { Box::Kind::RIGHT_DOWN_ARM,  ColorA8u(0x66, 0xcc, 0x66, 0xcc) },
    { Box::Kind::LEFT_UP_LEG,     ColorA8u(0xcc, 0xcc, 0x99, 0xcc) },
    { Box::Kind::RIGHT_UP_LEG,    ColorA8u(0xff, 0x66, 0xcc, 0xcc) },
    { Box::Kind::LEFT_DOWN_LEG,   ColorA8u(0x33, 0x99, 0xff, 0xcc) },
    { Box::Kind::RIGHT_DOWN_LEG,  ColorA8u(0x66, 0xcc, 0x66, 0xcc) }
  };

  CameraPersp camera_;
  CameraUi camera_ui_;

  gl::VertBatchRef grid_batch_;
  map<Box::Kind, gl::BatchRef> box_batches_;
  gl::GlslProgRef box_shader_;
  map<string, shared_ptr<Cloud>> clouds_;
  map<string, gl::VertBatchRef> cloud_batches_;
  map<Box::Kind, Box> boxes_;
  map<Box::Kind, vector<int>> limbs_indices;

  vec3 boxes_offset_;
  bool enable_cropping_;

  AppGui gui_;


  void directoryOpen(const OpenDirectoryEvent& event);
  void cloudUpdate(const UpdateCloudEvent& event);
  void boxUpdate(const UpdateBoxEvent& event);
  void boxOffsetUpdate(const UpdateBoxOffsetEvent& event);
  void enableCropping(const EnableCroppingEvent& event);

  void updateClouds();
  void updateClouds(string key);
};

AICPHelperApp::AICPHelperApp()
  : camera_ui_      (&camera_)
  , grid_batch_     (gl::VertBatch::create(GL_LINES))
  , box_shader_     (gl::context()->getStockShader(gl::ShaderDef().color()))
  , enable_cropping_(false)
{}

void AICPHelperApp::setup() {
  Signal<OpenDirectoryEvent>::connect(this, &AICPHelperApp::directoryOpen);
  Signal<UpdateCloudEvent>::connect(this, &AICPHelperApp::cloudUpdate);
  Signal<UpdateBoxEvent>::connect(this, &AICPHelperApp::boxUpdate);
  Signal<UpdateBoxOffsetEvent>::connect(this, &AICPHelperApp::boxOffsetUpdate);
  Signal<EnableCroppingEvent>::connect(this, &AICPHelperApp::enableCropping);

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

  camera_.getViewMatrix();
  camera_.getProjectionMatrix();
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
  gl::pointSize(1.0f);
  gl::lineWidth(1.0f);
  gl::color(1.0f, 1.0f, 1.0f);

  grid_batch_->draw();

  gl::pushMatrices();
  gl::translate(boxes_offset_);
  for (auto pair : box_batches_) {
    gl::color(kColorMap[pair.first]);
    pair.second->draw();
  }
  gl::popMatrices();

  for (auto pair : cloud_batches_) {
    pair.second->draw();
  }
}

void AICPHelperApp::directoryOpen(const OpenDirectoryEvent& event) {
  auto cloud = make_shared<Cloud>(event.path);
  clouds_[event.path.string()] = cloud;
  cloud->initialize();
}

void AICPHelperApp::cloudUpdate(const UpdateCloudEvent& event) {
  updateClouds(event.key);
}

void AICPHelperApp::boxUpdate(const UpdateBoxEvent &event) {
  auto cube = geom::WireCube().size(event.box.size);
  auto size = geom::Scale(event.box.size);
  auto trans = geom::Translate(event.box.trans);
  auto rot_x = geom::Rotate(event.box.rot.x, vec3(1, 0, 0));
  auto rot_y = geom::Rotate(event.box.rot.y, vec3(0, 1, 0));
  auto rot_z = geom::Rotate(event.box.rot.z, vec3(0, 0, 1));

  boxes_[event.box.kind] = event.box;

  if (enable_cropping_) {
    updateClouds();
  }

  box_batches_[event.box.kind] = gl::Batch::create(cube >> rot_x >> rot_y >> rot_z >> trans, box_shader_);
}

void AICPHelperApp::boxOffsetUpdate(const UpdateBoxOffsetEvent &event) {
  boxes_offset_ = event.offset;
}

void AICPHelperApp::enableCropping(const EnableCroppingEvent &event) {
  enable_cropping_ = event.enable;
  updateClouds();
}

void AICPHelperApp::updateClouds() {
  for (auto pair : clouds_) {
    updateClouds(pair.first);
  }
}

void AICPHelperApp::updateClouds(string key) {
  if (cloud_batches_.find(key) == cloud_batches_.end()) {
    cloud_batches_[key] = gl::VertBatch::create(GL_POINTS);
  }

  cloud_batches_[key]->clear();

  if (enable_cropping_) {
    for (auto pair : boxes_) {
      vector<int> indices;
      pcl::CropBox<PointT> filter;
      auto box = pair.second;
      Eigen::Vector4f max(box.size.x / 2, box.size.y / 2, box.size.z / 2, 0);
      Eigen::Vector4f min = -1 * max;
      Eigen::Vector3f rot(box.rot.x, box.rot.y, box.rot.z);
      Eigen::Vector3f t(box.trans.x, box.trans.y, box.trans.z);
      Eigen::Vector3f offset(boxes_offset_.x, boxes_offset_.y, boxes_offset_.z);
      filter.setMin(min);
      filter.setMax(max);
      filter.setRotation(rot);
      filter.setTranslation(t + offset);
      filter.setInputCloud(clouds_[key]->currentCloud());
      filter.filter(indices);
      limbs_indices[pair.first] = indices;

      for (auto i : indices) {
        auto p = clouds_[key]->currentCloud()->points[i];
        cloud_batches_[key]->color(ColorA8u(p.r, p.g, p.b, p.a));
        cloud_batches_[key]->vertex(vec3(p.x, p.y, p.z));
      }
    }
  } else {
    for (auto p : clouds_[key]->currentCloud()->points) {
      cloud_batches_[key]->color(ColorA8u(p.r, p.g, p.b, p.a));
      cloud_batches_[key]->vertex(vec3(p.x, p.y, p.z));
    }
  }
}

CINDER_APP(AICPHelperApp, RendererGl, [](App::Settings *settings) {
  settings->setHighDensityDisplayEnabled();
  settings->setWindowSize(1280, 960);
  settings->disableFrameRate();
})
