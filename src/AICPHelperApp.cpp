#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/CameraUi.h"
#include "cinder/gl/gl.h"


#include <yaml-cpp/yaml.h>

#include "AppGui.h"
#include "Clouds.h"
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

  vector<Box> kDefaultBoxes = {
    { Box::Kind::HEAD,            glm::vec3(0.3, 0.3, 0.3), glm::vec3(),  glm::vec3(+0.0, 1.55, 0.0)  },
    { Box::Kind::TRUNK,           glm::vec3(0.4, 0.7, 0.3), glm::vec3(),  glm::vec3(+0.0, 1.05, 0.0)  },
    { Box::Kind::LEFT_UP_ARM,     glm::vec3(0.2, 0.3, 0.2), glm::vec3(),  glm::vec3(-0.3, 1.25, 0.0)  },
    { Box::Kind::RIGHT_UP_ARM,    glm::vec3(0.2, 0.3, 0.2), glm::vec3(),  glm::vec3(+0.3, 1.25, 0.0)  },
    { Box::Kind::LEFT_DOWN_ARM,   glm::vec3(0.2, 0.4, 0.2), glm::vec3(),  glm::vec3(-0.3, 0.90, 0.0)  },
    { Box::Kind::RIGHT_DOWN_ARM,  glm::vec3(0.2, 0.4, 0.2), glm::vec3(),  glm::vec3(+0.3, 0.90, 0.0)  },
    { Box::Kind::LEFT_UP_LEG,     glm::vec3(0.2, 0.3, 0.2), glm::vec3(),  glm::vec3(-0.1, 0.55, 0.0)  },
    { Box::Kind::RIGHT_UP_LEG,    glm::vec3(0.2, 0.3, 0.2), glm::vec3(),  glm::vec3(+0.1, 0.55, 0.0)  },
    { Box::Kind::LEFT_DOWN_LEG,   glm::vec3(0.2, 0.4, 0.2), glm::vec3(),  glm::vec3(-0.1, 0.20, 0.0)  },
    { Box::Kind::RIGHT_DOWN_LEG,  glm::vec3(0.2, 0.4, 0.2), glm::vec3(),  glm::vec3(+0.1, 0.20, 0.0)  },
  };

  CameraPersp camera_;
  CameraUi camera_ui_;

  gl::VertBatchRef grid_batch_;
  map<Box::Kind, gl::BatchRef> box_batches_;
  gl::GlslProgRef box_shader_;
  map<string, gl::VertBatchRef> cloud_batches_;

  float point_size_;
  float slerp_t_;
  bool visible_current_cloud_;
  bool visible_next_cloud_;
  bool visible_transformed_cloud_;
  bool coloring_limbs_;

  bool enable_cropping_;

  shared_ptr<Clouds> clouds_;
  AppGui gui_;


  void appearanceUpdate(const UpdateAppearanceEvent& event);
  void cloudUpdate(const UpdateCloudEvent& event);
  void boxUpdate(const UpdateBoxEvent& event);
  void enableCropping(const EnableCroppingEvent& event);
  void saveCroppingParams(const SaveCroppingParamsEvent& event);

  void updateClouds();
  void updateClouds(string key);
};

AICPHelperApp::AICPHelperApp()
  : camera_ui_      (&camera_)
  , grid_batch_     (gl::VertBatch::create(GL_LINES))
  , box_shader_     (gl::context()->getStockShader(gl::ShaderDef().color()))
  , enable_cropping_(false)
  , clouds_(new Clouds)
  , gui_(clouds_)
{}

void AICPHelperApp::setup() {
  Signal<UpdateAppearanceEvent>::connect(this, &AICPHelperApp::appearanceUpdate);
  Signal<UpdateCloudEvent>::connect(this, &AICPHelperApp::cloudUpdate);
  Signal<UpdateBoxEvent>::connect(this, &AICPHelperApp::boxUpdate);
  Signal<EnableCroppingEvent>::connect(this, &AICPHelperApp::enableCropping);
  Signal<SaveCroppingParamsEvent>::connect(this, &AICPHelperApp::saveCroppingParams);

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

  for (auto box : kDefaultBoxes) {
    Limbs::get().boxes[box.kind] = box;
    Signal<UpdateBoxEvent>::emit({box});
  }
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
  gl::pointSize(point_size_);
  gl::lineWidth(1.0f);
  gl::color(1.0f, 1.0f, 1.0f);

  grid_batch_->draw();

  gl::pushMatrices();
  gl::translate(Limbs::get().offset);
  for (auto pair : box_batches_) {
    gl::color(kColorMap[pair.first]);
    pair.second->draw();
  }
  gl::popMatrices();

  for (auto pair : cloud_batches_) {
    pair.second->draw();
  }
}

void AICPHelperApp::appearanceUpdate(const UpdateAppearanceEvent &event) {
  point_size_ = event.point_size;
  slerp_t_ = event.slerp_t;
  visible_current_cloud_ = event.visible_current_cloud;
  visible_next_cloud_ = event.visible_next_cloud;
  visible_transformed_cloud_ = event.visible_transformed_cloud;
  coloring_limbs_ = event.coloring_limbs;
  updateClouds();
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

  box_batches_[event.box.kind] = gl::Batch::create(cube >> rot_x >> rot_y >> rot_z >> trans, box_shader_);
}

void AICPHelperApp::enableCropping(const EnableCroppingEvent &event) {
  enable_cropping_ = event.enable;
  updateClouds();
}

void AICPHelperApp::saveCroppingParams(const SaveCroppingParamsEvent &event) {
//  YAML::Node root_node;
//  for (auto pair : clouds_->clouds()) {
//    YAML::Node node;
//    node["offset"].push_back(boxes_offset_.x);
//    node["offset"].push_back(boxes_offset_.y);
//    node["offset"].push_back(boxes_offset_.z);
//    for (auto pair2 : boxes_) {
//      node["boxes"][static_cast<int>(pair2.first)]["size"].push_back(pair2.second.size.x);
//      node["boxes"][static_cast<int>(pair2.first)]["size"].push_back(pair2.second.size.y);
//      node["boxes"][static_cast<int>(pair2.first)]["size"].push_back(pair2.second.size.z);
//      node["boxes"][static_cast<int>(pair2.first)]["rot"].push_back(pair2.second.rot.x);
//      node["boxes"][static_cast<int>(pair2.first)]["rot"].push_back(pair2.second.rot.y);
//      node["boxes"][static_cast<int>(pair2.first)]["rot"].push_back(pair2.second.rot.z);
//      node["boxes"][static_cast<int>(pair2.first)]["trans"].push_back(pair2.second.trans.x);
//      node["boxes"][static_cast<int>(pair2.first)]["trans"].push_back(pair2.second.trans.y);
//      node["boxes"][static_cast<int>(pair2.first)]["trans"].push_back(pair2.second.trans.z);
//    }
//    root_node[pair.first] = node;
//  }
//  std::ofstream fout(event.path.string());
//  fout << root_node;
}

void AICPHelperApp::updateClouds() {
  for (auto pair : clouds_->clouds()) {
    updateClouds(pair.first.string());
  }
}

void AICPHelperApp::updateClouds(string key) {
  if (cloud_batches_.find(key) == cloud_batches_.end()) {
    cloud_batches_[key] = gl::VertBatch::create(GL_POINTS);
  }

  cloud_batches_[key]->clear();

  auto cloud = clouds_->cloud(key);

  if (visible_current_cloud_) {
    if (visible_transformed_cloud_) {
      for (auto pair : cloud->current_limbs_clouds_transformed(slerp_t_)) {
        for (auto p : *pair.second) {
          if (coloring_limbs_) {
            cloud_batches_[key]->color(kColorMap[pair.first]);
          } else {
            cloud_batches_[key]->color(ColorA8u(p.r, p.g, p.b, p.a));
          }
          cloud_batches_[key]->vertex(vec3(p.x, p.y, p.z));
        }
      }
    } else if (enable_cropping_) {
      for (auto pair : cloud->current_limbs_clouds()) {
        for (auto p : *pair.second) {
          if (coloring_limbs_) {
            cloud_batches_[key]->color(kColorMap[pair.first]);
          } else {
            cloud_batches_[key]->color(ColorA8u(p.r, p.g, p.b, p.a));
          }
          cloud_batches_[key]->vertex(vec3(p.x, p.y, p.z));
        }
      }
    } else {
      for (auto p : *(cloud->current_cloud())) {
        cloud_batches_[key]->color(ColorA8u(p.r, p.g, p.b, p.a));
        cloud_batches_[key]->vertex(vec3(p.x, p.y, p.z));
      }
    }
  }

  if (visible_next_cloud_) {
    for (auto p : *(cloud->next_cloud())) {
      cloud_batches_[key]->color(ColorA8u(p.r, p.g, p.b, p.a));
      cloud_batches_[key]->vertex(vec3(p.x, p.y, p.z));
    }
  }
//    for (auto pair : boxes_) {
//      vector<int> indices;
//      pcl::CropBox<PointT> filter;
//      auto box = pair.second;
//      Eigen::Vector4f max(box.size.x / 2, box.size.y / 2, box.size.z / 2, 0);
//      Eigen::Vector4f min = -1 * max;
//      Eigen::Vector3f rot(box.rot.x, box.rot.y, box.rot.z);
//      Eigen::Vector3f t(box.trans.x, box.trans.y, box.trans.z);
//      Eigen::Vector3f offset(boxes_offset_.x, boxes_offset_.y, boxes_offset_.z);
//      filter.setMin(min);
//      filter.setMax(max);
//      filter.setRotation(rot);
//      filter.setTranslation(t + offset);
//      filter.setInputCloud(clouds_[key]->currentCloud());
//      filter.filter(indices);
//      limbs_indices[pair.first] = indices;
//
//      for (auto i : indices) {
//        auto p = clouds_[key]->currentCloud()->points[i];
//        cloud_batches_[key]->color(ColorA8u(p.r, p.g, p.b, p.a));
//        cloud_batches_[key]->vertex(vec3(p.x, p.y, p.z));
//      }
//    }
//  } else {
//    for (auto p : clouds_[key]->currentCloud()->points) {
//      cloud_batches_[key]->color(ColorA8u(p.r, p.g, p.b, p.a));
//      cloud_batches_[key]->vertex(vec3(p.x, p.y, p.z));
//    }
//  }
}

CINDER_APP(AICPHelperApp, RendererGl, [](App::Settings *settings) {
  settings->setHighDensityDisplayEnabled();
  settings->setWindowSize(1280, 960);
  settings->disableFrameRate();
})
