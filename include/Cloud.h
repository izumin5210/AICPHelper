//
// Created by Masayuki IZUMI on 7/25/16.
//

#ifndef AICPHELPER_CLOUD_H
#define AICPHELPER_CLOUD_H

#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>

#include "Box.h"
#include "Events.h"
#include "Signal.h"

class Cloud {
public:
  using path          = boost::filesystem::path;
  using ditr          = boost::filesystem::directory_iterator;
  using PointT        = pcl::PointXYZRGBA;
  using PointCloud    = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;
  using Key           = path;

  Cloud(const path dir)
    : dir_            (dir)
    , current_cloud_  (new PointCloud)
    , next_cloud_     (new PointCloud)
    , transformed_    (false)
  {
    initialize();
  }

  inline path dir() const {
    return dir_;
  }

  inline std::vector<path> paths() const {
    return paths_;
  }

  inline PointCloudPtr current_cloud() const {
    return current_cloud_;
  }

  inline PointCloudPtr next_cloud() const {
    return next_cloud_;
  }

  inline bool has_transformed() const {
    return transformed_;
  }

  inline path current_path() const {
    return *current_path_itr_;
  }

  inline bool is_visible(Key key) const {
    return visible_clouds_.find(key) != visible_clouds_.end();
  }

  inline bool is_current(Key key) const {
    return *current_path_itr_ == key;
  }

  void emitUpdateEvent() {
    Signal<UpdateCloudEvent>::emit({dir_.string()});
  }

  void updateBox(const Box box) {
    limbs_[current_path()].boxes[box.kind] = box;
    filter();
  }

  void updateBoxOffset(const glm::vec3 offset) {
    limbs_[current_path()].offset = offset;
    filter();
  }

  std::map<Box::Kind, PointCloudPtr> current_limbs_clouds() const {
    std::map<Box::Kind, PointCloudPtr> map;
    for (auto pair : limbs_indices_) {
      PointCloudPtr cloud(new PointCloud);
      pcl::copyPointCloud(*current_cloud(), pair.second, *cloud);
      map[pair.first] = cloud;
    }
    return map;
  }

  std::map<Box::Kind, PointCloudPtr> current_limbs_clouds_transformed(float t) {
    std::map<Box::Kind, PointCloudPtr> map;
    Eigen::Quaterniond quat_base;
    for (auto pair : current_limbs_clouds()) {
      PointCloudPtr cloud_transformed(new PointCloud);
      auto mat = transform_matrices_[pair.first];
      Eigen::Matrix3d rot = mat.block(0, 0, 3, 3);
      Eigen::Quaterniond quat_target(rot);
      mat = mat * t;
      mat.block(0, 0, 3, 3) = quat_base.slerp(t, quat_target).toRotationMatrix();
      pcl::transformPointCloud(*pair.second, *cloud_transformed, mat);
      map[pair.first] = cloud_transformed;
    }
    return map;
  }

  void transform(int max_iterations) {
    for (auto pair : current_limbs_clouds()) {
      pcl::IterativeClosestPoint<PointT, PointT> icp;
      PointCloudPtr cloud(new PointCloud);
      icp.setInputCloud(pair.second);
      icp.setInputTarget(next_cloud());
      icp.setMaximumIterations(max_iterations);
      icp.align(*cloud);
      auto matrix = icp.getFinalTransformation().cast<double>();
      transform_matrices_[pair.first] = matrix;
      printf("Input cloud :\n%zu points\n", pair.second->size());
      printf("Target cloud :\n%zu points\n", next_cloud()->size());
      printf("Rotation matrix :\n");
      printf("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
      printf("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
      printf("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
      printf("Translation vector :\n");
      printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    }
    transformed_ = true;
  }


private:
  const path dir_;
  std::vector<Key> paths_;

  std::vector<Key>::iterator current_path_itr_;
  std::set<Key> visible_clouds_;

  PointCloudPtr current_cloud_;
  PointCloudPtr next_cloud_;

  std::map<Key, Limbs> limbs_;
  std::map<Box::Kind, std::vector<int>> limbs_indices_;

  std::map<Box::Kind, Eigen::Matrix4d> transform_matrices_;
  bool transformed_;

  void initialize() {
    for (auto file : boost::make_iterator_range(ditr(dir_), ditr())) {
      paths_.push_back(file.path());
    }
    current_path_itr_ = paths_.end();
    next();
  }

  void next() {
    if (current_path_itr_ == paths_.end()) {
      current_path_itr_ = paths_.begin();
    } else {
      current_path_itr_++;
    }
    pcl::io::loadPCDFile((*current_path_itr_).string(), *current_cloud_);
    limbs_[current_path()] = Limbs::get();
    if ((current_path_itr_ + 1) == paths_.end()) {
      pcl::io::loadPCDFile(paths_[0].string(), *next_cloud_);
    } else {
      pcl::io::loadPCDFile((*(current_path_itr_ + 1)).string(), *next_cloud_);
    }
    transformed_ = false;
  }

  void filter() {
    auto limbs = limbs_[current_path()];
    auto offset = limbs.offset;
    for (auto pair : limbs.boxes) {
      std::vector<int> indices;
      pcl::CropBox<PointT> filter;
      auto box = pair.second;
      Eigen::Vector4f max(box.size.x / 2, box.size.y / 2, box.size.z / 2, 0);
      Eigen::Vector4f min = -1 * max;
      Eigen::Vector3f rot(box.rot.x, box.rot.y, box.rot.z);
      Eigen::Vector3f t(box.trans.x + offset.x, box.trans.y + offset.y, box.trans.z + offset.z);
      filter.setMin(min);
      filter.setMax(max);
      filter.setRotation(rot);
      filter.setTranslation(t);
      filter.setInputCloud(current_cloud());
      filter.filter(indices);
      limbs_indices_[pair.first] = indices;
    }
    transformed_ = false;
    emitUpdateEvent();
  }
};

#endif //AICPHELPER_CLOUD_H
