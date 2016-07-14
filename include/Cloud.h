//
// Created by Masayuki IZUMI on 7/14/16.
//

#ifndef AICPHELPER_CLOUD_H
#define AICPHELPER_CLOUD_H

#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <map>

namespace bfs       = boost::filesystem;
using bpath         = bfs::path;
using bditr         = bfs::directory_iterator;

using PointT        = pcl::PointXYZRGBA;
using PointCloud    = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;

class Cloud {
public:
  struct LimbIndices {
    std::vector<int> head;
    std::vector<int> trunk;

    std::vector<int> left_up_arm;
    std::vector<int> right_up_arm;

    std::vector<int> left_down_arm;
    std::vector<int> right_down_arm;

    std::vector<int> left_up_leg;
    std::vector<int> right_up_leg;

    std::vector<int> left_down_leg;
    std::vector<int> right_down_leg;
  };

  Cloud(bpath dir);

  void initialize();
  bool nextFrame();

  inline PointCloudPtr currentCloud() {
    return current_cloud_;
  }

  inline PointCloudPtr nextCloud() {
    return next_cloud_;
  }

  inline bpath dir() {
    return dir_;
  }

  inline bpath current_path() {
    return paths_[current_path_idx_];
  }


private:
  const bpath dir_;
  std::vector<bpath> paths_;
  int current_path_idx_;

  std::map<bpath, LimbIndices> limbs_;

  PointCloudPtr current_cloud_;
  PointCloudPtr next_cloud_;
};

#endif //AICPHELPER_CLOUD_H
