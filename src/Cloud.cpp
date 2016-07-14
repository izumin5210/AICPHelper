//
// Created by Masayuki IZUMI on 7/14/16.
//

#include "Cloud.h"
#include "Events.h"
#include "Signal.h"

#include <pcl/io/pcd_io.h>

Cloud::Cloud(bpath dir)
  : dir_              (dir)
  , current_path_idx_ (0)
  , current_cloud_    (new PointCloud)
  , next_cloud_       (new PointCloud)
{
}

void Cloud::initialize() {
  for (auto file : boost::make_iterator_range(bditr(dir_), bditr())) {
    if (file.path().extension().string() == ".pcd") {
      paths_.push_back(file.path());
    }
  }
  pcl::io::loadPCDFile(paths_[current_path_idx_].string(), *current_cloud_);
  pcl::io::loadPCDFile(paths_[current_path_idx_ + 1].string(), *next_cloud_);
  Signal<UpdateCloudEvent>::emit({dir().string()});
}

