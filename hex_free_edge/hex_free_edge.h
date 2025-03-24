/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-14
 ****************************************************************/

#ifndef HEX_FREE_EDGE_HEX_FREE_EDGE_H_
#define HEX_FREE_EDGE_HEX_FREE_EDGE_H_

#include <vector>

#include "hex_free_edge/hex_utility.h"

using hex::utility::HexParameterFilter;
using hex::utility::HexParameterFlag;
using hex::utility::HexParameterObstacle;
using hex::utility::HexParameterSensor;
using hex::utility::HexStamp;
using hex::utility::HexStampedOdom;
using hex::utility::HexStampedPoints;
using hex::utility::HexStampedScan;

namespace hex {
namespace postprocess {

class HexFreeEdge {
 public:
  static HexFreeEdge& GetSingleton() {
    static HexFreeEdge singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  HexFreeEdge() = default;
  virtual ~HexFreeEdge() = default;

  // Work Handle
  void MapPoints();
  void MarkObstacle();
  void CreateObstacleScan();
  void CreateObstacleCloud();
  void ResetVariables();

  // Help Handle
  bool PointValid(const pcl::PointXYZ&);
  // index: (fov, distance)
  Eigen::Vector2i PointToIndex(const pcl::PointXYZ&);
  // metric: (range, max_gap), index: (fov, distance)
  Eigen::Vector2d EvaluateGrid(const Eigen::Vector2i&);

  // Parameters
  HexParameterFilter kparameter_filter_;
  HexParameterFlag kparameter_flag_;
  HexParameterObstacle kparameter_obstacle_;
  HexParameterSensor kparameter_sensor_;

  // Variables
  HexStampedPoints acc_lidar_;
  HexStampedPoints obstacle_cloud_;
  HexStampedScan obstacle_scan_;
  std::vector<int32_t> obstacle_index_;
  std::vector<std::vector<std::vector<double>>> height_map_;
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> point_map_;
};

}  // namespace postprocess
}  // namespace hex

#endif  // HEX_FREE_EDGE_HEX_FREE_EDGE_H_
