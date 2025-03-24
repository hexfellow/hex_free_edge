/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-14
 ****************************************************************/

#ifndef HEX_FREE_EDGE_LIDAR_ACCUMULATE_H_
#define HEX_FREE_EDGE_LIDAR_ACCUMULATE_H_

#include <vector>
#include <deque>

#include "hex_free_edge/hex_utility.h"

using hex::utility::HexParameterObstacle;
using hex::utility::HexParameterSensor;
using hex::utility::HexStamp;
using hex::utility::HexStampedOdom;
using hex::utility::HexStampedPoints;

namespace hex {
namespace postprocess {

class LidarAccumulate {
 public:
  static LidarAccumulate& GetSingleton() {
    static LidarAccumulate singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  void AddOdom(const std::vector<HexStampedOdom>&);
  const HexStampedPoints& AccumulateCloud(const std::vector<HexStampedPoints>&);

 private:
  LidarAccumulate() = default;
  virtual ~LidarAccumulate() = default;

  // Work Handle
  bool RemoveOldOdom(const HexStamp&);

  // Help Handle
  Eigen::Affine3d CurrentTrans(const HexStamp&);
  Eigen::Affine3d InterpolateTrans(const Eigen::Affine3d&,
                                   const Eigen::Affine3d&, double);
  Eigen::Affine3d PredictTrans(const Eigen::Affine3d&, const Eigen::Affine3d&,
                               double);

  // Parameters
  HexParameterObstacle kparameter_obstacle_;
  HexParameterSensor kparameter_sensor_;

  // Variables
  std::deque<HexStampedPoints> lidar_queue_;
  std::deque<HexStampedOdom> odom_queue_;
  HexStampedPoints acc_lidar_;
};

}  // namespace postprocess
}  // namespace hex

#endif  // HEX_FREE_EDGE_LIDAR_ACCUMULATE_H_
