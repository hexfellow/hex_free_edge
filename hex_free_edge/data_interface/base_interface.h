/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-14
 ****************************************************************/

#ifndef HEX_FREE_EDGE_DATA_INTERFACE_BASE_INTERFACE_H_
#define HEX_FREE_EDGE_DATA_INTERFACE_BASE_INTERFACE_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
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

enum class HexLogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };

class BaseInterface {
 public:
  BaseInterface() : lidar_flag_(false), odom_flag_(false) {}
  virtual ~BaseInterface() {}
  BaseInterface(const BaseInterface&) = delete;
  BaseInterface& operator=(const BaseInterface&) = delete;

  // Interface Handle
  virtual void Log(HexLogLevel, const char*, ...) = 0;
  virtual void Shutdown() = 0;
  virtual bool Ok() = 0;
  virtual HexStamp GetTime() = 0;
  virtual void Work() = 0;

  // Publisher Handle
  virtual void PublishObstacleCloud(const HexStampedPoints&) = 0;
  virtual void PublishObstacleScan(const HexStampedScan&) = 0;

  // Initialization Handle
  virtual void Init(int, char*[], std::string, double, void (*)()) = 0;
  virtual void Deinit() = 0;

  // Parameter Handle
  inline const HexParameterFilter& GetParameterFilter() const {
    return kparameter_filter_;
  }
  inline const HexParameterFlag& GetParameterFlag() const {
    return kparameter_flag_;
  }
  inline const HexParameterObstacle& GetParameterObstacle() const {
    return kparameter_obstacle_;
  }
  inline const HexParameterSensor& GetParameterSensor() const {
    return kparameter_sensor_;
  }

  // Subscriber Handle
  inline bool GetLidarFlag() { return lidar_flag_; }
  inline void ResetLidarFlag() { lidar_flag_ = false; }
  inline const std::vector<HexStampedPoints>& GetLidarBuffer() {
    static std::vector<HexStampedPoints> lidar_buffer;
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    lidar_buffer = std::vector<HexStampedPoints>(lidar_buffer_);
    lidar_buffer_.clear();
    return lidar_buffer;
  }
  inline void ResetLidarBuffer() { lidar_buffer_.clear(); }
  inline bool GetOdomFlag() { return odom_flag_; }
  inline void ResetOdomFlag() { odom_flag_ = false; }
  inline const std::vector<HexStampedOdom>& GetOdomBuffer() {
    static std::vector<HexStampedOdom> odom_buffer;
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer = std::vector<HexStampedOdom>(odom_buffer_);
    odom_buffer_.clear();
    return odom_buffer;
  }

 protected:
  // Initialization Handle
  virtual void ParameterInit() = 0;
  virtual void VariableInit() = 0;
  virtual void PublisherInit() = 0;
  virtual void SubscriberInit() = 0;
  virtual void TimerInit(double, void (*)()) = 0;

  // Timer Handle
  void (*timer_handle_)();

  // Parameters
  HexParameterSensor kparameter_sensor_;
  HexParameterFilter kparameter_filter_;
  HexParameterObstacle kparameter_obstacle_;
  HexParameterFlag kparameter_flag_;

  // Variables
  std::atomic<bool> lidar_flag_;
  mutable std::mutex lidar_mutex_;
  std::vector<HexStampedPoints> lidar_buffer_;
  std::atomic<bool> odom_flag_;
  mutable std::mutex odom_mutex_;
  std::vector<HexStampedOdom> odom_buffer_;
};

}  // namespace postprocess
}  // namespace hex

#endif  // HEX_FREE_EDGE_DATA_INTERFACE_BASE_INTERFACE_H_
