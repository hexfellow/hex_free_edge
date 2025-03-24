/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-14
 ****************************************************************/

#ifndef HEX_FREE_EDGE_DATA_INTERFACE_ROS1_INTERFACE_H_
#define HEX_FREE_EDGE_DATA_INTERFACE_ROS1_INTERFACE_H_

#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <string>

#include "hex_free_edge/data_interface/base_interface.h"
#include "hex_free_edge/hex_utility.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

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

class DataInterface : public BaseInterface {
 public:
  static DataInterface& GetSingleton() {
    static DataInterface singleton;
    return singleton;
  }

  // Interface Handle
  void Log(HexLogLevel, const char*, ...) override;
  inline void Shutdown() override { ros::shutdown(); }
  inline bool Ok() override { return ros::ok(); }
  inline HexStamp GetTime() override {
    ros::Time ros_time = ros::Time::now();
    return HexStamp(ros_time.sec, ros_time.nsec);
  }
  inline void Work() override {
    timer_->reset();
    while (ros::ok()) {
      timer_handle_();
      timer_->sleep();
    }
  }

  // Publisher Handle
  void PublishObstacleCloud(const HexStampedPoints&) override;
  void PublishObstacleScan(const HexStampedScan&) override;

  // Initialization Handle
  void Init(int, char*[], std::string, double, void (*)()) override;
  void Deinit() override;

 protected:
  // Subscriber Handle
  void LidarHandle(const sensor_msgs::PointCloud2Ptr&);
  void OdomHandle(const nav_msgs::OdometryPtr&);

 private:
  DataInterface() = default;
  virtual ~DataInterface() = default;

  // Initialization Handle
  void ParameterInit() override;
  void VariableInit() override;
  void PublisherInit() override;
  void SubscriberInit() override;
  void TimerInit(double, void (*)()) override;

  // Node Handle
  ros::NodeHandle* nh_ptr_;
  ros::NodeHandle* nh_local_ptr_;

  // Timer Handle
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  std::unique_ptr<ros::Rate> timer_;

  // Publisher Handle
  ros::Publisher obstacle_scan_pub_;
  ros::Publisher obstacle_cloud_pub_;

  // Subscriber Handle
  ros::Subscriber lidar_sub_;
  ros::Subscriber odom_sub_;
};

}  // namespace postprocess
}  // namespace hex

#endif  // HEX_FREE_EDGE_DATA_INTERFACE_ROS1_INTERFACE_H_
