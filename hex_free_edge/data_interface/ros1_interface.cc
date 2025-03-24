/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-15
 ****************************************************************/

#include "hex_free_edge/data_interface/ros1_interface.h"

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>

namespace hex {
namespace postprocess {

void DataInterface::Log(HexLogLevel level, const char* format, ...) {
  char* buffer;

  va_list args;
  va_start(args, format);
  int32_t len = vasprintf(&buffer, format, args);
  va_end(args);

  if (len < 0) {
    ROS_FATAL("### Wrong Log Message ###");
    return;
  }

  switch (level) {
    case HexLogLevel::kDebug: {
      ROS_DEBUG("%s", buffer);
      break;
    }
    case HexLogLevel::kInfo: {
      ROS_INFO("%s", buffer);
      break;
    }
    case HexLogLevel::kWarn: {
      ROS_WARN("%s", buffer);
      break;
    }
    case HexLogLevel::kError: {
      ROS_ERROR("%s", buffer);
      break;
    }
    case HexLogLevel::kFatal: {
      ROS_FATAL("%s", buffer);
      break;
    }
    default: {
      ROS_FATAL("### Wrong Log Level ###");
      ROS_FATAL("%s", buffer);
      break;
    }
  }

  free(buffer);
}

void DataInterface::Init(int argc, char* argv[], std::string name,
                         double period, void (*handle)()) {
  ros::init(argc, argv, name);
  static ros::NodeHandle nh;
  static ros::NodeHandle nh_local("~");
  nh_ptr_ = &nh;
  nh_local_ptr_ = &nh_local;

  ParameterInit();
  VariableInit();
  PublisherInit();
  SubscriberInit();
  TimerInit(period, handle);

  Log(HexLogLevel::kInfo,
      "\033[1;32m %s: ### data interface init finish ### \033[0m", name.data());
}

void DataInterface::Deinit() {
  spinner_->stop();
  ros::shutdown();
  spinner_.reset();
  timer_.reset();
}

void DataInterface::ParameterInit() {
  // filter parameter
  std::vector<double> param_filter_height;
  std::vector<double> param_filter_fov;
  std::vector<double> param_filter_distance;
  nh_local_ptr_->param<std::vector<double>>(
      "filter_height", param_filter_height, std::vector<double>({-1.0, 1.5}));
  nh_local_ptr_->param<std::vector<double>>("filter_fov", param_filter_fov,
                                            std::vector<double>({-M_PI, M_PI}));
  nh_local_ptr_->param<std::vector<double>>("filter_distance",
                                            param_filter_distance,
                                            std::vector<double>({0.5, 10.0}));
  nh_local_ptr_->param<int32_t>("filter_fov_num", kparameter_filter_.fov_num,
                                120);
  nh_local_ptr_->param<int32_t>("filter_distance_num",
                                kparameter_filter_.distance_num, 200);
  kparameter_filter_.height =
      Eigen::Vector2d(param_filter_height[0], param_filter_height[1]);
  kparameter_filter_.fov =
      Eigen::Vector2d(param_filter_fov[0], param_filter_fov[1]);
  kparameter_filter_.distance =
      Eigen::Vector2d(param_filter_distance[0], param_filter_distance[1]);
  kparameter_filter_.fov_step =
      (kparameter_filter_.fov[1] - kparameter_filter_.fov[0]) /
      (kparameter_filter_.fov_num - 1);
  kparameter_filter_.distance_step =
      (kparameter_filter_.distance[1] - kparameter_filter_.distance[0]) /
      (kparameter_filter_.distance_num - 1);

  // flag parameter
  nh_local_ptr_->param<bool>("flag_pub_cloud", kparameter_flag_.pub_cloud,
                             false);

  // obstacle parameter
  nh_local_ptr_->param<int32_t>("obstacle_cloud_num",
                                kparameter_obstacle_.cloud_num, 3);
  nh_local_ptr_->param<double>("obstacle_change_distance",
                               kparameter_obstacle_.change_distance, 0.1);
  nh_local_ptr_->param<double>("obstacle_far_range",
                               kparameter_obstacle_.far_range, 0.03);
  nh_local_ptr_->param<double>("obstacle_far_gap", kparameter_obstacle_.far_gap,
                               0.0);
  nh_local_ptr_->param<double>("obstacle_near_constant",
                               kparameter_obstacle_.near_constant, 0.35);
  nh_local_ptr_->param<double>("obstacle_near_factor",
                               kparameter_obstacle_.near_factor, 0.025);

  // sensor parameter
  std::vector<double> parameter_sensor_orientation;
  std::vector<double> parameter_sensor_sensor_in_base;
  nh_local_ptr_->param<std::string>("sensor_frame", kparameter_sensor_.frame,
                                    "rslidar");
  nh_local_ptr_->param<std::string>(
      "sensor_scan_frame", kparameter_sensor_.scan_frame, "rslidar_scan");
  nh_local_ptr_->param<int32_t>("sensor_scan_repeat",
                                kparameter_sensor_.repeat_times, 5);
  nh_local_ptr_->param<std::vector<double>>(
      "sensor_orientation", parameter_sensor_orientation,
      std::vector<double>({1.0, 0.0, 0.0, 0.0}));
  nh_local_ptr_->param<std::vector<double>>(
      "sensor_sensor_in_base", parameter_sensor_sensor_in_base,
      std::vector<double>({0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}));
  nh_local_ptr_->param<double>("sensor_lidar_period",
                               kparameter_sensor_.lidar_period, 0.1);
  nh_local_ptr_->param<double>("sensor_odom_period",
                               kparameter_sensor_.odom_period, 0.1);
  kparameter_sensor_.angle_increment =
      kparameter_filter_.fov_step / kparameter_sensor_.repeat_times;
  kparameter_sensor_.orientation.matrix().block<3, 3>(0, 0) =
      Eigen::Quaterniond(parameter_sensor_orientation.at(0),
                         parameter_sensor_orientation.at(1),
                         parameter_sensor_orientation.at(2),
                         parameter_sensor_orientation.at(3))
          .toRotationMatrix();
  kparameter_sensor_.sensor_in_base.matrix().block<3, 3>(0, 0) =
      Eigen::Quaterniond(parameter_sensor_sensor_in_base.at(3),
                         parameter_sensor_sensor_in_base.at(4),
                         parameter_sensor_sensor_in_base.at(5),
                         parameter_sensor_sensor_in_base.at(6))
          .toRotationMatrix();
  kparameter_sensor_.sensor_in_base.matrix().block<3, 1>(0, 3) =
      Eigen::Vector3d(parameter_sensor_sensor_in_base.at(0),
                      parameter_sensor_sensor_in_base.at(1),
                      parameter_sensor_sensor_in_base.at(2));
}

void DataInterface::VariableInit() {
  lidar_flag_ = false;
  lidar_buffer_.clear();

  odom_flag_ = false;
  odom_buffer_.clear();
}

void DataInterface::PublisherInit() {
  obstacle_scan_pub_ =
      nh_ptr_->advertise<sensor_msgs::LaserScan>("/obstacle_scan", 1);
  obstacle_cloud_pub_ =
      nh_ptr_->advertise<sensor_msgs::PointCloud2>("/obstacle_cloud", 1);
}

void DataInterface::SubscriberInit() {
  lidar_sub_ =
      nh_ptr_->subscribe("/point_cloud", 1, &DataInterface::LidarHandle, this);
  odom_sub_ = nh_ptr_->subscribe("/odom", 1, &DataInterface::OdomHandle, this);
}

void DataInterface::TimerInit(double period, void (*handle)()) {
  // spin thread
  spinner_ = std::unique_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
  spinner_->start();

  // work thread
  timer_handle_ = handle;
  timer_ = std::unique_ptr<ros::Rate>(new ros::Rate(1000.0 / period));
}

void DataInterface::PublishObstacleCloud(const HexStampedPoints& data) {
  sensor_msgs::PointCloud2Ptr obstacle_cloud_ptr(new sensor_msgs::PointCloud2);

  pcl::toROSMsg(*data.points, *obstacle_cloud_ptr);
  obstacle_cloud_ptr->header.stamp = ros::Time(data.time.sec, data.time.nsec);
  obstacle_cloud_ptr->header.frame_id = kparameter_sensor_.frame;

  obstacle_cloud_pub_.publish(obstacle_cloud_ptr);
}

void DataInterface::PublishObstacleScan(const HexStampedScan& data) {
  sensor_msgs::LaserScanPtr obstacle_scan_ptr(new sensor_msgs::LaserScan);

  obstacle_scan_ptr->header.stamp = ros::Time(data.time.sec, data.time.nsec);
  obstacle_scan_ptr->header.frame_id = kparameter_sensor_.scan_frame;
  obstacle_scan_ptr->angle_min = kparameter_filter_.fov[0];
  obstacle_scan_ptr->angle_max = kparameter_filter_.fov[1];
  obstacle_scan_ptr->angle_increment = kparameter_sensor_.angle_increment;
  obstacle_scan_ptr->scan_time = 1e-5;
  obstacle_scan_ptr->time_increment = 1e-8;
  obstacle_scan_ptr->range_min = kparameter_filter_.distance[0];
  obstacle_scan_ptr->range_max = kparameter_filter_.distance[1];

  obstacle_scan_ptr->ranges.clear();
  obstacle_scan_ptr->intensities.clear();
  int32_t fov_num = kparameter_filter_.fov_num;
  for (int32_t i = 0; i < fov_num; i++) {
    for (uint8_t j = 0; j < kparameter_sensor_.repeat_times; j++) {
      obstacle_scan_ptr->ranges.emplace_back(data.distances[i]);
      obstacle_scan_ptr->intensities.emplace_back(data.intensities[i]);
    }
  }

  obstacle_scan_pub_.publish(obstacle_scan_ptr);
}

void DataInterface::LidarHandle(const sensor_msgs::PointCloud2Ptr& msg) {
  static HexStampedPoints curr_lidar;
  if (ros::Time::now().toSec() - msg->header.stamp.toSec() < 0.2) {
    std::lock_guard<std::mutex> lock(lidar_mutex_);

    curr_lidar.time = HexStamp(msg->header.stamp.sec, msg->header.stamp.nsec);
    curr_lidar.points =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    curr_lidar.points->clear();
    pcl::fromROSMsg(*msg, *curr_lidar.points);

    lidar_buffer_.emplace_back(curr_lidar);
    lidar_flag_ = true;
  }
}

void DataInterface::OdomHandle(const nav_msgs::OdometryPtr& msg) {
  static HexStampedOdom curr_odom;
  if (ros::Time::now().toSec() - msg->header.stamp.toSec() < 0.1) {
    std::lock_guard<std::mutex> lock(odom_mutex_);

    curr_odom.time = HexStamp(msg->header.stamp.sec, msg->header.stamp.nsec);
    curr_odom.vel_lin =
        Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
    curr_odom.vel_ang =
        Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                        msg->twist.twist.angular.z);

    curr_odom.base_in_odom = Eigen::Affine3d::Identity();
    curr_odom.base_in_odom.translation() =
        Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    curr_odom.base_in_odom.linear() =
        Eigen::Quaterniond(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
            .toRotationMatrix();

    odom_buffer_.emplace_back(curr_odom);
    odom_flag_ = true;
  }
}

}  // namespace postprocess
}  // namespace hex
