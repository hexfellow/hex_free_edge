/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-15
 ****************************************************************/

#ifndef HEX_FREE_EDGE_HEX_UTILITY_H_
#define HEX_FREE_EDGE_HEX_UTILITY_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace hex {
namespace utility {

struct HexParameterFilter {
  Eigen::Vector2d height;
  Eigen::Vector2d fov;
  Eigen::Vector2d distance;
  int32_t fov_num;
  int32_t distance_num;
  double fov_step;
  double distance_step;
};

struct HexParameterFlag {
  bool pub_cloud;
};

struct HexParameterObstacle {
  int32_t cloud_num;
  double change_distance;
  double far_range;
  double far_gap;
  double near_constant;
  double near_factor;
};

struct HexParameterSensor {
  std::string frame;
  std::string scan_frame;
  int32_t repeat_times;
  Eigen::Affine3d orientation;
  Eigen::Affine3d sensor_in_base;
  double lidar_period;
  double odom_period;
  double angle_increment;
};

struct HexStamp {
  uint32_t sec;
  uint32_t nsec;
  // constructor
  HexStamp() : sec(0), nsec(0) {}
  HexStamp(uint32_t sec, uint64_t nsec) : sec(sec), nsec(nsec) {}
  // copy operator
  HexStamp& operator=(const HexStamp& that) {
    this->sec = that.sec;
    this->nsec = that.nsec;
    return *this;
  }
  // compare operator
  bool operator<(const HexStamp& that) const {
    if (this->sec < that.sec) {
      return true;
    } else if ((this->sec == that.sec) && (this->nsec < that.nsec)) {
      return true;
    } else {
      return false;
    }
  }
  bool operator>(const HexStamp& that) const {
    if (this->sec > that.sec) {
      return true;
    } else if ((this->sec == that.sec) && (this->nsec > that.nsec)) {
      return true;
    } else {
      return false;
    }
  }
  bool operator==(const HexStamp& that) const {
    return (this->sec == that.sec) && (this->nsec == that.nsec);
  }
  bool operator!=(const HexStamp& that) const { return !(*this == that); }
  bool operator<=(const HexStamp& that) const { return !(*this > that); }
  bool operator>=(const HexStamp& that) const { return !(*this < that); }
  // plus operator
  HexStamp operator+(double delta) const {
    if ((this->sec + this->nsec * 1e-9 + delta) < 0) {
      throw "Negative time is not allowed!";
    }

    HexStamp result = HexStamp();
    int64_t delta_sec = static_cast<int64_t>(delta);
    int64_t delta_nsec = static_cast<int64_t>((delta - delta_sec) * 1000000000);
    if (this->nsec + delta_nsec >= 1000000000) {
      result.sec = static_cast<uint32_t>(static_cast<int64_t>(this->sec) +
                                         delta_sec + 1);
      result.nsec = static_cast<uint32_t>(static_cast<int64_t>(this->nsec) +
                                          delta_nsec - 1000000000);
    } else if (this->nsec + delta_nsec < 0) {
      result.sec = static_cast<uint32_t>(static_cast<int64_t>(this->sec) +
                                         delta_sec - 1);
      result.nsec = static_cast<uint32_t>(static_cast<int64_t>(this->nsec) +
                                          delta_nsec + 1000000000);
    } else {
      result.sec =
          static_cast<uint32_t>(static_cast<int64_t>(this->sec) + delta_sec);
      result.nsec =
          static_cast<uint32_t>(static_cast<int64_t>(this->nsec) + delta_nsec);
    }
    return result;
  }
  // minus operator
  double operator-(const HexStamp& that) const {
    int64_t diff_sec =
        static_cast<int64_t>(this->sec) - static_cast<int64_t>(that.sec);
    int64_t diff_nsec =
        static_cast<int64_t>(this->nsec) - static_cast<int64_t>(that.nsec);
    double result = static_cast<double>(diff_sec) + diff_nsec * 1e-9;
    return result;
  }
  HexStamp operator-(double delta) const { return *this + (-delta); }
};

struct HexStampedOdom {
  HexStamp time;
  Eigen::Vector3d vel_lin;
  Eigen::Vector3d vel_ang;
  Eigen::Affine3d base_in_odom;
  // constructor
  HexStampedOdom() {
    this->time = HexStamp();
    this->vel_lin = Eigen::Vector3d::Zero();
    this->vel_ang = Eigen::Vector3d::Zero();
    this->base_in_odom = Eigen::Affine3d::Identity();
  }
  // copy operator
  HexStampedOdom& operator=(const HexStampedOdom& that) {
    this->time = that.time;
    this->vel_lin = Eigen::Vector3d(that.vel_lin);
    this->vel_ang = Eigen::Vector3d(that.vel_ang);
    this->base_in_odom = Eigen::Affine3d(that.base_in_odom);
    return *this;
  }
};

struct HexStampedPoints {
  HexStamp time;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  // constructor
  HexStampedPoints() {
    this->time = HexStamp();
    this->points =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  }
  // copy operator
  HexStampedPoints& operator=(const HexStampedPoints& that) {
    this->time = that.time;
    this->points = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>(*that.points));
    return *this;
  }
};

struct HexStampedScan {
  HexStamp time;
  std::vector<float> distances;
  std::vector<float> intensities;
  // constructor
  HexStampedScan() {
    this->time = HexStamp();
    this->distances = std::vector<float>();
    this->intensities = std::vector<float>();
  }
  // copy operator
  HexStampedScan& operator=(const HexStampedScan& that) {
    this->time = that.time;
    this->distances = std::vector<float>(that.distances);
    this->intensities = std::vector<float>(that.intensities);
    return *this;
  }
};

}  // namespace utility
}  // namespace hex

#endif  // HEX_FREE_EDGE_HEX_UTILITY_H_
