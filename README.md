# **hex_free_edge**

## **Overview**

This **hex_free_edge** repository provides an implementation to detect the free edge of the obstacle.

### **License**

This project is licensed under the terms of the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

### **Maintainer**

**[Dong Zhaorui](https://github.com/IBNBlank)**

### **Supported Platform**

- [x] **x64**
- [ ] **Jetson Orin Nano**
- [x] **Jetson Orin NX**
- [ ] **Jetson AGX Orin**
- [ ] **Horizon RDK X5**

### **Supported ROS Version**

- [x] **ROS Noetic**
- [ ] **ROS Humble**

---

## **Public APIs**

### **Publish**

| Topic             | Msg Type                  | Description               |
| ----------------- | ------------------------- | ------------------------- |
| `/obstacle_scan`  | `sensor_msgs/LaserScan`   | Safe edge of the obstacle |
| `/obstacle_cloud` | `sensor_msgs/PointCloud2` | Cloud outside of the edge |

### **Subscribe**

| Topic          | Msg Type                  | Description        |
| -------------- | ------------------------- | ------------------ |
| `/point_cloud` | `sensor_msgs/PointCloud2` | Sensor point cloud |

### **Parameters**

| Name                       | Data Type             | Description                                               |
| -------------------------- | --------------------- | --------------------------------------------------------- |
| `filter_height`            | `std::vector<double>` | Filter judgement of height                                |
| `filter_fov`               | `std::vector<double>` | Filter judgement of fov                                   |
| `filter_distance`          | `std::vector<double>` | Filter judgement of distance                              |
| `filter_fov_num`           | `int32`               | Grid number of fov                                        |
| `filter_distance_num`      | `int32`               | Grid number of distance                                   |
| `flag_pub_cloud`           | `bool`                | Whether to publish cloud                                  |
| `obstacle_cloud_num`       | `int32`               | Number of cloud to accumulate                             |
| `obstacle_change_distance` | `double`              | Obstacle judgement change distance                        |
| `obstacle_far_range`       | `double`              | Obstacle judgement for delta height in a single grid      |
| `obstacle_far_gap`         | `double`              | Obstacle judgement for delta height between sorted points |
| `obstacle_near_constant`   | `double`              | Near constant of obstacle                                 |
| `obstacle_near_factor`     | `double`              | Near factor of obstacle                                   |
| `sensor_frame`             | `std::string`         | Point cloud frame                                         |
| `sensor_scan_frame`        | `std::string`         | Scan frame                                                |
| `sensor_scan_repeat`       | `int32`               | Scan repeat times                                         |
| `sensor_orientation`       | `std::vector<double>` | Sensor orientation (qw, qx, qy, qz)                       |
| `sensor_senser_in_base`    | `std::vector<double>` | Sensor in base (x, y, z, qw, qx, qy, qz)                  |
| `sensor_lidar_period`      | `double`              | Lidar period                                              |
| `sensor_odom_period`       | `double`              | Odom period                                               |

---

## **Getting Started**

### **Dependencies**

- **ROS Noetic** or **ROS Humble**

### **Install**

1. Create a workspace `catkin_ws` and get into the `src`.

   ```shell
   mdkir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone this repo.

   ```shell
   git clone git@github.com:hexfellow/hex_free_edge.git
   ```

3. Go to `catkin_ws` directory and build the repo.

   ```shell
   cd ../
   catkin_make
   ```

4. Source the `setup.bash` and run the test blow

   ```shell
   source devel/setup.bash --extend
   ```

### **Usage**

1. If you are using Livox LiDAR, you can run the following command:

   ```shell
   terminal_1:
   $ roslaunch hex_free_edge hex_livox.launch

   terminal_2:
   $ roscd hex_free_edge/tests/
   $ rosbag play livox.bag --clock
   ```

2. If you are using RS-LiDAR, you can run the following command:

   ```shell
   terminal_1:
   $ roslaunch hex_free_edge hex_lidar.launch

   terminal_2:
   $ roscd hex_free_edge/tests/
   $ rosbag play lidar.bag --clock
   ```

3. If you are using depth camera, you can run the following command:

   ```shell
   terminal_1:
   $ roslaunch hex_free_edge hex_depth.launch

   terminal_2:
   $ roscd hex_free_edge/tests/
   $ rosbag play depth.bag --clock
   ```

## **Notes**

1. Near judgement: `h_ground < surround_factor * distance + surround_constant`
2. Far judgement: `delta > obstacle_far_range || gap > obstacle_far_gap`
