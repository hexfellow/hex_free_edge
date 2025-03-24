# hex_free_edge

This repo provides an implementation of livox pointcloud undistortion.

## Maintainer

Dong Zhaorui(<847235539@qq.com>)

## Public APIs

### Publish

| Topic             | Msg Type                  | Description      |
| ----------------- | ------------------------- | ---------------- |
| `/obstacle_scan`  | `sensor_msgs/LaserScan`   | 安全区域的边界   |
| `/obstacle_cloud` | `sensor_msgs/PointCloud2` | 安全区域外的点云 |

### Subscribe

| Topic          | Msg Type                  | Description |
| -------------- | ------------------------- | ----------- |
| `/point_cloud` | `sensor_msgs/PointCloud2` | 原始点云    |

### Parameters

| Name                        | Data Type             | Description                  |
| --------------------------- | --------------------- | ---------------------------- |
| `sensor_frame`              | `std::string`         | Lidar坐标系                  |
| `sensor_scan_frame`         | `std::string`         | 边界scan的坐标系             |
| `sensor_repeat_times`       | `int`                 | 每个水平区间中scan重复的次数 |
| `sensor_orientation`        | `std::vector<double>` | Lidar在base_link系下的方向   |
| `filter_fov_min`        | `double`              | 水平角度最小值               |
| `filter_fov_max`        | `double`              | 水平角度最大值               |
| `filter_fov_num`        | `int`                 | 水平区间个数                 |
| `filter_distance_min`       | `double`              | 距离最小值                   |
| `filter_distance_max`       | `double`              | 距离最大值                   |
| `filter_distance_num`       | `int`                 | 距离区间个数                 |
| `filter_height_min`         | `double`              | 点云高度过滤下界             |
| `filter_height_max`         | `double`              | 点云高度过滤上界             |
| `threshold_delta_height`    | `double`              | 区间上下限高度判据           |
| `threshold_interval_height` | `double`              | 区间高度间隔判据             |
| `surround_distance`         | `double`              | 近距离点云距离判据           |
| `surround_constant`         | `double`              | 近距离点云高度的负截距       |
| `surround_factor`           | `double`              | 近距离点云高度的斜率         |
| `flag_pub_cloud`            | `bool`                | 是否发布`/obstacle_cloud`    |

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

1. Create a workspace `catkin_ws` and get into the `src`.

   ```shell
   mdkir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone this repo.

   ```shell
   git clone git@gitlab.hexmove.cn:IBN_Blank/hex_free_edge.git
   ```

3. Run install script `hex_install.py`.

   ```shell
   python3 hex_free_edge/tools/hex_install.py
   ```

4. Go to `catkin_ws` dir and build the repo.

   ```shell
   cd ../
   catkin_make
   ```

5. Source the `setup.bash` and run the test blow

   ```shell
   source devel/setup.bash --extend
   ```

### Platforms

* [ ] **Jetson Orin NX**
* [ ] **Jetson Orin Nano**
* [ ] **Jetson AGX Orin**
* [ ] **RK3588**

### Prerequisites

What additional things you need to use the software

**Foxglove: (optional)**

```shell
wget https://get.foxglove.dev/desktop/latest/foxglove-studio-2.4.0-linux-amd64.deb
sudo apt install ./foxglove-studio-*.deb
```

### Usage

```shell
roslaunch hex_free_edge hex_free_edge.launch
```

## Running the tests

1. For Mid-360

   ```shell
   terminal_1:
   $ roslaunch hex_free_edge hex_livox.launch

   terminal_2:
   $ roscd hex_free_edge/tests/
   $ rosbag play livox.bag --clock
   ```

2. For RS-LiDAR

   ```shell
   terminal_1:
   $ roslaunch hex_free_edge hex_lidar.launch

   terminal_2:
   $ roscd hex_free_edge/tests/
   $ rosbag play lidar.bag --clock
   ```

3. For depth camera

   ```shell
   terminal_1:
   $ roslaunch hex_free_edge hex_depth.launch

   terminal_2:
   $ roscd hex_free_edge/tests/
   $ rosbag play depth.bag --clock
   ```

## Reminder

1. 近距离点云高度判据: `h_ground < surround_factor * distance + surround_constant`
