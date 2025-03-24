cmake_minimum_required(VERSION 3.0.2)
project(hex_free_edge)

set(HEX_ROS_VERSION $ENV{ROS_VERSION})
message(STATUS "Your ROS version is ROS${HEX_ROS_VERSION}")

add_definitions(-DHEX_ROS_VERSION=${HEX_ROS_VERSION}) 

if (${HEX_ROS_VERSION} STREQUAL "1")

    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "-std=c++14")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
    )
    find_package(OpenMP REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(PCL REQUIRED)

    catkin_package()

    include_directories(
        ../
        ./
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS} 
        ${PCL_INCLUDE_DIRS}
    )

    add_executable(hex_free_edge
        hex_free_edge/main.cc
        hex_free_edge/hex_free_edge.cc
        hex_free_edge/lidar_accumulate.cc
        hex_free_edge/data_interface/ros1_interface.cc
    )

    target_compile_options(hex_free_edge PRIVATE ${OpenMP_CXX_FLAGS})

    target_link_libraries(hex_free_edge
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OpenMP_CXX_FLAGS}
    )

    install(
        TARGETS hex_free_edge
        DESTINATION lib/${PROJECT_NAME}
    )

    install(
        DIRECTORY launch/ros1
        DESTINATION share/${PROJECT_NAME}/launch
    )

    install(
        DIRECTORY config/ros1
        DESTINATION share/${PROJECT_NAME}/config
    )

elseif (${HEX_ROS_VERSION} STREQUAL "2")

    cmake_minimum_required(VERSION 3.8)
    
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    include_directories(
        ../
        ./
        include
        ${colcon_INCLUDE_DIRS}
    )

    ament_package()

    add_executable(hex_free_edge
        hex_free_edge/main.cc
        hex_free_edge/hex_free_edge.cc
        hex_free_edge/data_interface/ros2_interface.cc
    )

    ament_target_dependencies(hex_free_edge
        rclcpp
        std_msgs
    )

    install(
        TARGETS hex_free_edge
        DESTINATION lib/${PROJECT_NAME}
    )

    install(
        DIRECTORY launch/ros2
        DESTINATION share/${PROJECT_NAME}/launch
    )

    install(
        DIRECTORY config/ros2
        DESTINATION share/${PROJECT_NAME}/config
    )

else ()
    message(FATAL_ERROR "Unknown ROS version, please install the ros_environment package")
endif ()
