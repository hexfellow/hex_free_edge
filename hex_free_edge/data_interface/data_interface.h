/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-14
 ****************************************************************/

#ifndef HEX_FREE_EDGE_DATA_INTERFACE_DATA_INTERFACE_H_
#define HEX_FREE_EDGE_DATA_INTERFACE_DATA_INTERFACE_H_

#define HEX_ROS_VERSION 1

#if HEX_ROS_VERSION == 1
#include "hex_free_edge/data_interface/ros1_interface.h"
#elif HEX_ROS_VERSION == 2
#include "hex_free_edge/data_interface/ros1_interface.h"
#else
#warning "HEX_ROS_VERSION is unknown"
#endif

#endif  // HEX_FREE_EDGE_DATA_INTERFACE_DATA_INTERFACE_H_
