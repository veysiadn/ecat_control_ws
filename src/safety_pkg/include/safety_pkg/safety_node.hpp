/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2021 Veysi ADIN, UST KIST
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS2 environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: veysi.adin@kist.re.kr
 *****************************************************************************/
/*****************************************************************************
 * \file  safety_node.hpp
 * \brief Header file for the Safety Node which is a manager node for the 
 * EtherCAT life cycle node.
 * This header file contains required include for lifecycle management and safety
 * state data structure.
 *******************************************************************************/
#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ecat_msgs/msg/gui_button_data.hpp"
#include "ecat_msgs/msg/data_received.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "std_msgs/msg/u_int16.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/rclcpp.hpp"
#include "ecat_globals.hpp"

using namespace std::chrono_literals;

// which node to handle
static constexpr char const * lifecycle_node = "ecat_node";

// Every lifecycle node has various services attached to it.
// By convention, we use the format of <node name>/<service name>.
// ecat_node/get_state
// ecat_node/change_state
static constexpr char const * node_get_state_topic    = "ecat_node/get_state";
static constexpr char const * node_change_state_topic = "ecat_node/change_state";


template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::milliseconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (status != std::future_status::ready);
  return status;
}

enum SafetyInfo{
  kSafe=0,
  kOverForce,
  kOverSpeed,
  kOverPositionLimit,
  kErrorInDrive,
  kEmergencyStop
};
