/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2022 Veysi ADIN, UST KIST
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
 * \file  timing_node.hpp
 * \brief Simple timing subscriber to get timing information from EtherCAT node.
 * And show statistics.
 *******************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "ecat_msgs/msg/data_received.hpp"
#include "rcutils/logging_macros.h"

namespace Timing
{
class TimingNode : public rclcpp::Node
{
public:
  TimingNode();
  ~TimingNode();

private:
  void HandleReceivedDataCallback(const ecat_msgs::msg::DataReceived::SharedPtr msg);
  void CalculateStatistics();
  void PrintStatistics();

private:
  rclcpp::Subscription<ecat_msgs::msg::DataReceived>::SharedPtr received_data_sub_;
  ecat_msgs::msg::DataReceived::SharedPtr received_data_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  uint32_t period_min_ns_ = 0xffffffff;
  uint32_t period_max_ns_ = 0;
  uint32_t exec_min_ns_ = 0xffffffff;
  uint32_t exec_max_ns_ = 0;
  int32_t jitter_min_ = 0xfffffff;
  int32_t jitter_max_ = 0;
  uint32_t print_begin_ = 20;
};

}  // namespace Timing