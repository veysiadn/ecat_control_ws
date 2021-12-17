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
 * \file  gui_node.hpp
 * \brief GUI node implementation to show slave status and controller commands in ROS2.
 *        Additionally it publishes information regarding buttons on mainscreen.
 *        GUI node is a ROS2 node which subscribes EthercatLifecycle Node topics and 
 *        controller topics and shows those values via GUI and publishes control ui
 *        button values.
 *******************************************************************************/

#pragma once
//ROS2
#include "rclcpp/rclcpp.hpp"

// Message file headers, -custom and built-in-
#include "sensor_msgs/msg/joy.hpp"
#include "ecat_msgs/msg/data_received.hpp"
#include "ecat_msgs/msg/data_sent.hpp"
#include "ecat_msgs/msg/gui_button_data.hpp"

#include <rclcpp/rclcpp.hpp>    // Standard ROS2 header API
//CPP
#include <vector>
#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cstdint>
#include <ctime>
#include <ratio>
#include <fstream>
#include <string>
#include "timing.hpp"
#include "gui_globals.hpp"

using namespace std::chrono_literals;

namespace GUI {

/**
 * @brief This node will be responsible from all GUI interaction and visualization of feedback information acquired
 * via EtherCAT communication.
 */

 class GuiNode : public rclcpp::Node
  {

  private:
      /**
       * @brief Publishes gui button value in specified interval.
       */

      void timer_callback();
      /**
       * @brief Function will be used for subscribtion callbacks from controller node
       *        for Controller topic.
       *
       * @param msg controller command structure published by controller node.
       */

        void HandleControllerCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg);
        /**
         * @brief Function will be used for subscribtion callbacks from EthercatLifecycle node
         *        for Master_Commands topic.
         *
         * @param msg Master commands structure published by EthercatLifecycle node
         */

        void HandleMasterCommandCallbacks(const ecat_msgs::msg::DataSent::SharedPtr msg);
        /**
         * @brief Function will be used for subscribtion callbacks from EthercatLifecycle node
         *        for Master_Commands topic.
         *
         * @param msg Slave feedback structure published by EthercatLifecycle node
         */

        void HandleSlaveFeedbackCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg);
        /**
         * @brief Resets control button values coming from control UI.
         */
        void ResetContolButtonValues();
   private:
        /// ROS2 subscriptions.
        /// Acquired feedback information from connected slaves
        rclcpp::Subscription<ecat_msgs::msg::DataReceived>::SharedPtr slave_feedback_;

        /// Subscribed to commands that's being sent by ecat_master
        rclcpp::Subscription<ecat_msgs::msg::DataSent>::SharedPtr master_commands_;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  controller_commands_;

        /// Timer for timer callbacks, publishing will be done in certain interval.
        rclcpp::TimerBase::SharedPtr timer_;

        /// Gui publisher, which contains clicked button information
        rclcpp::Publisher<ecat_msgs::msg::GuiButtonData>::SharedPtr gui_publisher_;

     public:
         GuiNode();
         virtual ~GuiNode();
     public:
         /// Received data structure to store all subscribed data.
         std::vector<ReceivedData> received_data_;
         /// GUI button value to publish emergency button state.
         ecat_msgs::msg::GuiButtonData ui_control_buttons_;
         /// For time measurements
         Timing time_info_;
  };// class GuiNode

 } // namespace GUI
