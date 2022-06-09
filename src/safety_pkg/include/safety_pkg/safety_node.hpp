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

using namespace std::chrono_literals;

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
// which node to handle
static constexpr char const * lifecycle_node = "ecat_node";

// Every lifecycle node has various services
// attached to it. By convention, we use the format of
// <node name>/<service name>.
// ecat_node/get_state
// ecat_node/change_state
static constexpr char const * node_get_state_topic    = "ecat_node/get_state";
static constexpr char const * node_change_state_topic = "ecat_node/change_state";

typedef struct
{
    float left_x_axis_;
    float left_y_axis_;
    float right_x_axis_;
    float right_y_axis_;
    uint8_t blue_button_;
    uint8_t green_button_;
    uint8_t red_button_;
    uint8_t yellow_button_;
    uint8_t left_r_button_;
    uint8_t left_l_button_;
    uint8_t left_u_button_;
    uint8_t left_d_button_ ;
    uint8_t left_rb_button_ ;
    uint8_t right_rb_button_ ;
    uint8_t left_start_button_ ;
    uint8_t right_start_button_ ; 
    uint8_t xbox_button_;
} Controller;

enum SafetyInfo{
  kSafe=0,
  kOverForce,
  kOverSpeed,
  kOverPositionLimit,
  kErrorInDrive,
  kEmergencyStop
};
