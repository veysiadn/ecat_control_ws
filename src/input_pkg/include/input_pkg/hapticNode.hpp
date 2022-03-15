// ---------------------------------------------------------------
// Ckim - Haptic Device Node (client) for ROS2
// Connects to haptic device PC (master) by TCP/IP
// reads incoming data and publishes HapticCmd
// Based on ROS2 joystick node code at
// https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy
// ---------------------------------------------------------------

#ifndef HAPTICNODE
#define HAPTICNODE

// CKim - SDL = Simple DirectMedia Layer is cross-platform development library designed 
// to provide low level access to audio, keyboard, mouse, joystick, and graphics hardware via
// OpenGL and Direct3D.  Used for joystickinput processing here
#include <SDL2/SDL.h>

// CKim - C++  headers
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "ecat_msgs/msg/haptic_cmd.hpp"  // CKim - Header for custom message

class HapticNode final : public rclcpp::Node  // keyword 'final' prevents further inheritance
{
public:

  // CKim - Keyword 'explicit' restricts implicit conversion
  explicit HapticNode(char * argv[]);
  
  // CKim - Restrict copy constructors and assignment operator =
  // keyword 'delete' tells these functions will not be implemented. 
  HapticNode(HapticNode && c) = delete;
  HapticNode & operator=(HapticNode && c) = delete;
  HapticNode(const HapticNode & c) = delete;
  HapticNode & operator=(const HapticNode & c) = delete;

  // CKim - Keyword 'override' tell compiler that this inherited function must be implemented
  ~HapticNode() override;

private:
  void commThread();

  // CKim - C++ standard thread, 
  // std::future/shared_future and promise provides a mechanism to
  // control therad....
  std::thread comm_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

  std::string m_IP;
  std::string m_Port;

  // CKim - Publisher
  rclcpp::Publisher<ecat_msgs::msg::HapticCmd>::SharedPtr haptic_publisher_;

  // CKim - Published message
  ecat_msgs::msg::HapticCmd hapticMsg;
  // rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  // rclcpp::Subscription<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_sub_;
  // sensor_msgs::msg::Joy joy_msg_;
};


#endif  // JOY__JOY_HPP_