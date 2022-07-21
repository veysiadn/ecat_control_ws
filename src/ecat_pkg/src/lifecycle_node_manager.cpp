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

#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

// which node to handle
static constexpr char const* lifecycle_node = "ecat_node";

// Every lifecycle node has various services
// attached to it. By convention, we use the format of
// <node name>/<service name>.
// ecat_node/get_state
// ecat_node/change_state
static constexpr char const* node_get_state_topic = "ecat_node/get_state";
static constexpr char const* node_change_state_topic = "ecat_node/change_state";

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
  uint8_t left_d_button_;
  uint8_t left_rb_button_;
  uint8_t right_rb_button_;
  uint8_t left_start_button_;
  uint8_t right_start_button_;
  uint8_t xbox_button_;
} Controller;

// template<typename FutureT, typename WaitTimeT>
// std::future_status
// wait_for_result(
//   FutureT & future,
//   WaitTimeT time_to_wait)
// {
//   auto end = std::chrono::steady_clock::now() + time_to_wait;
//   std::chrono::milliseconds wait_period(100);
//   std::future_status status = std::future_status::timeout;
//   do {
//     auto now = std::chrono::steady_clock::now();
//     auto time_left = end - now;
//     if (time_left <= std::chrono::milliseconds(0)) {break;}
//     status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
//   } while (status != std::future_status::ready);
//   return status;
// }

class LifecycleNodeManager : public rclcpp::Node
{
public:
  explicit LifecycleNodeManager(const std::string& node_name) : Node(node_name)
  {
    this->init();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    gui_button_subscriber_ = this->create_subscription<ecat_msgs::msg::GuiButtonData>(
        "gui_buttons", qos, std::bind(&LifecycleNodeManager::HandleGuiNodeCallbacks, this, std::placeholders::_1));

    /// Subscribtion for control node.
    /// Subscribtion for slave feedback values acquired from connected slaves.
    lifecycle_node_subscriber_ = this->create_subscription<ecat_msgs::msg::DataReceived>(
        "Slave_Feedback", qos,
        std::bind(&LifecycleNodeManager::HandleLifecycleNodeCallbacks, this, std::placeholders::_1));
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
  rclcpp::Subscription<ecat_msgs::msg::GuiButtonData>::SharedPtr gui_button_subscriber_;
  rclcpp::Subscription<ecat_msgs::msg::DataReceived>::SharedPtr lifecycle_node_subscriber_;
  ecat_msgs::msg::DataReceived lifecycle_node_data_;
  Controller controller_;
  void HandleGuiNodeCallbacks(const ecat_msgs::msg::GuiButtonData::SharedPtr msg)
  {
    // If state is unconfigured and buton_init_ecat trigger configuration transition.;
    if (get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      if (msg->b_init_ecat > 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
          return;
        }
      }
    }

    // Activate
    if (get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      if (msg->b_enter_cyclic_pdo > 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!rclcpp::ok())
        {
          return;
        }
        if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        {
          return;
        }
      }
    }

    // Deactivate
    if (get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      if (msg->b_stop_cyclic_pdo > 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!rclcpp::ok())
        {
          return;
        }
        if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
        {
          return;
        }
      }
    }
    // msg->b_init_ecat = 0;
    // msg->b_reinit_ecat = 0;
    // msg->b_enable_drives = 0;
    // msg->b_disable_drives = 0;
    // msg->b_enable_cyclic_pos = 0;
    // msg->b_enable_cyclic_vel = 0;
    // msg->b_enable_vel = 0;
    // msg->b_enable_pos = 0;
    // msg->b_enter_cyclic_pdo = 0;
    // msg->b_emergency_mode = 0;
    // msg->b_send = 0;
    // msg->b_stop_cyclic_pdo = 0;

    // Cleanup
    if (get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      if (msg->b_reinit_ecat > 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!rclcpp::ok())
        {
          return;
        }
        if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
        {
          return;
        }
      }
    }
    //   // Shutdown
    //   if(controller_.right_rb_button_ > 0){
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //     if (!rclcpp::ok()) {
    //       return;
    //     }
    //     if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
    //     {
    //       return;
    //     }
    //     if (!this->get_state()) {
    //       return;
    //     }
    //   }
  }

  void HandleLifecycleNodeCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg)
  {
    lifecycle_node_data_ = *msg;
  }
  void init()
  {
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
  }

  /// Requests the current state of the node
  /**
   * In this function, we send a service request
   * asking for the current state of the node
   * ecat_node.
   * If it does return within the given time_out,
   * we return the current state of the node, if
   * not, we return an unknown state.
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
  unsigned int get_state()
  {
    // auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    switch (lifecycle_node_data_.current_lifecycle_state)
    {
      case 0:
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
        break;
      case 1:
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
        break;
      case 2:
        return lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
        break;
      case 3:
        return lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        break;
      case 4:
        return lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
        break;
      case 10:
        return lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING;
        break;
      case 11:
        return lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP;
        break;
      case 12:
        return lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN;
        break;
      case 13:
        return lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING;
        break;
      case 14:
        return lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING;
        break;
      case 15:
        return lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING;
        break;
      default:
        break;
    };
    // if (!client_get_state_->wait_for_service(time_out)) {
    //   RCLCPP_ERROR(
    //     get_logger(),
    //     "Get State : Service %s is not available.",
    //     client_get_state_->get_service_name());
    //   return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    // }

    // // We send the service request for asking the current
    // // state of the ecat_node node.
    // auto future_result = client_get_state_->async_send_request(request);

    // // Let's wait until we have the answer from the node.
    // // If the request times out, we return an unknown state.
    // auto future_status = wait_for_result(future_result, time_out);

    // if (future_status != std::future_status::ready) {
    //   RCLCPP_ERROR(
    //     get_logger(), "Get State : Server time out while getting current state for node %s", lifecycle_node);
    //   return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    // }

    // // We have an succesful answer. So let's print the current state.
    // if (future_result.get()) {
    //   RCLCPP_INFO(
    //     get_logger(), "Get State : Node %s has current state %s.",
    //     lifecycle_node, future_result.get()->current_state.label.c_str());
    //   return future_result.get()->current_state.id;
    // } else {
    //   RCLCPP_ERROR(
    //     get_logger(), "Get State : Failed to get current state for node %s", lifecycle_node);
    //   return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    // }
  }

  /// Invokes a transition
  /**
   * We send a Service request and indicate
   * that we want to invoke transition with
   * the id "transition".
   * By default, these transitions are
   * - configure
   * - activate
   * - cleanup
   * - shutdown
   * \param transition id specifying which
   * transition to invoke
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */

  bool change_state(std::uint8_t transition)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(3s))
    {
      RCLCPP_ERROR(get_logger(), "Change State : Service %s is not available.",
                   client_change_state_->get_service_name());
      return false;
    }

    // // We send the request with the transition we want to invoke.

    auto future_result = client_change_state_->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    // auto future_status = wait_for_result(future_result, time_out);

    // if (future_status != std::future_status::ready) {
    //   RCLCPP_ERROR(
    //     get_logger(), "Change State : Server time out while changing state for node %s", lifecycle_node);
    //   return false;
    // }

    // We have an answer, let's print our success.
    // if (future_result.get()->success) {
    //   RCLCPP_INFO(
    //     get_logger(), "Change State : Transition %d successfully triggered.", static_cast<int>(transition));
    //   return true;
    // } else {
    //   RCLCPP_WARN(
    //     get_logger(), "Change State : Failed to trigger transition %u", static_cast<unsigned int>(transition));
    //   return false;
    // }
    return true;
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};

int main(int argc, char** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto lifecycle_manager = std::make_shared<LifecycleNodeManager>("lifecycle_manager_node");

#if 0
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(lc_client);

  std::shared_future<void> script = std::async(
    std::launch::async,
    std::bind(callee_script, lc_client));
  exe.spin_until_future_complete(script);
#else
  rclcpp::spin(lifecycle_manager);
#endif
  rclcpp::shutdown();

  return 0;
}

/**
 * This is a little independent
 * script which triggers the
 * default lifecycle of a node.
 * It starts with configure, activate,
 * deactivate, activate, deactivate,
 * cleanup and finally shutdown
 */
// void
// callee_script(std::shared_ptr<LifecycleNodeManager> lc_client)
// {
//   rclcpp::WallRate time_between_state_changes(0.5);  // 10s

//   // configure
//   {
//     if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
//       return;
//     }
//     if (!lc_client->get_state()) {
//       return;
//     }
//   }

//   // activate
//   {
//     time_between_state_changes.sleep();
//     if (!rclcpp::ok()) {
//       return;
//     }
//     if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
//       return;
//     }
//     if (!lc_client->get_state()) {
//       return;
//     }
//   }

//   // deactivate
//   {
//     time_between_state_changes.sleep();
//     if (!rclcpp::ok()) {
//       return;
//     }
//     if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
//       return;
//     }
//     if (!lc_client->get_state()) {
//       return;
//     }
//   }

//   // activate it again
//   {
//     time_between_state_changes.sleep();
//     if (!rclcpp::ok()) {
//       return;
//     }
//     if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
//       return;
//     }
//     if (!lc_client->get_state()) {
//       return;
//     }
//   }

//   // and deactivate it again
//   {
//     time_between_state_changes.sleep();
//     if (!rclcpp::ok()) {
//       return;
//     }
//     if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
//       return;
//     }
//     if (!lc_client->get_state()) {
//       return;
//     }
//   }

//   // we cleanup
//   {
//     time_between_state_changes.sleep();
//     if (!rclcpp::ok()) {
//       return;
//     }
//     if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
//       return;
//     }
//     if (!lc_client->get_state()) {
//       return;
//     }
//   }

//   // and finally shutdown
//   // Note: We have to be precise here on which shutdown transition id to call
//   // We are currently in the unconfigured state and thus have to call
//   // TRANSITION_UNCONFIGURED_SHUTDOWN
//   {
//     time_between_state_changes.sleep();
//     if (!rclcpp::ok()) {
//       return;
//     }
//     if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
//     {
//       return;
//     }
//     if (!lc_client->get_state()) {
//       return;
//     }
//   }
// }
