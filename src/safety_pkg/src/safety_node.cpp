#include <safety_node.hpp>

class SafetyNode : public rclcpp::Node
{
public:
  explicit SafetyNode(const std::string& node_name) : Node(node_name)
  {
    this->init();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    gui_button_subscriber_ = this->create_subscription<ecat_msgs::msg::GuiButtonData>(
        "gui_buttons", 10, std::bind(&SafetyNode::HandleGuiNodeCallbacks, this, std::placeholders::_1));

    /// Subscribtion for control node.
    /// Subscribtion for slave feedback values acquired from connected slaves.
    lifecycle_node_subscriber_ = this->create_subscription<ecat_msgs::msg::DataReceived>(
        "Slave_Feedback", qos, std::bind(&SafetyNode::HandleLifecycleNodeCallbacks, this, std::placeholders::_1));

    safety_state_publisher_ = this->create_publisher<std_msgs::msg::UInt16>("safety_info", 10);
    //  spin_thread_ = std::thread{std::bind(&SafetyNode::SpinThread, this)};
    safety_state_msg_.data = kSafe;
    timer_ = this->create_wall_timer(30ms, std::bind(&SafetyNode::PublishSafetyInfo, this));
  }

  std::thread spin_thread_;
  uint32_t state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
  rclcpp::Subscription<ecat_msgs::msg::GuiButtonData>::SharedPtr gui_button_subscriber_;
  rclcpp::Subscription<ecat_msgs::msg::DataReceived>::SharedPtr lifecycle_node_subscriber_;

  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr safety_state_publisher_;
  ecat_msgs::msg::DataReceived lifecycle_node_data_;
  Controller controller_;
  std_msgs::msg::UInt16 safety_state_msg_;
  uint8_t button_request_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;

  void PublishSafetyInfo()
  {
    for (uint32_t i = 0; i < lifecycle_node_data_.slave_com_status.size(); i++)
    {
      if (lifecycle_node_data_.error_code[i] > 0 && safety_state_msg_.data != kErrorInDrive)
      {
        RCLCPP_ERROR(get_logger(), "Drive in error state : %s.",
                     GetErrorMessage(lifecycle_node_data_.error_code[i]).c_str());
        this->safety_state_msg_.data = kErrorInDrive;
      }
      else if (lifecycle_node_data_.error_code[i] == 0)
      {
        safety_state_msg_.data = kSafe;
      }
    }
    safety_state_publisher_->publish(safety_state_msg_);
  }

  void SpinThread()
  {
    while (true)
    {
      auto timer = this->create_wall_timer(1s, [this]() {
        static auto task = std::async(std::launch::async, [this] {
          std::this_thread::sleep_for(1s);
          state_ = get_state();
        });
      });
      rclcpp::spin(this->get_node_base_interface());
    }
  }

  void HandleGuiNodeCallbacks(const ecat_msgs::msg::GuiButtonData::SharedPtr msg)
  {
    // If state is unconfigured and buton_init_ecat trigger configuration transition.;
    state_ = get_state();
    if (state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      if (msg->b_init_ecat > 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!rclcpp::ok())
        {
          return;
        }
        if (this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
          return;
        }
        button_request_ = 1;
      }
    }

    // Activate
    if (state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
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
        button_request_ = 1;
      }
    }

    // Deactivate
    if (state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
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
        button_request_ = 1;
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
    if (state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
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
        button_request_ = 1;
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

    if (msg->b_emergency_mode > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (!rclcpp::ok())
      {
        return;
      }
      safety_state_msg_.data = kEmergencyStop;
      button_request_ = 1;
    }
    safety_state_publisher_->publish(safety_state_msg_);
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
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        break;
    };
    // if (!client_get_state_->wait_for_service(5s)) {
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
    // auto future_status = wait_for_result(future_result, 5s);

    // if (future_status != std::future_status::ready) {
    //   RCLCPP_ERROR(
    //     get_logger(), "Get State : Server time out while getting current state for node %s", lifecycle_node);
    //   return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    // }

    // // We have an succesful answer. So let's print the current state.
    // if (future_result.get()) {
    //   // RCLCPP_INFO(
    //   //   get_logger(), "Get State : Node %s has current state %s.",
    //   //   lifecycle_node, future_result.get()->current_state.label.c_str());
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

void wake_executor(std::shared_future<void> future, rclcpp::executors::SingleThreadedExecutor& exec)
{
  future.wait();
  // Wake the executor when the script is done
  // https://github.com/ros2/rclcpp/issues/1916
  exec.cancel();
}

void callee_script(std::shared_ptr<SafetyNode> safety_node)
{
  rclcpp::WallRate time_to_sleep(3s);  // 10s
  while (true)
  {
    if (safety_node->button_request_)
    {
      time_to_sleep.sleep();
      safety_node->state_ = safety_node->get_state();
      safety_node->button_request_ = 0;
    }
  }
}

int main(int argc, char** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto safety_node = std::make_shared<SafetyNode>("safety_node");

#if 0
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(lc_client);

  std::shared_future<void> script = std::async(
    std::launch::async,
    std::bind(callee_script, lc_client));
  exe.spin_until_future_complete(script);
#else

  rclcpp::executors::StaticSingleThreadedExecutor exe;
  exe.add_node(safety_node->get_node_base_interface());
  exe.spin();

#endif
  rclcpp::shutdown();

  return 0;
}
