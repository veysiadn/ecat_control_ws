#include "../include/gui_pkg/gui_node.hpp"

  using namespace std::chrono_literals;

  using namespace GUI;

  GuiNode::GuiNode() : Node("gui_node")
  {
    slave_feedback_data_.actual_pos.resize(g_kNumberOfServoDrivers);
    ui_control_buttons_.spn_target_values.resize(g_kNumberOfServoDrivers);
    for(int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
        ui_control_buttons_.spn_target_values[i] = 0 ;
        slave_feedback_data_.emergency_switch_val = 1 ;
    }
    ResetContolButtonValues();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    /// Subscribtion for control node.
    controller_commands_= this->create_subscription<sensor_msgs::msg::Joy>("Controller", qos,
                                  std::bind(&GuiNode::HandleControllerCallbacks, this, std::placeholders::_1));
    /// Subscribtion for slave feedback values acquired from connected slaves.
    slave_feedback_ = this->create_subscription<ecat_msgs::msg::DataReceived>("Slave_Feedback", qos,
                                   std::bind(&GuiNode::HandleSlaveFeedbackCallbacks, this, std::placeholders::_1));
    /// Subscribtions for master commands to slaves.
    master_commands_ = this->create_subscription<ecat_msgs::msg::DataSent>("Master_Commands", qos,
                                   std::bind(&GuiNode::HandleMasterCommandCallbacks, this, std::placeholders::_1));
    /// Gui button value publisher
    gui_publisher_ = create_publisher<ecat_msgs::msg::GuiButtonData>("gui_buttons",10);
    /// Timer callback set to 33HZ.
    timer_ = this->create_wall_timer(30ms,std::bind(&GuiNode::timer_callback,this));
  }

  GuiNode::~GuiNode()
  {
    rclcpp::shutdown();
  }

  void GuiNode::timer_callback()
  {
      ui_control_buttons_.header.stamp = this->now();
      gui_publisher_->publish(ui_control_buttons_);
      ResetContolButtonValues();
  }

  void GuiNode::HandleControllerCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    //  for(int i=0; i < g_kNumberOfServoDrivers ; i++){
    //     received_data_[i].right_x_axis = msg->axes[3];
    //     received_data_[i].left_x_axis =  msg->axes[0];
    //  }
    // emit UpdateParameters(0);
  }

  void GuiNode::HandleMasterCommandCallbacks(const ecat_msgs::msg::DataSent::SharedPtr msg)
  {
      
      for(int i=0; i < g_kNumberOfServoDrivers ; i++){
         master_command_data_.target_pos[i]   =  msg->target_pos[i];
         master_command_data_.target_vel[i]   =  msg->target_vel[i];
         master_command_data_.target_tor[i]   =  msg->target_tor[i];
         master_command_data_.control_word[i] =  msg->control_word[i];
      }
  }

  void GuiNode::HandleSlaveFeedbackCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg)
  {

      for(int i=0; i < g_kNumberOfServoDrivers ; i++){
        /// Servo Drive feedbacks
        slave_feedback_data_.actual_pos[i]              =  msg->actual_pos[i];
        slave_feedback_data_.actual_vel[i]              =  msg->actual_vel[i];
        slave_feedback_data_.actual_tor[i]              =  msg->actual_tor[i];
        slave_feedback_data_.status_word[i]             =  msg->status_word[i];
        slave_feedback_data_.slave_com_status[i]        =  msg->slave_com_status[i];
        slave_feedback_data_.com_status                 =  msg->com_status;
        slave_feedback_data_.op_mode_display[i]         =  msg->op_mode_display[i]; 
        slave_feedback_data_.current_lifecycle_state    =  msg->current_lifecycle_state;
        slave_feedback_data_.error_code[i]              =  msg->error_code[i];
        /// Costum slave feedbacks.
        slave_feedback_data_.left_limit_switch_val   =  msg->left_limit_switch_val;
        slave_feedback_data_.right_limit_switch_val  =  msg->right_limit_switch_val;
        slave_feedback_data_.emergency_switch_val    =  msg->emergency_switch_val;
    }
  }

  void GuiNode::ResetContolButtonValues()
 {
    ui_control_buttons_.b_init_ecat = 0 ;
    ui_control_buttons_.b_reinit_ecat = 0 ;
    ui_control_buttons_.b_enable_drives = 0 ;
    ui_control_buttons_.b_disable_drives = 0 ;
    ui_control_buttons_.b_enable_cyclic_pos = 0 ;
    ui_control_buttons_.b_enable_cyclic_vel = 0 ;
    ui_control_buttons_.b_enable_vel = 0 ;
    ui_control_buttons_.b_enable_pos = 0 ;
    ui_control_buttons_.b_enter_cyclic_pdo = 0 ;
    ui_control_buttons_.b_emergency_mode = 0 ;
    ui_control_buttons_.b_send = 0 ;
    ui_control_buttons_.b_stop_cyclic_pdo = 0 ;
 }

