#include "../include/gui_pkg/gui_node.hpp"

  using namespace std::chrono_literals;
  using namespace GUI;
  GuiNode::GuiNode() : Node("gui_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    controller_commands_= this->create_subscription<sensor_msgs::msg::Joy>("Controller", qos,
                                  std::bind(&GuiNode::HandleControllerCallbacks, this, std::placeholders::_1));
    slave_feedback_ = this->create_subscription<ecat_msgs::msg::DataReceived>("Slave_Feedback", qos,
                                   std::bind(&GuiNode::HandleSlaveFeedbackCallbacks, this, std::placeholders::_1));
    master_commands_ = this->create_subscription<ecat_msgs::msg::DataSent>("Master_Commands", qos,
                                   std::bind(&GuiNode::HandleMasterCommandCallbacks, this, std::placeholders::_1));
    gui_publisher_ = create_publisher<ecat_msgs::msg::GuiButtonData>("gui_buttons", qos);
    timer_ = this->create_wall_timer(1ms,std::bind(&GuiNode::timer_callback,this));
    received_data_[0].p_emergency_switch_val=1;
    ResetContolButtonValues();
  }

  GuiNode::~GuiNode()
  {
    rclcpp::shutdown();
  }

  void GuiNode::timer_callback()
  {
      auto button_info = ecat_msgs::msg::GuiButtonData();
      button_info.b_init = emergency_button_val_;
      gui_publisher_->publish(button_info);
  }
  void GuiNode::HandleControllerCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
     for(int i=0; i < NUM_OF_SERVO_DRIVES ; i++){
        received_data_[i].right_x_axis = msg->axes[3];
        received_data_[i].left_x_axis =  msg->axes[0];
     }
    // emit UpdateParameters(0);
  }

  void GuiNode::HandleMasterCommandCallbacks(const ecat_msgs::msg::DataSent::SharedPtr msg)
  {
      
      for(int i=0; i < NUM_OF_SERVO_DRIVES ; i++){
         received_data_[i].target_pos   =  msg->target_pos[i];
         received_data_[i].target_vel   =  msg->target_vel[i];
         received_data_[i].control_word =  msg->control_word[i];
      }
     // emit UpdateParameters(0);
  }

  void GuiNode::HandleSlaveFeedbackCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg)
  {
//      time_info_.GetTime();
      for(int i=0; i < NUM_OF_SERVO_DRIVES ; i++){
        received_data_[i].actual_pos             =  msg->actual_pos[i];
        received_data_[i].actual_vel             =  msg->actual_vel[i];
        received_data_[i].status_word            =  msg->status_word[i];
        received_data_[i].left_limit_switch_val  =  msg->left_limit_switch_val;
        received_data_[i].right_limit_switch_val =  msg->right_limit_switch_val;
        received_data_[i].p_emergency_switch_val =  msg->emergency_switch_val;
        received_data_[i].com_status             =  msg->com_status;
    }
//    time_info_.MeasureTimeDifference();
//    if (time_info_.counter_ == NUMBER_OF_SAMPLES)
//      time_info_.OutInfoToFile();
     // emit UpdateParameters(0);

  }
 void GuiNode::ResetContolButtonValues()
 {
     control_buttons_.b_init = 0 ;
     control_buttons_.b_reinit = 0 ;
     control_buttons_.b_enable_motor = 0 ;
     control_buttons_.b_disable_motor = 0 ;
     control_buttons_.b_enable_cyclic_pos = 0 ;
     control_buttons_.b_enable_cyclic_vel = 0 ;
     control_buttons_.b_enable_vel = 0 ;
     control_buttons_.b_enable_pos = 0 ;
     control_buttons_.b_enter_cyclic_pdo = 0 ;
     control_buttons_.b_emergency_mode = 0 ;
     control_buttons_.b_send = 0 ;

 }
