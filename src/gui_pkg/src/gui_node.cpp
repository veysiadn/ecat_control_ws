#include "../include/gui_pkg/gui_node.hpp"

  using namespace std::chrono_literals;

  using namespace GUI;

  GuiNode::GuiNode() : Node("gui_node")
  {
    received_data_.resize(NUM_OF_SERVO_DRIVES);
    ui_control_buttons_.spn_target_values.resize(NUM_OF_SERVO_DRIVES);
    for(int i = 0; i < NUM_OF_SERVO_DRIVES; i++)
    {
        ui_control_buttons_.spn_target_values[i] = 0 ;
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
    gui_publisher_ = create_publisher<ecat_msgs::msg::GuiButtonData>("gui_buttons", qos);
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
         received_data_[i].target_tor   =  msg->target_tor[i];
         received_data_[i].control_word =  msg->control_word[i];
      }
  }

  void GuiNode::HandleSlaveFeedbackCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg)
  {

      for(int i=0; i < NUM_OF_SERVO_DRIVES ; i++){
        /// Servo Drive feedbacks
        received_data_[i].actual_pos              =  msg->actual_pos[i];
        received_data_[i].actual_vel              =  msg->actual_vel[i];
        received_data_[i].actual_tor              =  msg->actual_tor[i];
        received_data_[i].status_word             =  msg->status_word[i];
        received_data_[i].slave_com_status        =  msg->slave_com_status[i];
        received_data_[i].com_status              =  msg->com_status;
        current_lifecycle_state                   =  msg->current_lifecycle_state;
        /// Costum slave feedbacks.
        received_data_[i].left_limit_switch_val   =  msg->left_limit_switch_val;
        received_data_[i].right_limit_switch_val  =  msg->right_limit_switch_val;
        received_data_[i].p_emergency_switch_val  =  msg->emergency_switch_val;
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

