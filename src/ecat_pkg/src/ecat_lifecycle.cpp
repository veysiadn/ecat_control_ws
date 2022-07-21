#include <ecat_lifecycle.hpp>

using namespace EthercatLifeCycleNode;

EthercatLifeCycle::EthercatLifeCycle() : LifecycleNode("ecat_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  ecat_node_ = std::make_unique<EthercatNode>();
  received_data_.status_word.resize(g_kNumberOfServoDrivers);
  received_data_.actual_pos.resize(g_kNumberOfServoDrivers);
  received_data_.actual_vel.resize(g_kNumberOfServoDrivers);
  received_data_.actual_tor.resize(g_kNumberOfServoDrivers);
  received_data_.error_code.resize(g_kNumberOfServoDrivers);
  received_data_.op_mode_display.resize(g_kNumberOfServoDrivers);
  received_data_.slave_com_status.resize(g_kNumberOfServoDrivers);
  sent_data_.control_word.resize(g_kNumberOfServoDrivers);
  sent_data_.target_pos.resize(g_kNumberOfServoDrivers);
  sent_data_.target_vel.resize(g_kNumberOfServoDrivers);
  sent_data_.target_tor.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.spn_target_values.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.b_send.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.b_stop.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.b_enable.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.b_disable.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.b_vel.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.b_pos.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.b_tor.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.b_cyclic_vel.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.b_cyclic_pos.resize(g_kNumberOfServoDrivers);
  gui_buttons_status_.b_cyclic_tor.resize(g_kNumberOfServoDrivers);
  received_data_.emergency_switch_val = 1;
  measurement_time = this->declare_parameter("measure_time", std::int32_t(200));

  received_data_.current_lifecycle_state = PRIMARY_STATE_UNCONFIGURED;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  received_data_publisher_ = this->create_publisher<ecat_msgs::msg::DataReceived>("Slave_Feedback", qos);
  joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "Controller", qos, std::bind(&EthercatLifeCycle::HandleControlNodeCallbacks, this, std::placeholders::_1));
  gui_subscriber_ = this->create_subscription<ecat_msgs::msg::GuiButtonData>(
      "gui_buttons", 10, std::bind(&EthercatLifeCycle::HandleGuiNodeCallbacks, this, std::placeholders::_1));
  received_data_publisher_->on_activate();
  inactive_mode_callback_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&EthercatLifeCycle::InactiveModeCallback, this));
}

EthercatLifeCycle::~EthercatLifeCycle()
{
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_configure(const State&)
{
  received_data_.current_lifecycle_state = TRANSITION_STATE_CONFIGURING;
  PublishReceivedData();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring EtherCAT device...\n");
  auto qos = rclcpp::QoS(
      // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
      // are sent, to aid with recovery in the event of dropped messages.
      // "depth" specifies the size of this buffer.
      // In this example, we are optimizing for performance and limited resource usage (preventing
      // page faults), instead of reliability. Thus, we set the size of the history buffer to 1.
      rclcpp::KeepLast(1));
  // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
  // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
  qos.best_effort();

  if (InitEthercatCommunication())
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuration phase failed");
    received_data_.current_lifecycle_state = PRIMARY_STATE_UNCONFIGURED;
    PublishReceivedData();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating publishers...\n");
    sent_data_publisher_ = this->create_publisher<ecat_msgs::msg::DataSent>("Master_Commands", qos);
    received_data_.current_lifecycle_state = PRIMARY_STATE_INACTIVE;
    sent_data_publisher_->on_activate();
    PublishReceivedData();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_activate(const State&)
{
  received_data_.current_lifecycle_state = TRANSITION_STATE_ACTIVATING;
  PublishReceivedData();
  if (StartEthercatCommunication())
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Activation phase failed");
    received_data_.current_lifecycle_state = PRIMARY_STATE_INACTIVE;
    PublishReceivedData();
    /// TODO: clear fault in here. Assuming drive went into fault state. It will be better to clear fault in case of error 
    /// by informing the user.
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  else
  {
    sent_data_publisher_->on_activate();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activation complete, real-time communication started.");
    received_data_.current_lifecycle_state = PRIMARY_STATE_ACTIVE;
    PublishReceivedData();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_deactivate(const State&)
{
  received_data_.current_lifecycle_state = TRANSITION_STATE_DEACTIVATING;
  PublishReceivedData();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivating.\n");
  ecat_node_->DeactivateCommunication();
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    ecat_node_->slaves_[i].slave_pdo_domain_ = NULL;
    ecat_node_->slaves_[i].slave_config_ = NULL;
    ecat_node_->slaves_[i].slave_pdo_entry_info_ = NULL;
    ecat_node_->slaves_[i].slave_pdo_entry_reg_ = NULL;
    ecat_node_->slaves_[i].slave_pdo_info_ = NULL;
    ecat_node_->slaves_[i].slave_sync_info_ = NULL;
    g_master = NULL;
    g_master_domain = NULL;
    g_master_state = {};
  }
  if (InitEthercatCommunication())
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Deactivating phase failed.\n");
    received_data_.current_lifecycle_state = PRIMARY_STATE_ACTIVE;
    PublishReceivedData();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  else
  {
    received_data_.current_lifecycle_state = PRIMARY_STATE_INACTIVE;
    PublishReceivedData();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  // received_data_.current_lifecycle_state = PRIMARY_STATE_INACTIVE ;
  // PublishReceivedData();
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_cleanup(const State&)
{
  received_data_.current_lifecycle_state = TRANSITION_STATE_CLEANINGUP;
  PublishReceivedData();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cleaning up.");
  ecat_node_->ReleaseMaster();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (ecat_node_->RestartEthercatMaster())
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cannot restart EtherCAT master, reinitialization error.");
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    ecat_node_->slaves_[i].slave_pdo_domain_ = NULL;
    ecat_node_->slaves_[i].slave_config_ = NULL;
    ecat_node_->slaves_[i].slave_pdo_entry_info_ = NULL;
    ecat_node_->slaves_[i].slave_pdo_entry_reg_ = NULL;
    ecat_node_->slaves_[i].slave_pdo_info_ = NULL;
    ecat_node_->slaves_[i].slave_sync_info_ = NULL;
    g_master = NULL;
    g_master_domain = NULL;
    g_master_state = {};
  }
  received_data_.current_lifecycle_state = PRIMARY_STATE_UNCONFIGURED;
  PublishReceivedData();
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_shutdown(const State&)
{
  received_data_.current_lifecycle_state = TRANSITION_STATE_SHUTTINGDOWN;
  PublishReceivedData();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "On_Shutdown... Waiting for control thread.");
  sig = 0;
  pthread_cancel(ethercat_thread_);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Control thread terminated.");
  ecat_node_->ReleaseMaster();
  ecat_node_->ShutDownEthercatMaster();
  received_data_.current_lifecycle_state = PRIMARY_STATE_FINALIZED;
  PublishReceivedData();
  received_data_publisher_->on_deactivate();
  sent_data_publisher_->on_deactivate();
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_error(const State&)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error moving state to unconfigured.");
  if (received_data_.current_lifecycle_state == PRIMARY_STATE_ACTIVE)
  {
    on_deactivate(this->get_current_state());
    received_data_.current_lifecycle_state = PRIMARY_STATE_INACTIVE;
    PublishReceivedData();
    on_cleanup(this->get_current_state());
    received_data_.current_lifecycle_state = PRIMARY_STATE_UNCONFIGURED;
    PublishReceivedData();
  }
  if (received_data_.current_lifecycle_state == PRIMARY_STATE_INACTIVE)
  {
    on_cleanup(this->get_current_state());
  }
  received_data_.current_lifecycle_state = PRIMARY_STATE_UNCONFIGURED;
  PublishReceivedData();
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
void EthercatLifeCycle::InactiveModeCallback()
{
  if (received_data_.current_lifecycle_state == PRIMARY_STATE_INACTIVE)
  {
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      received_data_.actual_pos[i] = ecat_node_->ReadActualPositionViaSDO(i);
      received_data_.actual_vel[i] = ecat_node_->ReadActualVelocityViaSDO(i);
      received_data_.actual_tor[i] = ecat_node_->ReadActualTorqueViaSDO(i);
      ecat_node_->CheckMasterState();
      received_data_.com_status = g_master_state.al_states;
      received_data_.status_word[i] = ecat_node_->ReadStatusWordViaSDO(i);
      received_data_.op_mode_display[i] = ecat_node_->ReadOpModeViaSDO(i);
      if (g_kOperationMode == kProfilePosition || g_kOperationMode == kCSPosition)
      {
        sent_data_.target_pos[i] = gui_buttons_status_.spn_target_values[i];
        sent_data_.target_vel[i] = received_data_.actual_vel[i];
        sent_data_.target_tor[i] = received_data_.actual_tor[i];
      }
      else if(g_kOperationMode == kProfileVelocity || g_kOperationMode == kCSVelocity)
      {
        sent_data_.target_vel[i] = gui_buttons_status_.spn_target_values[i];
        sent_data_.target_pos[i] = received_data_.actual_pos[i];
        sent_data_.target_tor[i] = received_data_.actual_tor[i];
      }
      else
      {
        sent_data_.target_tor[i] = gui_buttons_status_.spn_target_values[i];
        sent_data_.target_pos[i] = received_data_.actual_pos[i];
        sent_data_.target_vel[i] = received_data_.actual_vel[i];
      }
    }
    PublishAllData();
  }
}
void EthercatLifeCycle::HandleControlNodeCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joy Msgs : %.2f, %.2f",msg->axes[0],msg->axes[2]);

  controller_.left_x_axis_ = msg->axes[0];
  controller_.left_y_axis_ = msg->axes[1];
  controller_.right_x_axis_ = msg->axes[3];
  controller_.right_y_axis_ = msg->axes[4];

  controller_.left_t_axis_ = (1.0 - msg->axes[2]) / 2.0;
  controller_.right_t_axis_ = (1.0 - msg->axes[5]) / 2.0;

  controller_.green_button_ = msg->buttons[0];
  controller_.red_button_ = msg->buttons[1];
  controller_.blue_button_ = msg->buttons[2];
  controller_.yellow_button_ = msg->buttons[3];
  controller_.left_rb_button_ = msg->buttons[4];
  controller_.right_rb_button_ = msg->buttons[5];
  controller_.left_start_button_ = msg->buttons[6];
  controller_.right_start_button_ = msg->buttons[7];
  controller_.xbox_button_ = msg->buttons[8];
  if (msg->axes[7] > 0)
  {
    controller_.left_d_button_ = 1;
    controller_.left_u_button_ = 0;
  }
  else if (msg->axes[7] < 0)
  {
    controller_.left_d_button_ = 0;
    controller_.left_u_button_ = 1;
  }
  else
  {
    controller_.left_d_button_ = 0;
    controller_.left_u_button_ = 0;
  }

  if (msg->axes[6] > 0)
  {
    controller_.left_r_button_ = 1;
    controller_.left_l_button_ = 0;
  }
  else if (msg->axes[6] < 0)
  {
    controller_.left_r_button_ = 0;
    controller_.left_l_button_ = 1;
  }
  else
  {
    controller_.left_r_button_ = 0;
    controller_.left_l_button_ = 0;
  }
}

void EthercatLifeCycle::HandleGuiNodeCallbacks(const ecat_msgs::msg::GuiButtonData::SharedPtr gui_sub)
{
  gui_buttons_status_ = *gui_sub;
  if (received_data_.current_lifecycle_state == PRIMARY_STATE_INACTIVE)
  {
    if (gui_buttons_status_.b_enable_cyclic_pos)
    {
      g_kOperationMode = kCSPosition;
      for (int i = 0; i < g_kNumberOfServoDrivers; i++)
      {
        ecat_node_->WriteOpModeViaSDO(i, kCSPosition);
        SetConfigurationParameters();
      }
    }

    if (gui_buttons_status_.b_enable_cyclic_vel)
    {
      g_kOperationMode = kCSVelocity;
      for (int i = 0; i < g_kNumberOfServoDrivers; i++)
      {
        ecat_node_->WriteOpModeViaSDO(i, kCSVelocity);
        SetConfigurationParameters();
      }
    }

    if (gui_buttons_status_.b_enable_pos)
    {
      g_kOperationMode = kProfilePosition;
      for (int i = 0; i < g_kNumberOfServoDrivers; i++)
      {
        ecat_node_->WriteOpModeViaSDO(i, kProfilePosition);
        SetConfigurationParameters();
      }
    }

    if (gui_buttons_status_.b_enable_vel)
    {
      g_kOperationMode = kProfileVelocity;
      for (int i = 0; i < g_kNumberOfServoDrivers; i++)
      {
        ecat_node_->WriteOpModeViaSDO(i, kProfileVelocity);
        SetConfigurationParameters();
      }
    }

    if (gui_buttons_status_.b_enable_drives)
    {
      for (int i = 0; i < g_kNumberOfServoDrivers; i++)
      {
        EnableDrivesViaSDO(i);
      }
    }

    if (gui_buttons_status_.b_disable_drives)
    {
      for (int i = 0; i < g_kNumberOfServoDrivers; i++)
      {
        DisableDrivesViaSDO(i);
      }
    }
    /// TODO: Update this statement with b_send.
    if (gui_buttons_status_.b_disable_drives)
    {
      switch (g_kOperationMode)
      {
        case kProfilePosition:
          for (int i = 0; i < g_kNumberOfServoDrivers; i++)
          {
            ecat_node_->WriteTargetPositionViaSDO(i, gui_buttons_status_.spn_target_values[i]);
          }
          break;
        case kProfileTorque:
          for (int i = 0; i < g_kNumberOfServoDrivers; i++)
          {
            ecat_node_->WriteTargetTorqueViaSDO(i, gui_buttons_status_.spn_target_values[i]);
          }
          break;
        case kCSPosition:
          for (int i = 0; i < g_kNumberOfServoDrivers; i++)
          {
            ecat_node_->WriteTargetPositionViaSDO(i, gui_buttons_status_.spn_target_values[i]);
          }
          break;
        case kCSTorque:
          for (int i = 0; i < g_kNumberOfServoDrivers; i++)
          {
            ecat_node_->WriteTargetTorqueViaSDO(i, gui_buttons_status_.spn_target_values[i]);
          }
          break;
        case kCSVelocity:
          for (int i = 0; i < g_kNumberOfServoDrivers; i++)
          {
            ecat_node_->WriteTargetVelocityViaSDO(i, gui_buttons_status_.spn_target_values[i]);
          }
          break;
        case kProfileVelocity:
          for (int i = 0; i < g_kNumberOfServoDrivers; i++)
          {
            ecat_node_->WriteTargetVelocityViaSDO(i, gui_buttons_status_.spn_target_values[i]);
          }
          break;
        default:
          for (int i = 0; i < g_kNumberOfServoDrivers; i++)
          {
            ecat_node_->WriteTargetVelocityViaSDO(i, gui_buttons_status_.spn_target_values[i]);
          }
          break;
      }
    }
  }
}

int EthercatLifeCycle::SetComThreadPriorities()
{
  ethercat_sched_param_.sched_priority = sched_get_priority_max(SCHED_FIFO);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using priority %i\n.", ethercat_sched_param_.sched_priority);

  if (sched_setscheduler(0, SCHED_FIFO, &ethercat_sched_param_) == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set scheduler failed. ! ");
    return -1;
  }
  err_ = pthread_attr_init(&ethercat_thread_attr_);
  if (err_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error initializing thread attribute  ! ");
    return -1;
  }
  /**********************************************************************************************/
  // This part is for CPU isolation to dedicate one core for EtherCAT communication.
  // for this feature to be active fist you have to modify GRUB_CMDLINE_LINUX_DEFAULT in /etc/default/grub
  // add isolcpus=3 so after editing it will be ; GRUB_CMDLINE_LINUX_DEFAULT = "quiet splash isolcpus=3"
  // save and exit, and type sudo update-grub and reboot.
  //  cpu_set_t mask;
  // CPU_ZERO(&mask);
  // CPU_SET(3,&mask);

  // int result = sched_setaffinity(0,sizeof(mask),&mask);
  /**********************************************************************************************/

  /* Set a specific stack size  */
  err_ = pthread_attr_setstacksize(&ethercat_thread_attr_, 4096 * 4096);
  if (err_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error setting thread stack size  ! ");
    return -1;
  }

  err_ = pthread_attr_setschedpolicy(&ethercat_thread_attr_, SCHED_FIFO);
  if (err_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Pthread setschedpolicy failed ! ");
    return -1;
  }
  err_ = pthread_attr_setschedparam(&ethercat_thread_attr_, &ethercat_sched_param_);
  if (err_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Pthread setschedparam failed ! ");
    return -1;
  }
  /* Use scheduling parameters of attr */
  err_ = pthread_attr_setinheritsched(&ethercat_thread_attr_, PTHREAD_EXPLICIT_SCHED);
  if (err_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Pthread setinheritsched failed ! ");
    return -1;
  }
}

int EthercatLifeCycle::InitEthercatCommunication()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening EtherCAT device...\n");
  if (ecat_node_->OpenEthercatMaster())
  {
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring EtherCAT master...\n");
  if (ecat_node_->ConfigureMaster())
  {
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting connected slave informations...\n");
  if (ecat_node_->GetNumberOfConnectedSlaves())
  {
    ecat_node_->ReleaseMaster();
    for (int i = 0; i < NUM_OF_SLAVES; i++)
    {
      ecat_node_->slaves_[i].slave_pdo_domain_ = NULL;
      ecat_node_->slaves_[i].slave_config_ = NULL;
      ecat_node_->slaves_[i].slave_pdo_entry_info_ = NULL;
      ecat_node_->slaves_[i].slave_pdo_entry_reg_ = NULL;
      ecat_node_->slaves_[i].slave_pdo_info_ = NULL;
      ecat_node_->slaves_[i].slave_sync_info_ = NULL;
      g_master = NULL;
      g_master_domain = NULL;
      g_master_state = {};
    }
    return -1;
  }

  ecat_node_->GetAllSlaveInformation();
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "\n"
                "--------------------Slave Info -------------------------\n"
                "Slave alias         = %d\n "
                "Slave position      = %d\n "
                "Slave vendor_id     = 0x%08x\n "
                "Slave product_code  = 0x%08x\n "
                "Slave name          = %s\n "
                "--------------------EOF %d'th Slave Info ----------------\n ",
                ecat_node_->slaves_[i].slave_info_.alias, ecat_node_->slaves_[i].slave_info_.position,
                ecat_node_->slaves_[i].slave_info_.vendor_id, ecat_node_->slaves_[i].slave_info_.product_code,
                ecat_node_->slaves_[i].slave_info_.name, i);
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring  slaves...\n");
  if (ecat_node_->ConfigureSlaves())
  {
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Registering SDOs ...\n");
  if (ecat_node_->MapDefaultSdos())
  {
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Assigning configuration parameters...\n");
  if (SetConfigurationParameters())
  {
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mapping default PDOs...\n");

  if (ecat_node_->MapDefaultPdos())
  {
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring DC synchronization...\n");
  if (DISTRIBUTED_CLOCK)
    ecat_node_->ConfigDcSyncDefault();
  return 0;
}

int EthercatLifeCycle::SetConfigurationParameters()
{
  int error = 0;
  if (g_kOperationMode == kProfilePosition)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting drives to Position mode...\n");
    ProfilePosParam P;
    uint32_t max_fol_err;
    P.profile_vel = 450;
    P.profile_acc = 1e4;
    P.profile_dec = 1e4;
    P.max_profile_vel = 1e3;
    P.quick_stop_dec = 3e4;
    P.motion_profile_type = 0;
    for (int i; i < g_kNumberOfServoDrivers; i++)
    {
      error = ecat_node_->SetProfilePositionParameters(P, i);
    }
  }
  else if (g_kOperationMode == kCSPosition)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting drives to CSP mode...\n");
    CSPositionModeParam P;
    uint32_t max_fol_err;
    P.profile_vel = 50;
    P.profile_acc = 3e4;
    P.profile_dec = 3e4;
    P.max_profile_vel = 1e3;
    P.quick_stop_dec = 3e4;
    P.interpolation_time_period = 0;
    for (int i; i < g_kNumberOfServoDrivers; i++)
    {
      error = ecat_node_->SetCyclicSyncPositionModeParameters(P, i);
    }
  }
  else if (g_kOperationMode == kCSVelocity)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting drives to CSV mode...\n");
    CSVelocityModeParam P;
    P.profile_dec = 3e4;
    P.max_profile_vel = 1e3;
    P.quick_stop_dec = 3e4;
    P.interpolation_time_period = 0;
    for (int i; i < g_kNumberOfServoDrivers; i++)
    {
      error = ecat_node_->SetCyclicSyncVelocityModeParameters(P, i);
    }
  }
  else if (g_kOperationMode == kCSTorque)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting drives to CST mode...\n");
    CSTorqueModeParam P;
    P.profile_dec = 3e4;
    P.quick_stop_dec = 3e4;
    for (int i; i < g_kNumberOfServoDrivers; i++)
    {
      error = ecat_node_->SetCyclicSyncTorqueModeParameters(P, i);
    }
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting drives to Velocity mode...\n");
    g_kOperationMode == kProfileVelocity;
    ProfileVelocityParam P;
#if 0
    P.profile_acc = 1e6;
    P.profile_dec = 1e6;
    P.max_profile_vel = 1e5;
    P.quick_stop_dec = 1e6;
    P.motion_profile_type = 0;
    for (int i = 0; i < 4; i++)
    {
      error = ecat_node_->SetProfileVelocityParameters(P, i);
    }
    P.profile_acc = 1e4;
    P.profile_dec = 1e4;
    P.max_profile_vel = 1e3;
    P.quick_stop_dec = 1e4;
    P.motion_profile_type = 0;
    for (int i = 4; i < g_kNumberOfServoDrivers; i++)
    {
      error = ecat_node_->SetProfileVelocityParameters(P, i);
    }
#else
    P.profile_acc = 3e4;
    P.profile_dec = 3e4;
    P.max_profile_vel = 5e3;
    P.quick_stop_dec = 3e4;
    P.motion_profile_type = 0;
    for (int i; i < g_kNumberOfServoDrivers; i++)
    {
      error = ecat_node_->SetProfileVelocityParameters(P, i);
    }
#endif
  }
  return error;
}

int EthercatLifeCycle::StartEthercatCommunication()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activating master...\n");
  if (ecat_node_->ActivateMaster())
  {
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Registering master domain...\n");
  if (ecat_node_->RegisterDomain())
  {
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for operational mode...\n");
  if (ecat_node_->WaitForOperationalMode())
  {
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialization succesfull...\n");

  if (SetComThreadPriorities())
  {
    return -1;
  }
  err_ = pthread_create(&ethercat_thread_, &ethercat_thread_attr_, &EthercatLifeCycle::PassCycylicExchange, this);
  if (err_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : Couldn't start communication thread.!");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Communication thread called.\n");
  return 0;
}

void* EthercatLifeCycle::PassCycylicExchange(void* arg)
{
  static_cast<EthercatLifeCycle*>(arg)->StartPdoExchange(arg);
}

void EthercatLifeCycle::StartPdoExchange(void* instance)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting PDO exchange....\n");
  // Measurement time in minutes, e.g.
  uint32_t print_max_min = measurement_time * 60000;
  uint32_t print_val = 1e4;
  int error_check = 0;
  int32_t status_var = 0;
  struct timespec wake_up_time, time, publish_time_start = {}, publish_time_end = {};
#if MEASURE_TIMING
  struct timespec start_time, end_time, last_start_time = {};
  uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0, latency_min_ns = 0xffffffff, latency_max_ns = 0,
           period_min_ns = 0xffffffff, period_max_ns = 0, exec_min_ns = 0xffffffff, exec_max_ns = 0, max_period = 0,
           max_latency = 0, exec_max = 0, min_period = 0xffffffff, exec_min = 0xffffffff, latency_min = 0xffffffff;
  int32_t publishing_time_ns = 1e4, publish_time_max = 0, publish_time_min = 0xfffffff;
  int32_t jitter = 0, jitter_min = 0xfffffff, jitter_max = 0, old_latency = 0;

#endif
  // get current time
  int begin = 1e4;
  int status_check_counter = 1000;

  // ------------------------------------------------------- //
  // CKim - Initialization loop before entring control loop.
  // Switch On and Enable Driver
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enabling motors...");
  clock_gettime(CLOCK_TO_USE, &wake_up_time);
  while (sig && !gui_buttons_status_.b_stop_cyclic_pdo)
  {
    // CKim - Sleep for 1 ms
    wake_up_time = timespec_add(wake_up_time, g_cycle_time);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
    ecrt_master_application_time(g_master, TIMESPEC2NS(wake_up_time));

    // CKim - Receive process data
    ecrt_master_receive(g_master);
    ecrt_domain_process(g_master_domain);
    ReadFromSlaves();

    // CKim - Initialize target pos and vel
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      sent_data_.target_pos[i] = received_data_.actual_pos[i];
      sent_data_.target_vel[i] = 0;
    }

    // CKim - Check status and update control words to enable drivers
    // Returns number of enabled drivers
    if (EnableDrivers() == g_kNumberOfServoDrivers)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "All drives enabled");
      break;
    }

    // CKim - Periodic printout
    if (status_check_counter)
    {
      status_check_counter--;
    }
    else
    {
      // Checking master/domain/slaves state every 1sec.
      if (ecat_node_->CheckMasterState() < 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Connection error, check your physical connection.");
        al_state_ = g_master_state.al_states;
        received_data_.emergency_switch_val = 0;
        emergency_status_ = 0;
        PublishAllData();
        error_check++;
        if (error_check == 5)
          return;
      }
      else
      {
        // ecat_node_->CheckMasterDomainState();
        // ecat_node_->CheckSlaveConfigurationState();
        error_check = 0;
        al_state_ = g_master_state.al_states;
        status_check_counter = 1000;

        for (int i = 0; i < g_kNumberOfServoDrivers; i++)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State of Drive %d : %d\n", i, motor_state_[i]);
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to enable motors");
        }
        PublishAllData();
      }
    }

    // CKim - Queue data
    PublishAllData();
    // WriteToSlavesInPositionMode();
    UpdateControlParameters();

    // CKim - Sync Timer
    if (g_sync_ref_counter)
    {
      g_sync_ref_counter--;
    }
    else
    {
      clock_gettime(CLOCK_TO_USE, &time);
      ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(time));
      g_sync_ref_counter = 1;
    }
    ecrt_master_sync_slave_clocks(g_master);
    ecrt_domain_queue(g_master_domain);
    // CKim - Send process data
    ecrt_master_send(g_master);
  }  // while(sig)
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "All motors enabled, entering control loop");

  // ------------------------------------------------------- //
  // CKim - All motors enabled. Start control loop
  while (sig && !gui_buttons_status_.b_stop_cyclic_pdo)
  {
    wake_up_time = timespec_add(wake_up_time, g_cycle_time);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
    ecrt_master_application_time(g_master, TIMESPEC2NS(wake_up_time));

#if MEASURE_TIMING
    clock_gettime(CLOCK_TO_USE, &start_time);
    old_latency = latency_ns;
    latency_ns = DIFF_NS(wake_up_time, start_time);
    period_ns = DIFF_NS(last_start_time, start_time);
    exec_ns = DIFF_NS(last_start_time, end_time);
    last_start_time = start_time;
    if (!begin)
    {
      jitter = latency_ns - old_latency;
      if (jitter < 0)
        jitter *= -1;
      if (jitter > jitter_max)
        jitter_max = jitter;
      if (latency_ns > max_latency)
        max_latency = latency_ns;
      if (period_ns > max_period)
        max_period = period_ns;
      if (exec_ns > exec_max)
        exec_max = exec_ns;
      if (period_ns < min_period)
        min_period = period_ns;
      if (exec_ns < exec_min)
        exec_min = exec_ns;
      if (latency_ns < latency_min)
        latency_min = latency_ns;
      if (jitter < jitter_min)
        jitter_min = jitter;
    }

    if (latency_ns > latency_max_ns)
    {
      latency_max_ns = latency_ns;
    }
    if (latency_ns < latency_min_ns)
    {
      latency_min_ns = latency_ns;
    }
    if (period_ns > period_max_ns)
    {
      period_max_ns = period_ns;
    }
    if (period_ns < period_min_ns)
    {
      period_min_ns = period_ns;
    }
    if (exec_ns > exec_max_ns)
    {
      exec_max_ns = exec_ns;
    }
    if (exec_ns < exec_min_ns)
    {
      exec_min_ns = exec_ns;
    }
#endif

    // receive process data
    ecrt_master_receive(g_master);
    ecrt_domain_process(g_master_domain);
    // wake_up_time = timespec_add(wake_up_time, g_half_cycle_time);
    // clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);

    if (status_check_counter)
    {
      status_check_counter--;
    }
    else
    {
      // Checking master/domain/slaves state every 1sec.
      if (ecat_node_->CheckMasterState() < 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Connection error, check your physical connection.");
        al_state_ = g_master_state.al_states;
        received_data_.emergency_switch_val = 0;
        emergency_status_ = 0;
        PublishAllData();
        error_check++;
        if (error_check == 5)
          return;
      }
      else
      {
        // ecat_node_->CheckMasterDomainState();
        // ecat_node_->CheckSlaveConfigurationState();
        error_check = 0;
        al_state_ = g_master_state.al_states;
        status_check_counter = 1000;
      }
    }

#if MEASURE_TIMING
// if you want to print timing stats
#if 0
            if(!print_val){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"-----------------------------------------------\n\n");
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tperiod   min   : %10u ns  | max : %10u ns\n",
                        period_min_ns, period_max_ns);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Texec     min   : %10u ns  | max : %10u ns\n",
                        exec_min_ns, exec_max_ns);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tlatency  min   : %10u ns  | max : %10u ns\n",
                        latency_min_ns, latency_max_ns);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tjitter max     : %10u ns  \n",
                        latency_max_ns-latency_min_ns);                       
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"-----------------------------------------------\n\n");       
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tperiod min     : %10u ns  | max : %10u ns\n",
                        min_period, max_period);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Texec  min      : %10u ns  | max : %10u ns\n",
                        exec_min, exec_max);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tjitter min     : %10u ns  | max : %10u ns\n",
                        jitter_min, jitter_max);  
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Publish time min: %10d ns  | max : %10d ns\n",
                      publish_time_min, publish_time_max);                             
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"-----------------------------------------------\n\n");
                
                // std::cout << min_period << " " << max_period << " " << exec_min << " " << exec_max << " " << jitter_min << " " << jitter_max << std::endl;
                /*  std::cout <<    "Left Switch   : " << unsigned(received_data_.left_limit_switch_val) << std::endl << 
                                    "Right Switch  : " << unsigned(received_data_.right_limit_switch_val) << std::endl;
                    std::cout << "Left X Axis    : " << controller_.left_x_axis_ << std::endl;
                    std::cout << "Right X XAxis  : " << controller_.right_x_axis_ << std::endl;*/
                    // std::cout << "Emergency button  : " << unsigned(gui_node_data_) << std::endl;
                //std::cout << std::dec << publishing_time_ns << std::endl;
                //std::cout << std::dec << time_span.count() << std::endl;
                    print_val=1000;

                    //  std::cout << "Finished...." << std::endl;
                    //  break;
            }else {
                //std::cout << std::dec << period_min_ns  << " " << period_max_ns << " " << exec_min_ns << " " << exec_max_ns << " " << 
               // latency_min_ns << " " <<  latency_max_ns << " " << latency_max_ns - latency_min_ns << std::endl;
                print_val--;
            }
#endif
    print_max_min--;
    period_max_ns = 0;
    period_min_ns = 0xffffffff;

    exec_max_ns = 0;
    exec_min_ns = 0xffffffff;

    latency_max_ns = 0;
    latency_min_ns = 0xffffffff;
    received_data_.period_ns = period_ns;
    received_data_.exec_ns = exec_ns;
    received_data_.jitter_ns = jitter;
#endif

    ReadFromSlaves();

    UpdateControlParameters();
#if MEASURE_TIMING
    if (!begin)
      clock_gettime(CLOCK_TO_USE, &publish_time_start);
#endif

    PublishAllData();
#if MEASURE_TIMING
    if (!begin)
      clock_gettime(CLOCK_TO_USE, &publish_time_end);
    publishing_time_ns = DIFF_NS(publish_time_start, publish_time_end);
    if (publishing_time_ns > publish_time_max)
      publish_time_max = publishing_time_ns;
    if (publishing_time_ns < publish_time_min)
      publish_time_min = publishing_time_ns;
#endif
    if (g_sync_ref_counter)
    {
      g_sync_ref_counter--;
    }
    else
    {
      clock_gettime(CLOCK_TO_USE, &time);
      ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(time));
      g_sync_ref_counter = 1;
    }
    ecrt_master_sync_slave_clocks(g_master);

    ecrt_domain_queue(g_master_domain);
    // send process data
    ecrt_master_send(g_master);

    if (begin)
      begin--;
#if MEASURE_TIMING
    clock_gettime(CLOCK_TO_USE, &end_time);
#endif
  }  // while(1/sig) //Ctrl+C signal

  // ------------------------------------------------------- //
  // CKim - Disable drivers before exiting
  wake_up_time = timespec_add(wake_up_time, g_cycle_time);
  clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
  ecrt_master_application_time(g_master, TIMESPEC2NS(wake_up_time));

  ecrt_master_receive(g_master);
  ecrt_domain_process(g_master_domain);

  ReadFromSlaves();
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    sent_data_.control_word[i] = SM_GO_SWITCH_ON_DISABLE;
  }
  UpdateControlParameters();

  ecrt_domain_queue(g_master_domain);
  ecrt_master_send(g_master);
  // ------------------------------------------------------- //

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Leaving control thread.");
  pthread_exit(NULL);
}  // StartPdoExchange end

void EthercatLifeCycle::UpdateControlParameters()
{
  if (g_kOperationMode == kProfilePosition)
  {
    UpdatePositionModeParameters();
    WriteToSlavesInPositionMode();
  }
  else if (g_kOperationMode == kCSPosition)
  {
    UpdateCyclicPositionModeParameters();
    WriteToSlavesInPositionMode();
  }
  else if (g_kOperationMode == kCSVelocity)
  {
    UpdateCyclicVelocityModeParameters();
    WriteToSlavesVelocityMode();
  }
  else if (g_kOperationMode == kCSTorque)
  {
    UpdateCyclicTorqueModeParameters();
    WriteToSlavesInCyclicTorqueMode();
  }
  else
  {
    UpdateVelocityModeParameters();
    WriteToSlavesVelocityMode();
  }
}

void EthercatLifeCycle::ReadFromSlaves()
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    received_data_.actual_pos[i] =
        EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.actual_pos);
    received_data_.actual_vel[i] =
        EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.actual_vel);
    received_data_.status_word[i] =
        EC_READ_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.status_word);
    received_data_.actual_tor[i] =
        EC_READ_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.actual_tor);
    received_data_.error_code[i] =
        EC_READ_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.error_code);
  }
  received_data_.com_status = al_state_;
#if CUSTOM_SLAVE
  received_data_.right_limit_switch_val = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +
                                                     ecat_node_->slaves_[FINAL_SLAVE].offset_.r_limit_switch);
  received_data_.left_limit_switch_val = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +
                                                    ecat_node_->slaves_[FINAL_SLAVE].offset_.l_limit_switch);
  received_data_.emergency_switch_val = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +
                                                   ecat_node_->slaves_[FINAL_SLAVE].offset_.emergency_switch);
  emergency_status_ = received_data_.emergency_switch_val;
#else
  emergency_status_ = 1;
  gui_node_data_ = 1;
  received_data_.emergency_switch_val = 1;
#endif
}  // ReadFromSlaves end

void EthercatLifeCycle::WriteToSlavesVelocityMode()
{
  //  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Writing to slaves....\n");
  if (!emergency_status_ || !gui_node_data_)
  {
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                   sent_data_.control_word[i]);
      EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_vel, 0);
    }
  }
  else
  {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Writing to slaves....\n");
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                   sent_data_.control_word[i]);
      EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_vel,
                   sent_data_.target_vel[i]);
    }
  }
}

int EthercatLifeCycle::PublishAllData()
{
  // received_data_.header.stamp = this->now();
  received_data_publisher_->publish(received_data_);
  // sent_data_.header.stamp  = this->now();
  sent_data_publisher_->publish(sent_data_);
}

int EthercatLifeCycle::PublishReceivedData()
{
  received_data_.header.stamp = this->now();
  received_data_publisher_->publish(received_data_);
}

int EthercatLifeCycle::GetComState()
{
  return al_state_;
}

void EthercatLifeCycle::UpdatePositionModeParameters()
{
  /// WRITE YOUR CUSTOM CONTROL ALGORITHM, VARIABLES DECLARATAION HERE, LIKE IN EXAMPLE BELOW.
  /// KEEP IN MIND THAT YOU WILL HAVE TO WAIT FOR THE MOTION TO FINISH IN POSITION MODE, THEREFORE
  /// YOU HAVE TO CHECK 10th BIT OF STATUS WORD TO CHECK WHETHER TARGET IS REACHED OR NOT.
  static uint8_t operation_ready = 0;
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    if (motor_state_[i] == kOperationEnabled || motor_state_[i] == kTargetReached || motor_state_[i] == kSwitchedOn)
    {
      if (controller_.xbox_button_)
      {
        for (int j = 0; j < g_kNumberOfServoDrivers; j++)
        {
          sent_data_.target_pos[j] = 0;
          if (!operation_ready)
          {
            sent_data_.control_word[j] = SM_RUN;
            if (TEST_BIT(received_data_.status_word[j], 10))
            {
              operation_ready = 1;
            }
          }
          else
          {
            sent_data_.control_word[0] = SM_GO_ENABLE;
            operation_ready = 0;
          }
        }
        break;
      }
      // Settings for motor 1;
      if (controller_.red_button_ > 0)
      {
        sent_data_.target_pos[0] = -FIVE_DEGREE_CCW;
      }
      if (controller_.blue_button_ > 0)
      {
        sent_data_.target_pos[0] = FIVE_DEGREE_CCW;
      }
      if (controller_.green_button_ > 0)
      {
        sent_data_.target_pos[0] = THIRTY_DEGREE_CCW;
      }
      if (controller_.yellow_button_ > 0)
      {
        sent_data_.target_pos[0] = -THIRTY_DEGREE_CCW;
      }

      if (controller_.red_button_ || controller_.blue_button_ || controller_.green_button_ ||
          controller_.yellow_button_)
      {
        sent_data_.control_word[0] = SM_GO_ENABLE;
        if (!operation_ready)
        {
          sent_data_.control_word[0] = SM_RUN;
          if (TEST_BIT(received_data_.status_word[0], 10))
          {
            operation_ready = 1;
          }
        }
        else
        {
          sent_data_.control_word[0] = SM_GO_ENABLE;
          operation_ready = 0;
        }
      }
      // Settings for motor 2
      if (controller_.left_r_button_ > 0)
      {
        sent_data_.target_pos[1] = FIVE_DEGREE_CCW;
      }
      if (controller_.left_l_button_ > 0)
      {
        sent_data_.target_pos[1] = -FIVE_DEGREE_CCW;
      }
      if (controller_.left_u_button_ > 0)
      {
        sent_data_.target_pos[1] = -THIRTY_DEGREE_CCW;
      }
      if (controller_.left_d_button_ > 0)
      {
        sent_data_.target_pos[1] = THIRTY_DEGREE_CCW;
      }

      if ((controller_.left_r_button_ || controller_.left_l_button_ || controller_.left_u_button_ ||
           controller_.left_d_button_))
      {
        sent_data_.control_word[1] = SM_GO_ENABLE;
      }

      // Settings for motor 3
      if (controller_.right_rb_button_ > 0)
      {
        sent_data_.target_pos[2] = -FIVE_DEGREE_CCW;
      }
      if (controller_.left_rb_button_ > 0)
      {
        sent_data_.target_pos[2] = FIVE_DEGREE_CCW;
      }
      if (controller_.left_start_button_ > 0)
      {
        sent_data_.target_pos[2] = THIRTY_DEGREE_CCW;
      }
      if (controller_.right_start_button_ > 0)
      {
        sent_data_.target_pos[2] = -THIRTY_DEGREE_CCW;
      }
      if ((controller_.right_rb_button_ || controller_.left_rb_button_ || controller_.left_start_button_ ||
           controller_.right_start_button_))
      {
        sent_data_.control_word[2] = SM_GO_ENABLE;
      }
    }
  }
}

void EthercatLifeCycle::UpdateMotorStatePositionMode()
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    if ((received_data_.status_word[i] & command_) == 0X08)
    {
      // if status is fault, reset fault state.
      command_ = 0X04F;
      sent_data_.control_word[i] = SM_FULL_RESET;
      motor_state_[i] = kFault;
    }
    if (motor_state_[i] != kSwitchedOn)
    {
      sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
      if ((received_data_.status_word[i] & command_) == 0x0040)
      {
        // If status is "Switch on disabled", \
                change state to "Ready to switch on"
        sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
        command_ = 0x006f;
        motor_state_[i] = kSwitchOnDisabled;
      }
      else if ((received_data_.status_word[i] & command_) == 0x0021)
      {
        // If status is "Ready to switch on", \
                        change state to "Switched on"
        sent_data_.control_word[i] = SM_GO_SWITCH_ON;
        command_ = 0x006f;
        motor_state_[i] = kReadyToSwitchOn;
      }
      else if ((received_data_.status_word[i] & command_) == 0x0023)
      {
        // If status is "Switched on", change state to "Operation enabled"
        sent_data_.control_word[i] = SM_GO_ENABLE;
        command_ = 0x006f;
        motor_state_[i] = kSwitchedOn;
      }
      else if ((received_data_.status_word[i] & command_) == 0X08)
      {
        // if status is fault, reset fault state.
        command_ = 0X04f;

        sent_data_.control_word[i] = SM_FULL_RESET;
        motor_state_[i] = kFault;
      }
    }
    else
    {
      sent_data_.control_word[i] = SM_GO_ENABLE;
      if (TEST_BIT(received_data_.status_word[i], 10) == 1)
      {
        sent_data_.control_word[i] = SM_RELATIVE_POS;
      }
    }
  }
  return;
}

int EthercatLifeCycle::GetDriveState(const int& statusWord)
{
  int state = 0;

  // bit 6 is 1
  if (TEST_BIT(statusWord, 6))
  {
    state = kSwitchOnDisabled;
    return state;
  }

  // bit 6 is 0 and bit 5 is 1
  if (TEST_BIT(statusWord, 5))
  {
    if (TEST_BIT(statusWord, 2))
    {
      state = kOperationEnabled;
      return state;
    }
    if (TEST_BIT(statusWord, 1))
    {
      state = kSwitchedOn;
      return state;
    }
    if (TEST_BIT(statusWord, 0))
    {
      state = kReadyToSwitchOn;
      return state;
    }
  }

  // bit 6 is 0 and bit 5 is 0
  if (TEST_BIT(statusWord, 3))
  {
    // For EPOS4, Fault or Fault Reaction Active,
    // See P2-14 of the Firmware Manual
    state = kFault;
    return state;
  }
  else
  {
    // For EPOS4, Quick Stop Active or Not Switched on
    // See P2-14 of the Firmware Manual
    state = kQuickStop;
    return state;
  }
  return state;
}

int EthercatLifeCycle::EnableDrivers()
{
  int cnt = 0;
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    motor_state_[i] = GetDriveState(received_data_.status_word[i]);

    // if status is fault, reset fault state.
    if (motor_state_[i] == kFault)
    {
      // RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Driver %d in Fault",i);
      sent_data_.control_word[i] = SM_FULL_RESET;
    }

    // If status is "Switch on disabled", change state to "Ready to switch on"
    if (motor_state_[i] == kSwitchOnDisabled)
    {
      sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
    }

    // If status is "Ready to switch on", change state to "Switched on"
    if (motor_state_[i] == kReadyToSwitchOn)
    {
      sent_data_.control_word[i] = SM_GO_SWITCH_ON;
    }

    // If status is "Switched on", change state to "Operation enabled"
    if (motor_state_[i] == kSwitchedOn)
    {
      sent_data_.control_word[i] = SM_GO_ENABLE;
    }

    // If status is "Switched on", change state to "Operation enabled"
    if (motor_state_[i] == kOperationEnabled)
    {
      cnt++;
    }
  }
  return cnt;
}

void EthercatLifeCycle::WriteToSlavesInPositionMode()
{
  // if(!received_data_.left_limit_switch_val || !received_data_.right_limit_switch_val){
  if (false)
  {
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      if (sent_data_.target_pos[i] > 0)
      {
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                     sent_data_.control_word[i]);
        EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_pos,
                     sent_data_.target_pos[i]);
      }
      else
      {
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                     SM_QUICKSTOP);
      }
    }
  }
  else
  {
    if (!emergency_status_ || !gui_node_data_)
    {
      for (int i = 0; i < g_kNumberOfServoDrivers; i++)
      {
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                     SM_QUICKSTOP);
        //      EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_pos,0);
      }
    }
    else
    {
      for (int i = 0; i < g_kNumberOfServoDrivers; i++)
      {
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                     sent_data_.control_word[i]);
        EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_pos,
                     sent_data_.target_pos[i]);
      }
    }
  }
}

// void EthercatLifeCycle::UpdateCyclicPositionModeParameters()
// {
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating control parameters....\n");
//     for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
//         if(motor_state_[i]==kOperationEnabled || motor_state_[i]==kTargetReached || motor_state_[i]==kSwitchedOn){
//             if (controller_.xbox_button_){
//                 for(int j = 0 ; j < g_kNumberOfServoDrivers ; j++){
//                     sent_data_.target_pos[j] = 0 ;
//                     sent_data_.control_word[j] = SM_GO_ENABLE ;
//                 }
//                 break;
//             }
//             // Settings for motor 1;
//             if(controller_.red_button_ > 0 ){
//                 sent_data_.target_pos[0] = received_data_.actual_pos[0] - FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.blue_button_ > 0){
//                 sent_data_.target_pos[0] = received_data_.actual_pos[0] + FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.green_button_ > 0 ){
//                 sent_data_.target_pos[0] = received_data_.actual_pos[0] + THIRTY_DEGREE_CCW/50 ;
//             }
//             if(controller_.yellow_button_ > 0){
//                 sent_data_.target_pos[0] = received_data_.actual_pos[0] -THIRTY_DEGREE_CCW/50 ;
//             }
//
//             if(controller_.red_button_ || controller_.blue_button_ || controller_.green_button_ ||
//             controller_.yellow_button_){
//                 sent_data_.control_word[0] = SM_GO_ENABLE;
//             }
//             // Settings for motor 2
//             if(controller_.left_r_button_ > 0 ){
//                 sent_data_.target_pos[1] = received_data_.actual_pos[1] + FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.left_l_button_ > 0){
//                 sent_data_.target_pos[1] = received_data_.actual_pos[1] - FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.left_u_button_ > 0 ){
//                 sent_data_.target_pos[1] = received_data_.actual_pos[1] -THIRTY_DEGREE_CCW/50 ;
//             }
//             if(controller_.left_d_button_ > 0){
//                 sent_data_.target_pos[1] = received_data_.actual_pos[1] + THIRTY_DEGREE_CCW/50 ;
//             }
//
//             if((controller_.left_r_button_ || controller_.left_l_button_ || controller_.left_u_button_ ||
//             controller_.left_d_button_)){
//                 sent_data_.control_word[1] = SM_GO_ENABLE;
//             }
//
//             // Settings for motor 3
//             if(controller_.right_rb_button_ > 0 ){
//                 sent_data_.target_pos[2] = received_data_.actual_pos[2] + FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.left_rb_button_ > 0){
//                 sent_data_.target_pos[2] = received_data_.actual_pos[2] - FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.left_start_button_ > 0 ){
//                 sent_data_.target_pos[2] = received_data_.actual_pos[2] + THIRTY_DEGREE_CCW/50 ;
//             }
//             if(controller_.right_start_button_ > 0){
//                 sent_data_.target_pos[2] = received_data_.actual_pos[2] - THIRTY_DEGREE_CCW/50 ;
//             }
//             if((controller_.right_rb_button_ || controller_.left_rb_button_ || controller_.left_start_button_ ||
//             controller_.right_start_button_)){
//                 sent_data_.control_word[2] = SM_GO_ENABLE;
//             }
//         }
//     }
// }

// CKim - Modifications
void EthercatLifeCycle::UpdateCyclicPositionModeParameters()
{
  /// WRITE YOUR CUSTOM CONTROL ALGORITHM, VARIABLES DECLARATAION HERE, LIKE IN EXAMPLE BELOW.
  float deadzone = 0.05;
  float amp = 1.0 - deadzone;
  float val;
  static uint8_t operation_ready = 0;
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    if (motor_state_[i] == kOperationEnabled || motor_state_[i] == kTargetReached || motor_state_[i] == kSwitchedOn)
    {
      // Settings for motor 1;
      val = controller_.left_x_axis_;
      if (val > deadzone)
      {
        sent_data_.target_pos[0] = received_data_.actual_pos[0] + (val - deadzone) / amp * THIRTY_DEGREE_CCW / 500;
      }
      else if (val < -deadzone)
      {
        sent_data_.target_pos[0] = received_data_.actual_pos[0] + (val + deadzone) / amp * THIRTY_DEGREE_CCW / 500;
      }
      // else {
      //     sent_data_.target_pos[0] = received_data_.actual_pos[0];
      // }
      // if((val > deadzone) || (val < -deadzone)){
      if (!operation_ready)
      {
        sent_data_.control_word[0] = SM_RUN;
        if (TEST_BIT(received_data_.status_word[0], 10))
          operation_ready = 1;
      }
      else
      {
        sent_data_.control_word[0] = SM_GO_ENABLE;
        operation_ready = 0;
      }
      //}

      // Settings for motor 2
      val = controller_.right_x_axis_;
      if (val > deadzone)
      {
        sent_data_.target_pos[1] = received_data_.actual_pos[1] + (val - deadzone) / amp * THIRTY_DEGREE_CCW / 50;
      }
      else if (val < -deadzone)
      {
        sent_data_.target_pos[1] = received_data_.actual_pos[1] + (val + deadzone) / amp * THIRTY_DEGREE_CCW / 50;
      }
      // else {
      //     sent_data_.target_pos[1] = received_data_.actual_pos[1];
      // }
      // if((val > deadzone) || (val < -deadzone)){
      sent_data_.control_word[1] = SM_GO_ENABLE;
      //}

      // Settings for motor 3
      if (controller_.right_rb_button_ > 0)
      {
        sent_data_.target_pos[2] = received_data_.actual_pos[2] + FIVE_DEGREE_CCW / 50 / 2.0;
      }
      else if (controller_.left_rb_button_ > 0)
      {
        sent_data_.target_pos[2] = received_data_.actual_pos[2] - FIVE_DEGREE_CCW / 50 / 2.0;
      }
      // else {
      //     sent_data_.target_pos[2] = received_data_.actual_pos[2];
      // }
      // if(controller_.right_rb_button_ || controller_.left_rb_button_){
      sent_data_.control_word[2] = SM_GO_ENABLE;
      //}
    }
  }
}

void EthercatLifeCycle::UpdateCyclicVelocityModeParameters()
{
  /// WRITE YOUR CUSTOM CONTROL ALGORITHM, VARIABLES DECLARATAION HERE, LIKE IN EXAMPLE BELOW.
  int m1_conversion_ratio =
      GEAR_RATIO * 2;  // if you multiply this value with the RPM value mechanism will move in that RPM.
  float m2_conversion_ratio = GEAR_RATIO * 2 * 12 * 40 / 30;
  float deadzone = 0.1;
  float maxSpeed_m1 = 3.0 * m1_conversion_ratio;  // Robot will move with 3 RPM (MAX). (Control input range 0~1)
  float maxSpeed_m2 = m2_conversion_ratio;
  float maxSpeed = 1500.0;
  float m1_left = 0.0, m1_right = 0.0, axis_m2 = 0.0, axis_m3 = 0.0;
  // Motor 1 controls roll axis.
  if (motor_state_[0] == kOperationEnabled || motor_state_[0] == kSwitchedOn)
  {
    m1_left = controller_.left_t_axis_;
    m1_right = controller_.right_t_axis_;
    if ((m1_right > deadzone))
    {
      sent_data_.target_vel[0] = int32_t(m1_right * maxSpeed_m1);
    }

    if ((m1_left > deadzone))
    {
      sent_data_.target_vel[0] = int32_t(-m1_left * maxSpeed_m1);
    }
    if (m1_left > 1 || m1_right > 1 || m1_left < -1 || m1_right < -1)
      sent_data_.target_vel[0] = 0;
    if (!(m1_left > deadzone || m1_right > deadzone))
      sent_data_.target_vel[0] = 0;
  }
  else
  {
    sent_data_.target_vel[0] = 0;
  }
  // sent_data_.control_word[0] = SM_GO_ENABLE;

  // Settings for motor 2
  if (motor_state_[1] == kOperationEnabled || motor_state_[1] == kSwitchedOn)
  {
    axis_m2 = controller_.right_y_axis_;
    if ((axis_m2 > deadzone) || (axis_m2 < -deadzone))
    {
      sent_data_.target_vel[1] = int32_t(axis_m2 * maxSpeed_m2);
    }
    else
    {
      sent_data_.target_vel[1] = 0;
    }
  }
  else
  {
    sent_data_.target_vel[1] = 0;
  }
  // sent_data_.control_word[1] = SM_GO_ENABLE;

  // Settings for motor 2
  if ((motor_state_[2] == kOperationEnabled || motor_state_[2] == kSwitchedOn))
  {
    axis_m3 = controller_.left_x_axis_;
    if ((axis_m3 > deadzone) || (axis_m3 < -deadzone))
    {
      sent_data_.target_vel[2] = int32_t(-axis_m3 * maxSpeed);
      sent_data_.target_vel[1] = sent_data_.target_vel[2];
    }
    else
    {
      sent_data_.target_vel[2] = 0;
    }
  }
  else
  {
    sent_data_.target_vel[2] = 0;
  }
  // sent_data_.control_word[2] = SM_GO_ENABLE;
}

void EthercatLifeCycle::UpdateVelocityModeParameters()
{
  /// WRITE YOUR CUSTOM CONTROL ALGORITHM VARIABLES DECLARATAION HERE
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    if (motor_state_[i] == kOperationEnabled || motor_state_[i] == kTargetReached || motor_state_[i] == kSwitchedOn)
    {
      /// WRITE YOUR CUSTOM CONTROL ALGORITHM HERE IF YOU WANT TO USE VELOCITY MODE
      /// YOU CAN CHECK  EXAMPLE CONTROL CODE BELOW.
      if (controller_.right_x_axis_ > 0.1 || controller_.right_x_axis_ < -0.1)
      {
        sent_data_.target_vel[0] = controller_.right_x_axis_ * 100000;
        sent_data_.target_vel[1] = controller_.right_x_axis_ * 100000;
      }
      else
      {
        sent_data_.target_vel[0] = 0;
        sent_data_.target_vel[1] = 0;
      }
      if (controller_.left_x_axis_ < -0.1 || controller_.left_x_axis_ > 0.1)
      {
        sent_data_.target_vel[2] = controller_.right_y_axis_ * 100000;
        sent_data_.target_vel[3] = controller_.right_y_axis_ * 100000;
      }
      else
      {
        sent_data_.target_vel[2] = 0;
        sent_data_.target_vel[3] = 0;
      }
      if (controller_.left_y_axis_ < -0.1 || controller_.left_y_axis_ > 0.1)
      {
        sent_data_.target_vel[4] = controller_.left_y_axis_ * 1000;
        sent_data_.target_vel[5] = controller_.left_y_axis_ * 10000;
      }
      else
      {
        sent_data_.target_vel[4] = 0;
        sent_data_.target_vel[5] = 0;
      }
      if (controller_.right_y_axis_ < -0.1 || controller_.left_y_axis_ > 0.1)
      {
        sent_data_.target_vel[6] = controller_.left_x_axis_ * 10000;
        sent_data_.target_vel[7] = controller_.left_x_axis_ * 10000;
      }
      else
      {
        sent_data_.target_vel[6] = 0;
        sent_data_.target_vel[7] = 0;
      }
    }
    else
    {
      sent_data_.target_vel[i] = 0;
    }
  }
}

void EthercatLifeCycle::UpdateMotorStateVelocityMode()
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    if ((received_data_.status_word[i] & command_) == 0X08)
    {
      // if status is fault, reset fault state.
      command_ = 0X04F;
      sent_data_.control_word[i] = SM_FULL_RESET;
      motor_state_[i] = kFault;
    }
    if (motor_state_[i] != kSwitchedOn)
    {
      if ((received_data_.status_word[i] & command_) == 0x0040)
      {
        // If status is "Switch on disabled", \
                change state to "Ready to switch on"
        sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
        command_ = 0x006f;
        motor_state_[i] = kSwitchOnDisabled;
      }
      else if ((received_data_.status_word[i] & command_) == 0x0021)
      {
        // If status is "Ready to switch on", \
                        change state to "Switched on"
        sent_data_.control_word[i] = SM_GO_SWITCH_ON;
        command_ = 0x006f;
        motor_state_[i] = kReadyToSwitchOn;
      }
      else if ((received_data_.status_word[i] & command_) == 0x0023)
      {
        // If status is "Switched on", change state to "Operation enabled"
        sent_data_.control_word[i] = SM_GO_ENABLE;
        command_ = 0x006f;
        motor_state_[i] = kSwitchedOn;
      }
      else if ((received_data_.status_word[i] & command_) == 0X08)
      {
        // if status is fault, reset fault state.
        command_ = 0X04f;

        sent_data_.control_word[i] = SM_FULL_RESET;
        motor_state_[i] = kFault;
      }
    }
  }
  return;
}

void EthercatLifeCycle::EnableMotors()
{
  // DS402 CANOpen over EtherCAT state machine
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
    if ((received_data_.status_word[i] & command_) == 0x0040)
    {
      // If status is "Switch on disabled", \
            change state to "Ready to switch on"
      sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
      command_ = 0x006f;
      motor_state_[i] = kSwitchOnDisabled;
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Transiting to -Ready to switch on state...- \n");
    }
    else if ((received_data_.status_word[i] & command_) == 0x0021)
    {  // If status is "Ready to switch on", \
                                                        change state to "Switched on"
      sent_data_.control_word[i] = SM_GO_SWITCH_ON;
      command_ = 0x006f;
      motor_state_[i] = kReadyToSwitchOn;
      //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Transiting to -Switched on state...- \n");
    }
    else if ((received_data_.status_word[i] & command_) == 0x0023)
    {
      // If status is "Switched on", change state to "Operation enabled"

      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Operation enabled...\n");
      sent_data_.control_word[i] = SM_GO_ENABLE;
      command_ = 0x006f;
      motor_state_[i] = kSwitchedOn;
    }
    else if ((received_data_.status_word[i] & command_) == 0X08)
    {
      // if status is fault, reset fault state.
      command_ = 0X04F;

      sent_data_.control_word[i] = SM_FULL_RESET;
      motor_state_[i] = kFault;
    }
  }
}

void EthercatLifeCycle::WriteToSlavesInCyclicTorqueMode()
{
  if (!emergency_status_ || !gui_node_data_)
  {
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                   sent_data_.control_word[i]);
      EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_tor, 0);
      EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.torque_offset, 0);
    }
  }
  else
  {
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                   sent_data_.control_word[i]);
      EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_tor,
                   sent_data_.target_tor[i]);
      EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.torque_offset, 0);
    }
  }
}

void EthercatLifeCycle::UpdateCyclicTorqueModeParameters()
{
  // Torque mode: sending target_torque value in per thousand of Motor Rated Torque value.
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    // sent_data_.control_word[i] = SM_GO_ENABLE;
    if (motor_state_[i] == kOperationEnabled || motor_state_[i] == kTargetReached || motor_state_[i] == kSwitchedOn)
    {
      if (controller_.right_x_axis_ < -0.1 || controller_.right_x_axis_ > 0.1)
      {
        sent_data_.target_tor[0] = controller_.right_x_axis_ * 300;
      }
      else
      {
        sent_data_.target_tor[0] = 0;
      }
      if (controller_.left_x_axis_ < -0.1 || controller_.left_x_axis_ > 0.1)
      {
        sent_data_.target_tor[1] = controller_.left_x_axis_ * 300;
      }
      else
      {
        sent_data_.target_tor[1] = 0;
      }
      if (controller_.left_y_axis_ < -0.1 || controller_.left_y_axis_ > 0.1)
      {
        sent_data_.target_tor[2] = controller_.left_y_axis_ * 300;
      }
      else
      {
        sent_data_.target_tor[2] = 0;
      }
    }
    else
    {
      sent_data_.target_tor[i] = 0;
    }
  }
}

int8_t EthercatLifeCycle::EnableDrivesViaSDO(int index)
{
  for (int i = 0; i < 10; i++)
  {
    received_data_.status_word[index] = ecat_node_->ReadStatusWordViaSDO(index);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d\n" ,received_data_.status_word[index]);
    if (EnableDrivers() == g_kNumberOfServoDrivers)
    {
      return 0;
    };
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d\n" ,sent_data_.control_word[index]);
    if (ecat_node_->WriteControlWordViaSDO(index, sent_data_.control_word[index]))
    {
      return -1;
    }
  }
  return 0;
}

int8_t EthercatLifeCycle::DisableDrivesViaSDO(int index)
{
  sent_data_.control_word[index] = SM_GO_SWITCH_ON_DISABLE;
  if (ecat_node_->WriteControlWordViaSDO(index, sent_data_.control_word[index]))
  {
    return -1;
  }
  return 0;
}
