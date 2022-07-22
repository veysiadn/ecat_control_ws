#include "ecat_node.hpp"

using namespace EthercatCommunication;

/*****************************************************************************************/
/// Extern global variable declaration.
ec_master_t* g_master = NULL;                  // EtherCAT master instance
ec_master_state_t g_master_state = {};         // EtherCAT master state
ec_domain_t* g_master_domain = NULL;           // Ethercat data passing master domain
ec_domain_state_t g_master_domain_state = {};  // EtherCAT master domain state
struct timespec g_sync_timer;
uint32_t g_sync_ref_counter = 0;
/*****************************************************************************************/

EthercatNode::EthercatNode()
{
  request_sdos_.resize(g_kNumberOfServoDrivers);
}

EthercatNode::~EthercatNode()
{
}

int EthercatNode::ConfigureMaster()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requesting EtherCAT master...\n");
  if (!g_master)
  {
    g_master = ecrt_request_master(0);
    if (!g_master)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Requesting master instance failed ! ");
      return -1;
    }
  }

  g_master_domain = ecrt_master_create_domain(g_master);
  if (!g_master_domain)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to create master domain ! ");
    return -1;
  }
  return 0;
}

void EthercatNode::GetAllSlaveInformation()
{
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    ecrt_master_get_slave(g_master, i, &slaves_[i].slave_info_);
  }
}

int EthercatNode::ConfigureSlaves()
{
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    slaves_[i].slave_config_ =
        ecrt_master_slave_config(g_master, slaves_[i].slave_info_.alias, slaves_[i].slave_info_.position,
                                 slaves_[i].slave_info_.vendor_id, slaves_[i].slave_info_.product_code);
    if (!slaves_[i].slave_config_)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to  configure slave ! ");
      return -1;
    }
  }

  return 0;
}

int EthercatNode::MapDefaultPdos()
{
  /**
   *  This part is specific for our Custom EASYCAT slave configuration
   *  To create your custom slave and variables you can add variables to \see OffsetPDO struct.
   *  Also you have add your variables to received data structure, you may have to create your custom msg files as well.
   **/
  ec_pdo_entry_info_t elmo_pdo_entries[12] = {
    { OD_TARGET_POSITION, 32 },       { OD_DIGITAL_OUTPUTS, 32 },     { OD_CONTROL_WORD, 16 },

    { OD_TARGET_VELOCITY, 32 },       { OD_TARGET_TORQUE, 16 },       { OD_TORQUE_OFFSET, 16 },

    { OD_POSITION_ACTUAL_VAL, 32 },   { OD_DIGITAL_INPUTS, 32 },      { OD_STATUS_WORD, 16 },

    { OD_VELOCITY_ACTUAL_VALUE, 32 }, { OD_TORQUE_ACTUAL_VALUE, 16 }, { OD_ERROR_CODE, 16 }

  };

  ec_pdo_info_t elmo_pdos[2] = { { 0x1607, 6, elmo_pdo_entries + 0 }, { 0x1a07, 6, elmo_pdo_entries + 6 } };

  ec_sync_info_t elmo_syncs[5] = { { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
                                   { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
                                   { 2, EC_DIR_OUTPUT, 1, elmo_pdos + 0, EC_WD_ENABLE },
                                   { 3, EC_DIR_INPUT, 1, elmo_pdos + 1, EC_WD_DISABLE },
                                   { 0xff } };

  /* Master 0, Slave 0, "EPOS4"
   * Vendor ID:       0x000000fb
   * Product code:    0x61500000
   * Revision number: 0x01600000
   */
  // CKim - First three entries will be read by slave (master sends command). RxPDO
  ec_pdo_entry_info_t maxon_epos_pdo_entries[11] = { { OD_CONTROL_WORD, 16 },
                                                     { OD_TARGET_VELOCITY, 32 },
                                                     { OD_TARGET_POSITION, 32 },
                                                     { OD_TARGET_TORQUE, 16 },
                                                     { OD_TORQUE_OFFSET, 16 },
                                                     // CKim - entries will be transmitted by slave (master receives the
                                                     // data). TxPDO
                                                     { OD_STATUS_WORD, 16 },
                                                     { OD_POSITION_ACTUAL_VAL, 32 },
                                                     { OD_VELOCITY_ACTUAL_VALUE, 32 },
                                                     { OD_TORQUE_ACTUAL_VALUE, 16 },
                                                     { OD_ERROR_CODE, 16 } ,
                                                     { OD_OPERATION_MODE_DISPLAY,8}};

  ec_pdo_info_t maxon_pdos[2] = {
    { 0x1600, 5, maxon_epos_pdo_entries + 0 },  // CKim - RxPDO index of the EPOS4
    { 0x1a00, 6, maxon_epos_pdo_entries + 5 }   // CKim - TxPDO index of the EPOS4
  };

  // CKim - Sync manager configuration of the EPOS4. 0,1 is reserved for SDO communications
  // EC_WD_ENABLE means that the sync manager of the slave will throw error
  // if it does not synchronize within certain interval
  ec_sync_info_t maxon_syncs[5] = { { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
                                    { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
                                    { 2, EC_DIR_OUTPUT, 1, maxon_pdos + 0, EC_WD_ENABLE },
                                    { 3, EC_DIR_INPUT, 1, maxon_pdos + 1, EC_WD_DISABLE },
                                    { 0xff } };
  /*********************************************************/
  ec_pdo_entry_info_t easycat_pdo_entries[16] = {
    { 0x0005, 0x01, 16 }, /* output_analog_01 */
    { 0x0005, 0x02, 16 }, /* output_analog_02 */
    { 0x0005, 0x03, 16 }, /* output_analog_03 */
    { 0x0005, 0x04, 8 },  /* output_digital_04 */
    { 0x0005, 0x05, 8 },  /* output_digital_05 */
    { 0x0005, 0x06, 8 },  /* output_digital_01 */
    { 0x0005, 0x07, 8 },  /* output_digital_02 */
    { 0x0005, 0x08, 8 },  /* output_digital_03 */
    { 0x0006, 0x01, 16 }, /* input_analog_01 */
    { 0x0006, 0x02, 16 }, /* input_analog_02 */
    { 0x0006, 0x03, 16 }, /* input_analog_03 */
    { 0x0006, 0x04, 8 },  /* input_digital_04 */
    { 0x0006, 0x05, 8 },  /* input_digital_05 */
    { 0x0006, 0x06, 8 },  /* left_limit_switch */
    { 0x0006, 0x07, 8 },  /* right_limit_switch */
    { 0x0006, 0x08, 8 },  /* input_digital_03 */
  };

  ec_pdo_info_t easycat_pdos[2] = {
    { 0x1600, 8, easycat_pdo_entries + 0 }, /* Outputs */
    { 0x1a00, 8, easycat_pdo_entries + 8 }, /* Inputs */
  };

  ec_sync_info_t easycat_syncs[3] = { { 0, EC_DIR_OUTPUT, 1, easycat_pdos + 0, EC_WD_ENABLE },
                                      { 1, EC_DIR_INPUT, 1, easycat_pdos + 1, EC_WD_DISABLE },
                                      { 0xff } };
#if DEV_TEST
  for (int i = 0; i < 4; i++)
  {
    if (ecrt_slave_config_pdos(slaves_[i].slave_config_, EC_END, elmo_syncs))
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Slave PDO configuration failed... ");
      return -1;
    }
  }
  for (int i = 4; i < g_kNumberOfServoDrivers; i++)
  {
    if (ecrt_slave_config_pdos(slaves_[i].slave_config_, EC_END, maxon_syncs))
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Slave PDO configuration failed... ");
      return -1;
    }
  }
#else
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    if (ecrt_slave_config_pdos(slaves_[i].slave_config_, EC_END, maxon_syncs))
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Slave PDO configuration failed... ");
      return -1;
    }
  }
#endif
#if CUSTOM_SLAVE
  if (ecrt_slave_config_pdos(slaves_[FINAL_SLAVE].slave_config_, EC_END, easycat_syncs))
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "EasyCAT slave PDO configuration failed... ");
    return -1;
  }
#endif
  // CKim - Registers a PDO entry for process data exchange in a domain. Obtain offsets
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    this->slaves_[i].offset_.actual_pos =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_POSITION_ACTUAL_VAL, g_master_domain, NULL);

    this->slaves_[i].offset_.status_word =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_STATUS_WORD, g_master_domain, NULL);

    this->slaves_[i].offset_.actual_vel = ecrt_slave_config_reg_pdo_entry(
        this->slaves_[i].slave_config_, OD_VELOCITY_ACTUAL_VALUE, g_master_domain, NULL);

    // this->slaves_[i].offset_.op_mode_display = ecrt_slave_config_reg_pdo_entry(
    //     this->slaves_[i].slave_config_, OD_OPERATION_MODE_DISPLAY, g_master_domain, NULL);

    this->slaves_[i].offset_.actual_tor =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_TORQUE_ACTUAL_VALUE, g_master_domain, NULL);

    this->slaves_[i].offset_.error_code =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_ERROR_CODE, g_master_domain, NULL);

    this->slaves_[i].offset_.torque_offset =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_TORQUE_OFFSET, g_master_domain, NULL);

    this->slaves_[i].offset_.target_pos =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_TARGET_POSITION, g_master_domain, NULL);
    this->slaves_[i].offset_.target_vel =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_TARGET_VELOCITY, g_master_domain, NULL);
    this->slaves_[i].offset_.target_tor =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_TARGET_TORQUE, g_master_domain, NULL);
    this->slaves_[i].offset_.control_word =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_CONTROL_WORD, g_master_domain, NULL);

    if ((slaves_[i].offset_.actual_pos < 0) || (slaves_[i].offset_.status_word < 0) ||
        (slaves_[i].offset_.actual_vel < 0) || (slaves_[i].offset_.target_vel < 0) ||
        (slaves_[i].offset_.target_pos < 0) || (slaves_[i].offset_.control_word < 0) ||
        (slaves_[i].offset_.target_tor < 0) || (slaves_[i].offset_.actual_tor < 0) ||
        (slaves_[i].offset_.torque_offset < 0) || (slaves_[i].offset_.error_code < 0))
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to configure  PDOs for motors.!");
      return -1;
    }
  }
#if CUSTOM_SLAVE
  slaves_[FINAL_SLAVE].offset_.r_limit_switch =
      ecrt_slave_config_reg_pdo_entry(slaves_[FINAL_SLAVE].slave_config_, 0x006, 0x006, g_master_domain, NULL);
  if (slaves_[FINAL_SLAVE].offset_.r_limit_switch < 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EasyCAT right limit switch PDO configuration failed...\n");
    return -1;
  }
  slaves_[FINAL_SLAVE].offset_.l_limit_switch =
      ecrt_slave_config_reg_pdo_entry(slaves_[FINAL_SLAVE].slave_config_, 0x006, 0x07, g_master_domain, NULL);
  if (slaves_[FINAL_SLAVE].offset_.l_limit_switch < 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EasyCAT left limit switch PDO configuration failed...\n");
    return -1;
  }
  slaves_[FINAL_SLAVE].offset_.emergency_switch =
      ecrt_slave_config_reg_pdo_entry(slaves_[FINAL_SLAVE].slave_config_, 0x006, 0x05, g_master_domain, NULL);
  if (slaves_[FINAL_SLAVE].offset_.emergency_switch < 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EasyCAT left limit switch PDO configuration failed...\n");
    return -1;
  }
#endif
  return 0;
}

void EthercatNode::ConfigDcSyncDefault()
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    ecrt_slave_config_dc(slaves_[i].slave_config_, 0X0300, PERIOD_NS, slaves_[i].kSync0_shift_, 0, 0);
  }
#if CUSTOM_SLAVE
  ecrt_slave_config_dc(slaves_[FINAL_SLAVE].slave_config_, 0X0300, PERIOD_NS, 2000200000, 0, 0);
#endif
}

int EthercatNode::ActivateMaster()
{
  if (ecrt_master_activate(g_master))
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Master activation error ! ");
    return -1;
  }
  return 0;
}

int EthercatNode::RegisterDomain()
{
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    slaves_[i].slave_pdo_domain_ = ecrt_domain_data(g_master_domain);
    // if(!(slaves_[i].slave_pdo_domain_) )
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Domain PDO registration error");
    //     return -1;
    // }
  }
  return 0;
}

int EthercatNode::SetProfilePositionParameters(ProfilePosParam& P, int position)
{
  // Operation mode to ProfilePositionMode for slave on that position.
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_OPERATION_MODE, kProfilePosition))
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
    return -1;
  }
  // profile velocity
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_VELOCITY, P.profile_vel) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity config error ! ");
    return -1;
  }
  // max profile velocity
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_MAX_PROFILE_VELOCITY, P.max_profile_vel) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile velocity config error !");
    return -1;
  }
  // profile acceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_ACCELERATION, P.profile_acc) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
    return -1;
  }
  // profile deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
    return -1;
  }
  // quick stop deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
    return -1;
  }
  // max following error
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_MAX_FOLLOWING_ERROR, P.max_fol_err) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max following error failed ! ");
    return -1;
  }
  return 0;
}

int EthercatNode::SetProfilePositionParametersAll(ProfilePosParam& P)
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    // Set operation mode to ProfilePositionMode for all motors.
    if (ecrt_slave_config_sdo8(slaves_[i].slave_config_, OD_OPERATION_MODE, kProfilePosition))
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
      return -1;
    }
    // profile velocity
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_PROFILE_VELOCITY, P.profile_vel) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity failed ! ");
      return -1;
    }
    // max profile velocity
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_MAX_PROFILE_VELOCITY, P.max_profile_vel) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile velocity failed ! ");
      return -1;
    }
    // profile acceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_PROFILE_ACCELERATION, P.profile_acc) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
      return -1;
    }
    // profile deceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
      return -1;
    }
    // quick stop deceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
      return -1;
    }
    if (ecrt_slave_config_sdo16(slaves_[i].slave_config_, OD_MOTION_PROFILE_TYPE, P.motion_profile_type) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
      return -1;
    }
  }
  return 0;
}

int EthercatNode::SetProfileVelocityParameters(ProfileVelocityParam& P, int position)
{
  // Set operation mode to ProfileVelocityMode for slave on that position.
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_OPERATION_MODE, kProfileVelocity))
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
    return -1;
  }
  // motionProfileType
  if (ecrt_slave_config_sdo16(slaves_[position].slave_config_, OD_MOTION_PROFILE_TYPE, P.motion_profile_type) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity config error ! ");
    return -1;
  }
  // max profile velocity
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_MAX_PROFILE_VELOCITY, P.max_profile_vel) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile  velocity config error ! ");
    return -1;
  }
  // profile acceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed !");
    return -1;
  }
  // profile deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_ACCELERATION, P.profile_acc) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
    return -1;
  }
  // quick stop deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed ! ");
    return -1;
  }
  return 0;
}

int EthercatNode::SetProfileVelocityParametersAll(ProfileVelocityParam& P)
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    // Set operation mode to ProfileVelocityMode for all motors.
    if (ecrt_slave_config_sdo8(slaves_[i].slave_config_, OD_OPERATION_MODE, kProfileVelocity))
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
      return -1;
    }
    // motionProfileType
    if (ecrt_slave_config_sdo16(slaves_[i].slave_config_, OD_MOTION_PROFILE_TYPE, P.motion_profile_type) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity config error ! ");
      return -1;
    }
    // max profile velocity
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_MAX_PROFILE_VELOCITY, P.max_profile_vel) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile  velocity config error ! ");
      return -1;
    }
    // profile acceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed !");
      return -1;
    }
    // profile deceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_PROFILE_ACCELERATION, P.profile_acc) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
      return -1;
    }
    // quick stop deceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed ! ");
      return -1;
    }
  }
  return 0;
}

int EthercatNode::SetCyclicSyncPositionModeParameters(CSPositionModeParam& P, int position)
{
  // Set operation mode to Cyclic Synchronous Position mode for motor in specified physical position w.r.t master.
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_OPERATION_MODE, kCSPosition))
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
    return -1;
  }
  // profile velocity
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_VELOCITY, P.profile_vel) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity failed ! ");
    return -1;
  }
  // max profile velocity
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_MAX_PROFILE_VELOCITY, P.max_profile_vel) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile velocity failed ! ");
    return -1;
  }
  // profile acceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_ACCELERATION, P.profile_acc) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
    return -1;
  }
  // profile deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
    return -1;
  }
  // quick stop deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
    return -1;
  }
  // Interpolation time period is 1ms by default.Default unit is milliseconds (ms)
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_INTERPOLATION_TIME_PERIOD,
                             P.interpolation_time_period) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
    return -1;
  }
  return 0;
}

int EthercatNode::SetCyclicSyncPositionModeParametersAll(CSPositionModeParam& P)
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    // Set operation mode to Cyclic Synchronous Position mode for all motors.
    if (ecrt_slave_config_sdo8(slaves_[i].slave_config_, OD_OPERATION_MODE, kCSPosition))
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
      return -1;
    }
    // //profile velocity
    // if(ecrt_slave_config_sdo32(slaves_[i].slave_config_,OD_PROFILE_VELOCITY, P.profile_vel) < 0) {
    //     RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity failed ! ");
    //     return -1;
    // }
    // //max profile velocity
    // if(ecrt_slave_config_sdo32(slaves_[i].slave_config_,OD_MAX_PROFILE_VELOCITY,P.max_profile_vel) < 0) {
    //     RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile velocity failed ! ");
    //     return -1;
    // }
    // //profile acceleration
    // if(ecrt_slave_config_sdo32(slaves_[i].slave_config_,OD_PROFILE_ACCELERATION, P.profile_acc) < 0) {
    //     RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
    //     return -1;
    // }
    // //profile deceleration
    // if(ecrt_slave_config_sdo32(slaves_[i].slave_config_,OD_PROFILE_DECELERATION,P.profile_dec) < 0) {
    //     RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
    //     return -1;
    // }
    // quick stop deceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
      return -1;
    }
    // Interpolation time period is 1ms by default.Default unit is milliseconds (ms)
    if (ecrt_slave_config_sdo8(slaves_[i].slave_config_, OD_INTERPOLATION_TIME_PERIOD, P.interpolation_time_period) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
      return -1;
    }
  }
  return 0;
}

int EthercatNode::SetCyclicSyncVelocityModeParameters(CSVelocityModeParam& P, int position)
{
  // Set operation mode to Cyclic Synchronous Velocity mode for motor in specified physical position w.r.t master.
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_OPERATION_MODE, kCSVelocity))
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
    return -1;
  }
  // // Velocity control parameter set, P, I gain only
  // if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_VELOCITY_CONTROLLER_PGAIN,
  //                             P.velocity_controller_gain.Pgain) < 0)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set velocity Pgain failed ! ");
  //   return -1;
  // }
  // if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_VELOCITY_CONTROLLER_IGAIN,
  //                             P.velocity_controller_gain.Igain) < 0)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set velocity Igain failed ! ");
  //   return -1;
  // }
  // profile deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
    return -1;
  }
  // quick stop deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
    return -1;
  }
  // Interpolation time period is 1ms by default.Default unit is milliseconds (ms)
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_INTERPOLATION_TIME_PERIOD,
                             P.interpolation_time_period) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
    return -1;
  }
  return 0;
}

int EthercatNode::SetCyclicSyncTorqueModeParameters(CSTorqueModeParam& P, int position)
{
  // Set operation mode to Cyclic Synchronous Velocity mode for motor in specified physical position w.r.t master.
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_OPERATION_MODE, kCSTorque))
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
    return -1;
  }
  // profile deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
    return -1;
  }
  // quick stop deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
    return -1;
  }
  return 0;
}

int EthercatNode::SetCyclicSyncVelocityModeParametersAll(CSVelocityModeParam& P)
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    // Set operation mode to Cyclic Synchronous Velocity mode for motor in specified physical position w.r.t master.
    if (ecrt_slave_config_sdo8(slaves_[i].slave_config_, OD_OPERATION_MODE, kCSVelocity))
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
      return -1;
    }
    // Velocity control parameter set, P, I gain only
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_VELOCITY_CONTROLLER_PGAIN,
                                P.velocity_controller_gain.Pgain) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set velocity Pgain failed ! ");
      return -1;
    }
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_VELOCITY_CONTROLLER_IGAIN,
                                P.velocity_controller_gain.Igain) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set velocity Igain failed ! ");
      return -1;
    }
    // Max profile velocity.
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_MAX_PROFILE_VELOCITY, P.max_profile_vel) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile velocity failed ! ");
      return -1;
    }
    // profile deceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
      return -1;
    }
    // quick stop deceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
      return -1;
    }
    // Interpolation time period is 1ms by default.Default unit is milliseconds (ms)
    if (ecrt_slave_config_sdo8(slaves_[i].slave_config_, OD_INTERPOLATION_TIME_PERIOD, P.interpolation_time_period) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
      return -1;
    }
  }
  return 0;
}

int EthercatNode::SetCyclicSyncTorqueModeParametersAll(CSTorqueModeParam& P)
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    // Set operation mode to Cyclic Synchronous Velocity mode for motor in specified physical position w.r.t master.
    if (ecrt_slave_config_sdo8(slaves_[i].slave_config_, OD_OPERATION_MODE, kCSTorque))
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
      return -1;
    }
    // profile deceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
      return -1;
    }
    // quick stop deceleration
    if (ecrt_slave_config_sdo32(slaves_[i].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
      return -1;
    }
  }
  return 0;
}

int EthercatNode::WaitForOperationalMode()
{
  int try_counter = 0;
  int check_state_count = 0;
  int time_out = 20e3;
  while (g_master_state.al_states != EC_AL_STATE_OP)
  {
    if (try_counter < time_out)
    {
      clock_gettime(CLOCK_MONOTONIC, &g_sync_timer);
      ecrt_master_application_time(g_master, TIMESPEC2NS(g_sync_timer));

      ecrt_master_receive(g_master);
      ecrt_domain_process(g_master_domain);
      usleep(PERIOD_US);
      if (!check_state_count)
      {
        CheckMasterState();
        CheckMasterDomainState();
        CheckSlaveConfigurationState();
        check_state_count = PERIOD_US;
      }

      ecrt_domain_queue(g_master_domain);
      ecrt_master_sync_slave_clocks(g_master);
      ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(g_sync_timer));
      ecrt_master_send(g_master);

      try_counter++;
      check_state_count--;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : Time out occurred while waiting for OP mode.!  ");
        // ecrt_master_deactivate_slaves(g_master);
        ecrt_master_deactivate(g_master);
        ecrt_release_master(g_master);
      return -1;
    }
  }
  return 0;
}

void EthercatNode::SetCustomSlave(EthercatSlave c_slave, int position)
{
  slaves_[position] = c_slave;
}

int EthercatNode::MapCustomPdos(EthercatSlave c_slave, int position)
{
  slaves_[position] = c_slave;
  int err = ecrt_slave_config_pdos(slaves_[position].slave_config_, EC_END, slaves_[position].slave_sync_info_);
  if (err)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to configure  PDOs!  ");
    return -1;
  }
  err = ecrt_domain_reg_pdo_entry_list(g_master_domain, slaves_[position].slave_pdo_entry_reg_);
  if (err)
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to register PDO entries ");
    return -1;
  }
  return 0;
}

void EthercatNode::ConfigDcSync(uint16_t assign_activate, int position)
{
  return ecrt_slave_config_dc(slaves_[position].slave_config_, assign_activate, PERIOD_NS,
                              slaves_[position].kSync0_shift_, 0, 0);
}

void EthercatNode::CheckSlaveConfigurationState()
{
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    slaves_[i].CheckSlaveConfigState();
  }
}

int EthercatNode::CheckMasterState()
{
  ec_master_state_t ms;
  ecrt_master_state(g_master, &ms);
  if (ms.slaves_responding != g_master_state.slaves_responding)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%u slave(s).\n", ms.slaves_responding);
    if (ms.slaves_responding < 1)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Connection error,no response from slaves.");
      return -1;
    }
  }
  if (ms.al_states != g_master_state.al_states)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AL states: 0x%02X.\n", ms.al_states);
  }
  if (ms.link_up != g_master_state.link_up)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Link is %s.\n", ms.link_up ? "up" : "down");
    if (!ms.link_up)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Master state link down");
      return -1;
    }
  }
  g_master_state = ms;
  return 0;
}

void EthercatNode::CheckMasterDomainState()
{
  ec_domain_state_t ds;  // Domain instance
  ecrt_domain_state(g_master_domain, &ds);
  if (ds.working_counter != g_master_domain_state.working_counter)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "masterDomain: WC %u.\n", ds.working_counter);
  if (ds.wc_state != g_master_domain_state.wc_state)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "masterDomain: State %u.\n", ds.wc_state);
  if (g_master_domain_state.wc_state == EC_WC_COMPLETE)
  {
    g_master_domain_state = ds;
  }
  g_master_domain_state = ds;
}

int EthercatNode::GetNumberOfConnectedSlaves()
{
  unsigned int number_of_slaves;
  usleep(1e6);
  ecrt_master_state(g_master, &g_master_state);
  number_of_slaves = g_master_state.slaves_responding;
  if (NUM_OF_SLAVES != number_of_slaves)
  {
    std::cout << "Please enter correct number of slaves... " << std::endl;
    std::cout << "Entered number of slave : " << NUM_OF_SLAVES << std::endl
              << "Connected slaves        : " << number_of_slaves << std::endl;
    return -1;
  }
  return 0;
}

void EthercatNode::DeactivateCommunication()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivating EtherCAT master : Cyclic PDO ended...\n");
  ecrt_master_deactivate(g_master);
  ReleaseMaster();
}

void EthercatNode::ReleaseMaster()
{
  ecrt_release_master(g_master);
}

int EthercatNode::OpenEthercatMaster()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to open EtherCAT master...\n");
  fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
  if (fd)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening EtherCAT master...\n");
    std::system("sudo ethercatctl start");
    usleep(2e6);
    fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
    if (fd)
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : EtherCAT device not found.\n");
      return -1;
    }
    else
    {
      return 0;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EtherCAT master opened...\n");
  return 0;
}

int EthercatNode::ShutDownEthercatMaster()
{
  fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
  if (!fd)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down EtherCAT master...");
    std::system("cd ~; sudo ethercatctl stop\n");
    usleep(1e6);
    fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
    if (fd)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EtherCAT shut down succesfull.");
      return 0;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : EtherCAT shutdown error.");
      return -1;
    }
  }
  return 0;
}

int EthercatNode::RestartEthercatMaster()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Restarting EtherCAT master...");
  std::system("cd ~; sudo ethercatctl restart\n");
  usleep(2e6);
  fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
  if (!fd)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EtherCAT restart succesfull.");
    return 0;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : EtherCAT restart error.");
    return -1;
  }
}

int8_t EthercatNode::SdoRead(SDO_data& pack)
{
  if (ecrt_master_sdo_upload(g_master, pack.slave_position, pack.index, pack.sub_index, (uint8_t*)(&pack.data),
                             pack.data_sz, &pack.result_sz, &pack.err_code))
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "SDO read error, code: %d \n", &pack.err_code);
    return -1;
  }
  return 0;
}

int8_t EthercatNode::SdoWrite(SDO_data& pack)
{
  if (ecrt_master_sdo_download(g_master, pack.slave_position, pack.index, pack.sub_index, (uint8_t*)(&pack.data),
                               pack.data_sz, &pack.err_code))
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "SDO write error, code : %d \n ", &pack.err_code);
    return -1;
  }
  return 0;
}

uint16_t EthercatNode::ReadStatusWordViaSDO(int index)
{
  SDO_data pack;
  uint16_t status_word;
  pack.slave_position = index;
  pack.index = OD_STATUS_WORD;
  pack.sub_index = 0;
  pack.data_sz = sizeof(uint16_t);
  if (SdoRead(pack))
  {
    std::cout << "Error while reading Status Word " << std::endl;
    return -1;
  }
  status_word = (uint16_t)(pack.data);
  return status_word;
}

int16_t EthercatNode::WriteControlWordViaSDO(int index, uint16_t control_word)
{
  SDO_data pack;
  pack.index = OD_CONTROL_WORD;
  pack.slave_position = index;
  pack.sub_index = 0;
  pack.data_sz = sizeof(uint16_t);
  pack.data = uint32_t(control_word);
  if (SdoWrite(pack))
  {
    std::cout << "Error while writing Control Word " << std::endl;
    return -1;
  }
  return 0;
}

uint8_t EthercatNode::ReadOpModeViaSDO(int index)
{
  SDO_data pack;
  uint8_t op_mode;
  pack.slave_position = index;
  pack.index = OD_OPERATION_MODE;
  pack.sub_index = 0;
  pack.data_sz = sizeof(uint8_t);
  if (SdoRead(pack))
  {
    std::cout << "Error while reading Operation Mode " << std::endl;
    return -1;
  }
  op_mode = (uint8_t)(pack.data);
  return op_mode;
}

int16_t EthercatNode::WriteOpModeViaSDO(int index, uint8_t op_mode)
{
  SDO_data pack;
  pack.index = OD_OPERATION_MODE;
  pack.slave_position = index;
  pack.sub_index = 0;
  pack.data_sz = sizeof(uint8_t);
  pack.data = uint32_t(op_mode);
  if (SdoWrite(pack))
  {
    std::cout << "Error while writing Op Mode " << std::endl;
    return -1;
  }
  return 0;
}

int32_t EthercatNode::ReadActualVelocityViaSDO(int index)
{
  SDO_data pack;
  int32_t actual_vel;
  pack.slave_position = index;
  pack.index = OD_VELOCITY_ACTUAL_VALUE;
  pack.sub_index = 0;
  pack.data_sz = sizeof(int32_t);
  if (SdoRead(pack))
  {
    std::cout << "Error while reading Actual Velocity " << std::endl;
    return -1;
  }
  actual_vel = (int32_t)(pack.data);
  return actual_vel;
}

int16_t EthercatNode::WriteTargetVelocityViaSDO(int index, int32_t target_vel)
{
  SDO_data pack;
  pack.index = OD_TARGET_VELOCITY;
  pack.slave_position = index;
  pack.sub_index = 0;
  pack.data_sz = sizeof(int32_t);
  pack.data = int32_t(target_vel);
  if (SdoWrite(pack))
  {
    std::cout << "Error while writing Target Velocity " << std::endl;
    return -1;
  }
  if (WriteControlWordViaSDO(index, SM_RUN))
  {
    return -1;
  }
  return 0;
}

int32_t EthercatNode::ReadActualPositionViaSDO(int index)
{
  SDO_data pack;
  int32_t actual_pos;
  pack.slave_position = index;
  pack.index = OD_POSITION_ACTUAL_VAL;
  pack.sub_index = 0;
  pack.data_sz = sizeof(int32_t);
  if (SdoRead(pack))
  {
    std::cout << "Error while reading Actual Position " << std::endl;
    return -1;
  }
  actual_pos = (int32_t)(pack.data);
  return actual_pos;
}

int16_t EthercatNode::WriteTargetPositionViaSDO(int index, int32_t target_pos)
{
  SDO_data pack;
  pack.index = OD_TARGET_POSITION;
  pack.slave_position = index;
  pack.sub_index = 0;
  pack.data_sz = sizeof(int32_t);
  pack.data = int32_t(target_pos);
  if (SdoWrite(pack))
  {
    std::cout << "Error while writing Target Position " << std::endl;
    return -1;
  }
  if (TEST_BIT(ReadStatusWordViaSDO(index), 10) == 1)
  {
    WriteControlWordViaSDO(index, SM_RELATIVE_POS);
    WriteControlWordViaSDO(index, SM_GO_ENABLE);
  }
  else
  {
    if (WriteControlWordViaSDO(index, SM_GO_ENABLE))
    {
      return -1;
    }
  }
  return 0;
}

int16_t EthercatNode::ReadActualTorqueViaSDO(int index)
{
  SDO_data pack;
  int16_t actual_tor;
  pack.slave_position = index;
  pack.index = OD_TORQUE_ACTUAL_VALUE;
  pack.sub_index = 0;
  pack.data_sz = sizeof(int16_t);
  if (SdoRead(pack))
  {
    std::cout << "Error while reading SDO " << std::endl;
    return -1;
  }
  actual_tor = (int16_t)(pack.data);
  return actual_tor;
}

int16_t EthercatNode::WriteTargetTorqueViaSDO(int index, uint16_t target_tor)
{
  SDO_data pack;
  pack.index = OD_TARGET_TORQUE;
  pack.slave_position = index;
  pack.sub_index = 0;
  pack.data_sz = sizeof(int16_t);
  pack.data = uint32_t(target_tor);
  if (SdoWrite(pack))
  {
    std::cout << "Error while writing SDO " << std::endl;
    return -1;
  }
  if (WriteControlWordViaSDO(index, SM_RUN))
  {
    return -1;
  }
  return 0;
}
uint16_t EthercatNode::ReadErrorCodeViaSDO(int index)
{
  SDO_data pack;
  uint16_t error_code;
  pack.slave_position = index;
  pack.index = OD_ERROR_CODE;
  pack.sub_index = 0;
  pack.data_sz = sizeof(uint16_t);
  if (SdoRead(pack))
  {
    std::cout << "Error while reading Error Code " << std::endl;
    return -1;
  }
  error_code = (uint16_t)(pack.data);
  return error_code;
}

uint16_t EthercatNode::ClearFaultsViaSDO(int index)
{
  WriteControlWordViaSDO(index, SM_START);
  WriteControlWordViaSDO(index, SM_FULL_RESET);
}

int EthercatNode::MapDefaultSdos()
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    if (!(request_sdos_[i].target_vel =
              ecrt_slave_config_create_sdo_request(slaves_[i].slave_config_, OD_TARGET_VELOCITY, sizeof(int32_t))))
    {
      fprintf(stderr, "Failed to create SDO request for Target Velocity.\n");
      return -1;
    }
    ecrt_sdo_request_timeout(request_sdos_[i].target_vel, 500);  // ms

    if (!(request_sdos_[i].actual_vel =
              ecrt_slave_config_create_sdo_request(slaves_[i].slave_config_, OD_TARGET_VELOCITY, sizeof(int32_t))))
    {
      fprintf(stderr, "Failed to create SDO request for Actual Velocity.\n");
      return -1;
    }
    ecrt_sdo_request_timeout(request_sdos_[i].actual_vel, 500);  // ms

    if (!(request_sdos_[i].target_pos =
              ecrt_slave_config_create_sdo_request(slaves_[i].slave_config_, OD_TARGET_POSITION, sizeof(int32_t))))
    {
      fprintf(stderr, "Failed to create SDO request for Target Position.\n");
      return -1;
    }
    ecrt_sdo_request_timeout(request_sdos_[i].target_pos, 500);  // ms

    if (!(request_sdos_[i].actual_pos =
              ecrt_slave_config_create_sdo_request(slaves_[i].slave_config_, OD_TARGET_POSITION, sizeof(int32_t))))
    {
      fprintf(stderr, "Failed to create SDO request for Actual Position.\n");
      return -1;
    }
    ecrt_sdo_request_timeout(request_sdos_[i].actual_pos, 500);  // ms

    if (!(request_sdos_[i].target_tor =
              ecrt_slave_config_create_sdo_request(slaves_[i].slave_config_, OD_TARGET_TORQUE, sizeof(int16_t))))
    {
      fprintf(stderr, "Failed to create SDO request for Target Torque\n");
      return -1;
    }
    ecrt_sdo_request_timeout(request_sdos_[i].target_tor, 500);  // ms

    if (!(request_sdos_[i].actual_tor =
              ecrt_slave_config_create_sdo_request(slaves_[i].slave_config_, OD_TARGET_TORQUE, sizeof(int16_t))))
    {
      fprintf(stderr, "Failed to create SDO request for Actual Torque\n");
      return -1;
    }
    ecrt_sdo_request_timeout(request_sdos_[i].actual_tor, 500);  // ms

    if (!(request_sdos_[i].control_word =
              ecrt_slave_config_create_sdo_request(slaves_[i].slave_config_, OD_CONTROL_WORD, sizeof(uint16_t))))
    {
      fprintf(stderr, "Failed to create SDO request for Control Word\n");
      return -1;
    }
    ecrt_sdo_request_timeout(request_sdos_[i].control_word, 500);  // ms

    if (!(request_sdos_[i].status_word =
              ecrt_slave_config_create_sdo_request(slaves_[i].slave_config_, OD_STATUS_WORD, sizeof(uint16_t))))
    {
      fprintf(stderr, "Failed to create SDO request for Status Word\n");
      return -1;
    }
    ecrt_sdo_request_timeout(request_sdos_[i].status_word, 500);  // ms

    if (!(request_sdos_[i].op_mode =
              ecrt_slave_config_create_sdo_request(slaves_[i].slave_config_, OD_OPERATION_MODE, sizeof(uint8_t))))
    {
      fprintf(stderr, "Failed to create SDO request for Op Mode \n");
      return -1;
    }
    ecrt_sdo_request_timeout(request_sdos_[i].op_mode, 500);  // ms

    if (!(request_sdos_[i].op_mode_display =
              ecrt_slave_config_create_sdo_request(slaves_[i].slave_config_, OD_OPERATION_MODE_DISPLAY, 1)))
    {
      fprintf(stderr, "Failed to create SDO request for Op Mode Display\n");
      return -1;
    }
    ecrt_sdo_request_timeout(request_sdos_[i].op_mode_display, 500);  // ms
  }
  return 0;
}

void EthercatNode::WriteSDO(ec_sdo_request_t* req, int32_t data, int size)
{
  switch (ecrt_sdo_request_state(req))
  {
    case EC_REQUEST_UNUSED:         // request was not used yet
      ecrt_sdo_request_write(req);  // trigger first write
      break;
    case EC_REQUEST_BUSY:
      fprintf(stderr, "SDO write still busy...\n");
      break;
    case EC_REQUEST_SUCCESS:
      if (size == 1)
        EC_WRITE_U8(ecrt_sdo_request_data(req), data & 0xff);
      else if (size == 2)
        EC_WRITE_U16(ecrt_sdo_request_data(req), data & 0xffff);
      else if (size == 4)
        EC_WRITE_U32(ecrt_sdo_request_data(req), data & 0Xffffffff);
      else
        fprintf(stderr, "Size value is invalid, try 1,2 or 4\n");
      break;
    case EC_REQUEST_ERROR:
      fprintf(stderr, "Failed to write SDO!\n");
      ecrt_sdo_request_write(req);  // retry writing
      break;
  }
}

uint16_t EthercatNode::ReadSDO(ec_sdo_request_t* req, uint16_t& status_word)
{
  switch (ecrt_sdo_request_state(req))
  {
    case EC_REQUEST_UNUSED:        // request was not used yet
      ecrt_sdo_request_read(req);  // trigger first read
      break;
    case EC_REQUEST_BUSY:
      fprintf(stderr, "SDO still busy...\n");
      break;
    case EC_REQUEST_SUCCESS:
      status_word = EC_READ_U32(ecrt_sdo_request_data(req));
      ecrt_sdo_request_read(req);  // trigger next read
      break;
    case EC_REQUEST_ERROR:
      fprintf(stderr, "Failed to read SDO!\n");
      ecrt_sdo_request_read(req);  // retry reading
      break;
  }
}