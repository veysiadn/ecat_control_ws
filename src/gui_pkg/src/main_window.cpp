#include "../include/gui_pkg/main_window.hpp"

MainWindow::MainWindow(int argc, char** argv, QWidget* parent)
  : QMainWindow(parent), ui(new Ui::MainWindow), argc_(argc), argv_(argv)
{
  ui->setupUi(this);
  ros_spin_thread_ = std::thread{ std::bind(&MainWindow::rosSpinThread, this) };
  this->my_timer.setInterval(30);  // Update rate 30 ms for GUI.
  this->my_timer.start();
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    connect(ui->b_send[i], &MyButton::buttonClicked, this, &MainWindow::b_send_clicked);
    connect(ui->b_stop[i], SIGNAL(buttonClicked(int)), this, SLOT(b_stop_clicked(int)));
    connect(ui->b_enable[i], SIGNAL(buttonClicked(int)), this, SLOT(b_enable_clicked(int)));
    connect(ui->b_disable[i], SIGNAL(buttonClicked(int)), this, SLOT(b_disable_clicked(int)));
    connect(ui->b_vel[i], SIGNAL(buttonClicked(int)), this, SLOT(b_vel_clicked(int)));
    connect(ui->b_pos[i], SIGNAL(buttonClicked(int)), this, SLOT(b_pos_clicked(int)));
    connect(ui->b_cyclic_vel[i], SIGNAL(buttonClicked(int)), this, SLOT(b_cyclic_vel_clicked(int)));
    connect(ui->b_cyclic_pos[i], SIGNAL(buttonClicked(int)), this, SLOT(b_cyclic_pos_clicked(int)));
    connect(ui->b_cyclic_tor[i], SIGNAL(buttonClicked(int)), this, SLOT(b_cyclic_tor_clicked(int)));
  }
  connect(&my_timer, SIGNAL(timeout()), this, SLOT(UpdateGUI()));
  CallUnconfiguredStateUI();
}

MainWindow::~MainWindow()
{
  rclcpp::shutdown();
  delete ui;
}

// Start ROS2 NODE
void MainWindow::rosSpinThread()
{
  rclcpp::init(argc_, argv_);
  gui_node_ = std::make_shared<GuiNode>();
  gui_node_->slave_feedback_data_.status_word.resize(g_kNumberOfServoDrivers);
  gui_node_->slave_feedback_data_.actual_pos.resize(g_kNumberOfServoDrivers);
  gui_node_->slave_feedback_data_.actual_vel.resize(g_kNumberOfServoDrivers);
  gui_node_->slave_feedback_data_.actual_tor.resize(g_kNumberOfServoDrivers);
  gui_node_->slave_feedback_data_.error_code.resize(g_kNumberOfServoDrivers);
  gui_node_->slave_feedback_data_.op_mode_display.resize(g_kNumberOfServoDrivers);
  gui_node_->slave_feedback_data_.slave_com_status.resize(g_kNumberOfServoDrivers);
  gui_node_->master_command_data_.control_word.resize(g_kNumberOfServoDrivers);
  gui_node_->master_command_data_.target_pos.resize(g_kNumberOfServoDrivers);
  gui_node_->master_command_data_.target_vel.resize(g_kNumberOfServoDrivers);
  gui_node_->master_command_data_.target_tor.resize(g_kNumberOfServoDrivers);
  gui_node_->ui_control_buttons_.spn_target_values.resize(g_kNumberOfServoDrivers);
  rclcpp::spin(gui_node_);
  rclcpp::shutdown();
}

void MainWindow::UpdateGUI()
{
  ShowEmergencyStatus();
  ShowComStatus();
  ShowAllMotorStatus();
  ShowOperationMode();
}

void MainWindow::b_send_clicked(int m_no)
{
  gui_node_->ui_control_buttons_.b_send[m_no] = 1;
  gui_node_->ui_control_buttons_.spn_target_values[m_no] = ui->spn_target_val_[m_no]->value();
  gui_node_->PublishGuiEvents();
}

void MainWindow::b_stop_clicked(int m_no)
{
  gui_node_->ui_control_buttons_.b_stop[m_no] = 1;
  gui_node_->PublishGuiEvents();
}

void MainWindow::b_enable_clicked(int m_no)
{
  gui_node_->ui_control_buttons_.b_enable[m_no] = 1;
  gui_node_->PublishGuiEvents();
}

void MainWindow::b_disable_clicked(int m_no)
{
  gui_node_->ui_control_buttons_.b_disable[m_no] = 1;
  gui_node_->PublishGuiEvents();
}

void MainWindow::b_vel_clicked(int m_no)
{
  if (ui->b_vel[m_no]->isChecked())
  {
    DisableOtherModes(ui->b_vel[m_no], m_no);
    gui_node_->ui_control_buttons_.b_vel[m_no] = 1;
    gui_node_->PublishGuiEvents();
  }
  else
  {
    EnableAllModes(m_no);
  }
}

void MainWindow::b_cyclic_vel_clicked(int m_no)
{
  if (ui->b_cyclic_vel[m_no]->isChecked())
  {
    DisableOtherModes(ui->b_cyclic_vel[m_no], m_no);
    gui_node_->ui_control_buttons_.b_cyclic_vel[m_no] = 1;
    gui_node_->PublishGuiEvents();
  }
  else
  {
    EnableAllModes(m_no);
  }
}

void MainWindow::b_pos_clicked(int m_no)
{
  if (ui->b_pos[m_no]->isChecked())
  {
    DisableOtherModes(ui->b_pos[m_no], m_no);
    gui_node_->ui_control_buttons_.b_pos[m_no] = 1;
    gui_node_->PublishGuiEvents();
  }
  else
  {
    EnableAllModes(m_no);
  }
}

void MainWindow::b_cyclic_pos_clicked(int m_no)
{
  if (ui->b_cyclic_pos[m_no]->isChecked())
  {
    DisableOtherModes(ui->b_cyclic_pos[m_no], m_no);
    gui_node_->ui_control_buttons_.b_cyclic_pos[m_no] = 1;
    gui_node_->PublishGuiEvents();
  }
  else
  {
    EnableAllModes(m_no);
  }
}

void MainWindow::b_cyclic_tor_clicked(int m_no)
{
  if (ui->b_cyclic_tor[m_no]->isChecked())
  {
    DisableOtherModes(ui->b_cyclic_tor[m_no], m_no);
    gui_node_->ui_control_buttons_.b_cyclic_tor[m_no] = 1;
    gui_node_->PublishGuiEvents();
  }
  else
  {
    EnableAllModes(m_no);
  }
}

void MainWindow::DisableOtherModes(QPushButton* button, int index)
{
  ui->b_vel[index]->setDisabled(true);
  ui->b_pos[index]->setDisabled(true);
  ui->b_cyclic_vel[index]->setDisabled(true);
  ui->b_cyclic_pos[index]->setDisabled(true);
  ui->b_cyclic_tor[index]->setDisabled(true);
  button->setEnabled(true);
}

void MainWindow::EnableAllModes(int index)
{
  ui->b_vel[index]->setEnabled(true);
  ui->b_pos[index]->setEnabled(true);
  ui->b_cyclic_vel[index]->setEnabled(true);
  ui->b_cyclic_pos[index]->setEnabled(true);
  ui->b_cyclic_tor[index]->setEnabled(true);
}

void MainWindow::SetDisabledStyleSheet(QPushButton* button)
{
  button->setEnabled(false);
  // button->setStyleSheet(
  //     "color: rgb(33, 33, 33);"
  //     "background-color:gray;"
  //     "font: bold 75 15pt;");
}

void MainWindow::SetEnabledStyleSheetSDO(QPushButton* button)
{
  button->setEnabled(true);
  // button->setStyleSheet(sdo_style_sheet_);
}

void MainWindow::SetEnabledStyleSheetPDO(QPushButton* button)
{
  button->setEnabled(true);
  // button->setStyleSheet(blue_style_sheet_);
}

void MainWindow::on_b_enable_cyclic_pos_clicked()
{
  gui_node_->ui_control_buttons_.b_enable_cyclic_pos = 1;
  gui_node_->PublishGuiEvents();
}

void MainWindow::on_b_enable_cyclic_vel_clicked()
{
  gui_node_->ui_control_buttons_.b_enable_cyclic_vel = 1;
  gui_node_->PublishGuiEvents();
}

void MainWindow::on_b_enable_vel_clicked()
{
  gui_node_->ui_control_buttons_.b_enable_vel = 1;
  gui_node_->PublishGuiEvents();
}
void MainWindow::on_b_enable_cyclic_torque_clicked()
{
  gui_node_->ui_control_buttons_.b_enable_cyclic_torque = 1;
  gui_node_->PublishGuiEvents();
}
void MainWindow::on_b_enable_pos_clicked()
{
  gui_node_->ui_control_buttons_.b_enable_pos = 1;
  gui_node_->PublishGuiEvents();
}

void MainWindow::on_b_init_ecat_clicked()
{
  gui_node_->ui_control_buttons_.b_init_ecat = 1;
  gui_node_->PublishGuiEvents();
  int time_out_counter = 0;
  while ((gui_node_->slave_feedback_data_.current_lifecycle_state == kConfiguring ||
          gui_node_->slave_feedback_data_.current_lifecycle_state != kInactive) &&
         time_out_counter != 10)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    time_out_counter++;
  }
  if (gui_node_->slave_feedback_data_.current_lifecycle_state != kInactive)
    return;
  CallInactiveStateUI();
}

void MainWindow::on_b_reinit_ecat_clicked()
{
  gui_node_->ui_control_buttons_.b_reinit_ecat = 1;
  gui_node_->PublishGuiEvents();
  int time_out_counter = 0;
  while ((gui_node_->slave_feedback_data_.current_lifecycle_state == kCleaningUp ||
          gui_node_->slave_feedback_data_.current_lifecycle_state != kUnconfigured) &&
         time_out_counter != 10)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    time_out_counter++;
  }
  if (gui_node_->slave_feedback_data_.current_lifecycle_state != kUnconfigured)
    return;
  CallUnconfiguredStateUI();
}

void MainWindow::on_b_enable_drives_clicked()
{
  gui_node_->ui_control_buttons_.b_enable_drives = 1;
  gui_node_->PublishGuiEvents();
}

void MainWindow::on_b_disable_drives_clicked()
{
  gui_node_->ui_control_buttons_.b_disable_drives = 1;
  gui_node_->PublishGuiEvents();
}

void MainWindow::on_b_enter_cyclic_pdo_clicked()
{
  gui_node_->ui_control_buttons_.b_enter_cyclic_pdo = 1;
  gui_node_->PublishGuiEvents();
  int time_out_counter = 0;
  while ((gui_node_->slave_feedback_data_.current_lifecycle_state == kActivating ||
          gui_node_->slave_feedback_data_.current_lifecycle_state != kActive) &&
         time_out_counter != 10)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    time_out_counter++;
  }
  if (gui_node_->slave_feedback_data_.current_lifecycle_state != kActive)
    return;
  CallActiveStateUI();
}

void MainWindow::on_b_stop_cyclic_pdo_clicked()
{
  gui_node_->ui_control_buttons_.b_stop_cyclic_pdo = 1;
  gui_node_->PublishGuiEvents();
  int time_out_counter = 0;
  while ((gui_node_->slave_feedback_data_.current_lifecycle_state == kDeactivating ||
          gui_node_->slave_feedback_data_.current_lifecycle_state != kInactive) &&
         time_out_counter != 10)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    time_out_counter++;
  }
  if (gui_node_->slave_feedback_data_.current_lifecycle_state != kInactive)
    return;
  CallInactiveStateUI();
}

void MainWindow::on_b_clear_fault_clicked()
{
  gui_node_->ui_control_buttons_.b_clear_fault = 1;
  gui_node_->PublishGuiEvents();
}

void MainWindow::on_b_emergency_mode_clicked()
{
  if (!em_state_)
  {
    em_state_ = 1;
    gui_node_->ui_control_buttons_.b_emergency_mode = 1;
    gui_node_->PublishGuiEvents();
    ui->b_emergency_mode->setText("RESET");
    ui->b_emergency_mode->setStyleSheet(
        "QPushButton:pressed {"
        "background-color: rgb(5, 153, 44);}"
        "QPushButton { "
        "color: rgb(255, 255, 255);"
        "selection-background-color: rgb(238, 238, 236);"
        "selection-color: rgb(238, 238, 236);"
        "background-color: rgb(19, 61, 128);"
        "font: bold 75 16pt \"Noto Sans\";}");
  }
  else
  {
    gui_node_->ui_control_buttons_.b_emergency_mode = 0;
    gui_node_->PublishGuiEvents();
    em_state_ = 0;
    ui->b_emergency_mode->setText("Emergency Mode");
    ui->b_emergency_mode->setStyleSheet(
        "QPushButton:pressed {"
        "background-color: rgb(19, 61, 128);}"
        "QPushButton {"
        "color: rgb(255, 255, 255);"
        "background-color: rgb(252, 0, 0);"
        "font: bold 75 15pt \"Noto Sans\";}");
  }
}

void MainWindow::ResetControlButtonValues(unsigned char& button_val)
{
  gui_node_->ui_control_buttons_.b_init_ecat = 0;
  gui_node_->ui_control_buttons_.b_reinit_ecat = 0;
  gui_node_->ui_control_buttons_.b_enable_drives = 0;
  gui_node_->ui_control_buttons_.b_disable_drives = 0;
  gui_node_->ui_control_buttons_.b_enable_cyclic_pos = 0;
  gui_node_->ui_control_buttons_.b_enable_cyclic_vel = 0;
  gui_node_->ui_control_buttons_.b_enable_vel = 0;
  gui_node_->ui_control_buttons_.b_enable_pos = 0;
  gui_node_->ui_control_buttons_.b_enter_cyclic_pdo = 0;
  gui_node_->ui_control_buttons_.b_emergency_mode = 0;
  gui_node_->ui_control_buttons_.b_stop_cyclic_pdo = 0;
  button_val = 1;
}

void MainWindow::CallUnconfiguredStateUI()
{
  SetEnabledStyleSheetPDO(ui->b_init_ecat);
  SetDisabledStyleSheet(ui->b_reinit_ecat);
  SetDisabledStyleSheet(ui->b_enter_cyclic_pdo);
  SetDisabledStyleSheet(ui->b_stop_cyclic_pdo);
  SetDisabledStyleSheet(ui->b_enable_drives);
  SetDisabledStyleSheet(ui->b_disable_drives);
  SetDisabledStyleSheet(ui->b_enable_cyclic_pos);
  SetDisabledStyleSheet(ui->b_enable_cyclic_vel);
  SetDisabledStyleSheet(ui->b_enable_pos);
  SetDisabledStyleSheet(ui->b_enable_vel);
  SetDisabledStyleSheet(ui->b_enable_cyclic_torque);
  SetDisabledStyleSheet(ui->b_clear_fault);

  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    SetDisabledStyleSheet(ui->b_send[i]);
    SetDisabledStyleSheet(ui->b_stop[i]);
    SetDisabledStyleSheet(ui->b_enable[i]);
    SetDisabledStyleSheet(ui->b_disable[i]);
    SetDisabledStyleSheet(ui->b_disable[i]);
    SetDisabledStyleSheet(ui->b_vel[i]);
    SetDisabledStyleSheet(ui->b_pos[i]);
    SetDisabledStyleSheet(ui->b_cyclic_vel[i]);
    SetDisabledStyleSheet(ui->b_cyclic_pos[i]);
    SetDisabledStyleSheet(ui->b_cyclic_tor[i]);

    ui->lb_actual_pos[i]->setText("0");
    ui->lb_target_pos[i]->setText("0");
    ui->lb_actual_vel[i]->setText("0");
    ui->lb_target_vel[i]->setText("0");
    ui->lb_actual_tor[i]->setText("0");
    ui->lb_target_tor[i]->setText("0");
    ui->lb_status_word[i]->setText("NOT READY");
    ui->lb_control_word[i]->setText("0");
    ui->lb_op_mode[i]->setText("Not Selected");
  }
}

void MainWindow::CallInactiveStateUI()
{
  SetDisabledStyleSheet(ui->b_init_ecat);
  SetDisabledStyleSheet(ui->b_stop_cyclic_pdo);

  SetEnabledStyleSheetPDO(ui->b_reinit_ecat);
  SetEnabledStyleSheetPDO(ui->b_enter_cyclic_pdo);
  SetEnabledStyleSheetSDO(ui->b_enable_drives);
  SetEnabledStyleSheetSDO(ui->b_disable_drives);
  SetEnabledStyleSheetSDO(ui->b_enable_vel);
  SetEnabledStyleSheetSDO(ui->b_enable_pos);
  SetEnabledStyleSheetSDO(ui->b_enable_cyclic_vel);
  SetEnabledStyleSheetSDO(ui->b_enable_cyclic_pos);
  SetEnabledStyleSheetSDO(ui->b_enable_cyclic_torque);
  SetEnabledStyleSheetSDO(ui->b_clear_fault);

  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    SetEnabledStyleSheetSDO(ui->b_send[i]);
    SetEnabledStyleSheetSDO(ui->b_stop[i]);
    SetEnabledStyleSheetSDO(ui->b_enable[i]);
    SetEnabledStyleSheetSDO(ui->b_disable[i]);
    SetEnabledStyleSheetSDO(ui->b_vel[i]);
    SetEnabledStyleSheetSDO(ui->b_pos[i]);
    SetEnabledStyleSheetSDO(ui->b_cyclic_vel[i]);
    SetEnabledStyleSheetSDO(ui->b_cyclic_pos[i]);
    SetEnabledStyleSheetSDO(ui->b_cyclic_tor[i]);
  }
}

void MainWindow::CallActiveStateUI()
{
  SetEnabledStyleSheetPDO(ui->b_stop_cyclic_pdo);
  SetDisabledStyleSheet(ui->b_enter_cyclic_pdo);
  SetDisabledStyleSheet(ui->b_init_ecat);
  SetDisabledStyleSheet(ui->b_reinit_ecat);
  SetDisabledStyleSheet(ui->b_enable_drives);
  SetDisabledStyleSheet(ui->b_disable_drives);
  SetDisabledStyleSheet(ui->b_enable_pos);
  SetDisabledStyleSheet(ui->b_enable_vel);
  SetDisabledStyleSheet(ui->b_enable_cyclic_vel);
  SetDisabledStyleSheet(ui->b_enable_cyclic_pos);
  SetDisabledStyleSheet(ui->b_enable_cyclic_torque);
  SetDisabledStyleSheet(ui->b_clear_fault);

  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    SetDisabledStyleSheet(ui->b_send[i]);
    SetDisabledStyleSheet(ui->b_stop[i]);
    SetDisabledStyleSheet(ui->b_enable[i]);
    SetDisabledStyleSheet(ui->b_disable[i]);
    SetDisabledStyleSheet(ui->b_vel[i]);
    SetDisabledStyleSheet(ui->b_pos[i]);
    SetDisabledStyleSheet(ui->b_cyclic_vel[i]);
    SetDisabledStyleSheet(ui->b_cyclic_pos[i]);
    SetDisabledStyleSheet(ui->b_cyclic_tor[i]);
  }
}
/// TODO: Implement this function
void MainWindow::ShowEmergencyStatus()
{
  QString qstr;
  if (!gui_node_->ui_control_buttons_.b_emergency_mode)
  {
    QTextStream(&qstr) << "IDLE";
    // ui->lb_emergency_status->setText(qstr);
    // ui->lb_emergency_status->setStyleSheet("QLabel{background:green;"
    //                                        "color:white;"
    //                                        "font:bold 75 12pt \"Noto Sans\";}");
    qstr.clear();
  }
  else
  {
    // QTextStream(&qstr) << "EMERGENCY MODE";
    // ui->lb_emergency_status->setText(qstr);
    // ui->lb_emergency_status->setStyleSheet("QLabel{background:red;"
    //                                        "color:white;"
    //                                        "font:bold 75 12pt \"Noto Sans\";}");
    qstr.clear();
  }
}

void MainWindow::ShowComStatus()
{
  QString qstr;
  int state = gui_node_->slave_feedback_data_.com_status;
  if (state == EC_AL_STATE_OP)
  {
    QTextStream(&qstr) << "OPERATIONAL";
    ui->lb_com_status->setText(qstr);
    ui->lb_com_status->setStyleSheet(
        "QLabel{background:green;"
        "color:white;"
        "font:bold 75 12pt \"Noto Sans\";}");
    qstr.clear();
  }
  else if (state == EC_AL_STATE_SAFEOP)
  {
    QTextStream(&qstr) << "SAFE OPERATIONAL";
    ui->lb_com_status->setText(qstr);
    ui->lb_com_status->setStyleSheet(
        "QLabel{background:yellow;"
        "color:black;"
        "font:bold 75 12pt \"Noto Sans\";}");
    qstr.clear();
  }
  else if (state == EC_AL_STATE_PREOP)
  {
    QTextStream(&qstr) << "PRE OPERATIONAL";
    ui->lb_com_status->setText(qstr);
    ui->lb_com_status->setStyleSheet(
        "QLabel{background:yellow;"
        "color:black;"
        "font:bold 75 12pt \"Noto Sans\";}");
    qstr.clear();
  }
  else if (state == EC_AL_STATE_INIT)
  {
    QTextStream(&qstr) << "INIT";
    ui->lb_com_status->setText(qstr);
    ui->lb_com_status->setStyleSheet(
        "QLabel{background:red;"
        "color:white;"
        "font:bold 75 12pt \"Noto Sans\";}");
    qstr.clear();
  }
  else
  {
    QTextStream(&qstr) << "NO CONNECTION";
    ui->lb_com_status->setText(qstr);
    ui->lb_com_status->setStyleSheet(
        "QLabel{background:red;"
        "color:white;"
        "font:bold 75 12pt \"Noto Sans\";}");
    qstr.clear();
  }
}

void MainWindow::ShowAllMotorStatus()
{
  QString qstr;
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    // Target vel.
    QTextStream(&qstr) << gui_node_->master_command_data_.target_vel[i];
    ui->lb_target_vel[i]->setText(qstr);
    qstr.clear();
    // Actual vel
    QTextStream(&qstr) << gui_node_->slave_feedback_data_.actual_vel[i];
    ui->lb_actual_vel[i]->setText(qstr);
    qstr.clear();
    // Target pos
    QTextStream(&qstr) << gui_node_->master_command_data_.target_pos[i];
    ui->lb_target_pos[i]->setText(qstr);
    qstr.clear();
    // Actual Pos
    QTextStream(&qstr) << gui_node_->slave_feedback_data_.actual_pos[i];
    ui->lb_actual_pos[i]->setText(qstr);
    qstr.clear();
    // Target torque
    QTextStream(&qstr) << gui_node_->master_command_data_.target_tor[i];
    ui->lb_target_tor[i]->setText(qstr);
    qstr.clear();
    // Actual torque
    QTextStream(&qstr) << gui_node_->slave_feedback_data_.actual_tor[i];
    ui->lb_actual_tor[i]->setText(qstr);
    qstr.clear();

    QTextStream(&qstr) << gui_node_->master_command_data_.control_word[i];
    ui->lb_control_word[i]->setText(qstr);
    qstr.clear();

    qstr = GetReadableStatusWord(i);
    qstr.clear();

    QTextStream(&qstr) << GetDriveErrorMessage(gui_node_->slave_feedback_data_.error_code[i]);
    ui->lb_motor_error_code[i]->setText(qstr);
    qstr.clear();
  }
}

int MainWindow::GetDriveStates(const int& statusWord)
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

void MainWindow::ShowOperationMode()
{
  QString qstr;
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    switch (gui_node_->slave_feedback_data_.op_mode_display[i])
    {
      case kProfileVelocity:
        QTextStream(&qstr) << "Velocity";
        break;
      case kProfilePosition:
        QTextStream(&qstr) << "Position";
        break;
      case kProfileTorque:
        QTextStream(&qstr) << "Torque";
        break;
      case kCSVelocity:
        QTextStream(&qstr) << "CSV";
        break;
      case kCSPosition:
        QTextStream(&qstr) << "CSP";
        break;
      case kCSTorque:
        QTextStream(&qstr) << "CST";
        break;
      default:
        QTextStream(&qstr) << "Not Selected";
        break;
    }
    ui->lb_op_mode[i]->setText(qstr);
    qstr.clear();
  }
}

QString MainWindow::GetReadableStatusWord(int index)
{
  QString qstr;
  qstr.clear();
  if (GetDriveStates(gui_node_->slave_feedback_data_.status_word[index]) == kOperationEnabled)
  {
    if (gui_node_->slave_feedback_data_.op_mode_display[index] == kProfilePosition)
    {
      if (TEST_BIT(gui_node_->slave_feedback_data_.status_word[index], 10))
      {
        QTextStream(&qstr) << "READY";
        ui->lb_status_word[index]->setText(qstr);
        ui->lb_status_word[index]->setStyleSheet(
            "QLabel{background:green;"
            "color:black;"
            "font:bold 75 12pt \"Noto Sans\";}");
      }
      else
      {
        QTextStream(&qstr) << "MOVING";
        ui->lb_status_word[index]->setText(qstr);
        ui->lb_status_word[index]->setStyleSheet(
            "QLabel{background:yellow;"
            "color:black;"
            "font:bold 75 12pt \"Noto Sans\";}");
      }
    }
    else if (gui_node_->slave_feedback_data_.op_mode_display[index] == kProfileVelocity)
    {
      if (TEST_BIT(gui_node_->slave_feedback_data_.status_word[index], 12))
      {
        QTextStream(&qstr) << "READY";

        ui->lb_status_word[index]->setText(qstr);
        ui->lb_status_word[index]->setStyleSheet(
            "QLabel{background:green;"
            "color:black;"
            "font:bold 75 12pt \"Noto Sans\";}");
      }
      else
      {
        QTextStream(&qstr) << "MOVING";

        ui->lb_status_word[index]->setText(qstr);
        ui->lb_status_word[index]->setStyleSheet(
            "QLabel{background:yellow;"
            "color:black;"
            "font:bold 75 12pt \"Noto Sans\";}");
      }
    }
    else if (gui_node_->slave_feedback_data_.op_mode_display[index] == kProfileTorque ||
             gui_node_->slave_feedback_data_.op_mode_display[index] == kCSTorque ||
             gui_node_->slave_feedback_data_.op_mode_display[index] == kCSVelocity ||
             gui_node_->slave_feedback_data_.op_mode_display[index] == kCSPosition)
    {
      if (TEST_BIT(gui_node_->slave_feedback_data_.status_word[index], 12))
      {
        QTextStream(&qstr) << "READY";

        ui->lb_status_word[index]->setText(qstr);
        ui->lb_status_word[index]->setStyleSheet(
            "QLabel{background:green;"
            "color:black;"
            "font:bold 75 12pt \"Noto Sans\";}");
      }
      else
      {
        QTextStream(&qstr) << "MOVING";

        ui->lb_status_word[index]->setText(qstr);
        ui->lb_status_word[index]->setStyleSheet(
            "QLabel{background:yellow;"
            "color:black;"
            "font:bold 75 12pt \"Noto Sans\";}");
      }
    }
    else
    {
      QTextStream(&qstr) << "DISABLED";
      ui->lb_status_word[index]->setText(qstr);
      ui->lb_status_word[index]->setStyleSheet(
          "QLabel{background:white;"
          "color:black;"
          "font:bold 75 12pt \"Noto Sans\";}");
    }
  }
  else if (GetDriveStates(gui_node_->slave_feedback_data_.status_word[index]) == kQuickStop)
  {
    QTextStream(&qstr) << "STOPPED";
    ui->lb_status_word[index]->setText(qstr);
    ui->lb_status_word[index]->setStyleSheet(
        "QLabel{background:white;"
        "color:black;"
        "font:bold 75 12pt \"Noto Sans\";}");
  }
  else
  {
    QTextStream(&qstr) << "DISABLED";
    ui->lb_status_word[index]->setText(qstr);
    ui->lb_status_word[index]->setStyleSheet(
        "QLabel{background:white;"
        "color:black;"
        "font:bold 75 12pt \"Noto Sans\";}");
  }
  return qstr;
}

QString MainWindow::GetDriveErrorMessage(const int& err_code)
{
  switch (err_code)
  {
    case NO_ERROR:
      return "No error";
    case GENERIC_ERROR:
      return "Generic error";
    case GENERIC_INIT_ERROR:
      return "Generic initialization error";
    case GENERIC_INIT_ERROR_1:
      return "Generic initialization error 1";
    case GENERIC_INIT_ERROR_2:
      return "Generic initialization error 2";
    case GENERIC_INIT_ERROR_3:
      return "Generic initialization error 3";
    case GENERIC_INIT_ERROR_4:
      return "Generic initialization error 4";
    case GENERIC_INIT_ERROR_5:
      return "Generic initialization error 5";
    case GENERIC_INIT_ERROR_6:
      return "Generic initialization error 6";
    case GENERIC_INIT_ERROR_7:
      return "Generic initialization error 7";
    case GENERIC_INIT_ERROR_8:
      return "Generic initialization error 8";
    case FIRMWARE_INCOMPATIBLITY_ERROR:
      return "Firmware incompatibility error";
    case OVER_CURRENT_ERROR:
      return "Over current error";
    case POWER_STAGE_PROTECTION_ERROR:
      return "Power stage protection error";
    case OVER_VOLTAGE_ERROR:
      return "Over voltage error";
    case UNDER_VOLTAGE_ERROR:
      return "Under voltage error";
    case THERMAL_OVERLOAD_ERROR:
      return "Thermal overload error";
    case THERMAL_MOTOR_OVERLOAD_ERRROR:
      return "Thermal motor overload error";
    case LOGIC_SUPPLY_TOO_LOW_ERROR:
      return "Logic supply too low error";
    case HARDWARE_DEFECT_ERROR:
      return "Hardware defect error";
    case HARDWARE_INCOMPATIBLITY_ERROR:
      return "Hardware incompatibility error";
    case HARDWARE_ERROR:
      return "Hardware error";
    case HARDWARE_ERROR_1:
      return "Hardware error 1";
    case HARDWARE_ERROR_2:
      return "Hardware error 2";
    case HARDWARE_ERROR_3:
      return "Hardware error 3";
    case SIGN_OF_LIFE_ERROR:
      return "Sign of life error";
    case EXTENSION_1_WATCHDOG_ERROR:
      return "Extension 1 watchdog error";
    case INTERNAL_SOFTWARE_ERROR:
      return "Internal software error";
    case SOFTWARE_PARAMETER_ERROR:
      return "Software parameter error";
    case PERSISTENT_PARAMETER_CORRUPT_ERROR:
      return "Persistent parameter corrupt error";
    case POSITION_SENSOR_ERROR:
      return "Position sensor error";
    case POSITION_SENSOR_BREACH_ERROR:
      return "Position sensor breach error";
    case POSITION_SENSOR_RESOLUTION_ERROR:
      return "Position sensor resolution error";
    case POSITION_SENSOR_INDEX_ERROR:
      return "Position sensor index error";
    case HALL_SENSOR_ERROR:
      return "Hall sensor error";
    case HALL_SENSOR_NOT_FOUND_ERROR:
      return "Hall sensor not found error";
    case HALL_ANGLE_DETECTION_ERROR:
      return "Hall angle detection error";
    case SSI_SENSOR_ERROR:
      return "SSI sensor error";
    case SSI_SENSOR_FRAME_ERROR:
      return "SSI sensor frame error";
    case MISSING_MAIN_SENSOR_ERROR:
      return "Missing main sensor error";
    case MISSING_COMMUTATION_SENSOR_ERROR:
      return "Missing commutation sensor error";
    case MAIN_SENSOR_DIRECTION_ERROR:
      return "Main sensor direction error";
    case ETHERCAT_COMMUNCATION_ERROR:
      return "Ethercat communication error";
    case ETHERCAT_INITIALIZATION_ERROR:
      return "Ethercat initialization error";
    case ETHERCAT_RX_QUEUE_OVERFLOW_ERROR:
      return "Ethercat RX queue overflow error";
    case ETHERCAT_COMMUNICATION_ERROR_INTERNAL:
      return "Ethercat communication error internal";
    case ETHERCAT_COMMUNICATION_CYCLE_TIME_ERROR:
      return "Ethercat communication cycle time error";
    case ETHERCAT_PDO_COMMUNICATION_ERROR:
      return "Ethercat PDO communication error";
    case ETHERCAT_SDO_COMMUNICATION_ERROR:
      return "Ethercat SDO communication error";
    case FOLLOWING_ERROR:
      return "Following error";
    case NEGATIVE_LIMIT_SWITCH_ERROR:
      return "Negative limit switch error";
    case POSITIVE_LIMIT_SWITCH_ERROR:
      return "Positive limit switch error";
    case SOFTWARE_POSITION_LIMIT_ERROR:
      return "Software position limit error";
    case STO_ERROR:
      return "STO error";
    case SYSTEM_OVERLOADED_ERROR:
      return "System overloaded error";
    case WATCHDOG_ERROR:
      return "Watchdog error";
    case SYSTEM_PEAK_OVERLOADED_ERROR:
      return "System peak overloaded error";
    case CONTROLLER_GAIN_ERROR:
      return "Controller gain error";
    case AUTO_TUNING_INDENTIFICATION_ERROR:
      return "Auto tuning identification error";
    case AUTO_TUNING_CURRENT_LIMIT_ERROR:
      return "Auto tuning current limit error";
    case AUTO_TUNING_IDENTIFICATION_CURRENT_ERROR:
      return "Auto tuning identification current error";
    case AUTO_TUNING_DATA_SAMPLING_ERROR:
      return "Auto tuning data sampling error";
    case AUTO_TUNING_SAMPLE_MISMATCH_ERROR:
      return "Auto tuning sample mismatch error";
    case AUTO_TUNING_PARAMETER_ERROR:
      return "Auto tuning parameter error";
    case AUTO_TUNING_AMPLITUDE_MISMATCH_ERROR:
      return "Auto tuning amplitude mismatch error";
    case AUTO_TUNING_TIMEOUT_ERROR:
      return "Auto tuning timeout error";
    case AUTO_TUNING_STAND_STILL_ERROR:
      return "Auto tuning stand still error";
    case AUTO_TUNING_TORQUE_INVALID_ERROR:
      return "Auto tuning torque invalid error";
    case AUTO_TUNING_MAX_SYSTEM_SPEED_ERROR:
      return "Auto tuning max system speed error";
    case AUTO_TUNING_MOTOR_CONNECTION_ERROR:
      return "Auto tuning motor connection error";
    case AUTO_TUNING_SENSOR_SIGNAL_ERROR:
      return "Auto tuning sensor signal error";
    default:
      return "Unknown error";
  }
}
