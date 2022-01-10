#include "../include/gui_pkg/main_window.hpp"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
: QMainWindow(parent),
  ui(new Ui::MainWindow),
  argc_(argc),
  argv_(argv)
{
    ui->setupUi(this);
    ros_spin_thread_ = std::thread{std::bind(&MainWindow::rosSpinThread, this)};
    this->my_timer.setInterval(30);  // Update rate 30 ms for GUI.
    this->my_timer.start();
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
    rclcpp::spin(gui_node_);
    rclcpp::shutdown();
}

void MainWindow::UpdateGUI()
{

    // gui_node_->ui_control_buttons_.b_init_ecat = 0;
    //  Updating Additional GUI Part Veysi ADN
       ShowEmergencyStatus();
       ShowComStatus();
       ShowAllMotorStatus();
       ShowOperationMode();
}


void MainWindow::SetDisabledStyleSheet(QPushButton *button)
{
    button->setEnabled(false);
    button->setStyleSheet("color: rgb(33, 33, 33);"
                                         "background-color:gray;"
                                         "font: bold 75 15pt;");
}

void MainWindow::SetEnabledStyleSheetSDO(QPushButton *button)
{
    button->setEnabled(true);
    button->setStyleSheet(sdo_style_sheet_);
}

void MainWindow::SetEnabledStyleSheetPDO(QPushButton *button)
{
    button->setEnabled(true);
    button->setStyleSheet(blue_style_sheet_);
}

void MainWindow::on_b_enable_cyclic_pos_clicked()
{
    gui_node_->ui_control_buttons_.b_enable_cyclic_pos = 1 ;
    // Todo : Disable buttons/ apply disable stylesheet.Make other buttons 0;
    //this->setDisabledStyleSheet(ui->b_enable_cyclic_pos);
}

void MainWindow::on_b_enable_cylic_vel_clicked()
{
    gui_node_->ui_control_buttons_.b_enable_cyclic_vel = 1 ;
}

void MainWindow::on_b_enable_vel_clicked()
{
    gui_node_->ui_control_buttons_.b_enable_vel = 1 ;
}

void MainWindow::on_b_enable_pos_clicked()
{
    gui_node_->ui_control_buttons_.b_enable_pos = 1 ;
}

void MainWindow::on_b_init_ecat_clicked()
{
    gui_node_->ui_control_buttons_.b_init_ecat = 1 ;
    int time_out_counter = 0;
    while((gui_node_->current_lifecycle_state==kConfiguring || 
    gui_node_->current_lifecycle_state !=kInactive) && time_out_counter!=15){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        time_out_counter++;
    }
    CallInactiveStateUI();
}

void MainWindow::on_b_reinit_ecat_clicked()
{
    gui_node_->ui_control_buttons_.b_reinit_ecat = 1 ;
    int time_out_counter = 0;
    while((gui_node_->current_lifecycle_state==kCleaningUp || 
    gui_node_->current_lifecycle_state !=kUnconfigured) && time_out_counter!=15){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        time_out_counter++;
    }
    CallUnconfiguredStateUI();
}

void MainWindow::on_b_enable_drives_clicked()
{
    gui_node_->ui_control_buttons_.b_enable_drives = 1 ;
}

void MainWindow::on_b_disable_drives_clicked()
{
    gui_node_->ui_control_buttons_.b_disable_drives = 1 ;
}

void MainWindow::on_b_enter_cyclic_pdo_clicked()
{
    gui_node_->ui_control_buttons_.b_enter_cyclic_pdo = 1 ;
    int time_out_counter = 0;
    while((gui_node_->current_lifecycle_state==kActivating || 
    gui_node_->current_lifecycle_state !=kActive) && time_out_counter!=15){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        time_out_counter++;
    }
    CallActiveStateUI();
}

void MainWindow::on_b_stop_cyclic_pdo_clicked()
{
    gui_node_->ui_control_buttons_.b_stop_cyclic_pdo = 1 ;
    int time_out_counter = 0;
    while((gui_node_->current_lifecycle_state==kDeactivating || 
    gui_node_->current_lifecycle_state !=kInactive) && time_out_counter!=15){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        time_out_counter++;
    }
    if(gui_node_->current_lifecycle_state==kInactive)
        CallInactiveStateUI();
}

void MainWindow::on_b_emergency_mode_clicked()
{
    gui_node_->ui_control_buttons_.b_emergency_mode = 1 ;
}

void MainWindow::on_b_send_clicked()
{
    gui_node_->ui_control_buttons_.b_send = 1 ;
    for(int i = 0 ; i < NUM_OF_SERVO_DRIVES; i++){
        switch (i) {
            case 0:
                gui_node_->ui_control_buttons_.spn_target_values[i] = static_cast<int> (ui->spn_target_val_1->value());
                break;
            case 1:
                gui_node_->ui_control_buttons_.spn_target_values[i] = static_cast<int> (ui->spn_target_val_2->value());
                break;
            case 2:
                gui_node_->ui_control_buttons_.spn_target_values[i] = static_cast<int> (ui->spn_target_val_3->value());
                break;
        }
    }
}

void MainWindow::ResetControlButtonValues(unsigned char &button_val)
{
   gui_node_->ui_control_buttons_.b_init_ecat = 0 ;
   gui_node_->ui_control_buttons_.b_reinit_ecat = 0 ;
   gui_node_->ui_control_buttons_.b_enable_drives = 0 ;
   gui_node_->ui_control_buttons_.b_disable_drives = 0 ;
   gui_node_->ui_control_buttons_.b_enable_cyclic_pos = 0 ;
   gui_node_->ui_control_buttons_.b_enable_cyclic_vel = 0 ;
   gui_node_->ui_control_buttons_.b_enable_vel = 0 ;
   gui_node_->ui_control_buttons_.b_enable_pos = 0 ;
   gui_node_->ui_control_buttons_.b_enter_cyclic_pdo = 0 ;
   gui_node_->ui_control_buttons_.b_emergency_mode = 0 ;
   gui_node_->ui_control_buttons_.b_send = 0 ;
   gui_node_->ui_control_buttons_.b_stop_cyclic_pdo = 0 ;
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
    SetDisabledStyleSheet(ui->b_send);
    SetDisabledStyleSheet(ui->b_enable_cyclic_pos);
    SetDisabledStyleSheet(ui->b_enable_cylic_vel);
    SetDisabledStyleSheet(ui->b_enable_pos);
    SetDisabledStyleSheet(ui->b_enable_vel);

    ui->lb_actual_pos_m1->setText("0");
    ui->lb_actual_pos_m2->setText("0");
    ui->lb_actual_pos_m3->setText("0");

    ui->lb_target_pos_m1->setText("0");
    ui->lb_target_pos_m2->setText("0");
    ui->lb_target_pos_m3->setText("0");    

    ui->lb_actual_vel_m1->setText("0");
    ui->lb_actual_vel_m2->setText("0");
    ui->lb_actual_vel_m3->setText("0");  

    ui->lb_target_vel_m1->setText("0");
    ui->lb_target_vel_m2->setText("0");
    ui->lb_target_vel_m3->setText("0");           
    
    ui->lb_actual_tor_m1->setText("0");
    ui->lb_actual_tor_m2->setText("0");
    ui->lb_actual_tor_m3->setText("0");  

    ui->lb_target_tor_m1->setText("0");
    ui->lb_target_tor_m2->setText("0");
    ui->lb_target_tor_m3->setText("0");  


    ui->lb_status_word_m1->setText("NOT READY");
    ui->lb_control_word_m1->setText("0");

    ui->lb_status_word_m2->setText("NOT READY");
    ui->lb_control_word_m2->setText("0");

    ui->lb_status_word_m3->setText("NOT READY");
    ui->lb_control_word_m3->setText("0");

    ui->lb_op_mode_m1->setText("Not Selected");
    ui->lb_op_mode_m2->setText("Not Selected");
    ui->lb_op_mode_m3->setText("Not Selected");
}

void MainWindow::CallInactiveStateUI()
{
    SetDisabledStyleSheet(ui->b_init_ecat);
    SetDisabledStyleSheet(ui->b_stop_cyclic_pdo);

    SetEnabledStyleSheetPDO(ui->b_reinit_ecat);
    SetEnabledStyleSheetPDO(ui->b_enter_cyclic_pdo);

    SetEnabledStyleSheetSDO(ui->b_enable_drives);
    SetEnabledStyleSheetSDO(ui->b_disable_drives);
    SetEnabledStyleSheetSDO(ui->b_send);
    SetEnabledStyleSheetSDO(ui->b_enable_cyclic_pos);
    SetEnabledStyleSheetSDO(ui->b_enable_cylic_vel);
    SetEnabledStyleSheetSDO(ui->b_enable_pos);
    SetEnabledStyleSheetSDO(ui->b_enable_vel);
}

void MainWindow::CallActiveStateUI()
{
    SetEnabledStyleSheetPDO(ui->b_stop_cyclic_pdo);
    SetEnabledStyleSheetPDO(ui->b_enter_cyclic_pdo);

    SetDisabledStyleSheet(ui->b_init_ecat);
    SetDisabledStyleSheet(ui->b_reinit_ecat);
    SetDisabledStyleSheet(ui->b_enable_drives);
    SetDisabledStyleSheet(ui->b_disable_drives);
    SetDisabledStyleSheet(ui->b_send);
    SetDisabledStyleSheet(ui->b_enable_cyclic_pos);
    SetDisabledStyleSheet(ui->b_enable_cylic_vel);
    SetDisabledStyleSheet(ui->b_enable_pos);
    SetDisabledStyleSheet(ui->b_enable_vel);  
}

void MainWindow::ShowEmergencyStatus()
{   
   QString qstr;
   if(!gui_node_->received_data_[0].p_emergency_switch_val){
         SetDisabledStyleSheet(ui->b_emergency_mode);
         gui_node_->ui_control_buttons_.b_emergency_mode = 0;
   }else{
       if(!ui->b_emergency_mode->isEnabled()){
            ui->b_emergency_mode->setEnabled(true);
            ui->b_emergency_mode->setStyleSheet(red_style_sheet);
            gui_node_->ui_control_buttons_.b_emergency_mode = 0;
       }
   }
   if(gui_node_->received_data_[0].p_emergency_switch_val && !gui_node_->ui_control_buttons_.b_emergency_mode ){
         QTextStream(&qstr) << "IDLE";
         ui->lb_emergency_status->setText(qstr);
         ui->lb_emergency_status->setStyleSheet("QLabel{background:green;"
                                                  "color:white;"
                                                  "font:bold 75 12pt \"Noto Sans\";}");
         qstr.clear();
   }else{
         QTextStream(&qstr) << "EMERGENCY MODE";
         ui->lb_emergency_status->setText(qstr);
         ui->lb_emergency_status->setStyleSheet("QLabel{background:red;"
                                                  "color:white;"
                                                  "font:bold 75 12pt \"Noto Sans\";}");
         qstr.clear();
   }
}

void MainWindow::ShowComStatus()
{
   QString qstr;
   int state = gui_node_->received_data_[0].com_status;
   if(state == 0x08){

        QTextStream(&qstr) << "OPERATIONAL";
        ui->lb_com_status->setText(qstr);
        ui->lb_com_status->setStyleSheet("QLabel{background:green;"
                                           "color:white;"
                                           "font:bold 75 12pt \"Noto Sans\";}");
        qstr.clear();
   }
   else if (state == 0x04){
       QTextStream(&qstr) << "SAFE OPERATIONAL";
       ui->lb_com_status->setText(qstr);
       ui->lb_com_status->setStyleSheet("QLabel{background:yellow;"
                                          "color:black;"
                                          "font:bold 75 12pt \"Noto Sans\";}");
       qstr.clear();

   }
   else if (state == 0x02){
           QTextStream(&qstr) << "PRE OPERATIONAL";
           ui->lb_com_status->setText(qstr);
           ui->lb_com_status->setStyleSheet("QLabel{background:yellow;"
                                              "color:black;"
                                              "font:bold 75 12pt \"Noto Sans\";}");
           qstr.clear();

       }
   else if (state == 0x01 && state==0){
           QTextStream(&qstr) << "INIT";
           ui->lb_com_status->setText(qstr);
           ui->lb_com_status->setStyleSheet("QLabel{background:red;"
                                              "color:white;"
                                              "font:bold 75 12pt \"Noto Sans\";}");
           qstr.clear();

   }
   else {
       QTextStream(&qstr) << "NO CONNECTION";
       ui->lb_com_status->setText(qstr);
       ui->lb_com_status->setStyleSheet("QLabel{background:red;"
                                          "color:white;"
                                          "font:bold 75 12pt \"Noto Sans\";}");
       qstr.clear();

   }

}

void MainWindow::ShowAllMotorStatus()
{
   QString  qstr;
   for(int i = 0; i < NUM_OF_SERVO_DRIVES ;i++){
       switch (i) {
       case 0:
           QTextStream(&qstr) << gui_node_->received_data_[i].target_vel;
           ui->lb_target_vel_m1->setText(qstr);
         /*  ui->line_target_velocity_m1->setStyleSheet("QLabel{background:white;"
                                                      "color:black;"
                                                      "font:bold 75 12pt \"Noto Sans\";}");*/
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].control_word;
           ui->lb_control_word_m1->setText(qstr);
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].actual_vel;
           ui->lb_actual_vel_m1->setText(qstr);
           qstr.clear();

           qstr = GetReadableStatusWord(i);

           qstr.clear();
            
           QTextStream(&qstr) << gui_node_->received_data_[i].actual_pos;
           ui->lb_actual_pos_m1->setText(qstr);
           qstr.clear();

            QTextStream(&qstr) << gui_node_->received_data_[i].target_pos;
           ui->lb_target_pos_m1->setText(qstr);
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].target_tor;
           ui->lb_target_tor_m1->setText(qstr);
           qstr.clear();
           
           QTextStream(&qstr) << gui_node_->received_data_[i].actual_tor;
           ui->lb_actual_tor_m1->setText(qstr);
           qstr.clear();
           
           break;
       case 1:
           QTextStream(&qstr) << gui_node_->received_data_[i].target_vel;
           ui->lb_target_vel_m2->setText(qstr);
         /*  ui->line_target_velocity_m2->setStyleSheet("QLabel{background:white;"
                                                      "color:black;"
                                                      "font:bold 75 12pt \"Noto Sans\";}");*/
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].control_word;
           ui->lb_control_word_m2->setText(qstr);
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].actual_vel;
           ui->lb_actual_vel_m2->setText(qstr);
           qstr.clear();

           qstr = GetReadableStatusWord(i);

           qstr.clear();
            
           QTextStream(&qstr) << gui_node_->received_data_[i].actual_pos;
           ui->lb_actual_pos_m2->setText(qstr);
           qstr.clear();

            QTextStream(&qstr) << gui_node_->received_data_[i].target_pos;
           ui->lb_target_pos_m2->setText(qstr);
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].target_tor;
           ui->lb_target_tor_m2->setText(qstr);
           qstr.clear();
           
           QTextStream(&qstr) << gui_node_->received_data_[i].actual_tor;
           ui->lb_actual_tor_m2->setText(qstr);
           qstr.clear();
           break;
        case 2:
           QTextStream(&qstr) << gui_node_->received_data_[i].target_vel;
           ui->lb_target_vel_m3->setText(qstr);
         /*  ui->line_target_velocity_m3->setStyleSheet("QLabel{background:white;"
                                                      "color:black;"
                                                      "font:bold 75 12pt \"Noto Sans\";}");*/
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].control_word;
           ui->lb_control_word_m3->setText(qstr);
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].actual_vel;
           ui->lb_actual_vel_m3->setText(qstr);
           qstr.clear();

           qstr = GetReadableStatusWord(i);

           qstr.clear();
            
           QTextStream(&qstr) << gui_node_->received_data_[i].actual_pos;
           ui->lb_actual_pos_m3->setText(qstr);
           qstr.clear();

            QTextStream(&qstr) << gui_node_->received_data_[i].target_pos;
           ui->lb_target_pos_m3->setText(qstr);
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].target_tor;
           ui->lb_target_tor_m3->setText(qstr);
           qstr.clear();
           
           QTextStream(&qstr) << gui_node_->received_data_[i].actual_tor;
           ui->lb_actual_tor_m3->setText(qstr);
           qstr.clear();
           break;
       default:
           ui->lb_actual_pos_m1->setText("0");
           ui->lb_actual_pos_m2->setText("0");
           ui->lb_actual_pos_m3->setText("0");

           ui->lb_target_pos_m1->setText("0");
           ui->lb_target_pos_m2->setText("0");
           ui->lb_target_pos_m3->setText("0");    

           ui->lb_actual_vel_m1->setText("0");
           ui->lb_actual_vel_m2->setText("0");
           ui->lb_actual_vel_m3->setText("0");  

           ui->lb_target_vel_m1->setText("0");
           ui->lb_target_vel_m2->setText("0");
           ui->lb_target_vel_m3->setText("0");           
           
           ui->lb_actual_tor_m1->setText("0");
           ui->lb_actual_tor_m2->setText("0");
           ui->lb_actual_tor_m3->setText("0");  

           ui->lb_target_tor_m1->setText("0");
           ui->lb_target_tor_m2->setText("0");
           ui->lb_target_tor_m3->setText("0");  


           ui->lb_status_word_m1->setText("NOT READY");
           ui->lb_control_word_m1->setText("0");

           ui->lb_status_word_m2->setText("NOT READY");
           ui->lb_control_word_m2->setText("0");

           ui->lb_status_word_m3->setText("NOT READY");
           ui->lb_control_word_m3->setText("0");
           break;
       }

   }
}

int MainWindow::GetDriveStates(const int & statusWord)
{
    int state = 0;
    // bit 6 is 1
    if (TEST_BIT(statusWord,6))
    {
        state = kSwitchOnDisabled;  return state;
    }

    // bit 6 is 0 and bit 5 is 1
    if (TEST_BIT(statusWord,5))
    {
        if (TEST_BIT(statusWord,2)) {
            state = kOperationEnabled;  return state;
        }
        if (TEST_BIT(statusWord,1)) {
            state = kSwitchedOn;        return state;
        }
        if (TEST_BIT(statusWord,0)) {
            state = kReadyToSwitchOn;   return state;
        }
    }

    // bit 6 is 0 and bit 5 is 0
    if (TEST_BIT(statusWord,3)) {
        // For EPOS4, Fault or Fault Reaction Active,
        // See P2-14 of the Firmware Manual
        state = kFault;         return state;
    }
    else {
        // For EPOS4, Quick Stop Active or Not Switched on
        // See P2-14 of the Firmware Manual
        state = kQuickStop;     return state;
    }
    return state;
}

void MainWindow::ShowOperationMode()
{
    QString qstr;
    for(int i = 0; i < NUM_OF_SERVO_DRIVES ; i++){
        switch (gui_node_->received_data_[i].op_mode_display)
        {
        case kCSPosition:
            QTextStream(&qstr) << "Cyclic Sync Position";
            ui->lb_op_mode_m1->setText(qstr);
            ui->lb_op_mode_m2->setText(qstr);
            ui->lb_op_mode_m3->setText(qstr);
            break;
        case kCSVelocity:
            QTextStream(&qstr) << "Cyclic Sync Velocity";
            ui->lb_op_mode_m1->setText(qstr);
            ui->lb_op_mode_m2->setText(qstr);
            ui->lb_op_mode_m3->setText(qstr);
            break;
        case kProfilePosition:
            QTextStream(&qstr) << "Profile Position";
            ui->lb_op_mode_m1->setText(qstr);
            ui->lb_op_mode_m2->setText(qstr);
            ui->lb_op_mode_m3->setText(qstr);
            break;
        case kProfileVelocity:
            QTextStream(&qstr) << "Profile Velocity";
            ui->lb_op_mode_m1->setText(qstr);
            ui->lb_op_mode_m2->setText(qstr);
            ui->lb_op_mode_m3->setText(qstr);
            break;            
        default:
            QTextStream(&qstr) << "Not Selected";
            ui->lb_op_mode_m1->setText(qstr);
            ui->lb_op_mode_m2->setText(qstr);
            ui->lb_op_mode_m3->setText(qstr);
            break;
        }
    }
    
}

QString MainWindow::GetReadableStatusWord(int index)
{
   QString qstr;
    if(GetDriveStates(gui_node_->received_data_[index].status_word)==kOperationEnabled){
        if(gui_node_->received_data_[index].op_mode_display==kProfilePosition || 
            gui_node_->received_data_[index].op_mode_display==kCSPosition)
        {
            if (TEST_BIT(gui_node_->received_data_[index].status_word,10)){
                    QTextStream(&qstr) << "READY";
                    switch (index) {
                    case 0:
                        ui->lb_status_word_m1->setText(qstr);
                        ui->lb_status_word_m1->setStyleSheet("QLabel{background:green;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    case 1:
                        ui->lb_status_word_m2->setText(qstr);
                        ui->lb_status_word_m2->setStyleSheet("QLabel{background:green;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    case 2:
                        ui->lb_status_word_m3->setText(qstr);
                        ui->lb_status_word_m3->setStyleSheet("QLabel{background:green;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    default:
                        break;
                    }
            } else {
                    QTextStream(&qstr) << "MOVING";
                    switch (index) {
                    case 0:
                        ui->lb_status_word_m1->setText(qstr);
                        ui->lb_status_word_m1->setStyleSheet("QLabel{background:yellow;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    case 1:
                        ui->lb_status_word_m2->setText(qstr);
                        ui->lb_status_word_m2->setStyleSheet("QLabel{background:yellow;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    case 2:
                        ui->lb_status_word_m3->setText(qstr);
                        ui->lb_status_word_m3->setStyleSheet("QLabel{background:yellow;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    default:
                        break;
                    }
                }
        
        }else{
            if (TEST_BIT(gui_node_->received_data_[index].status_word,12)){
                    QTextStream(&qstr) << "READY";
                    switch (index) {
                    case 0:
                        ui->lb_status_word_m1->setText(qstr);
                        ui->lb_status_word_m1->setStyleSheet("QLabel{background:green;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    case 1:
                        ui->lb_status_word_m2->setText(qstr);
                        ui->lb_status_word_m2->setStyleSheet("QLabel{background:green;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    case 2:
                        ui->lb_status_word_m3->setText(qstr);
                        ui->lb_status_word_m3->setStyleSheet("QLabel{background:green;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    default:
                        break;
                    }
            } else {
                    QTextStream(&qstr) << "MOVING";
                    switch (index) {
                    case 0:
                        ui->lb_status_word_m1->setText(qstr);
                        ui->lb_status_word_m1->setStyleSheet("QLabel{background:yellow;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    case 1:
                        ui->lb_status_word_m2->setText(qstr);
                        ui->lb_status_word_m2->setStyleSheet("QLabel{background:yellow;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    case 2:
                        ui->lb_status_word_m3->setText(qstr);
                        ui->lb_status_word_m3->setStyleSheet("QLabel{background:yellow;"
                                                                "color:black;"
                                                                "font:bold 75 12pt \"Noto Sans\";}");
                        break;
                    default:
                        break;
                    }
                }
                
        }  
    }else{
         QTextStream(&qstr) << "NOT READY";
        ui->lb_status_word_m1->setText(qstr);
        ui->lb_status_word_m1->setStyleSheet("QLabel{background:white;"
                                                "color:black;"
                                                "font:bold 75 12pt \"Noto Sans\";}");
        ui->lb_status_word_m2->setText(qstr);
        ui->lb_status_word_m2->setStyleSheet("QLabel{background:white;"
                                                "color:black;"
                                                "font:bold 75 12pt \"Noto Sans\";}");
        ui->lb_status_word_m3->setText(qstr);
        ui->lb_status_word_m3->setStyleSheet("QLabel{background:white;"
                                                "color:black;"
                                                "font:bold 75 12pt \"Noto Sans\";}");                                                

    }
   return qstr;
}