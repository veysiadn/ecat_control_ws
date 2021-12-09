#include "../include/gui_pkg/main_window.hpp"
#include "ui_main_window.h"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
: QMainWindow(parent),
  ui(new Ui::MainWindow),
  argc_(argc),
  argv_(argv)
{
    ui->setupUi(this);
    // Activating ROS2 spinning functionality for subscribtion callbacks.
    control_ui_.setWindowTitle("Spine Robot Control UI");
    control_ui_.show();
    ros_spin_thread_ = std::thread{std::bind(&MainWindow::rosSpinThread, this)};
    this->my_timer.setInterval(25);  // Update rate 25 ms for GUI.
    this->my_timer.start();
    connect(&my_timer, SIGNAL(timeout()), this, SLOT(UpdateGUI()));

    opencv_video_cap =  new VideoCapture(this);
    connect(opencv_video_cap, &VideoCapture::NewPixmapCapture, this, [&]()
    {
       ui->image_frame->setPixmap(opencv_video_cap->pixmap().scaled(1100,720));
    });
    opencv_video_cap->start(QThread::HighestPriority);
}

MainWindow::~MainWindow()
{
  rclcpp::shutdown();
  delete ui;
  opencv_video_cap->terminate();
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
    // Updating Additional GUI Part Veysi ADN
    ShowEmergencyStatus();
    ShowComStatus();
    ShowAllMotorStatus();

}

void MainWindow::ShowEmergencyStatus()
{
    QString qstr;
    if(!gui_node_->received_data_[0].p_emergency_switch_val){
          setDisabledStyleSheet();
          gui_node_->emergency_button_val_ = 0;
    }

    if(gui_node_->received_data_[0].p_emergency_switch_val && gui_node_->emergency_button_val_ ){
          QTextStream(&qstr) << "IDLE";
          ui->line_emergency_switch->setText(qstr);
          ui->line_emergency_switch->setStyleSheet("QLabel{background:green;"
                                                   "color:white;"
                                                   "font:bold 75 12pt \"Noto Sans\";}");
          qstr.clear();
    }else{
          QTextStream(&qstr) << "EMERGENCY MODE";
          ui->line_emergency_switch->setText(qstr);
          ui->line_emergency_switch->setStyleSheet("QLabel{background:red;"
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
         ui->line_com_status->setText(qstr);
         ui->line_com_status->setStyleSheet("QLabel{background:green;"
                                            "color:white;"
                                            "font:bold 75 12pt \"Noto Sans\";}");
         qstr.clear();
    }
    else if (state == 0x04){
        QTextStream(&qstr) << "SAFE OPERATIONAL";
        ui->line_com_status->setText(qstr);
        ui->line_com_status->setStyleSheet("QLabel{background:yellow;"
                                           "color:white;"
                                           "font:bold 75 12pt \"Noto Sans\";}");
        qstr.clear();

    }
    else if (state == 0x02){
            QTextStream(&qstr) << "PRE OPERATIONAL";
            ui->line_com_status->setText(qstr);
            ui->line_com_status->setStyleSheet("QLabel{background:yello;"
                                               "color:white;"
                                               "font:bold 75 12pt \"Noto Sans\";}");
            qstr.clear();

        }
    else if (state == 0x01){
            QTextStream(&qstr) << "INIT";
            ui->line_com_status->setText(qstr);
            ui->line_com_status->setStyleSheet("QLabel{background:red;"
                                               "color:white;"
                                               "font:bold 75 12pt \"Noto Sans\";}");
            qstr.clear();

    }
    else {
        QTextStream(&qstr) << "NO CONNECTION";
        ui->line_com_status->setText(qstr);
        ui->line_com_status->setStyleSheet("QLabel{background:red;"
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
            QTextStream(&qstr) << gui_node_->received_data_[i].target_pos;
            ui->line_target_velocity_m1->setText(qstr);
          /*  ui->line_target_velocity_m1->setStyleSheet("QLabel{background:white;"
                                                       "color:black;"
                                                       "font:bold 75 12pt \"Noto Sans\";}");*/
            qstr.clear();

            QTextStream(&qstr) << gui_node_->received_data_[i].control_word;
            ui->line_control_word_m1->setText(qstr);
            qstr.clear();

            QTextStream(&qstr) << gui_node_->received_data_[i].actual_pos;
            ui->line_actual_velocity_m1->setText(qstr);
            qstr.clear();

            qstr = GetReadableStatusWord(i);

            qstr.clear();
            break;
        case 1:
            QTextStream(&qstr) << gui_node_->received_data_[i].target_pos;
            ui->line_target_velocity_m2->setText(qstr);
          /*  ui->line_target_velocity_m1->setStyleSheet("QLabel{background:white;"
                                                       "color:black;"
                                                       "font:bold 75 12pt \"Noto Sans\";}");*/
            qstr.clear();

            QTextStream(&qstr) << gui_node_->received_data_[i].control_word;
            ui->line_control_word_m2->setText(qstr);
            qstr.clear();

            QTextStream(&qstr) << gui_node_->received_data_[i].actual_pos;
            ui->line_actual_velocity_m2->setText(qstr);
            qstr.clear();
            qstr = GetReadableStatusWord(i);
            qstr.clear();
            break;
        case 2:
            QTextStream(&qstr) << gui_node_->received_data_[i].target_pos;
            ui->line_target_velocity_m3->setText(qstr);
          /*  ui->line_target_velocity_m1->setStyleSheet("QLabel{background:white;"
                                                       "color:black;"
                                                       "font:bold 75 12pt \"Noto Sans\";}");*/
            qstr.clear();

            QTextStream(&qstr) << gui_node_->received_data_[i].control_word;
            ui->line_control_word_m3->setText(qstr);
            qstr.clear();

            QTextStream(&qstr) << gui_node_->received_data_[i].actual_pos;
            ui->line_actual_velocity_m3->setText(qstr);
            qstr.clear();
            qstr = GetReadableStatusWord(i);
            qstr.clear();
            break;
        default:
            ui->line_actual_velocity_m3->setText("0");
            ui->line_status_word_m3->setText("NOT READY");
            ui->line_control_word_m3->setText("0");
            ui->line_target_velocity_m3->setText("0");

            ui->line_actual_velocity_m2->setText("0");
            ui->line_status_word_m2->setText("NOT READY");
            ui->line_control_word_m2->setText("0");
            ui->line_target_velocity_m2->setText("0");

            ui->line_actual_velocity_m1->setText("0");
            ui->line_status_word_m1->setText("NOT READY");
            ui->line_control_word_m1->setText("0");
            ui->line_target_velocity_m1->setText("0");
            break;
        }

    }
}

QString MainWindow::GetReadableStatusWord(int index)
{
    QString qstr;

    if (gui_node_->received_data_[index].status_word==4663 || gui_node_->received_data_[index].status_word==567){
        QTextStream(&qstr) << "READY";
        switch (index) {
        case 0:
            ui->line_status_word_m1->setText(qstr);
            ui->line_status_word_m1->setStyleSheet("QLabel{background:green;"
                                                   "color:black;"
                                                   "font:bold 75 12pt \"Noto Sans\";}");
            break;
        case 1:
            ui->line_status_word_m2->setText(qstr);
            ui->line_status_word_m2->setStyleSheet("QLabel{background:green;"
                                                   "color:black;"
                                                   "font:bold 75 12pt \"Noto Sans\";}");
            break;
        case 2:
            ui->line_status_word_m3->setText(qstr);
            ui->line_status_word_m3->setStyleSheet("QLabel{background:green;"
                                                   "color:black;"
                                                   "font:bold 75 12pt \"Noto Sans\";}");
            break;
        default:
            break;
        }
    } else if(TEST_BIT(gui_node_->received_data_[index].status_word,10)){
        QTextStream(&qstr) << "ON TARGET";
        switch (index) {
        case 0:
            ui->line_status_word_m1->setText(qstr);
            ui->line_status_word_m1->setStyleSheet("QLabel{background:yellow;"
                                                   "color:black;"
                                                   "font:bold 75 12pt \"Noto Sans\";}");
            break;
        case 1:
            ui->line_status_word_m2->setText(qstr);
            ui->line_status_word_m2->setStyleSheet("QLabel{background:yellow;"
                                                   "color:black;"
                                                   "font:bold 75 12pt \"Noto Sans\";}");
            break;
        case 2:
            ui->line_status_word_m3->setText(qstr);
            ui->line_status_word_m3->setStyleSheet("QLabel{background:yellow;"
                                                   "color:black;"
                                                   "font:bold 75 12pt \"Noto Sans\";}");
            break;
        default:
            break;
        }
    }else{
        QTextStream(&qstr) << "MOVING";
        switch (index) {
        case 0:
            ui->line_status_word_m1->setText(qstr);
            ui->line_status_word_m1->setStyleSheet("QLabel{background:red;"
                                                   "color:black;"
                                                   "font:bold 75 12pt \"Noto Sans\";}");
            break;
        case 1:
            ui->line_status_word_m2->setText(qstr);
            ui->line_status_word_m2->setStyleSheet("QLabel{background:red;"
                                                   "color:black;"
                                                   "font:bold 75 12pt \"Noto Sans\";}");
            break;
        case 2:
            ui->line_status_word_m3->setText(qstr);
            ui->line_status_word_m3->setStyleSheet("QLabel{background:red;"
                                                   "color:black;"
                                                   "font:bold 75 12pt \"Noto Sans\";}");
            break;
        default:
            break;
        }
    }
    return qstr;
}

void MainWindow::on_button_reset_clicked()
{
    for (int i = 0 ; i < NUM_OF_SERVO_DRIVES ; i++ ){
        gui_node_->received_data_[i].status_word = 0 ;
        gui_node_->received_data_[i].actual_pos = 0 ;
        gui_node_->received_data_[i].actual_vel = 0 ;

        gui_node_->received_data_[i].control_word = 0 ;
        gui_node_->received_data_[i].target_pos  = 0 ;
        gui_node_->received_data_[i].target_vel = 0 ;
        gui_node_->received_data_[0].left_limit_switch_val = 0 ;
        gui_node_->received_data_[0].right_limit_switch_val = 0;
        gui_node_->received_data_[0].right_x_axis = 0 ;
        gui_node_->received_data_[0].left_x_axis = 0 ;
    }
    if (gui_node_->received_data_[0].p_emergency_switch_val){
       gui_node_->emergency_button_val_ = 1;
       setEnabledStyleSheet();
    }
}

void MainWindow::setDisabledStyleSheet()
{
    ui->button_emergency->setEnabled(false);
    ui->button_emergency->setStyleSheet("color: rgb(255, 255, 255);"
                                         "background-color: rgb(0, 0,0);"
                                         "font: bold 75 15pt;");
}

void MainWindow::setEnabledStyleSheet()
{
    ui->button_emergency->setEnabled(true);
    ui->button_emergency->setStyleSheet("color: rgb(255, 255, 255);"
                                        "background-color: rgb(255, 0,0);"
                                        "font: bold 75 15pt;");
}

void MainWindow::on_button_emergency_clicked()
{
    gui_node_->emergency_button_val_ = 0;
    setDisabledStyleSheet();
}
