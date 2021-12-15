/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2021 Veysi ADIN, UST KIST
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS2 environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: veysi.adin@kist.re.kr
 *****************************************************************************/
/*****************************************************************************
 * \file  main_window.hpp
 * \brief MainWindow implementation to show slave status and controller commands
 *        and endoscope camera view in GUI in ROS2.
 *        All GUI functionality and updating GUI handled by this MainWindow class.
 *******************************************************************************/
#pragma once

//CPP
#include <chrono>
#include <memory>
#include <iostream>

//ROS2
#include "rclcpp/rclcpp.hpp"

//GUI_Node Headers
#include "gui_node.hpp"
#include "video_capture.hpp"
#include "gui_globals.hpp"

// QT
#include "ui_mainwindow.h"
#include <QMainWindow>
#include <QApplication>
#include <QStandardItemModel>
#include <QTableView>
#include <QHeaderView>
#include <QString>
#include <QTextStream>
#include <QTimer>


using namespace GUI;
namespace Ui {
class MainWindow;
}
  class MainWindow : public QMainWindow {
  Q_OBJECT
  public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();

  private slots:
//  /**
//   * @brief Resets all received values to 0.
//   *
//   */
//    void on_button_reset_clicked();
//    /**
//     * @brief Updates GUI based on timer callback in this case 25ms.
//     *
//     */
    void UpdateGUI();
    /**
     * @brief Sets GUI appearance for disabled buttons.
     */
    void setDisabledStyleSheet(QPushButton *button);
    /**
     * @brief Sets GUI appearance for enabled buttons.
     */
    void setEnabledStyleSheet(QPushButton *button);


    void ResetControlButtonValues(unsigned char &button_val);
//    /**
//     * @brief Stops motor movement while maintining EtherCAT communication in case of emergency.
//     */
//    void on_button_emergency_clicked();

    void on_b_enable_cyclic_pos_clicked();

    void on_b_enable_cylic_vel_clicked();

    void on_b_enable_vel_clicked();

    void on_b_enable_pos_clicked();

    void on_b_init_ecat_clicked();

    void on_b_reinit_ecat_clicked();

    void on_b_enable_drives_clicked();

    void on_b_disable_drives_clicked();

    void on_b_enter_cyclic_pdo_clicked();

    void on_b_stop_cyclic_pdo_clicked();

    void on_b_emergency_mode_clicked();

    void on_b_send_clicked();

  private:
    /**
     * @brief To use ROS2 spinining functionality in our specific thread.
     *
     */
    void rosSpinThread();
    /**
     * @brief Shows motor status in GUI.
//     */
//    void ShowAllMotorStatus();
//    /**
//     * @brief Shows communication status in GUI.
//     */
//    void ShowComStatus();
//    /**
//     * @brief Shows emergency button and switch status in GUI.
//     */
//    void ShowEmergencyStatus();
//    /**
//     * @brief Reads status word and turns it to readable string format.
//     * @param index slave index
//     * @return QString readable status word format.
//     */
//    QString GetReadableStatusWord(int index);

    Ui::MainWindow *ui;
    int argc_;
    char** argv_;
    /**
     * @brief timer to update GUI in specific intervals.In this case 25ms.
     * 
     */
    int GetDriveStates(const int & statusWord);
    QTimer my_timer;
    // To get data from gui_node_ .
    std::shared_ptr<GuiNode> gui_node_;
    VideoCapture* opencv_video_cap;
    // Thread for ROS2 spinning.
    std::thread ros_spin_thread_;
  };
