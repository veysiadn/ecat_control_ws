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

// CPP
#include <chrono>
#include <memory>
#include <iostream>

// ROS2
#include "rclcpp/rclcpp.hpp"

// GUI_Node Headers
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
namespace Ui
{
class MainWindow;
}
class MainWindow : public QMainWindow
{
  Q_OBJECT
public:
  MainWindow(int argc, char** argv, QWidget* parent = nullptr);
  ~MainWindow();

private:
  void DisableOtherModes(QPushButton* button, int index);
  void EnableAllModes(int index);

private slots:

  void b_send_clicked(int m_no);
  void b_stop_clicked(int m_no);
  void b_enable_clicked(int m_no);
  void b_disable_clicked(int m_no);
  void b_vel_clicked(int m_no);
  void b_cyclic_vel_clicked(int m_no);
  void b_pos_clicked(int m_no);
  void b_cyclic_pos_clicked(int m_no);
  void b_tor_clicked(int m_no);
  void b_cyclic_tor_clicked(int m_no);

private slots:

  void UpdateGUI();

  void ResetControlButtonValues(unsigned char& button_val);

  void on_b_enable_cyclic_pos_clicked();

  void on_b_enable_cyclic_vel_clicked();

  void on_b_enable_vel_clicked();

  void on_b_enable_pos_clicked();

  void on_b_init_ecat_clicked();

  void on_b_reinit_ecat_clicked();

  void on_b_enable_drives_clicked();

  void on_b_disable_drives_clicked();

  void on_b_enter_cyclic_pdo_clicked();

  void on_b_stop_cyclic_pdo_clicked();

  void on_b_emergency_mode_clicked();

private:
  /**
   * @brief To use ROS2 spinining functionality in our specific thread.
   *
   */
  void rosSpinThread();
  /**
   * @brief Shows motor status in GUI.
   */
  void ShowAllMotorStatus();
  /**
   * @brief Shows communication status in GUI.
   */
  void ShowComStatus();
  /**
   * @brief Shows emergency button and switch status in GUI.
   */
  void ShowEmergencyStatus();
  /**
   * @brief Reads status word and turns it to readable string format.
   * @param index slave index
   * @return QString readable status word format.
   */
  QString GetReadableStatusWord(int index);
  /**
   * @brief This function will update UI style (enable-disable buttons) based on unconfigured state style
   * @note In unconfigured state only Initialize Ethercat button will be enabled.
   * @note That this state naming comes from lifecycle node in ROS2.
   */
  void CallUnconfiguredStateUI();

  /**
   * @brief This function will update UI style (enable-disable buttons) based on inactive state style.
   * @note  In this state state only Initialize/Reinitialize Ethercat buttons and STOP Cyclic Exchange button will
   * be disabled other will be enabled.
   */
  void CallInactiveStateUI();
  /**
   * @brief This function will update UI style (enable-disable buttons) based on active state style.
   * @note In this state only Emergency Mode and Stop Cyclic Exchange buttons will be enabled.
   */
  void CallActiveStateUI();

  /**
   * @brief Sets GUI appearance for disabled buttons.
   */
  void SetDisabledStyleSheet(QPushButton* button);
  /**
   * @brief Sets GUI appearance for enabled buttons.
   */
  void SetEnabledStyleSheetSDO(QPushButton* button);

  void SetEnabledStyleSheetPDO(QPushButton* button);

  void ShowOperationMode();

  QString GetDriveErrorMessage(const int& err_code);
  Ui::MainWindow* ui;
  int argc_;
  char** argv_;
  /**
   * @brief timer to update GUI in specific intervals.In this case 25ms.
   *
   */
  int GetDriveStates(const int& statusWord);
  const QString blue_style_sheet_ =
      "QPushButton:pressed {background-color: rgb(5, 153, 44);}"
      "QPushButton {color: rgb(255, 255, 255);"
      "selection-background-color: rgb(238, 238, 236);"
      "selection-color: rgb(238, 238, 236);"
      "background-color: rgb(19, 61, 128);"
      "alternate-background-color: rgb(0, 0, 0);"
      "font: bold 75 16pt \"Noto Sans\";}";
  const QString sdo_style_sheet_ =
      "QPushButton:pressed {background-color: rgb(5, 153, 44);}"
      "QPushButton {color: rgb(255, 255, 255);"
      "background-color: rgb(252, 119, 3);"
      "font: bold 75 15pt \"Noto Sans\";}";
  const QString red_style_sheet =
      "QPushButton:pressed {background-color: rgb(252, 186, 3);}"
      "QPushButton {color: rgb(255, 255, 255);"
      "background-color: rgb(252, 0, 0);"
      "font: bold 75 15pt \"Noto Sans\";}";
  QTimer my_timer;
  // To get data from gui_node_ .
  std::shared_ptr<GuiNode> gui_node_;
  VideoCapture* opencv_video_cap;
  // Thread for ROS2 spinning.
  std::thread ros_spin_thread_;
  uint8_t em_state_ = 0;
};
