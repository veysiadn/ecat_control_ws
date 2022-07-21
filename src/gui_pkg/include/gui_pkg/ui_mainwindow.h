#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include "ecat_globals.hpp"
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <QMainWindow>

QT_BEGIN_NAMESPACE

class MyButton : public QPushButton
{
  Q_OBJECT

public:
  MyButton(QWidget* parent = nullptr, int no = 0)
  {
    connect(this, SIGNAL(clicked()), this, SLOT(handleClick()));
    button_no = no;
  }

signals:
  void buttonClicked(int);

private slots:
  void handleClick()
  {
    emit buttonClicked(button_no);
  }

public:
  int button_no;
};

class Ui_MainWindow
{
public:
  QWidget* centralwidget;
  QGridLayout* gridLayout;
  QHBoxLayout* bottom_horizontal_layout;
  QVBoxLayout* left_sdo_button_layout;
  MyButton* b_enable_cyclic_pos;
  MyButton* b_enable_torque;
  MyButton* b_clear_fault;
  MyButton* b_enable_cyclic_torque;
  MyButton* b_enable_cyclic_vel;
  MyButton* b_enable_vel;
  MyButton* b_enable_pos;
  QVBoxLayout* right_verticalLayout;
  QHBoxLayout* top_horizontal_layout;
  MyButton* b_init_ecat;
  MyButton* b_reinit_ecat;
  MyButton* b_enable_drives;
  MyButton* b_disable_drives;
  QHBoxLayout* up_horizontalLayout;
  MyButton* b_enter_cyclic_pdo;
  MyButton* b_stop_cyclic_pdo;
  MyButton* b_emergency_mode;
  QVBoxLayout* send_verticalLayout;
  QGridLayout* singleButtonGridLayout;
  QVBoxLayout* target_vertical_layout;
  QHBoxLayout* send_spin_horizontal_layout;
  QLabel* ls_enter_target_vals;

  QSpinBox* spn_target_val_[g_kNumberOfServoDrivers];
  MyButton* b_send[g_kNumberOfServoDrivers];

  MyButton* b_stop[g_kNumberOfServoDrivers];
  MyButton* b_enable[g_kNumberOfServoDrivers];
  MyButton* b_disable[g_kNumberOfServoDrivers];
  MyButton* b_vel[g_kNumberOfServoDrivers];
  MyButton* b_cyclic_vel[g_kNumberOfServoDrivers];
  MyButton* b_pos[g_kNumberOfServoDrivers];
  MyButton* b_cyclic_pos[g_kNumberOfServoDrivers];
  MyButton* b_tor[g_kNumberOfServoDrivers];
  MyButton* b_cyclic_tor[g_kNumberOfServoDrivers];

  QLabel* ls_status_bar;
  QGridLayout* status_bar_gridLayout;

  QLabel* lb_target_pos[g_kNumberOfServoDrivers];
  QLabel* lb_actual_pos[g_kNumberOfServoDrivers];
  QLabel* lb_status_word[g_kNumberOfServoDrivers];
  QLabel* lb_control_word[g_kNumberOfServoDrivers];
  QLabel* lb_actual_vel[g_kNumberOfServoDrivers];
  QLabel* lb_target_vel[g_kNumberOfServoDrivers];
  QLabel* lb_actual_tor[g_kNumberOfServoDrivers];
  QLabel* lb_target_tor[g_kNumberOfServoDrivers];
  QLabel* lb_op_mode[g_kNumberOfServoDrivers];
  QLabel* label_motor[g_kNumberOfServoDrivers];

  QLabel* ls_op_mode;
  QLabel* ls_tar_vel;
  QLabel* lb_com_status;
  QLabel* ls_tar_pos;
  QLabel* ls_control_word;
  QLabel* ls_target_tor;
  QLabel* lb_safety_state;
  QLabel* lb_motor_error_code;
  QLabel* ls_motor_error_code;
  QLabel* ls_ecat_status_bar;
  QLabel* ls_safety_state;
  QLabel* ls_com_status;
  QLabel* label_motor_no;

  void setupUi(QMainWindow* MainWindow)
  {
    if (MainWindow->objectName().isEmpty())
      MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
    centralwidget = new QWidget(MainWindow);
    centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
    gridLayout = new QGridLayout(centralwidget);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    singleButtonGridLayout = new QGridLayout(centralwidget);
    singleButtonGridLayout->setObjectName(QString::fromUtf8("singleButtonGridLayout"));

    bottom_horizontal_layout = new QHBoxLayout();
    bottom_horizontal_layout->setObjectName(QString::fromUtf8("bottom_horizontal_layout"));
    left_sdo_button_layout = new QVBoxLayout();
    left_sdo_button_layout->setObjectName(QString::fromUtf8("left_sdo_button_layout"));

    QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    b_enable_vel = new MyButton(centralwidget);
    b_enable_vel->setObjectName(QString::fromUtf8("b_enable_vel"));
    sizePolicy.setHeightForWidth(b_enable_vel->sizePolicy().hasHeightForWidth());
    b_enable_vel->setSizePolicy(sizePolicy);
    b_enable_vel->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "background-color: rgb(252, 119, 3);\n"
                          "font: bold 75 15pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    left_sdo_button_layout->addWidget(b_enable_vel);

    b_enable_cyclic_vel = new MyButton(centralwidget);
    b_enable_cyclic_vel->setObjectName(QString::fromUtf8("b_enable_cyclic_vel"));
    sizePolicy.setHeightForWidth(b_enable_cyclic_vel->sizePolicy().hasHeightForWidth());
    b_enable_cyclic_vel->setSizePolicy(sizePolicy);
    b_enable_cyclic_vel->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "background-color: rgb(252, 119, 3);\n"
                          "font: bold 75 15pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    left_sdo_button_layout->addWidget(b_enable_cyclic_vel);

    b_enable_pos = new MyButton(centralwidget);
    b_enable_pos->setObjectName(QString::fromUtf8("b_enable_pos"));
    sizePolicy.setHeightForWidth(b_enable_pos->sizePolicy().hasHeightForWidth());
    b_enable_pos->setSizePolicy(sizePolicy);
    b_enable_pos->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "background-color: rgb(252, 119, 3);\n"
                          "font: bold 75 15pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    left_sdo_button_layout->addWidget(b_enable_pos);

    b_enable_cyclic_pos = new MyButton(centralwidget);
    b_enable_cyclic_pos->setObjectName(QString::fromUtf8("b_enable_cyclic_pos"));
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(b_enable_cyclic_pos->sizePolicy().hasHeightForWidth());
    b_enable_cyclic_pos->setSizePolicy(sizePolicy);
    b_enable_cyclic_pos->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "background-color: rgb(252, 119, 3);\n"
                          "font: bold 75 15pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    left_sdo_button_layout->addWidget(b_enable_cyclic_pos);

    b_enable_torque = new MyButton(centralwidget);
    b_enable_torque->setObjectName(QString::fromUtf8("b_enable_torque"));
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(b_enable_cyclic_pos->sizePolicy().hasHeightForWidth());
    b_enable_torque->setSizePolicy(sizePolicy);
    b_enable_torque->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "background-color: rgb(252, 119, 3);\n"
                          "font: bold 75 15pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    left_sdo_button_layout->addWidget(b_enable_torque);

    b_enable_cyclic_torque = new MyButton(centralwidget);
    b_enable_cyclic_torque->setObjectName(QString::fromUtf8("b_enable_cyclic_torque"));
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(b_enable_cyclic_pos->sizePolicy().hasHeightForWidth());
    b_enable_cyclic_torque->setSizePolicy(sizePolicy);
    b_enable_cyclic_torque->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "background-color: rgb(252, 119, 3);\n"
                          "font: bold 75 15pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    left_sdo_button_layout->addWidget(b_enable_cyclic_torque);

    b_clear_fault = new MyButton(centralwidget);
    b_clear_fault->setObjectName(QString::fromUtf8("b_clear_fault"));
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(b_enable_cyclic_pos->sizePolicy().hasHeightForWidth());
    b_clear_fault->setSizePolicy(sizePolicy);
    b_clear_fault->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "background-color: rgb(252, 119, 3);\n"
                          "font: bold 75 15pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    left_sdo_button_layout->addWidget(b_clear_fault);
    bottom_horizontal_layout->addLayout(left_sdo_button_layout);

    right_verticalLayout = new QVBoxLayout();
    right_verticalLayout->setObjectName(QString::fromUtf8("right_verticalLayout"));
    top_horizontal_layout = new QHBoxLayout();
    top_horizontal_layout->setObjectName(QString::fromUtf8("top_horizontal_layout"));
    b_init_ecat = new MyButton(centralwidget);
    b_init_ecat->setObjectName(QString::fromUtf8("b_init_ecat"));
    sizePolicy.setHeightForWidth(b_init_ecat->sizePolicy().hasHeightForWidth());
    b_init_ecat->setSizePolicy(sizePolicy);
    b_init_ecat->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color: rgb(19, 61, 128);\n"
                          "font: bold 75 16pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    top_horizontal_layout->addWidget(b_init_ecat);

    b_reinit_ecat = new MyButton(centralwidget);
    b_reinit_ecat->setObjectName(QString::fromUtf8("b_reinit_ecat"));
    sizePolicy.setHeightForWidth(b_reinit_ecat->sizePolicy().hasHeightForWidth());
    b_reinit_ecat->setSizePolicy(sizePolicy);
    b_reinit_ecat->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color: rgb(19, 61, 128);\n"
                          "font: bold 75 16pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    top_horizontal_layout->addWidget(b_reinit_ecat);

    b_enable_drives = new MyButton(centralwidget);
    b_enable_drives->setObjectName(QString::fromUtf8("b_enable_drives"));
    sizePolicy.setHeightForWidth(b_enable_drives->sizePolicy().hasHeightForWidth());
    b_enable_drives->setSizePolicy(sizePolicy);
    b_enable_drives->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "background-color: rgb(252, 119, 3);\n"
                          "font: bold 75 15pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    top_horizontal_layout->addWidget(b_enable_drives);

    b_disable_drives = new MyButton(centralwidget);
    b_disable_drives->setObjectName(QString::fromUtf8("b_disable_drives"));
    sizePolicy.setHeightForWidth(b_disable_drives->sizePolicy().hasHeightForWidth());
    b_disable_drives->setSizePolicy(sizePolicy);
    b_disable_drives->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "background-color: rgb(252, 119, 3);\n"
                          "font: bold 75 15pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    top_horizontal_layout->addWidget(b_disable_drives);

    right_verticalLayout->addLayout(top_horizontal_layout);

    up_horizontalLayout = new QHBoxLayout();
    up_horizontalLayout->setObjectName(QString::fromUtf8("up_horizontalLayout"));
    up_horizontalLayout->setSizeConstraint(QLayout::SetFixedSize);
    b_enter_cyclic_pdo = new MyButton(centralwidget);
    b_enter_cyclic_pdo->setObjectName(QString::fromUtf8("b_enter_cyclic_pdo"));
    sizePolicy.setHeightForWidth(b_enter_cyclic_pdo->sizePolicy().hasHeightForWidth());
    b_enter_cyclic_pdo->setSizePolicy(sizePolicy);
    b_enter_cyclic_pdo->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color: rgb(19, 61, 128);\n"
                          "font: bold 75 16pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    up_horizontalLayout->addWidget(b_enter_cyclic_pdo);

    b_stop_cyclic_pdo = new MyButton(centralwidget);
    b_stop_cyclic_pdo->setObjectName(QString::fromUtf8("b_stop_cyclic_pdo"));
    sizePolicy.setHeightForWidth(b_stop_cyclic_pdo->sizePolicy().hasHeightForWidth());
    b_stop_cyclic_pdo->setSizePolicy(sizePolicy);
    b_stop_cyclic_pdo->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "    background-color: rgb(5, 153, 44);\n"
                          "}\n"
                          "MyButton:disabled {\n"
                          "color: rgb(33, 33, 33);\n"
                          "background-color:gray;\n"
                          "font: bold 75 15pt;\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color: rgb(19, 61, 128);\n"
                          "font: bold 75 16pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    up_horizontalLayout->addWidget(b_stop_cyclic_pdo);

    b_emergency_mode = new MyButton(centralwidget);
    b_emergency_mode->setObjectName(QString::fromUtf8("b_emergency_mode"));
    QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Preferred);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(b_emergency_mode->sizePolicy().hasHeightForWidth());
    b_emergency_mode->setSizePolicy(sizePolicy1);
    b_emergency_mode->setMinimumSize(QSize(0, 0));
    b_emergency_mode->setStyleSheet(
        QString::fromUtf8("MyButton:pressed {\n"
                          "background-color: rgb(193, 5, 10);\n"
                          "}\n"
                          "MyButton { \n"
                          "color: rgb(255, 255, 255);\n"
                          "background-color: rgb(252, 0, 0);\n"
                          "font: bold 75 15pt \"Noto Sans\";\n"
                          "}\n"
                          ""));

    up_horizontalLayout->addWidget(b_emergency_mode);

    right_verticalLayout->addLayout(up_horizontalLayout);

    send_verticalLayout = new QVBoxLayout();
    send_verticalLayout->setObjectName(QString::fromUtf8("send_horizontalLayout"));
    send_spin_horizontal_layout = new QHBoxLayout();
    send_spin_horizontal_layout->setObjectName(QString::fromUtf8("send_spin_horizontal_layout"));
    target_vertical_layout = new QVBoxLayout();
    target_vertical_layout->setObjectName(QString::fromUtf8("target_vertical_layout"));
    ls_enter_target_vals = new QLabel(centralwidget);
    ls_enter_target_vals->setObjectName(QString::fromUtf8("lb_enter_target_vals"));
    ls_enter_target_vals->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "background-color:  none;\n"
                          "font: bold 75 14pt \"Noto Sans\";"));
    QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Fixed);
    sizePolicy2.setHorizontalStretch(0);
    sizePolicy2.setVerticalStretch(0);
    sizePolicy2.setHeightForWidth(ls_enter_target_vals->sizePolicy().hasHeightForWidth());
    ls_enter_target_vals->setSizePolicy(sizePolicy2);

    right_verticalLayout->addWidget(ls_enter_target_vals);
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      spn_target_val_[i] = new QSpinBox(centralwidget);
      spn_target_val_[i]->setObjectName(QString::fromUtf8("spn_target_val_1" + i));
      QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Fixed);
      sizePolicy2.setHorizontalStretch(0);
      sizePolicy2.setVerticalStretch(0);
      sizePolicy2.setHeightForWidth(spn_target_val_[i]->sizePolicy().hasHeightForWidth());
      spn_target_val_[i]->setSizePolicy(sizePolicy2);
      spn_target_val_[i]->setMinimum(-10000);
      spn_target_val_[i]->setMaximum(10000);
      spn_target_val_[i]->setStepType(QAbstractSpinBox::AdaptiveDecimalStepType);

      target_vertical_layout->addWidget(spn_target_val_[i]);
    }

    send_spin_horizontal_layout->addLayout(target_vertical_layout);

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      b_send[i] = new MyButton(centralwidget, i);
      b_send[i]->setObjectName(QString::fromUtf8("b_send_") + QString::number(i));
      QSizePolicy sizePolicy3(QSizePolicy::Maximum, QSizePolicy::Minimum);
      sizePolicy3.setHorizontalStretch(0);
      sizePolicy3.setVerticalStretch(0);
      sizePolicy3.setHeightForWidth(b_send[i]->sizePolicy().hasHeightForWidth());
      b_send[i]->setSizePolicy(sizePolicy3);
      b_send[i]->setStyleSheet(
          QString::fromUtf8("MyButton:pressed {\n"
                            "    background-color: rgb(5, 153, 44);\n"
                            "}\n"
                            "MyButton:disabled {\n"
                            "color: rgb(33, 33, 33);\n"
                            "background-color:gray;\n"
                            "font: bold 75 15pt;\n"
                            "}\n"
                            "MyButton { \n"
                            "color: rgb(255, 255, 255);\n"
                            "background-color: rgb(252, 119, 3);\n"
                            "font: bold 75 15pt \"Noto Sans\";\n"
                            "}\n"
                            ""));
      send_verticalLayout->addWidget(b_send[i]);
    }
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      b_stop[i] = new MyButton(centralwidget, i);
      b_stop[i]->setObjectName(QString::fromUtf8("b_stop_") + QString::number(i));
      QSizePolicy sizePolicy3(QSizePolicy::Maximum, QSizePolicy::Minimum);
      sizePolicy3.setHorizontalStretch(0);
      sizePolicy3.setVerticalStretch(0);
      sizePolicy3.setHeightForWidth(b_stop[i]->sizePolicy().hasHeightForWidth());
      b_stop[i]->setSizePolicy(sizePolicy3);
      b_stop[i]->setStyleSheet(
          QString::fromUtf8("MyButton:pressed {\n"
                            "background-color: rgb(193, 5, 10);\n"
                            "}\n"
                            "MyButton:disabled {\n"
                            "color: rgb(33, 33, 33);\n"
                            "background-color:gray;\n"
                            "font: bold 75 15pt;\n"
                            "}\n"
                            "MyButton { \n"
                            "color: rgb(255, 255, 255);\n"
                            "background-color: rgb(252, 0, 0);\n"
                            "font: bold 75 15pt \"Noto Sans\";\n"
                            "}\n"
                            ""));

      singleButtonGridLayout->addWidget(b_stop[i], i, 0, 1, 1);
    }

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      b_enable[i] = new MyButton(centralwidget, i);
      b_enable[i]->setObjectName(QString::fromUtf8("b_enable_") + QString::number(i));
      QSizePolicy sizePolicy3(QSizePolicy::Maximum, QSizePolicy::Minimum);
      sizePolicy3.setHorizontalStretch(0);
      sizePolicy3.setVerticalStretch(0);
      sizePolicy3.setHeightForWidth(b_enable[i]->sizePolicy().hasHeightForWidth());
      b_enable[i]->setSizePolicy(sizePolicy3);
      b_enable[i]->setStyleSheet(
          QString::fromUtf8("MyButton:pressed {\n"
                            "    background-color: rgb(5, 153, 44);\n"
                            "}\n"
                            "MyButton:disabled {\n"
                            "color: rgb(33, 33, 33);\n"
                            "background-color:gray;\n"
                            "font: bold 75 15pt;\n"
                            "}\n"
                            "MyButton:checked {\n"
                            "    background-color: rgb(5, 153, 44);\n"
                            "}\n"
                            "MyButton { \n"
                            "color: rgb(255, 255, 255);\n"
                            "background-color: rgb(43, 143, 62);\n"
                            "font: bold 75 15pt \"Noto Sans\";\n"
                            "}\n"
                            ""));
      singleButtonGridLayout->addWidget(b_enable[i], i, 1, 1, 1);
    }
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      b_disable[i] = new MyButton(centralwidget, i);
      b_disable[i]->setObjectName(QString::fromUtf8("b_disable_") + QString::number(i));
      QSizePolicy sizePolicy3(QSizePolicy::Maximum, QSizePolicy::Minimum);
      sizePolicy3.setHorizontalStretch(0);
      sizePolicy3.setVerticalStretch(0);
      sizePolicy3.setHeightForWidth(b_disable[i]->sizePolicy().hasHeightForWidth());
      b_disable[i]->setSizePolicy(sizePolicy3);
      b_disable[i]->setStyleSheet(
          QString::fromUtf8("MyButton:pressed {\n"
                            "    background-color: rgb(118, 126, 130);\n"
                            "}\n"
                            "MyButton:checked {\n"
                            "    background-color: rgb(5, 153, 44);\n"
                            "}\n"
                            "MyButton:disabled {\n"
                            "color: rgb(33, 33, 33);\n"
                            "background-color:gray;\n"
                            "font: bold 75 15pt;\n"
                            "}\n"
                            "MyButton { \n"
                            "color: rgb(255, 255, 255);\n"
                            "background-color: rgb(10, 60, 87);\n"
                            "font: bold 75 15pt \"Noto Sans\";\n"
                            "}\n"
                            ""));
      singleButtonGridLayout->addWidget(b_disable[i], i, 2, 1, 1);
    }

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      b_vel[i] = new MyButton(centralwidget, i);
      b_vel[i]->setObjectName(QString::fromUtf8("b_vel_") + QString::number(i));
      QSizePolicy sizePolicy3(QSizePolicy::Maximum, QSizePolicy::Minimum);
      sizePolicy3.setHorizontalStretch(0);
      sizePolicy3.setVerticalStretch(0);
      sizePolicy3.setHeightForWidth(b_vel[i]->sizePolicy().hasHeightForWidth());
      b_vel[i]->setSizePolicy(sizePolicy3);
      b_vel[i]->setStyleSheet(
          QString::fromUtf8("MyButton:pressed {\n"
                            "    background-color: rgb(5, 153, 44);\n"
                            "}\n"
                            "MyButton:checked {\n"
                            "    background-color: rgb(5, 153, 44);\n"
                            "}\n"
                            "MyButton { \n"
                            //                                                       "color: rgb(255, 255, 255);\n"
                            "font: bold 75 15pt \"Noto Sans\";\n"
                            "}\n"
                            ""));
      b_vel[i]->setCheckable(true);
      singleButtonGridLayout->addWidget(b_vel[i], i, 3, 1, 1);
    }
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      b_cyclic_vel[i] = new MyButton(centralwidget, i);
      b_cyclic_vel[i]->setObjectName(QString::fromUtf8("b_cyclic_vel_") + QString::number(i));
      QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Minimum);
      sizePolicy3.setHorizontalStretch(0);
      sizePolicy3.setVerticalStretch(0);
      sizePolicy3.setHeightForWidth(b_cyclic_vel[i]->sizePolicy().hasHeightForWidth());
      b_cyclic_vel[i]->setSizePolicy(sizePolicy3);
      b_cyclic_vel[i]->setStyleSheet(QString::fromUtf8(
          "MyButton:pressed {\n"
          "    background-color: rgb(5, 153, 44);\n"
          "}\n"
          "MyButton:checked {\n"
          "    background-color: rgb(5, 153, 44);\n"
          "}\n"
          "MyButton { \n"
          //                                                       "color: rgb(255, 255, 255);\n"
          //                                                       "background-color: rgb(252, 119, 3);\n"
          "font: bold 75 15pt \"Noto Sans\";\n"
          "}\n"
          ""));
      b_cyclic_vel[i]->setCheckable(true);
      singleButtonGridLayout->addWidget(b_cyclic_vel[i], i, 4, 1, 1);
    }

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      b_pos[i] = new MyButton(centralwidget, i);
      b_pos[i]->setObjectName(QString::fromUtf8("b_pos_") + QString::number(i));
      QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Minimum);
      sizePolicy3.setHorizontalStretch(0);
      sizePolicy3.setVerticalStretch(0);
      sizePolicy3.setHeightForWidth(b_pos[i]->sizePolicy().hasHeightForWidth());
      b_pos[i]->setSizePolicy(sizePolicy3);
      b_pos[i]->setStyleSheet(QString::fromUtf8(
          "MyButton:pressed {\n"
          "    background-color: rgb(5, 153, 44);\n"
          "}\n"
          "MyButton:checked {\n"
          "    background-color: rgb(5, 153, 44);\n"
          "}\n"
          "MyButton { \n"
          //                                                       "color: rgb(255, 255, 255);\n"
          //                                                       "background-color: rgb(252, 119, 3);\n"
          "font: bold 75 15pt \"Noto Sans\";\n"
          "}\n"
          ""));
      b_pos[i]->setCheckable(true);
      singleButtonGridLayout->addWidget(b_pos[i], i, 5, 1, 1);
    }
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      b_cyclic_pos[i] = new MyButton(centralwidget, i);
      b_cyclic_pos[i]->setObjectName(QString::fromUtf8("b_cyclic_pos_") + QString::number(i));
      QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Minimum);
      sizePolicy3.setHorizontalStretch(0);
      sizePolicy3.setVerticalStretch(0);
      sizePolicy3.setHeightForWidth(b_cyclic_pos[i]->sizePolicy().hasHeightForWidth());
      b_cyclic_pos[i]->setSizePolicy(sizePolicy3);
      b_cyclic_pos[i]->setStyleSheet(QString::fromUtf8(
          "MyButton:pressed {\n"
          "    background-color: rgb(5, 153, 44);\n"
          "}\n"
          "MyButton:checked {\n"
          "    background-color: rgb(5, 153, 44);\n"
          "}\n"
          "MyButton { \n"
          //                                                       "color: rgb(255, 255, 255);\n"
          //                                                       "background-color: rgb(252, 119, 3);\n"
          "font: bold 75 15pt \"Noto Sans\";\n"
          "}\n"
          ""));
      b_cyclic_pos[i]->setCheckable(true);
      singleButtonGridLayout->addWidget(b_cyclic_pos[i], i, 6, 1, 1);
    }

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      b_tor[i] = new MyButton(centralwidget, i);
      b_tor[i]->setObjectName(QString::fromUtf8("b_tor_") + QString::number(i));
      QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Minimum);
      sizePolicy3.setHorizontalStretch(0);
      sizePolicy3.setVerticalStretch(0);
      sizePolicy3.setHeightForWidth(b_tor[i]->sizePolicy().hasHeightForWidth());
      b_tor[i]->setSizePolicy(sizePolicy3);
      b_tor[i]->setStyleSheet(QString::fromUtf8(
          "MyButton:pressed {\n"
          "    background-color: rgb(5, 153, 44);\n"
          "}\n"
          "MyButton:checked {\n"
          "    background-color: rgb(5, 153, 44);\n"
          "}\n"
          "MyButton { \n"
          //                                                       "color: rgb(255, 255, 255);\n"
          //                                                       "background-color: rgb(252, 119, 3);\n"
          "font: bold 75 15pt \"Noto Sans\";\n"
          "}\n"
          ""));
      b_tor[i]->setCheckable(true);
      singleButtonGridLayout->addWidget(b_tor[i], i, 7, 1, 1);
    }

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      b_cyclic_tor[i] = new MyButton(centralwidget, i);
      b_cyclic_tor[i]->setObjectName(QString::fromUtf8("b_cyclic_tor_") + QString::number(i));
      QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Minimum);
      sizePolicy3.setHorizontalStretch(0);
      sizePolicy3.setVerticalStretch(0);
      sizePolicy3.setHeightForWidth(b_cyclic_tor[i]->sizePolicy().hasHeightForWidth());
      b_cyclic_tor[i]->setSizePolicy(sizePolicy3);
      b_cyclic_tor[i]->setStyleSheet(QString::fromUtf8(
          "MyButton:pressed {\n"
          "    background-color: rgb(5, 153, 44);\n"
          "}\n"
          "MyButton:checked {\n"
          "    background-color: rgb(5, 153, 44);\n"
          "}\n"
          "MyButton { \n"
          //                                                       "color: rgb(255, 255, 255);\n"
          //                                                       "background-color: rgb(252, 119, 3);\n"
          "font: bold 75 15pt \"Noto Sans\";\n"
          "}\n"
          ""));
      b_cyclic_tor[i]->setCheckable(true);
      singleButtonGridLayout->addWidget(b_cyclic_tor[i], i, 8, 1, 1);
    }

    send_spin_horizontal_layout->addLayout(send_verticalLayout);
    send_spin_horizontal_layout->addLayout(singleButtonGridLayout);
    send_spin_horizontal_layout->setStretch(1, 0);
    send_spin_horizontal_layout->setStretch(2, 0);
    right_verticalLayout->addLayout(send_spin_horizontal_layout);

    ls_status_bar = new QLabel(centralwidget);
    ls_status_bar->setObjectName(QString::fromUtf8("ls_status_bar"));
    QSizePolicy sizePolicy4(QSizePolicy::Minimum, QSizePolicy::Minimum);
    sizePolicy4.setHorizontalStretch(0);
    sizePolicy4.setVerticalStretch(0);
    sizePolicy4.setHeightForWidth(ls_status_bar->sizePolicy().hasHeightForWidth());
    ls_status_bar->setSizePolicy(sizePolicy4);
    ls_status_bar->setStyleSheet(
        QString::fromUtf8("color: rgb(255, 255, 255);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color: rgb(19, 61, 128);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 16pt \"Noto Sans\";"));
    ls_status_bar->setAlignment(Qt::AlignCenter);

    right_verticalLayout->addWidget(ls_status_bar);
    status_bar_gridLayout = new QGridLayout();
    status_bar_gridLayout->setObjectName(QString::fromUtf8("status_bar_gridLayout"));
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      lb_target_pos[i] = new QLabel(centralwidget);
      lb_target_pos[i]->setObjectName(QString::fromUtf8("lb_target_pos_") + QString::number(i));
      sizePolicy4.setHeightForWidth(lb_target_pos[i]->sizePolicy().hasHeightForWidth());
      lb_target_pos[i]->setSizePolicy(sizePolicy4);
      lb_target_pos[i]->setMinimumSize(QSize(0, 0));
      lb_target_pos[i]->setStyleSheet(
          QString::fromUtf8("color: rgb(0, 0, 0);\n"
                            "selection-background-color: rgb(238, 238, 236);\n"
                            "selection-color: rgb(238, 238, 236);\n"
                            "background-color:  rgb(255, 255, 255);\n"
                            "alternate-background-color: rgb(0, 0, 0);\n"
                            "font: bold 75 14pt \"Noto Sans\";"));
      lb_target_pos[i]->setAlignment(Qt::AlignCenter);
      status_bar_gridLayout->addWidget(lb_target_pos[i], 2, 2 * i + 1, 1, 1);
    }

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      label_motor[i] = new QLabel(centralwidget);
      label_motor[i]->setObjectName(QString::fromUtf8("label_motor") + QString::number(i));
      label_motor[i]->setStyleSheet(
          QString::fromUtf8("color: rgb(0, 0, 0);\n"
                            "selection-background-color: rgb(238, 238, 236);\n"
                            "selection-color: rgb(238, 238, 236);\n"
                            "background-color:  rgb(255, 255, 255);\n"
                            "alternate-background-color: rgb(0, 0, 0);\n"
                            "font: bold 75 14pt \"Noto Sans\";"));
      label_motor[i]->setAlignment(Qt::AlignCenter);

      status_bar_gridLayout->addWidget(label_motor[i], 0, 2 * i + 1, 1, 2);
    }
    ls_op_mode = new QLabel(centralwidget);
    ls_op_mode->setObjectName(QString::fromUtf8("ls_op_mode"));
    QSizePolicy sizePolicy5(QSizePolicy::Minimum, QSizePolicy::Minimum);

    sizePolicy5.setHeightForWidth(ls_op_mode->sizePolicy().hasHeightForWidth());
    ls_op_mode->setSizePolicy(sizePolicy5);
    ls_op_mode->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";\n"
                          "padding-left: 5px;\n"
                          "padding-left: 5px;\n"
                          ""));
    ls_op_mode->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(ls_op_mode, 5, 0, 1, 1);

    ls_tar_vel = new QLabel(centralwidget);
    ls_tar_vel->setObjectName(QString::fromUtf8("ls_tar_vel"));
    sizePolicy.setHeightForWidth(ls_tar_vel->sizePolicy().hasHeightForWidth());
    ls_tar_vel->setSizePolicy(sizePolicy);
    ls_tar_vel->setMinimumSize(QSize(0, 0));
    ls_tar_vel->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";\n"
                          "padding-left: 8px;\n"
                          "padding-left: 8px;\n"
                          ""));
    ls_tar_vel->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(ls_tar_vel, 1, 0, 1, 1);

    lb_com_status = new QLabel(centralwidget);
    lb_com_status->setObjectName(QString::fromUtf8("lb_com_status"));
    QSizePolicy sizePolicy6(QSizePolicy::Minimum, QSizePolicy::Minimum);
    sizePolicy6.setHorizontalStretch(0);
    sizePolicy6.setVerticalStretch(0);
    sizePolicy6.setHeightForWidth(lb_com_status->sizePolicy().hasHeightForWidth());
    lb_com_status->setSizePolicy(sizePolicy6);
    lb_com_status->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";"));
    lb_com_status->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(lb_com_status, 7, 1, 1, 2 * g_kNumberOfServoDrivers);

    ls_com_status = new QLabel(centralwidget);
    ls_com_status->setObjectName(QString::fromUtf8("ls_com_status"));
    sizePolicy6.setHeightForWidth(ls_com_status->sizePolicy().hasHeightForWidth());
    ls_com_status->setSizePolicy(sizePolicy6);
    ls_com_status->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";\n"
                          "padding-left: 5px;\n"
                          "padding-left: 5px;\n"
                          ""));
    ls_com_status->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(ls_com_status, 7, 0, 1, 1);

    lb_motor_error_code = new QLabel(centralwidget);
    lb_motor_error_code->setObjectName(QString::fromUtf8("lb_motor_error_code"));
    sizePolicy.setHeightForWidth(lb_motor_error_code->sizePolicy().hasHeightForWidth());
    lb_motor_error_code->setSizePolicy(sizePolicy);
    lb_motor_error_code->setMinimumSize(QSize(0, 0));
    lb_motor_error_code->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";"));
    lb_motor_error_code->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(lb_motor_error_code, 9, 1, 1, 2 * g_kNumberOfServoDrivers);

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      lb_status_word[i] = new QLabel(centralwidget);
      lb_status_word[i]->setObjectName(QString::fromUtf8("lb_status_word_") + QString::number(i));
      sizePolicy.setHeightForWidth(lb_status_word[i]->sizePolicy().hasHeightForWidth());
      lb_status_word[i]->setSizePolicy(sizePolicy);
      lb_status_word[i]->setMinimumSize(QSize(0, 0));
      lb_status_word[i]->setStyleSheet(
          QString::fromUtf8("color: rgb(0, 0, 0);\n"
                            "selection-background-color: rgb(238, 238, 236);\n"
                            "selection-color: rgb(238, 238, 236);\n"
                            "background-color:  rgb(255, 255, 255);\n"
                            "alternate-background-color: rgb(0, 0, 0);\n"
                            "font: bold 75 14pt \"Noto Sans\";"));
      lb_status_word[i]->setAlignment(Qt::AlignCenter);

      status_bar_gridLayout->addWidget(lb_status_word[i], 4, 2 * i + 2, 1, 1);
    }
    ls_tar_pos = new QLabel(centralwidget);
    ls_tar_pos->setObjectName(QString::fromUtf8("ls_tar_pos"));
    sizePolicy.setHeightForWidth(ls_tar_pos->sizePolicy().hasHeightForWidth());
    ls_tar_pos->setSizePolicy(sizePolicy);
    ls_tar_pos->setMinimumSize(QSize(0, 0));
    ls_tar_pos->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";\n"
                          "padding-left: 5px;\n"
                          "padding-left: 5px;\n"
                          ""));
    ls_tar_pos->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(ls_tar_pos, 2, 0, 1, 1);
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      lb_actual_tor[i] = new QLabel(centralwidget);
      lb_actual_tor[i]->setObjectName(QString::fromUtf8("lb_actual_tor_") + QString::number(i));
      sizePolicy.setHeightForWidth(lb_actual_tor[i]->sizePolicy().hasHeightForWidth());
      lb_actual_tor[i]->setSizePolicy(sizePolicy);
      lb_actual_tor[i]->setMinimumSize(QSize(0, 0));
      lb_actual_tor[i]->setStyleSheet(
          QString::fromUtf8("color: rgb(0, 0, 0);\n"
                            "selection-background-color: rgb(238, 238, 236);\n"
                            "selection-color: rgb(238, 238, 236);\n"
                            "background-color:  rgb(255, 255, 255);\n"
                            "alternate-background-color: rgb(0, 0, 0);\n"
                            "font: bold 75 14pt \"Noto Sans\";"));
      lb_actual_tor[i]->setAlignment(Qt::AlignCenter);

      status_bar_gridLayout->addWidget(lb_actual_tor[i], 3, 2 * i + 2, 1, 1);
    }

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      lb_op_mode[i] = new QLabel(centralwidget);
      lb_op_mode[i]->setObjectName(QString::fromUtf8("lb_op_mode_") + QString::number(i));
      sizePolicy.setHeightForWidth(lb_op_mode[i]->sizePolicy().hasHeightForWidth());
      lb_op_mode[i]->setSizePolicy(sizePolicy);
      lb_op_mode[i]->setMinimumSize(QSize(0, 0));
      lb_op_mode[i]->setStyleSheet(
          QString::fromUtf8("color: rgb(0, 0, 0);\n"
                            "selection-background-color: rgb(238, 238, 236);\n"
                            "selection-color: rgb(238, 238, 236);\n"
                            "background-color:  rgb(255, 255, 255);\n"
                            "alternate-background-color: rgb(0, 0, 0);\n"
                            "font: bold 75 14pt \"Noto Sans\";"));
      lb_op_mode[i]->setAlignment(Qt::AlignCenter);

      status_bar_gridLayout->addWidget(lb_op_mode[i], 5, 2 * i + 1, 1, 2);
    }
    ls_control_word = new QLabel(centralwidget);
    ls_control_word->setObjectName(QString::fromUtf8("ls_control_word"));
    sizePolicy.setHeightForWidth(ls_control_word->sizePolicy().hasHeightForWidth());
    ls_control_word->setSizePolicy(sizePolicy);
    ls_control_word->setMinimumSize(QSize(0, 0));
    ls_control_word->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";\n"
                          "padding-left: 5px;\n"
                          "padding-left: 5px;\n"
                          ""));
    ls_control_word->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(ls_control_word, 4, 0, 1, 1);
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      lb_target_vel[i] = new QLabel(centralwidget);
      lb_target_vel[i]->setObjectName(QString::fromUtf8("lb_target_vel_" + i));
      sizePolicy.setHeightForWidth(lb_target_vel[i]->sizePolicy().hasHeightForWidth());
      lb_target_vel[i]->setSizePolicy(sizePolicy);
      lb_target_vel[i]->setMinimumSize(QSize(20, 20));
      lb_target_vel[i]->setStyleSheet(
          QString::fromUtf8("color: rgb(0, 0, 0);\n"
                            "selection-background-color: rgb(238, 238, 236);\n"
                            "selection-color: rgb(238, 238, 236);\n"
                            "background-color:  rgb(255, 255, 255);\n"
                            "alternate-background-color: rgb(0, 0, 0);\n"
                            "font: bold 75 14pt \"Noto Sans\";"));
      lb_target_vel[i]->setAlignment(Qt::AlignCenter);

      status_bar_gridLayout->addWidget(lb_target_vel[i], 1, 2 * i + 1, 1, 1);
    }
    ls_target_tor = new QLabel(centralwidget);
    ls_target_tor->setObjectName(QString::fromUtf8("ls_target_tor"));
    sizePolicy.setHeightForWidth(ls_target_tor->sizePolicy().hasHeightForWidth());
    ls_target_tor->setSizePolicy(sizePolicy);
    ls_target_tor->setMinimumSize(QSize(0, 0));
    ls_target_tor->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";\n"
                          "padding-left: 5px;\n"
                          "padding-left: 5px;\n"
                          ""));
    ls_target_tor->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(ls_target_tor, 3, 0, 1, 1);

    lb_safety_state = new QLabel(centralwidget);
    lb_safety_state->setObjectName(QString::fromUtf8("lb_emergency_status"));
    sizePolicy.setHeightForWidth(lb_safety_state->sizePolicy().hasHeightForWidth());
    lb_safety_state->setSizePolicy(sizePolicy);
    lb_safety_state->setMinimumSize(QSize(0, 0));
    lb_safety_state->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";"));
    lb_safety_state->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(lb_safety_state, 8, 1, 1, 2 * g_kNumberOfServoDrivers);

    ls_ecat_status_bar = new QLabel(centralwidget);
    ls_ecat_status_bar->setObjectName(QString::fromUtf8("ls_ecat_status_bar"));
    sizePolicy4.setHeightForWidth(ls_ecat_status_bar->sizePolicy().hasHeightForWidth());
    ls_ecat_status_bar->setSizePolicy(sizePolicy4);
    ls_ecat_status_bar->setStyleSheet(
        QString::fromUtf8("color: rgb(255, 255, 255);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color: rgb(19, 61, 128);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 16pt \"Noto Sans\";"));
    ls_ecat_status_bar->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(ls_ecat_status_bar, 6, 0, 1, 2 * g_kNumberOfServoDrivers + 1);

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      lb_target_tor[i] = new QLabel(centralwidget);
      lb_target_tor[i]->setObjectName(QString::fromUtf8("lb_target_tor_") + QString::number(i));
      sizePolicy.setHeightForWidth(lb_target_tor[i]->sizePolicy().hasHeightForWidth());
      lb_target_tor[i]->setSizePolicy(sizePolicy);
      lb_target_tor[i]->setMinimumSize(QSize(0, 0));
      lb_target_tor[i]->setStyleSheet(
          QString::fromUtf8("color: rgb(0, 0, 0);\n"
                            "selection-background-color: rgb(238, 238, 236);\n"
                            "selection-color: rgb(238, 238, 236);\n"
                            "background-color:  rgb(255, 255, 255);\n"
                            "alternate-background-color: rgb(0, 0, 0);\n"
                            "font: bold 75 14pt \"Noto Sans\";"));
      lb_target_tor[i]->setAlignment(Qt::AlignCenter);
      status_bar_gridLayout->addWidget(lb_target_tor[i], 3, 2 * i + 1, 1, 1);
    }
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      lb_actual_pos[i] = new QLabel(centralwidget);
      lb_actual_pos[i]->setObjectName(QString::fromUtf8("lb_actual_pos_") + QString::number(i));
      sizePolicy.setHeightForWidth(lb_actual_pos[i]->sizePolicy().hasHeightForWidth());
      lb_actual_pos[i]->setSizePolicy(sizePolicy);
      lb_actual_pos[i]->setMinimumSize(QSize(0, 0));
      lb_actual_pos[i]->setStyleSheet(
          QString::fromUtf8("color: rgb(0, 0, 0);\n"
                            "selection-background-color: rgb(238, 238, 236);\n"
                            "selection-color: rgb(238, 238, 236);\n"
                            "background-color:  rgb(255, 255, 255);\n"
                            "alternate-background-color: rgb(0, 0, 0);\n"
                            "font: bold 75 14pt \"Noto Sans\";"));
      lb_actual_pos[i]->setAlignment(Qt::AlignCenter);
      status_bar_gridLayout->addWidget(lb_actual_pos[i], 2, 2 * i + 2, 1, 1);
    }
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      lb_control_word[i] = new QLabel(centralwidget);
      lb_control_word[i]->setObjectName(QString::fromUtf8("lb_control_word_") + QString::number(i));
      sizePolicy.setHeightForWidth(lb_control_word[i]->sizePolicy().hasHeightForWidth());
      lb_control_word[i]->setSizePolicy(sizePolicy);
      lb_control_word[i]->setMinimumSize(QSize(0, 0));
      lb_control_word[i]->setStyleSheet(
          QString::fromUtf8("color: rgb(0, 0, 0);\n"
                            "selection-background-color: rgb(238, 238, 236);\n"
                            "selection-color: rgb(238, 238, 236);\n"
                            "background-color:  rgb(255, 255, 255);\n"
                            "alternate-background-color: rgb(0, 0, 0);\n"
                            "font: bold 75 14pt \"Noto Sans\";"));
      lb_control_word[i]->setAlignment(Qt::AlignCenter);
      status_bar_gridLayout->addWidget(lb_control_word[i], 4, 2 * i + 1, 1, 1);
    }

    ls_safety_state = new QLabel(centralwidget);
    ls_safety_state->setObjectName(QString::fromUtf8("ls_emergency_status"));
    sizePolicy.setHeightForWidth(ls_safety_state->sizePolicy().hasHeightForWidth());
    ls_safety_state->setSizePolicy(sizePolicy);
    ls_safety_state->setMinimumSize(QSize(0, 0));
    ls_safety_state->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";\n"
                          "padding-left: 5px;\n"
                          "padding-left: 5px;\n"
                          ""));
    ls_safety_state->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(ls_safety_state, 8, 0, 1, 1);

    ls_motor_error_code = new QLabel(centralwidget);
    ls_motor_error_code->setObjectName(QString::fromUtf8("ls_motor_error_code"));
    sizePolicy.setHeightForWidth(ls_motor_error_code->sizePolicy().hasHeightForWidth());
    ls_motor_error_code->setSizePolicy(sizePolicy);
    ls_motor_error_code->setMinimumSize(QSize(0, 0));
    ls_motor_error_code->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";\n"
                          "padding-left: 5px;\n"
                          "padding-left: 5px;\n"
                          ""));
    ls_motor_error_code->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(ls_motor_error_code, 9, 0, 1, 1);

    label_motor_no = new QLabel(centralwidget);
    label_motor_no->setObjectName(QString::fromUtf8("label_motor_no"));
    label_motor_no->setStyleSheet(
        QString::fromUtf8("color: rgb(0, 0, 0);\n"
                          "selection-background-color: rgb(238, 238, 236);\n"
                          "selection-color: rgb(238, 238, 236);\n"
                          "background-color:  rgb(255, 255, 255);\n"
                          "alternate-background-color: rgb(0, 0, 0);\n"
                          "font: bold 75 14pt \"Noto Sans\";"));
    label_motor_no->setAlignment(Qt::AlignCenter);

    status_bar_gridLayout->addWidget(label_motor_no, 0, 0, 1, 1);

    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      lb_actual_vel[i] = new QLabel(centralwidget);
      lb_actual_vel[i]->setObjectName(QString::fromUtf8("lb_actual_vel_") + QString::number(i));
      sizePolicy4.setHeightForWidth(lb_actual_vel[i]->sizePolicy().hasHeightForWidth());
      lb_actual_vel[i]->setSizePolicy(sizePolicy4);
      lb_actual_vel[i]->setStyleSheet(
          QString::fromUtf8("color: rgb(0, 0, 0);\n"
                            "selection-background-color: rgb(238, 238, 236);\n"
                            "selection-color: rgb(238, 238, 236);\n"
                            "background-color:  rgb(255, 255, 255);\n"
                            "alternate-background-color: rgb(0, 0, 0);\n"
                            "font: bold 75 14pt \"Noto Sans\";"));
      lb_actual_vel[i]->setAlignment(Qt::AlignCenter);

      status_bar_gridLayout->addWidget(lb_actual_vel[i], 1, 2 * i + 2, 1, 1);
    }

    right_verticalLayout->addLayout(status_bar_gridLayout);
    right_verticalLayout->setStretch(0, 0);
    right_verticalLayout->setStretch(1, 0);
    right_verticalLayout->setStretch(2, 0);
    right_verticalLayout->setStretch(3, 1);
    right_verticalLayout->setStretch(4, 0);

    bottom_horizontal_layout->addLayout(right_verticalLayout);
    bottom_horizontal_layout->setStretch(0, 0);
    bottom_horizontal_layout->setStretch(1, 5);
    gridLayout->addLayout(bottom_horizontal_layout, 0, 0, 1, 1);

    MainWindow->setCentralWidget(centralwidget);

    retranslateUi(MainWindow);

    QMetaObject::connectSlotsByName(MainWindow);
  }  // setupUi

  void retranslateUi(QMainWindow* MainWindow)
  {
    MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
    b_enable_cyclic_pos->setText(QApplication::translate("MainWindow",
                                                         "Enable Cyclic\n"
                                                         "Position Mode All",
                                                         nullptr));

    b_enable_cyclic_vel->setText(QApplication::translate("MainWindow",
                                                         "Enable Cyclic\n"
                                                         "Velocity Mode All",
                                                         nullptr));
    b_enable_torque->setText(QApplication::translate("MainWindow",
                                                     "Enable\n"
                                                     "Torque Mode All",
                                                     nullptr));

    b_enable_cyclic_torque->setText(QApplication::translate("MainWindow",
                                                            "Enable Cyclic\n"
                                                            "Torque Mode All",
                                                            nullptr));

    b_enable_vel->setText(QApplication::translate("MainWindow",
                                                  "Enable \n"
                                                  "Velocity Mode All",
                                                  nullptr));
    b_enable_pos->setText(QApplication::translate("MainWindow",
                                                  "Enable \n"
                                                  " Position Mode All",
                                                  nullptr));
    b_clear_fault->setText(QApplication::translate("MainWindow",
                                                   "Clear\n"
                                                   "Faults All",
                                                   nullptr));
    b_init_ecat->setText(QApplication::translate("MainWindow",
                                                 "Initialize \n"
                                                 " EtherCAT",
                                                 nullptr));
    b_reinit_ecat->setText(QApplication::translate("MainWindow",
                                                   "Restart \n"
                                                   "EtherCAT Master",
                                                   nullptr));
    b_enable_drives->setText(QApplication::translate("MainWindow",
                                                     "Enable All\n"
                                                     "Drives",
                                                     nullptr));
    b_disable_drives->setText(QApplication::translate("MainWindow",
                                                      "Disable All\n"
                                                      " Drives",
                                                      nullptr));
    b_enter_cyclic_pdo->setText(QApplication::translate("MainWindow",
                                                        "Enter Cyclic \n"
                                                        " Exchange PDO",
                                                        nullptr));
    b_stop_cyclic_pdo->setText(QApplication::translate("MainWindow",
                                                       "STOP Cyclic \n"
                                                       " Exchange PDO",
                                                       nullptr));
    b_emergency_mode->setText(QApplication::translate("MainWindow", "Emergency Mode", nullptr));
    ls_enter_target_vals->setText(
        QApplication::translate("MainWindow", "Enter Target Value (Velocity, Torque,Position)", nullptr));
    ls_status_bar->setText(QApplication::translate("MainWindow", "STATUS BAR", nullptr));
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      label_motor[i]->setText(QApplication::translate("MainWindow", "Motor 1", nullptr));
      label_motor[i]->setText(QString("Motor ") + QString::number(i + 1));
      b_send[i]->setText(QApplication::translate("MainWindow", "SEND" + i, nullptr));
      b_send[i]->setText(QString("Send M") + QString::number(i + 1));

      b_stop[i]->setText(QApplication::translate("MainWindow", "STOP" + i, nullptr));
      b_stop[i]->setText(QString("STOP M") + QString::number(i + 1));

      b_enable[i]->setText(QApplication::translate("MainWindow", "ENABLE" + i, nullptr));
      b_enable[i]->setText(QString("ENABLE"));

      b_disable[i]->setText(QApplication::translate("MainWindow", "DISABLE" + i, nullptr));
      b_disable[i]->setText(QString("DISABLE"));

      b_vel[i]->setText(QApplication::translate("MainWindow", "VEL" + i, nullptr));
      b_vel[i]->setText(QString("VEL"));

      b_cyclic_vel[i]->setText(QApplication::translate("MainWindow", "Cyclic Velocity" + i, nullptr));
      b_cyclic_vel[i]->setText(QString("CSV"));

      b_pos[i]->setText(QApplication::translate("MainWindow", "Position" + i, nullptr));
      b_pos[i]->setText(QString("POS"));

      b_cyclic_pos[i]->setText(QApplication::translate("MainWindow", "Cyclic Position" + i, nullptr));
      b_cyclic_pos[i]->setText(QString("CSP"));

      b_tor[i]->setText(QApplication::translate("MainWindow", "Torque" + i, nullptr));
      b_tor[i]->setText(QString("TOR"));

      b_cyclic_tor[i]->setText(QApplication::translate("MainWindow", "Cyclic Torque" + i, nullptr));
      b_cyclic_tor[i]->setText(QString("CST"));

      lb_actual_tor[i]->setText(QString());
      lb_op_mode[i]->setText(QString());
      lb_target_vel[i]->setText(QString());
      lb_actual_vel[i]->setText(QString());
      lb_target_tor[i]->setText(QString());
      lb_actual_pos[i]->setText(QString());
      lb_control_word[i]->setText(QString());
      lb_target_pos[i]->setText(QString());
    }
    ls_op_mode->setText(QApplication::translate("MainWindow", "Operational Mode", nullptr));
    ls_tar_vel->setText(QApplication::translate("MainWindow", "Target Velocity  / Actual Velocity ", nullptr));
    lb_com_status->setText(QString());
    ls_tar_pos->setText(QApplication::translate("MainWindow", "Target Position / Actual Position", nullptr));
    ls_control_word->setText(QApplication::translate("MainWindow", "Control Word / Status Word", nullptr));
    ls_target_tor->setText(QApplication::translate("MainWindow", "Target Torque / Actual Torque", nullptr));
    lb_safety_state->setText(QString());
    ls_ecat_status_bar->setText(QApplication::translate("MainWindow", "EtherCAT State", nullptr));
    ls_safety_state->setText(QApplication::translate("MainWindow", "Safety State", nullptr));
    ls_com_status->setText(QApplication::translate("MainWindow", "COM Status", nullptr));
    ls_motor_error_code->setText(QApplication::translate("MainWindow", "Error Registers", nullptr));
    label_motor_no->setText(QApplication::translate("MainWindow", "MOTOR No", nullptr));
  }  // retranslateUi
};

namespace Ui
{
class MainWindow : public Ui_MainWindow
{
};
}  // namespace Ui

QT_END_NAMESPACE

#endif  // UI_MAINWINDOW_H
