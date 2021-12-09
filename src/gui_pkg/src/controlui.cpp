#include "../include/gui_pkg/controlui.hpp"
#include "ui_controlui.h"

ControlUI::ControlUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ControlUI)
{
    ui->setupUi(this);
}

ControlUI::~ControlUI()
{
    delete ui;
}

void ControlUI::on_b_init_ecat_clicked()
{

}

void ControlUI::on_b_reinit_ecat_clicked()
{

}

void ControlUI::on_b_enable_motor_clicked()
{

}

void ControlUI::on_b_disable_motor_clicked()
{

}

void ControlUI::on_b_enable_cyclic_pos_clicked()
{

}

void ControlUI::on_b_enable_cylic_vel_clicked()
{

}

void ControlUI::on_b_enable_vel_clicked()
{

}

void ControlUI::on_b_enable_pos_clicked()
{

}

void ControlUI::on_b_enter_cyclic_pdo_clicked()
{

}

void ControlUI::on_b_emergency_mode_clicked()
{

}

void ControlUI::on_b_send_clicked()
{

}
