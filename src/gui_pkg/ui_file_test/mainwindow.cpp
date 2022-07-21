#include "mainwindow.h"
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
}
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      argc_(argc),
      argv_(argv)
{
    ui->setupUi(this);
    for(int i = 0 ; i < NUM_OF_SERVO_DRIVERS;i++){
        connect(ui->b_send[i],SIGNAL(buttonClicked(int)),this,SLOT(on_send_clicked(int)));
        connect(ui->b_stop[i],SIGNAL(buttonClicked(int)),this,SLOT(on_stop_clicked(int)));
        connect(ui->b_enable[i],SIGNAL(buttonClicked(int)),this,SLOT(on_enable_clicked(int)));
        connect(ui->b_disable[i],SIGNAL(buttonClicked(int)),this,SLOT(on_disable_clicked(int)));
        connect(ui->b_vel[i],SIGNAL(buttonClicked(int)),this,SLOT(on_vel_clicked(int)));
        connect(ui->b_pos[i],SIGNAL(buttonClicked(int)),this,SLOT(on_cyclic_vel_clicked(int)));
        connect(ui->b_tor[i],SIGNAL(buttonClicked(int)),this,SLOT(on_pos_clicked(int)));
        connect(ui->b_cyclic_vel[i],SIGNAL(buttonClicked(int)),this,SLOT(on_cyclic_pos_clicked(int)));
        connect(ui->b_cyclic_pos[i],SIGNAL(buttonClicked(int)),this,SLOT(on_tor_clicked(int)));
        connect(ui->b_cyclic_tor[i],SIGNAL(buttonClicked(int)),this,SLOT(on_cyclic_tor_clicked(int)));
    }
}

MainWindow::~MainWindow()
{
}

void MainWindow::on_send_clicked(int m_no)
{

    ui->lb_target_vel[m_no]->setText(QString("Sent ")+ QString::number(m_no+1));
    if (ui->b_send[m_no]->isChecked())
        ui->lb_target_vel[m_no]->clear();
}
void MainWindow::on_stop_clicked(int m_no)
{
    ui->lb_target_vel[m_no]->setText(QString("Stopped ")+ QString::number(m_no+1));

}
void MainWindow::on_enable_clicked(int m_no)
{
    ui->lb_target_vel[m_no]->setText(QString("Enabled ")+ QString::number(m_no+1));

}
void MainWindow::on_disable_clicked(int m_no)
{
    ui->lb_target_vel[m_no]->setText(QString("Disabled ")+ QString::number(m_no+1));

}
void MainWindow::on_vel_clicked(int m_no)
{
    ui->lb_target_vel[m_no]->setText(QString("Vel Mode ")+ QString::number(m_no+1));
    if (ui->b_vel[m_no]->isChecked()){
        DisableOtherModes(ui->b_vel[m_no],m_no);
    }else{
        EnableAllModes(m_no);
        ui->lb_target_vel[m_no]->clear();
    }
}
void MainWindow::DisableOtherModes(QPushButton *button,int index)
{
    ui->b_vel[index]->setDisabled(true);
    ui->b_pos[index]->setDisabled(true);
    ui->b_tor[index]->setDisabled(true);
    ui->b_cyclic_vel[index]->setDisabled(true);
    ui->b_cyclic_pos[index]->setDisabled(true);
    ui->b_cyclic_tor[index]->setDisabled(true);
    button->setEnabled(true);

}

void MainWindow::EnableAllModes(int index)
{
    ui->b_vel[index]->setDisabled(false);
    ui->b_pos[index]->setDisabled(false);
    ui->b_tor[index]->setDisabled(false);
    ui->b_cyclic_vel[index]->setDisabled(false);
    ui->b_cyclic_pos[index]->setDisabled(false);
    ui->b_cyclic_tor[index]->setDisabled(false);

}
void MainWindow::on_cyclic_vel_clicked(int m_no)
{
    ui->lb_target_vel[m_no]->setText(QString("Cyc Vel Mode ")+ QString::number(m_no+1));

}
void MainWindow::on_pos_clicked(int m_no)
{
    ui->lb_target_vel[m_no]->setText(QString("Pos Mode ")+ QString::number(m_no+1));

}
void MainWindow::on_cyclic_pos_clicked(int m_no)
{
    ui->lb_target_vel[m_no]->setText(QString("Cyc Pos Mode ")+ QString::number(m_no+1));

}
void MainWindow::on_tor_clicked(int m_no)
{
    ui->lb_target_vel[m_no]->setText(QString("Tor Mode ")+ QString::number(m_no+1));
}
void MainWindow::on_cyclic_tor_clicked(int m_no)
{
    ui->lb_target_vel[m_no]->setText(QString("Cyc Tor Mode ")+ QString::number(m_no+1));

}
