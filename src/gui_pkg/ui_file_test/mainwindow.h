#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"
namespace Ui {

class MainWindow;
}
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    Ui::MainWindow *ui;
    int argc_;
    char** argv_;
private:
    void DisableOtherModes(QPushButton *button,int index);
    void EnableAllModes(int index);
private slots:
    void on_send_clicked(int m_no);
    void on_stop_clicked(int m_no);
    void on_enable_clicked(int m_no);
    void on_disable_clicked(int m_no);
    void on_vel_clicked(int m_no);
    void on_cyclic_vel_clicked(int m_no);
    void on_pos_clicked(int m_no);
    void on_cyclic_pos_clicked(int m_no);
    void on_tor_clicked(int m_no);
    void on_cyclic_tor_clicked(int m_no);
};
#endif // MAINWINDOW_H
