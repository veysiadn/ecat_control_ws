#ifndef CONTROLUI_HPP
#define CONTROLUI_HPP

#include <QMainWindow>

namespace Ui {
class ControlUI;
}

class ControlUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit ControlUI(QWidget *parent = nullptr);
    ~ControlUI();

private slots:

    void on_b_init_ecat_clicked();

    void on_b_reinit_ecat_clicked();

    void on_b_enable_motor_clicked();

    void on_b_disable_motor_clicked();

    void on_b_enable_cyclic_pos_clicked();

    void on_b_enable_cylic_vel_clicked();

    void on_b_enable_vel_clicked();

    void on_b_enable_pos_clicked();

    void on_b_enter_cyclic_pdo_clicked();

    void on_b_emergency_mode_clicked();

    void on_b_send_clicked();

private:
    Ui::ControlUI *ui;
};

#endif // CONTROLUI_HPP
