#include <QApplication>
#include "../include/gui_pkg/main_window.hpp"
#include "rclcpp/rclcpp.hpp"
#include "../include/gui_pkg/controlui.hpp"

int main(int argc, char **argv) {

    QApplication app(argc, argv);
    MainWindow w(argc,argv);
    w.setWindowTitle("Spine Robot Endoscope Viewer");
    w.show();
    int result = app.exec();
    return result;

}
