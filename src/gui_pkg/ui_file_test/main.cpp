#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w(argc,argv);
    w.setWindowTitle("Control UI by Veysi ADIN © ");
    w.show();
    return a.exec();
}
