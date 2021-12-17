#include "../include/gui_pkg/endoscope_viewer.hpp"

EndoscopeViewer::EndoscopeViewer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::EndoscopeViewer)
{
    ui->setupUi(this);
    opencv_video_cap_ =  new VideoCapture(this);
    connect(opencv_video_cap_, &VideoCapture::NewPixmapCapture, this, [&]()
    {
       ui->image_frame->setPixmap(opencv_video_cap_->pixmap().scaled(1280,720,Qt::KeepAspectRatioByExpanding));
    });

}

EndoscopeViewer::~EndoscopeViewer()
{
    delete ui;
}

void EndoscopeViewer::on_b_start_capture_clicked()
{
    opencv_video_cap_->start(QThread::NormalPriority);
}

void EndoscopeViewer::on_b_stop_capture_clicked()
{
    if (opencv_video_cap_->isRunning()){
        opencv_video_cap_->requestInterruption();
    }
}

void EndoscopeViewer::on_b_exit_clicked()
{
    if (opencv_video_cap_->isRunning()){
        opencv_video_cap_->requestInterruption();
    }
    EndoscopeViewer::close();
}
