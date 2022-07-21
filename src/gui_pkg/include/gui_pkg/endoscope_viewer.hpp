#ifndef ENDOSCOPE_VIEWER_HPP
#define ENDOSCOPE_VIEWER_HPP

#include <QMainWindow>
#include "ui_endoscope_viewer.h"
#include <QApplication>
#include <Qt>

#include "video_capture.hpp"

namespace Ui
{
class EndoscopeViewer;
}

class EndoscopeViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit EndoscopeViewer(QWidget* parent = nullptr);
  ~EndoscopeViewer();

private slots:

  void on_b_start_capture_clicked();

  void on_b_stop_capture_clicked();

  void on_b_exit_clicked();

private:
  Ui::EndoscopeViewer* ui;
  VideoCapture* opencv_video_cap_;
};

#endif  // ENDOSCOPE_VIEWER_HPP
