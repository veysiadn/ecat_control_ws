#include "../include/gui_pkg/video_capture.hpp"
#include <QDebug>

VideoCapture::VideoCapture(QObject* parent) : QThread{ parent }, video_cap{ ID_CAMERA }
{
}

void VideoCapture::run()
{
  if (!video_cap.isOpened())
  {
    video_cap.open(cam_id);
  }
  while (true)
  {
    video_cap >> frame_cap;
    if (!frame_cap.empty())
    {
      pixmap_cap = cvMatToQPixmap(frame_cap);
      emit NewPixmapCapture();
    }
    if (isInterruptionRequested())
    {
      video_cap.release();
      std::cout << "Release requested " << std::endl;
      break;
    }
    QThread::msleep((1 / frame_rate) * 1000);
  }
  return;
}

QImage VideoCapture::cvMatToQImage(const cv::Mat& inMat)
{
  switch (inMat.type())
  {
    // 8-bit, 4 channel
    case CV_8UC4: {
      QImage image(inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_ARGB32);

      return image;
    }

      // 8-bit, 3 channel
    case CV_8UC3: {
      QImage image(inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_RGB888);

      return image.rgbSwapped();
    }

      // 8-bit, 1 channel
    case CV_8UC1: {
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
      QImage image(inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_Grayscale8);
#else
      static QVector<QRgb> sColorTable;

      // only create our color table the first time
      if (sColorTable.isEmpty())
      {
        sColorTable.resize(256);

        for (int i = 0; i < 256; ++i)
        {
          sColorTable[i] = qRgb(i, i, i);
        }
      }

      QImage image(inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_Indexed8);

      image.setColorTable(sColorTable);
#endif

      return image;
    }

    default:
      qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
      break;
  }

  return QImage();
}

QPixmap VideoCapture::cvMatToQPixmap(const cv::Mat& inMat)
{
  return QPixmap::fromImage(cvMatToQImage(inMat));
}
