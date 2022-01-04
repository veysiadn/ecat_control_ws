/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2021 Veysi ADIN, UST KIST
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS2 environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: veysi.adin@kist.re.kr
 *****************************************************************************/
/*****************************************************************************
 * \file  video_capture.hpp
 * \brief Class header for video capture from endoscope camera by using OpenCV to show in GUI
 *******************************************************************************/
#ifndef VIDEO_CAPTURE_HPP
#define VIDEO_CAPTURE_HPP

// QT headers for image and reading thread.
#include <QPixmap>
#include <QImage>
#include <QThread>
// OpenCV headers for camera capture.
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>

/**
 * Camera ID will be your camera's device ID.
 * If you don't have any camera attached to usb default ID will be 0.
 * If you're sure about camera ID, open terminal and type
 *  $ sudo apt-get install v4l-utils
 *  $ v4l2-ctl --list-devices
 * You can take your camera by taking the last digit in /dev/videoX
 * X will be your device ID.
*/

#include "gui_globals.hpp"

class VideoCapture : public QThread
{
    Q_OBJECT
public:
    VideoCapture(QObject *parent = nullptr);
    QPixmap pixmap() const { return pixmap_cap;}
signals:
    //capture a frame
    void NewPixmapCapture();
protected:
    void run() override;
private:

    QPixmap pixmap_cap;              //Qt image
    cv::Mat frame_cap;               //OpenCV image
    cv::VideoCapture video_cap;      //video capture

    unsigned long frame_rate = 30;
    uint8_t cam_id  = ID_CAMERA ; 
    // Convert opencv readings to QImage && QPixmap format
    QImage cvMatToQImage(const cv::Mat &inMat);
    QPixmap cvMatToQPixmap(const cv::Mat &inMat );
};

#endif // MYVIDEOCAPTURE_H
