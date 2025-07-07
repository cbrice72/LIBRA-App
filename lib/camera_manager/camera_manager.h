/******************************************************************************
 * @file   camera_manager.h
 * @brief  QMediaCaptureSession convenience class header file.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <unordered_map>

// Other Library Headers
#include <QtMultimedia>  // Qt::Multimedia
#include <QVideoWidget>  // Qt::MultimediaWidgets
#ifdef BUILD_WITH_ROS2
# include <opencv2/videoio.hpp>        // OpenCV
# include <QVideoSink>                 // Qt::Multimedia
# include <rclcpp/rclcpp.hpp>          // ROS2 Core
# include <sensor_msgs/msg/image.hpp>  // ROS2 Messages
#endif

// Project Headers
#include "logger.h"

/**
 * @brief Manages a single `QCamera` object, allowing for image capture, video
 *        recording, and streaming to Qt Multimedia Widgets.
 */
class CameraManager : public QObject
#ifdef BUILD_WITH_ROS2
    ,
                      public rclcpp::Node
#endif
{
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    CameraManager(QString id, QVideoWidget* viewfinder,
                  const bool& debug_mode = false, QObject* parent = nullptr);
    ~CameraManager();

    void SetDebugMode(const bool& enabled) {
        debug_mode_ = enabled;
        logger_->SetDebugMode(debug_mode_);
    };

    void Start();
    void Stop();
    void Capture();
    bool Record();

  private:
    // --- Helper Functions ---

    bool CameraIsActive();

#ifdef BUILD_WITH_ROS2
    void StartRos2NodeViaLaunchFile();
    void ProcessRos2Image(const sensor_msgs::msg::Image::SharedPtr msg);
#endif

    // --- Data Members ---

    QString id_;

    std::unique_ptr<Logger> logger_;
    bool debug_mode_{false};

    std::string output_dir_;
    QCamera* camera_{nullptr};
    bool use_ros2_node_{false};

    QMediaCaptureSession session_;
    QImageCapture* capture_{nullptr};
    QMediaRecorder* recorder_{nullptr};

    std::string image_filename_;
    std::atomic<bool> is_recording_{false};
    std::string video_filename_;

#ifdef BUILD_WITH_ROS2
    const std::string kCameraRgbTopic = "/camera/color/image_raw";

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    QProcess* ros2_process_{nullptr};
    QTimer* spin_timer_{nullptr};

    std::atomic<bool> capture_requested_{false};

    QVideoSink* video_sink_{nullptr};  // for displaying in Qt
    const int video_codec_;
    cv::VideoWriter video_writer_;
#endif
};
