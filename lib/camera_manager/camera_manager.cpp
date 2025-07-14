/******************************************************************************
 * @file   camera_manager.cpp
 * @brief  QMediaCaptureSession convenience class implementation file.
 *
 * @author brice.c.aa
 ******************************************************************************/

// Related Header
#include "camera_manager.h"

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QDateTime>  // Qt::Core
#include <QDir>       // Qt::Core
#ifdef BUILD_WITH_ROS2
# include <cv_bridge/cv_bridge.h>  // OpenCV
# include <opencv2/imgcodecs.hpp>  // OpenCV
# include <QVideoFrame>            // Qt::Multimedia
#endif

// Project Headers
#ifdef BUILD_WITH_ROS2
# include "ros2_logger.h"
#else
# include "qt_logger.h"
#endif

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Class Helpers
 * !Camera Commands
 */

constexpr int kWaitForTimeout = 3000;  // ms

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

#ifdef BUILD_WITH_ROS2
bool IsRealSenseCamera(const QCameraDevice& device) {
    return device.description().toLower().contains("realsense");
}
#endif

/**
 * @brief Provides a filename-safe string of the current date and time.
 *
 * @return std::string Date-time string in the format "yyyy-MM-ddTHH-mm-ss"
 */
std::string GetDateTimeStr() {
    auto dts = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    dts.replace(":", "-");         // replace colons (invalid in filenames)
    dts = dts.section('.', 0, 0);  // remove milliseconds
    return dts.toStdString();
}

}  // namespace

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param id Camera location (e.g., `/dev/video0`); will be used to find the device
 * @param viewfinder The QVideoWidget object to display the camera feeed in
 * @param debug_mode Whether to output verbose debug text
 * @param parent Owning Qt widget (default: nullptr)
 */
CameraManager::CameraManager(QString id, QVideoWidget* viewfinder,
                             const bool& debug_mode, QObject* parent)
    : QObject(parent),
      id_(std::move(id)),
      debug_mode_(debug_mode),
      output_dir_(QDir::currentPath().toStdString() + "/")
#ifdef BUILD_WITH_ROS2
      ,
      rclcpp::Node("camera_manager_" + id.section('/', -1).toStdString()),
      video_codec_(cv::VideoWriter::fourcc('M', 'J', 'P', 'G'))
#endif
{
    // Initialize the logger
#ifdef BUILD_WITH_ROS2
    logger_ = std::make_unique<Ros2Logger>(debug_mode_, this->get_logger());
#else
    logger_ = std::make_unique<QtLogger>(debug_mode_);
#endif

    // Find requested camera
    const auto cameras = QMediaDevices::videoInputs();
    QCameraDevice selected_camera;

    for (const auto& camera_device : cameras) {
        if (camera_device.id() == id_) {
            // NOTE: We don't immediately initialize the QCamera object `camera_`
            //       here since, if it's a RealSense RGB-D camera, we'd rather
            //       let a ROS2 realsense2_camera node do all the work.
            selected_camera = camera_device;
            break;
        }
    }

    if (selected_camera.isNull()) {
        logger_->Error("CameraManager - Camera " + id_.toStdString()
                       + " not found!");
        return;
    }

#ifdef BUILD_WITH_ROS2
    // ========== Special Case: RealSense Depth Cameras ==========

    if (IsRealSenseCamera(selected_camera)) {
        logger_->Info("CameraManager - RealSense camera detected! Deferring to "
                      "ROS2 node...");
        use_ros2_node_ = true;

        // Get video sink from the QVideoWidget for direct frame injection
        video_sink_ = viewfinder->videoSink();

        return;  // Qt Multimedia setup not necessary
    }
#endif

    // ========== Regular Cameras use Qt Multimedia ==========

    camera_ = new QCamera(selected_camera, this);

    // Initialize central media capture object
    session_.setCamera(camera_);

    // Register image capture object
    capture_ = new QImageCapture(this);
    session_.setImageCapture(capture_);

    // Register video recorder object
    recorder_ = new QMediaRecorder(camera_);
    session_.setRecorder(recorder_);

    // Register video output to existing UI VideoWidget
    session_.setVideoOutput(viewfinder);
    viewfinder->show();  // enable the video feed

    // If an invalid CameraFormat is detected, select a safe default
    if (camera_->cameraFormat().pixelFormat()
        == QVideoFrameFormat::Format_Invalid) {
        logger_->Warn("CameraManager - Invalid camera format detected; "
                      "applying defaults");

        // Check all supported formats
        // (we can't edit a QCameraFormat object, so try looking for it)
        bool selected = false;
        QList<QCameraFormat> supported_formats = camera_->cameraDevice()
                                                     .videoFormats();
        for (const QCameraFormat& format : supported_formats) {
            // Our "safe default" is a JPEG-formatted 720p (HD) stream
            if (format.pixelFormat() == QVideoFrameFormat::Format_Jpeg
                && format.resolution() == QSize(1280, 720)) {
                camera_->setCameraFormat(format);
                selected = true;
                break;
            }
        }

        if (!selected) {
            logger_->Warn(
                "CameraManager - This camera doesn't seem to support "
                "a JPEG 720p stream; defaulting to the first supported format");
            camera_->setCameraFormat(supported_formats.first());
        }
    }

    // Start camera
    camera_->start();

    // Set output format for video
    QMediaFormat format(QMediaFormat::MPEG4);
    // format.setAudioCodec(QMediaFormat::AudioCodec::MP3);
    format.setVideoCodec(QMediaFormat::VideoCodec::MPEG4);

    recorder_->setMediaFormat(format);
    recorder_->setQuality(QMediaRecorder::Quality::VeryHighQuality);

    const auto& camera_format = camera_->cameraFormat();
    recorder_->setVideoResolution(camera_format.resolution());
    recorder_->setVideoFrameRate(camera_format.maxFrameRate());

    // Ensure output directories exist
    const QDir img_dir(QString::fromStdString(output_dir_ + "img"));
    if (!img_dir.exists()) {
        img_dir.mkpath(".");
    }
    const QDir vid_dir(QString::fromStdString(output_dir_ + "vid"));
    if (!vid_dir.exists()) {
        vid_dir.mkpath(".");
    }
};

/**
 * @brief Standard desctructor.
 */
CameraManager::~CameraManager() {
    // Stop camera
    Stop();

    if (use_ros2_node_) {
#ifdef BUILD_WITH_ROS2
        // Stop helper objects used by ROS2 message processing
        if (video_writer_.isOpened()) {
            video_writer_.release();
        }
#else
        logger_->Error("CameraManager was built with BUILD_WITH_ROS2 set "
                       "to \"OFF\". You shouldn't be able to get here!");
#endif
    }

    // NOTE: raw pointers to QObjects (such as camera_) are automatically
    //       cleaned up by virtue of Qt's parenting structure (passing in "this"
    //       as a constructor parameter)

    logger_->Debug("Cleaned up CameraManager");
};

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

/**
 * @brief Convenience function for checking camera status regardless of whether
 *        it was instantiated through the owned QCamera object (`camera_`) or
 *        delegated to a separate process running the ROS2 node.
 *
 * @return true Camera is running
 * @return false Camera is not initialized
 */
bool CameraManager::CameraIsActive() {
    return (use_ros2_node_) ? ros2_connected_ : camera_ != nullptr;
}

#ifdef BUILD_WITH_ROS2
/**
 * @brief TODO: documentation.
 */
void CameraManager::CheckRos2Connectivity() {
    // TODO: the following line will need to change if I ever implement
    //       multi-topic selection (e.g., RealSense has RGB, depth, infra, etc.)
    auto active_publishers = this->get_publishers_info_by_topic(kCameraRgbTopic);

    const auto was_connected = ros2_connected_;    // save previous state
    ros2_connected_ = !active_publishers.empty();  // get current state

    if (was_connected && !ros2_connected_) {  // yes -> no
        logger_->Warn("CameraManager - Lost connection to publishers on "
                      + kCameraRgbTopic);
    } else if (!was_connected && ros2_connected_) {  // no -> yes
        logger_->Info("CameraManager - Connected to publishers on "
                      + kCameraRgbTopic);
    } else if (!was_connected && !ros2_connected_) {  // no
        logger_->Debug("CameraManager - Not detecting publishers on "
                       + kCameraRgbTopic);
    }
}

/**
 * @brief Applies the current flip states to an image.
 *
 * @param input The input image to transform
 * @return The transformed image
 */
cv::Mat CameraManager::TransformImage(const cv::Mat& input) const {
    if (!flip_horizontal_ && !flip_vertical)
        return input;
    cv::Mat result;
    if (flip_horizontal_ && flip_vertical) {
        cv::flip(input, result, -1);
    } else if (flip_horizontal_) {
        cv::flip(input, result, 1);
    } else if (flip_vertical) {
        cv::flip(input, result, 0);
    }
    return result;
}

/**
 * @brief Processes RGB image frames in a Qt GUI.
 *
 * @param msg A ROS2 RGB image message (e.g., `/camera/color/image_raw`)
 */
void CameraManager::ProcessRos2Image(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    // Recording settings
    constexpr double kVideoFps = 30.0;

    try {
        // Convert to OpenCV for processing
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

        // Flip frame if needed, then convert to QVideoFrame for streaming to GUI
        cv::Mat frame = TransformImage(cv_ptr->image);
        if (video_sink_) {
            QImage qimg(frame.data, frame.cols, frame.rows, frame.step,
                        QImage::Format_RGB888);
            QVideoFrame frame(qimg);
            video_sink_->setVideoFrame(frame);
        }

        // If capture was requested, save to file
        if (capture_requested_.load()) {
            if (!cv::imwrite(image_filename_, cv_ptr->image)) {
                logger_->Error("CameraManager - Failed to save image data");
            }
            capture_requested_.store(false);
        }

        // If recording, write to video file
        if (is_recording_.load()) {
            // If this is a new recording, start the VideoWriter
            if (!video_writer_.isOpened()) {
                cv::Size frame_size(cv_ptr->image.cols, cv_ptr->image.rows);
                video_writer_.open(video_filename_, video_codec_, kVideoFps,
                                   frame_size);

                if (!video_writer_.isOpened()) {
                    logger_->Error(
                        "CameraManager - Failed to initialize VideoWriter");
                    is_recording_.store(false);
                    return;
                }
            }

            // Save the current frame
            cv::Mat bgr_frame;
            cv::cvtColor(cv_ptr->image, bgr_frame, cv::COLOR_RGB2BGR);
            video_writer_.write(bgr_frame);
        }
    } catch (cv_bridge::Exception& e) {
        logger_->Error("CameraManager - OpenCV Bridge error: "
                       + std::string(e.what()));
    }
}
#endif

//------------------------------------------------------------------------------
// !Camera Commands
//------------------------------------------------------------------------------

/**
 * @brief Starts the camera feed.
 */
void CameraManager::Start() {
    // If there is already an active connection, gracefully terminate it
    Stop();

    if (use_ros2_node_) {
#ifdef BUILD_WITH_ROS2
        // Set up image subscription
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            kCameraRgbTopic, 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                this->ProcessRos2Image(msg);
            });

        // Start timer for periodic ROS2 spinning
        // (NOTE: this is useful for triggering ROS2 callbacks without blocking
        //        the Qt event loop)
        spin_timer_ = new QTimer(this);
        connect(spin_timer_, &QTimer::timeout, [this]() {
            if (rclcpp::ok()) {
                rclcpp::spin_some(this->get_node_base_interface());
            }
        });
        spin_timer_->start(10);  // ms (100 Hz)

        // Set up periodic connectivity check
        connectivity_timer_ = new QTimer(this);
        connect(connectivity_timer_, &QTimer::timeout,
                [this]() { CheckRos2Connectivity(); });
        connectivity_timer_->start(5000);  // ms (0.2 Hz)
#else
        logger_->Error("CameraManager was built with BUILD_WITH_ROS2 set "
                       "to \"OFF\". You shouldn't be able to get here!");
#endif
    } else {
        if (camera_ == nullptr) {
            logger_->Error("CameraManager - Cannot start camera; QCamera not "
                           "initialized!");
            return;
        }

        // Directly start the QCamera object
        camera_->start();
    }
}

/**
 * @brief Stops the camera feed.
 */
void CameraManager::Stop() {
    if (use_ros2_node_) {
#ifdef BUILD_WITH_ROS2
        // Gracefully stop the timers
        if (spin_timer_ != nullptr) {
            spin_timer_->stop();
            spin_timer_->deleteLater();
            spin_timer_ = nullptr;

            logger_->Debug("CameraManager - ROS2 spin timer stopped");
        }

        if (connectivity_timer_ != nullptr) {
            connectivity_timer_->stop();
            connectivity_timer_->deleteLater();
            connectivity_timer_ = nullptr;

            logger_->Debug("CameraManager - ROS2 connectivity timer stopped");
        }
#else
        logger_->Error("CameraManager was built with BUILD_WITH_ROS2 set "
                       "to \"OFF\". You shouldn't be able to get here!");
#endif
    } else {
        // Directly stop the QCamera object
        if (camera_ != nullptr) {
            camera_->stop();
        }
    }
}

/**
 * @brief Saves a still image of the current frame.
 */
void CameraManager::Capture() {
    if (!CameraIsActive()) {
        logger_->Error("CameraManager - Cannot capture image; camera not "
                       "initialized!");
        return;
    }

    image_filename_ = output_dir_ + "img/" + GetDateTimeStr() + ".jpg";

    // Capture a single frame
    if (use_ros2_node_) {
#ifdef BUILD_WITH_ROS2
        // NOTE: this does not immediately save the current frame! Rather, it
        //       sets a bool that is checked by the ROS2 subscriber callback
        //       ProcessRos2Image(), which then saves the latest frame.
        //       Although this ensures the "freshness" of the image data, it
        //       slightly obfuscates the capture logic.
        capture_requested_.store(true);
#else
        logger_->Error("CameraManager was built with BUILD_WITH_ROS2 set "
                       "to \"OFF\". You shouldn't be able to get here!");
        return;
#endif
    } else {
        auto id = capture_->captureToFile(
            QString::fromStdString(image_filename_));
        if (id == -1) {
            logger_->Error("CameraManager - "
                           + capture_->errorString().toStdString());
            return;
        }
    }

    logger_->Info("CameraManager - Saving image data to " + image_filename_);
}

/**
 * @brief Toggles video recording of the current feed.
 *
 * @return true Recording is active
 * @return false Not recording
 */
bool CameraManager::Record() {
    if (!CameraIsActive()) {
        logger_->Error(
            "CameraManager - Cannot record video; camera not initialized!");
        return false;
    }

    if (!is_recording_.load()) {
        video_filename_ = output_dir_ + "vid/" + GetDateTimeStr() + ".mp4";

        // Start recording
        if (use_ros2_node_) {
#ifdef BUILD_WITH_ROS2
            // NOTE: similarly to Capture(), we simply need to set a bool that
            //       is checked by the subscriber callback ProcessRos2Image().
            //       Since, in Record(), this bool is also used by the non-ROS2
            //       branch, it is set after this conditional block. In other
            //       words, this use_ros2_node_ conditional serves no function
            //       but visual symmetry with the "Stop recording" block.
#else
            logger_->Error("CameraManager was built with BUILD_WITH_ROS2 set "
                           "to \"OFF\". You shouldn't be able to get here!");
            return false;
#endif
        } else {
            recorder_->setOutputLocation(
                QUrl::fromLocalFile(QString::fromStdString(video_filename_)));
            recorder_->record();

            if (recorder_->recorderState() != QMediaRecorder::RecordingState) {
                logger_->Error("CameraManager - "
                               + recorder_->errorString().toStdString());
                return false;
            }
        }

        // Set common members
        is_recording_.store(true);

    } else {
        // Stop recording
        if (use_ros2_node_) {
#ifdef BUILD_WITH_ROS2
            if (video_writer_.isOpened()) {
                video_writer_.release();
            }
#else
            logger_->Error("CameraManager was built with BUILD_WITH_ROS2 set "
                           "to \"OFF\". You shouldn't be able to get here!");
            return false;
#endif
        } else {
            recorder_->stop();
        }

        logger_->Info("CameraManager - Saving video recording to "
                      + video_filename_);

        // Reset common members
        video_filename_.clear();
        is_recording_.store(false);
    }

    return is_recording_.load();
}
