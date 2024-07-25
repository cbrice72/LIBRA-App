/******************************************************************************
 * @file   camera_manager.cpp
 * @brief  QMediaCaptureSession convenience class implementation file.
 *
 * @author brice.c.aa
 * @date   2024/7/25
 ******************************************************************************/

// Related Header
#include "camera_manager.h"
// C++ Standard Library Headers
//   (none)
// Other Libraries' Headers
//   Qt
#include <QDateTime>

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Camera Commands
 */

/**
 * @brief Standard constructor.
 *
 * @param name Camera descriptor; will be used to find the device
 * @param viewfinder The QVideoWidget object to display the camera feeed in
 * @param parent Owning Qt widget (default: nullptr)
 */
CameraManager::CameraManager(const std::string& name, QVideoWidget* viewfinder,
                             QObject* parent = nullptr)
    : QObject(parent), camera_(nullptr), name_(QString::fromStdString(name)),
      viewfinder_(viewfinder) {
    // Find requested camera
    const auto cameras = QMediaDevices::videoInputs();
    for (const auto& camera_device : cameras) {
        qDebug() << "[INFO] found camera: " << camera_device.description();
        if (camera_device.description() == name_) {
            camera_ = new QCamera(camera_device);
            break;
        }
    }

    // Set up Qt multimedia objects
    if (camera_ != nullptr) {
        // Initialize central media capture object
        session_.setCamera(camera_);

        // Register video output to existing UI VideoWidget
        session_.setVideoOutput(viewfinder_);
        viewfinder_->show();  // enable the video feed

        // Register image capture object
        capture_ = new QImageCapture;
        session_.setImageCapture(capture_);

        // Register video recorder object
        recorder_ = new QMediaRecorder(camera_);
        session_.setRecorder(recorder_);

        // Start camera
        camera_->start();

        // Set output location for image captures
        // TODO

        // Set output format for video
        QMediaFormat format(QMediaFormat::MPEG4);
        format.setVideoCodec(QMediaFormat::VideoCodec::H264);
        format.setAudioCodec(QMediaFormat::AudioCodec::MP3);
        recorder_->setMediaFormat(format);
    } else {
        qDebug() << "[ERROR] Camera " << name_ << " not found!";
    }
};

/**
 * @brief Standard desctructor.
 */
CameraManager::~CameraManager() {
    // Stop camera
    Stop();  // checks if camera_ is null

    // Clean up raw pointers
    delete camera_;
    delete capture_;
    delete recorder_;
};

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

namespace {

/**
 * @brief Provides a formatted string of the current date and time.
 *
 * @return std::string Formatted as "yyyy-MM-ddTHH:mm:ss.zzz"
 */
std::string GetDateTimeString() {
    return QDateTime::currentDateTime().toString(Qt::ISODateWithMs).toStdString();
}

}  // namespace

//------------------------------------------------------------------------------
// !Camera Commands
//------------------------------------------------------------------------------

/**
 * @brief Starts the camera feed.
 */
void CameraManager::Start() {
    if (camera_ != nullptr) {
        camera_->start();
    } else {
        qDebug() << "[ERROR] Cannot start camera; camera not initialized!";
    }
}

/**
 * @brief Stops the camera feed.
 */
void CameraManager::Stop() {
    if (camera_ != nullptr) {
        camera_->stop();
    }
}

/**
 * @brief Saves a still image of the current frame.
 */
void CameraManager::Capture() {
    if (camera_ != nullptr) {
        capture_->captureToFile(
            QString::fromStdString(GetDateTimeString() + "_img.jpg"));
    } else {
        qDebug() << "[ERROR] Cannot capture image; camera not initialized!";
    }
}

/**
 * @brief Toggles video recording of the current feed.
 *
 * @return true if recording is active, false otherwise
 */
bool CameraManager::Record() {
    if (camera_ == nullptr) {
        qDebug() << "[ERROR] Cannot capture video; camera not initialized!";
        return false;
    }

    if (!is_recording_) {
        recorder_->setOutputLocation(QUrl::fromLocalFile(
            QString::fromStdString(GetDateTimeString() + "_video.mp4")));
        recorder_->record();
    } else {
        recorder_->stop();
    }

    return is_recording_ = !is_recording_;
}
