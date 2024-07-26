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
#include <QDir>

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Camera Commands
 */

/**
 * @brief Standard constructor.
 *
 * @param id Camera location (e.g., `/dev/video0`); will be used to find the device
 * @param viewfinder The QVideoWidget object to display the camera feeed in
 * @param parent Owning Qt widget (default: nullptr)
 */
CameraManager::CameraManager(const QString& id, QVideoWidget* viewfinder,
                             QObject* parent = nullptr)
    : QObject(parent), id_(id), viewfinder_(viewfinder), camera_(nullptr),
      output_dir_(QDir::currentPath().toStdString() + "/") {
    // Find requested camera
    const auto cameras = QMediaDevices::videoInputs();
    for (const auto& camera_device : cameras) {
        if (camera_device.id() == id_) {
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
        // format.setAudioCodec(QMediaFormat::AudioCodec::MP3);
        recorder_->setMediaFormat(format);
    } else {
        qDebug() << "[ERROR] Camera " << id_ << " not found!";
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
        auto filename = QString::fromStdString(output_dir_ + "img/"
                                               + GetDateTimeString() + ".jpg");
        capture_->captureToFile(filename);
        qDebug() << "[INFO] Saved image data to " << filename;
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
    if (!is_recording_) {
        video_filename_ = QString::fromStdString(
            output_dir_ + "vid/" + GetDateTimeString() + ".mp4");
        recorder_->setOutputLocation(QUrl::fromLocalFile(video_filename_));
        recorder_->record();
    } else {
        recorder_->stop();
        qDebug() << "[INFO] Saved video recording to " << video_filename_;
        video_filename_.clear();
    }

    return is_recording_ = !is_recording_;
}
