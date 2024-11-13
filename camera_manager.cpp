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
#include <QDateTime>  // Qt
#include <QDir>       // Qt

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
    : QObject(parent), id_(id), camera_(nullptr),
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

        // Register image capture object
        capture_ = new QImageCapture;
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
            qDebug()
                << "[WARN] Invalid camera format detected; applying defaults.";

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
                qDebug()
                    << "[WARN] This camera doesn't seem to support a Jpeg 720p "
                       "stream. Defaulting to the first supported format.";
                camera_->setCameraFormat(supported_formats.first());
            }
        }

        // Start camera
        camera_->start();

        // Set output format for video
        QMediaFormat format(QMediaFormat::MPEG4);  // init with FileFormat
        // format.setAudioCodec(QMediaFormat::AudioCodec::MP3);
        format.setVideoCodec(QMediaFormat::VideoCodec::MotionJPEG);

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
    delete recorder_;
    delete capture_;
    delete camera_;
};

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

namespace {

/**
 * @brief Provides a filename-safe string of the current date and time.
 *
 * @return std::string Formatted as "yyyy-MM-ddTHH-mm-ss"
 */
std::string GetDateTimeStr() {
    auto dts = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    dts.replace(":", "-");         // replace colons (invalid in filenames)
    dts = dts.section('.', 0, 0);  // remove milliseconds
    return dts.toStdString();
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
        // Save image capture
        auto filename = QString::fromStdString(output_dir_ + "img/"
                                               + GetDateTimeStr() + ".jpg");
        auto id = capture_->captureToFile(filename);

        // Error checking
        if (id != -1) {
            qDebug() << "[INFO] Saved image data to" << filename;
        } else {
            qDebug() << "[ERROR]" << capture_->errorString();
        }
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
        // Start recording
        video_filename_ = QString::fromStdString(output_dir_ + "vid/"
                                                 + GetDateTimeStr() + ".mp4");
        recorder_->setOutputLocation(QUrl::fromLocalFile(video_filename_));
        recorder_->record();

        // Error checking
        auto status = recorder_->recorderState();
        if (status == QMediaRecorder::RecordingState) {
            is_recording_ = true;
        } else {
            qDebug() << "[ERROR]" << recorder_->errorString();
        }
    } else {
        // Stop recording and reset
        recorder_->stop();
        qDebug() << "[INFO] Saved video recording to" << video_filename_;
        video_filename_.clear();
        is_recording_ = false;
    }

    return is_recording_;
}
