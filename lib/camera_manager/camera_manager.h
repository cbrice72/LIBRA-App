/******************************************************************************
 * @file   main_window.h
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

// Project Headers
//   (none)

class CameraManager : public QObject {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    CameraManager(const QString& id, QVideoWidget* viewfinder, QObject* parent);
    ~CameraManager();

    void Start();
    void Stop();
    void Capture();
    bool Record();

  private:
    // --- Helper Functions ---

    // --- Data Members ---

    QString id_;

    std::string output_dir_;
    QCamera* camera_;

    QMediaCaptureSession session_;
    QImageCapture* capture_;
    QMediaRecorder* recorder_;

    bool is_recording_{false};
    QString video_filename_;
};
