/******************************************************************************
 * @file   main_window.h
 * @brief  QMediaCaptureSession convenience class header file.
 *
 * @author brice.c.aa
 * @date   2024/2/22
 ******************************************************************************/

// C++ Standard Library Headers
#include <unordered_map>
// Other Libraries' Headers
//   Qt
#include <QtMultimedia>
#include <QVideoWidget>
// Project Headers
//   (none)

#pragma once

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
