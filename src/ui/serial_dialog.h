/******************************************************************************
 * @file   serial_dialog.h
 * @brief  Serial (USB) selection dialog box header file.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QDialog>  // Qt::Widgets

// Project Headers
//   (none)

namespace Ui {  // NOLINT: Qt-generated
class SerialDialog;
}  // namespace Ui

/**
 * @brief TODO: documentation.
 */
class SerialDialog : public QDialog {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit SerialDialog(const std::string& device_type,
                          const bool debug_mode = false,
                          QWidget* parent = nullptr);
    ~SerialDialog();

    std::string GetDeviceName();  // has to be public so MainWindow can access it

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Main Window ---

    void on_cb_serial_name_textActivated(const QString& sel);

    void on_pb_connect_clicked();
    void on_pb_cancel_clicked();

  private:
    // NOLINTEND
    // --- Helper Functions ---

    static bool isRunningOnWSL();
    QStringList GetDeviceList();

    // --- Data Members ---

    Ui::SerialDialog* ui_;
    bool debug_mode_;
    std::string device_type_;

    std::string selected_device_;
};
