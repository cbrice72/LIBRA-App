/******************************************************************************
 * @file   serial_dialog.h
 * @brief  Serial (USB) selection dialog box header file.
 *
 * @author brice.c.aa
 * @date   2024/3/1
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)
// Other Libraries' Headers
//   Qt
#include <QDialog>

// Project Headers
//   (none)

namespace Ui {  // NOLINT: Qt-generated
class SerialDialog;
}  // namespace Ui

class SerialDialog : public QDialog {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit SerialDialog(const std::string& device_type,
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
    std::string device_type_;

    std::string selected_device_;
};
