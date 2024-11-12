/******************************************************************************
 * @file   open_epos_window.h
 * @brief  Open EPOS device window header file.
 *
 * @author brice.c.aa
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
class OpenEPOSWindow;
}  // namespace Ui

/**
 * @brief The "Open EPOS device" window, a sub-window of MainWindow.
 *        Used to retrieve the information required for the call to
 *        `VCS_OpenDevice()`.
 */
class OpenEPOSWindow : public QDialog {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit OpenEPOSWindow(QWidget* parent = nullptr);
    ~OpenEPOSWindow() override;

    // --- Getters & Setters ---

    void* GetEPOSHandle();  // has to be public so MainWindow can access it

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Main Window ---

    void on_cb_device_name_textActivated(const QString& sel);
    void on_cb_protocol_name_textActivated(const QString& sel);
    void on_cb_interface_name_textActivated(const QString& sel);
    void on_cb_port_name_textActivated(const QString& sel);
    void on_cb_baud_rate_textActivated(const QString& sel);

    void on_pb_connect_clicked();
    void on_pb_cancel_clicked();

  private:
    // NOLINTEND
    // --- Data Members ---
    Ui::OpenEPOSWindow* ui_;

    std::string device_name_;
    std::string protocol_name_;
    std::string interface_name_;
    std::string port_name_;
    uint baud_rate_;

    void* handle_{nullptr};  // void* are dangerous, but Maxon handles use them
};
