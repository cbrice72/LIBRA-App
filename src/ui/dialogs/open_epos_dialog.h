/******************************************************************************
 * @file   open_epos_dialog.h
 * @brief  Open EPOS device window header file.
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
class OpenEposDialog;
}  // namespace Ui

/**
 * @brief Struct grouping all parameters required to connect to an EPOS device.
 */
struct EposDeviceParams {
    std::string device_name;
    std::string protocol_name;
    std::string interface_name;
    std::string port_name;
    uint baud_rate;
    // Convenience flag indicating whether all params have been set
    bool is_set{false};
};

/**
 * @brief A dialog for retrieving the information required for the EPOS (Maxon)
 *        library call `VCS_OpenDevice()`.
 */
class OpenEposDialog : public QDialog {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit OpenEposDialog(QWidget* parent = nullptr);
    ~OpenEposDialog() override;

    // --- Getters & Setters ---

    EposDeviceParams GetDeviceParams();

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Dialog Window ---

    void on_cb_device_name_textActivated(const QString& sel);
    void on_cb_protocol_name_textActivated(const QString& sel);
    void on_cb_interface_name_textActivated(const QString& sel);
    void on_cb_port_name_textActivated(const QString& sel);
    void on_cb_baud_rate_textActivated(const QString& sel);

    void on_pb_confirm_clicked();
    void on_pb_cancel_clicked();

  private:
    // NOLINTEND
    // --- Data Members ---
    Ui::OpenEposDialog* ui_{nullptr};

    EposDeviceParams params_{};
};
