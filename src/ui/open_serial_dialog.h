/******************************************************************************
 * @file   open_serial_dialog.h
 * @brief  Serial (USB) selection dialog box header file.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QDialog>          // Qt::Widgets
#include <QSerialPortInfo>  // Qt::SerialPort

// Project Headers
//   (none)

namespace Ui {  // NOLINT: Qt-generated
class OpenSerialDialog;
}  // namespace Ui

/**
 * @brief A dialog for retrieving the information required for the QtSerialPort
 *        library call `setPortName()` (necessary for `QtSerialPort::open()`).
 */
class OpenSerialDialog : public QDialog {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit OpenSerialDialog(QWidget* parent = nullptr,
                              QString device_to_prompt = "Serial");
    ~OpenSerialDialog();

    QString GetSelectedPortName();  // has to be public so MainWindow can access it

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Main Window ---

    void on_cb_ports_currentIndexChanged(int sel);

  private:
    // NOLINTEND
    // --- Helper Functions ---

    void PopulatePorts();

    // --- Data Members ---

    Ui::OpenSerialDialog* ui_{nullptr};

    QList<QSerialPortInfo> valid_ports_;
};
