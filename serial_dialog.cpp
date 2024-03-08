/******************************************************************************
 * @file   serial_dialog.cpp
 * @brief  Serial (USB) selection dialog box implementation file.
 *
 * @author brice.c.aa
 * @date   2024/3/1
 ******************************************************************************/

// Related Header
#include "serial_dialog.h"
// C++ Standard Library Headers
#include <sys/stat.h>
// Other Libraries' Headers
//   Qt
#include <QDebug>
//   Userspace Devices
#include <libudev.h>
// Project Headers
#include "ui_serial_dialog.h"

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Main Window
 */

SerialDialog::SerialDialog(QWidget* parent)
    : QDialog(parent), ui_(new Ui::SerialDialog) {
    ui_->setupUi(this);

    // Populate the combobox
    auto devices = GetDeviceList();
    if (!devices.empty()) {
        ui_->cb_serial_name->addItems(devices);
        ui_->pb_connect->setEnabled(true);
    }
}

SerialDialog::~SerialDialog() {
    delete ui_;
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief TODO
 *
 * @return
 */
bool SerialDialog::isRunningOnWSL() {
    const char* wslPath = "/run/WSL";
    struct stat buf;

    return (stat(wslPath, &buf) == 0 && S_ISDIR(buf.st_mode));
}

/**
 * @brief TODO
 *
 * @return
 */
QStringList SerialDialog::GetDeviceList() {
    QStringList device_list;

    // Create udev context
    struct udev* udev = udev_new();
    if (!udev) {
        qCritical() << "[ERROR] Unable to create udev context";
        return device_list;
    }

    // Initialize udev enumerator
    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    // Get list of devices
    struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry* entry;

    // Iterate through list
    udev_list_entry_foreach(entry, devices) {
        // Retrieve device information
        const char* path = udev_list_entry_get_name(entry);
        struct udev_device* device = udev_device_new_from_syspath(udev, path);
        const char* devnode = udev_device_get_devnode(device);

        // Only list physical connections
        if (device && strstr(devnode, "ttyUSB")) {
            device_list.append(QString::fromUtf8(devnode));
        }

        // Free udev device object before moving on to next one
        udev_device_unref(device);
    }

    // Clean up udev objects
    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    // Return device list
    device_list.removeDuplicates();  // just in case
    return device_list;
}

//------------------------------------------------------------------------------
// !Main Window
//------------------------------------------------------------------------------

/**
 * @brief Event handler for "Connect" button (single click).
 *        Opens the specified serial device.
 */
void SerialDialog::on_pb_connect_clicked() {
    // Close the dialog and return `QDialog::Accepted`
    SerialDialog::accept();
}

/**
 * @brief Give the user another way to close the dialog
 *        (in addition to the X in the menu bar).
 */
void SerialDialog::on_pb_cancel_clicked() {
    // Close the dialog and return `QDialog::Rejected`
    SerialDialog::reject();
}
