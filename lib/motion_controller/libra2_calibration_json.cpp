/******************************************************************************
 * @file   libra2_calibration_json.cpp
 * @brief  Saves/loads LIBRA-II calibration records as JSON; implementation
 *         file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "libra2_calibration_json.h"

// C++ Standard Library Headers
#include <optional>
#include <stdexcept>

// Other Library Headers
#include <QDateTime>      // Qt::Core
#include <QDir>           // Qt::Core
#include <QFile>          // Qt::Core
#include <QFileInfo>      // Qt::Core
#include <QJsonArray>     // Qt::Core
#include <QJsonDocument>  // Qt::Core
#include <QJsonObject>    // Qt::Core
#include <QSaveFile>      // Qt::Core

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !File I/O
 */

// File Format

constexpr const char* kFormatName = "libra2-calibration";
constexpr int kFormatVersion = 1;

namespace Motion {

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

/**
 * @brief Reads a number from a JSON object.
 *
 * @param obj Object to read from
 * @param key Key to read
 * @param context Where `obj` sits in the file (for error messages)
 *
 * @throws std::runtime_error If missing or non-numeric
 */
double ReadNumber(const QJsonObject& obj, const QString& key,
                  const std::string& context) {
    const QJsonValue value = obj.value(key);
    if (!value.isDouble()) {
        throw std::runtime_error("missing or non-numeric \"" + context
                                 + key.toStdString() + "\"");
    }

    return value.toDouble();
}

/**
 * @brief Reads an object from a JSON object.
 *
 * @param obj Object to read from
 * @param key Key to read
 * @param context Where `obj` sits in the file (for error messages)
 *
 * @throws std::runtime_error If missing or not an object
 */
QJsonObject ReadObject(const QJsonObject& obj, const QString& key,
                       const std::string& context) {
    const QJsonValue value = obj.value(key);
    if (!value.isObject()) {
        throw std::runtime_error("missing or non-object \"" + context
                                 + key.toStdString() + "\"");
    }

    return value.toObject();
}

/**
 * @brief Reads an optional object from a JSON object.
 *
 * @param obj Object to read from
 * @param key Key to read
 * @param context Where `obj` sits in the file (for error messages)
 *
 * @note Returns none if the key is missing or null.
 *
 * @throws std::runtime_error If present but not an object
 */
std::optional<QJsonObject> ReadOptionalObject(const QJsonObject& obj,
                                              const QString& key,
                                              const std::string& context) {
    const QJsonValue value = obj.value(key);
    if (value.isNull() || value.isUndefined()) {
        return std::nullopt;
    }

    if (!value.isObject()) {
        throw std::runtime_error("\"" + context + key.toStdString()
                                 + "\" must be an object or null");
    }

    return value.toObject();
}

/**
 * @brief Converts a CW model to JSON.
 *
 * @param params CW model parameters
 */
QJsonObject CwModelToJson(const CwModelParams& params) {
    QJsonObject obj;
    obj["pivot_dist_m"] = params.pivot_dist;
    obj["pivot_slope_m_per_l"] = params.pivot_slope;
    obj["mean_sq_dist_m2"] = params.mean_sq_dist;
    obj["mean_sq_dist_slope_m2_per_l"] = params.mean_sq_dist_slope;
    obj["hang_offset_m"] = params.hang_offset;
    obj["capacity_l"] = params.capacity;

    return obj;
}

/**
 * @brief Reads a CW model from JSON.
 *
 * @param obj JSON object
 * @param context Where `obj` sits in the file (for error messages)
 *
 * @note The mean r_bag² terms are optional and stay 0 if missing. They only
 *       affect CW inertia, not the gravity terms an arm model is checked against.
 *
 * @throws std::runtime_error If a value is missing
 */
CwModelParams CwModelFromJson(const QJsonObject& obj,
                              const std::string& context) {
    CwModelParams params;
    params.pivot_dist = ReadNumber(obj, "pivot_dist_m", context);
    params.pivot_slope = ReadNumber(obj, "pivot_slope_m_per_l", context);
    params.hang_offset = ReadNumber(obj, "hang_offset_m", context);
    params.capacity = ReadNumber(obj, "capacity_l", context);

    // Optional
    if (obj.contains("mean_sq_dist_m2")) {
        params.mean_sq_dist = ReadNumber(obj, "mean_sq_dist_m2", context);
    }
    if (obj.contains("mean_sq_dist_slope_m2_per_l")) {
        params.mean_sq_dist_slope = ReadNumber(obj,
                                               "mean_sq_dist_slope_m2_per_l",
                                               context);
    }

    return params;
}

/**
 * @brief Converts an arm model to JSON.
 *
 * @param arm Arm model record
 */
QJsonObject ArmModelToJson(const ArmModel& arm) {
    QJsonObject obj;
    obj["source"] = QString::fromStdString(arm.source);
    obj["level_torque_nm"] = arm.level_torque;
    obj["hang_torque_nm"] = arm.hang_torque;
    obj["cw_model"] = CwModelToJson(arm.cw_model);

    return obj;
}

/**
 * @brief Reads an arm model from JSON.
 *
 * @param obj JSON object
 * @param context Where `obj` sits in the file (for error messages)
 *
 * @throws std::runtime_error If a value is missing
 */
ArmModel ArmModelFromJson(const QJsonObject& obj, const std::string& context) {
    ArmModel arm;

    // How the model was computed
    arm.source = obj.value("source").toString().toStdString();

    // Model data
    arm.level_torque = ReadNumber(obj, "level_torque_nm", context);
    arm.hang_torque = ReadNumber(obj, "hang_torque_nm", context);
    arm.cw_model = CwModelFromJson(ReadObject(obj, "cw_model", context),
                                   context + "cw_model.");

    return arm;
}

/**
 * @brief Converts a friction result to JSON.
 *
 * @param friction Friction result
 */
QJsonObject FrictionToJson(const FrictionModel& friction) {
    // Raw data
    QJsonArray points;
    for (const auto& p : friction.points) {
        QJsonObject point;
        point["speed_rad_s"] = p.vel;
        point["drag_nm"] = p.drag;
        point["spread_nm"] = p.spread;
        point["n_bins"] = p.n_bins;
        point["steady_fraction"] = p.steady_fraction;

        QJsonArray bins;
        for (const auto& bin : p.bins) {
            QJsonObject b;
            b["angle_rad"] = bin.angle;
            b["drag_nm"] = bin.drag;
            b["holding_nm"] = bin.holding;
            b["n_up"] = bin.n_up;
            b["n_down"] = bin.n_down;
            bins.append(b);
        }
        point["bins"] = bins;

        points.append(point);
    }

    // Model data
    QJsonObject obj;
    obj["coulomb_nm"] = friction.coulomb;
    obj["viscous_nm_s_per_rad"] = friction.viscous;
    obj["residual_rms_nm"] = friction.residual_rms;
    obj["angle_lo_rad"] = friction.angle_lo;
    obj["angle_hi_rad"] = friction.angle_hi;
    obj["volume_l"] = friction.volume;
    obj["points"] = points;

    return obj;
}

/**
 * @brief Reads a friction result from JSON.
 *
 * @param obj JSON object
 * @param context Where `obj` sits in the file (for error messages)
 *
 * @throws std::runtime_error If a value is missing or negative
 */
FrictionModel FrictionFromJson(const QJsonObject& obj,
                               const std::string& context) {
    FrictionModel friction;

    // Model data
    friction.coulomb = ReadNumber(obj, "coulomb_nm", context);
    friction.viscous = ReadNumber(obj, "viscous_nm_s_per_rad", context);
    friction.residual_rms = ReadNumber(obj, "residual_rms_nm", context);
    friction.angle_lo = ReadNumber(obj, "angle_lo_rad", context);
    friction.angle_hi = ReadNumber(obj, "angle_hi_rad", context);
    friction.volume = ReadNumber(obj, "volume_l", context);

    if (friction.coulomb < 0.0 || friction.viscous < 0.0) {
        throw std::runtime_error("\"" + context + "coulomb_nm\" and \"" + context
                                 + "viscous_nm_s_per_rad\" must not be "
                                   "negative");
    }

    // Raw data
    const auto points = obj.value("points").toArray();
    for (int i = 0; i < points.size(); ++i) {
        const std::string point_context = context + "points["
                                          + std::to_string(i) + "].";
        const auto p = points.at(i).toObject();

        FrictionPoint point;
        point.vel = ReadNumber(p, "speed_rad_s", point_context);
        point.drag = ReadNumber(p, "drag_nm", point_context);
        point.spread = ReadNumber(p, "spread_nm", point_context);
        point.n_bins = static_cast<int>(ReadNumber(p, "n_bins", point_context));

        // Optional
        point.steady_fraction = p.value("steady_fraction").toDouble(0.0);

        const auto bins = p.value("bins").toArray();
        for (int j = 0; j < bins.size(); ++j) {
            const std::string bin_context = point_context + "bins["
                                            + std::to_string(j) + "].";
            const auto b = bins.at(j).toObject();

            FrictionBin bin;
            bin.angle = ReadNumber(b, "angle_rad", bin_context);
            bin.drag = ReadNumber(b, "drag_nm", bin_context);
            bin.holding = ReadNumber(b, "holding_nm", bin_context);
            bin.n_up = static_cast<int>(ReadNumber(b, "n_up", bin_context));
            bin.n_down = static_cast<int>(ReadNumber(b, "n_down", bin_context));
            point.bins.push_back(bin);
        }

        friction.points.push_back(point);
    }

    return friction;
}

}  // namespace

//------------------------------------------------------------------------------
// !File I/O
//------------------------------------------------------------------------------

/**
 * @brief Writes a calibration record to a JSON file.
 *
 * @param record Record to save
 * @param file_path File to write (overwrites if exists)
 *
 * @throws std::runtime_error If saving failed
 */
void SaveCalibrationJson(const CalibrationRecord& record,
                         const std::string& file_path) {
    QJsonObject root;

    // For validation purposes
    root["format"] = kFormatName;
    root["format_version"] = kFormatVersion;

    // Metadata
    root["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    root["extension_mm"] = record.extension ? QJsonValue(*record.extension)
                                            : QJsonValue(QJsonValue::Null);
    root["note"] = QString::fromStdString(record.note);

    // Models
    root["cw_model"] = CwModelToJson(record.cw_model);
    root["arm_inertia_kg_m2"] = record.arm_inertia;
    root["arm_model"] = record.arm_model
                            ? QJsonValue(ArmModelToJson(*record.arm_model))
                            : QJsonValue(QJsonValue::Null);
    root["friction"] = record.friction
                           ? QJsonValue(FrictionToJson(*record.friction))
                           : QJsonValue(QJsonValue::Null);

    // Write to file
    const auto path = QString::fromStdString(file_path);
    const QString dir = QFileInfo(path).absolutePath();
    if (!QDir().mkpath(dir)) {
        throw std::runtime_error("can't create folder " + dir.toStdString());
    }

    // (NOTE: QSaveFile only replaces the old file once writing finishes)
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        throw std::runtime_error(file.errorString().toStdString());
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if (!file.commit()) {
        throw std::runtime_error(file.errorString().toStdString());
    }
}

/**
 * @brief Reads a calibration record from a JSON file.
 *
 * @param file_path File to read
 *
 * @throws std::runtime_error If the file can't be read or isn't a valid
 *         calibration record
 */
CalibrationRecord LoadCalibrationJson(const std::string& file_path) {
    // Open and parse the file
    QFile file(QString::fromStdString(file_path));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        throw std::runtime_error(file.errorString().toStdString());
    }

    QJsonParseError parse_error;
    const auto doc = QJsonDocument::fromJson(file.readAll(), &parse_error);
    if (doc.isNull() || !doc.isObject()) {
        throw std::runtime_error(
            "not valid JSON (" + parse_error.errorString().toStdString() + ")");
    }
    const QJsonObject root = doc.object();

    // Validation
    if (root.value("format").toString() != kFormatName) {
        throw std::runtime_error("not a LIBRA-II calibration file");
    }
    const int version = root.value("format_version").toInt(-1);
    if (version != kFormatVersion) {
        throw std::runtime_error("unsupported format version "
                                 + std::to_string(version));
    }

    // Metadata
    CalibrationRecord record;
    record.timestamp = root.value("timestamp").toString().toStdString();
    if (root.value("extension_mm").isDouble()) {
        record.extension = root.value("extension_mm").toDouble();
    }
    record.note = root.value("note").toString().toStdString();

    // Models
    record.cw_model = CwModelFromJson(ReadObject(root, "cw_model", ""),
                                      "cw_model.");
    record.arm_inertia = ReadNumber(root, "arm_inertia_kg_m2", "");

    if (const auto arm_obj = ReadOptionalObject(root, "arm_model", "")) {
        record.arm_model = ArmModelFromJson(*arm_obj, "arm_model.");
    }
    if (const auto friction_obj = ReadOptionalObject(root, "friction", "")) {
        record.friction = FrictionFromJson(*friction_obj, "friction.");
    }

    return record;
}

}  // namespace Motion
