/******************************************************************************
 * @file   libra2_calibration_file.cpp
 * @brief  Saves/loads LIBRA-II calibration records as JSON; implementation
 *         file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "libra2_calibration_file.h"

// C++ Standard Library Headers
#include <utility>

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
 * @param[out] out Value read
 * @param[out] error Reason, if missing or NaN
 */
bool ReadNumber(const QJsonObject& obj, const QString& key,
                const std::string& context, double& out, std::string& error) {
    const QJsonValue value = obj.value(key);

    if (!value.isDouble()) {
        error = "missing or non-numeric \"" + context + key.toStdString()
                + "\"";
        return false;
    }

    out = value.toDouble();
    return true;
}

/**
 * @brief Reads an object from a JSON object.
 *
 * @param obj Object to read from
 * @param key Key to read
 * @param context Where `obj` sits in the file (for error messages)
 * @param[out] out Object read
 * @param[out] error Reason, if missing or not an object
 */
bool ReadObject(const QJsonObject& obj, const QString& key,
                const std::string& context, QJsonObject& out,
                std::string& error) {
    const QJsonValue value = obj.value(key);

    if (!value.isObject()) {
        error = "missing or non-object \"" + context + key.toStdString() + "\"";
        return false;
    }

    out = value.toObject();
    return true;
}

/**
 * @brief Reads an optional object from a JSON object.
 *
 * @param obj Object to read from
 * @param key Key to read
 * @param context Where `obj` sits in the file (for error messages)
 * @param[out] out Object read, if any
 * @param[out] error Reason, if present but not an object
 */
bool ReadOptionalObject(const QJsonObject& obj, const QString& key,
                        const std::string& context,
                        std::optional<QJsonObject>& out, std::string& error) {
    const QJsonValue value = obj.value(key);
    if (value.isNull() || value.isUndefined()) {
        out.reset();
        return true;
    }

    if (!value.isObject()) {
        error = "\"" + context + key.toStdString()
                + "\" must be an object or null";
        return false;
    }

    out = value.toObject();
    return true;
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
    obj["hang_offset_m"] = params.hang_offset;
    obj["capacity_l"] = params.capacity;

    return obj;
}

/**
 * @brief Reads a CW model from JSON.
 *
 * @param obj JSON object
 * @param context Where `obj` sits in the file (for error messages)
 * @param[out] params CW model parameters
 * @param[out] error Reason, if a value is missing
 */
bool CwModelFromJson(const QJsonObject& obj, const std::string& context,
                     CwModelParams& params, std::string& error) {
    return ReadNumber(obj, "pivot_dist_m", context, params.pivot_dist, error)
           && ReadNumber(obj, "pivot_slope_m_per_l", context,
                         params.pivot_slope, error)
           && ReadNumber(obj, "hang_offset_m", context, params.hang_offset,
                         error)
           && ReadNumber(obj, "capacity_l", context, params.capacity, error);
}

/**
 * @brief Converts an arm model to JSON.
 *
 * @param arm Arm model record
 */
QJsonObject ArmModelToJson(const ArmModel& arm) {
    // Raw data
    QJsonArray samples;
    for (const auto& s : arm.samples) {
        QJsonObject sample;
        sample["theta_rad"] = s.theta;
        sample["torque_nm"] = s.torque;
        sample["volume_l"] = s.volume;
        samples.append(sample);
    }

    // Model data
    QJsonObject obj;
    obj["source"] = QString::fromStdString(arm.source);
    obj["level_torque_nm"] = arm.level_torque;
    obj["hang_torque_nm"] = arm.hang_torque;
    obj["cw_model"] = CwModelToJson(arm.cw_model);
    obj["samples"] = samples;

    return obj;
}

/**
 * @brief Reads an arm model from JSON.
 *
 * @param obj JSON object
 * @param context Where `obj` sits in the file (for error messages)
 * @param[out] arm Arm model record
 * @param[out] error Reason, if a value is missing
 */
bool ArmModelFromJson(const QJsonObject& obj, const std::string& context,
                      ArmModel& arm, std::string& error) {
    // How the model was computed
    arm.source = obj.value("source").toString().toStdString();

    // Model data
    QJsonObject cw_obj;
    if (!ReadNumber(obj, "level_torque_nm", context, arm.level_torque, error)
        || !ReadNumber(obj, "hang_torque_nm", context, arm.hang_torque, error)
        || !ReadObject(obj, "cw_model", context, cw_obj, error)
        || !CwModelFromJson(cw_obj, context + "cw_model.", arm.cw_model,
                            error)) {
        return false;
    }

    // Raw data
    arm.samples.clear();
    const QJsonArray samples = obj.value("samples").toArray();
    for (int i = 0; i < samples.size(); ++i) {
        const std::string sample_context = context + "samples["
                                           + std::to_string(i) + "].";
        const QJsonObject s = samples.at(i).toObject();
        RestSample sample;

        if (!ReadNumber(s, "theta_rad", sample_context, sample.theta, error)
            || !ReadNumber(s, "torque_nm", sample_context, sample.torque, error)
            || !ReadNumber(s, "volume_l", sample_context, sample.volume,
                           error)) {
            return false;
        }

        arm.samples.push_back(sample);
    }

    return true;
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
        point["speed_rad_s"] = p.speed;
        point["drag_nm"] = p.drag;
        point["spread_nm"] = p.spread;
        point["n_bins"] = p.n_bins;
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
 * @param[out] friction Friction result
 * @param[out] error Reason, if a value is missing or negative
 */
bool FrictionFromJson(const QJsonObject& obj, const std::string& context,
                      FrictionModel& friction, std::string& error) {
    // Model data
    if (!ReadNumber(obj, "coulomb_nm", context, friction.coulomb, error)
        || !ReadNumber(obj, "viscous_nm_s_per_rad", context, friction.viscous,
                       error)
        || !ReadNumber(obj, "residual_rms_nm", context, friction.residual_rms,
                       error)
        || !ReadNumber(obj, "angle_lo_rad", context, friction.angle_lo, error)
        || !ReadNumber(obj, "angle_hi_rad", context, friction.angle_hi, error)
        || !ReadNumber(obj, "volume_l", context, friction.volume, error)) {
        return false;
    }

    if (friction.coulomb < 0.0 || friction.viscous < 0.0) {
        error = "\"" + context + "coulomb_nm\" and \"" + context
                + "viscous_nm_s_per_rad\" must not be negative";
        return false;
    }

    // Raw data
    friction.points.clear();
    const QJsonArray points = obj.value("points").toArray();
    for (int i = 0; i < points.size(); ++i) {
        const std::string point_context = context + "points["
                                          + std::to_string(i) + "].";
        const QJsonObject p = points.at(i).toObject();
        FrictionPoint point;
        double n_bins = 0.0;
        if (!ReadNumber(p, "speed_rad_s", point_context, point.speed, error)
            || !ReadNumber(p, "drag_nm", point_context, point.drag, error)
            || !ReadNumber(p, "spread_nm", point_context, point.spread, error)
            || !ReadNumber(p, "n_bins", point_context, n_bins, error)) {
            return false;
        }
        point.n_bins = static_cast<int>(n_bins);
        friction.points.push_back(point);
    }
    return true;
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
 * @param[out] error Reason, if saving failed
 */
bool SaveCalibrationFile(const CalibrationRecord& record,
                         const std::string& file_path, std::string& error) {
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
    const QString path = QString::fromStdString(file_path);
    const QString dir = QFileInfo(path).absolutePath();
    if (!QDir().mkpath(dir)) {
        error = "can't create folder " + dir.toStdString();
        return false;
    }

    // (NOTE: QSaveFile only replaces the old file once writing finishes)
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        error = file.errorString().toStdString();
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if (!file.commit()) {
        error = file.errorString().toStdString();
        return false;
    }

    return true;
}

/**
 * @brief Reads a calibration record from a JSON file.
 *
 * @param file_path File to read
 * @param[out] error Reason, if loading failed
 *
 * @note Returns none on failure.
 */
std::optional<CalibrationRecord> LoadCalibrationFile(
    const std::string& file_path, std::string& error) {
    // Open and parse the file
    QFile file(QString::fromStdString(file_path));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        error = file.errorString().toStdString();
        return std::nullopt;
    }

    QJsonParseError parse_error;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(),
                                                      &parse_error);
    if (doc.isNull() || !doc.isObject()) {
        error = "not valid JSON (" + parse_error.errorString().toStdString()
                + ")";
        return std::nullopt;
    }
    const QJsonObject root = doc.object();

    // Validation
    if (root.value("format").toString() != kFormatName) {
        error = "not a LIBRA-II calibration file";
        return std::nullopt;
    }
    const int version = root.value("format_version").toInt(-1);
    if (version != kFormatVersion) {
        error = "unsupported format version " + std::to_string(version);
        return std::nullopt;
    }

    // Metadata
    CalibrationRecord record;
    record.timestamp = root.value("timestamp").toString().toStdString();
    if (root.value("extension_mm").isDouble()) {
        record.extension = root.value("extension_mm").toDouble();
    }
    record.note = root.value("note").toString().toStdString();

    // Models
    QJsonObject cw_obj;
    if (!ReadObject(root, "cw_model", "", cw_obj, error)
        || !CwModelFromJson(cw_obj, "cw_model.", record.cw_model, error)
        || !ReadNumber(root, "arm_inertia_kg_m2", "", record.arm_inertia,
                       error)) {
        return std::nullopt;
    }

    std::optional<QJsonObject> arm_obj;
    std::optional<QJsonObject> friction_obj;
    if (!ReadOptionalObject(root, "arm_model", "", arm_obj, error)
        || !ReadOptionalObject(root, "friction", "", friction_obj, error)) {
        return std::nullopt;
    }

    if (arm_obj) {
        ArmModel arm;
        if (!ArmModelFromJson(*arm_obj, "arm_model.", arm, error)) {
            return std::nullopt;
        }
        record.arm_model = std::move(arm);
    }
    if (friction_obj) {
        FrictionModel friction;
        if (!FrictionFromJson(*friction_obj, "friction.", friction, error)) {
            return std::nullopt;
        }
        record.friction = std::move(friction);
    }

    return record;
}

}  // namespace Motion
