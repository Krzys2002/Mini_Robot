#include "calibration/QtCalibrationProfileStore.hpp"
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>

#include <exception>

namespace micro_robot {
QtCalibrationProfileStore::QtCalibrationProfileStore(QString directory)
    : directory_(QDir(directory).absolutePath()) {}
QString QtCalibrationProfileStore::path(std::string_view id, std::uint32_t width, std::uint32_t height) const
{
    // Length-prefixed identity plus delivered size: no user-controlled path components.
    QByteArray key = QByteArray::number(static_cast<qulonglong>(id.size())) + ':';
    key.append(id.data(), static_cast<qsizetype>(id.size()));
    key += ':' + QByteArray::number(width) + 'x' + QByteArray::number(height);
    return QDir(directory_).filePath(QString::fromLatin1(
        QCryptographicHash::hash(key, QCryptographicHash::Sha256).toHex()) + QStringLiteral(".intrinsics"));
}
ProfileLoadOutcome QtCalibrationProfileStore::load(std::string_view id, std::uint32_t width, std::uint32_t height)
{
    const auto name = path(id, width, height);
    if (!QFileInfo::exists(name)) return {};
    QFile file{name};
    if (!file.open(QIODevice::ReadOnly)) return {ProfileLoadState::Rejected, {}, "Cannot read profile: " + file.errorString().toStdString()};
    const auto bytes = file.read(static_cast<qint64>(MaximumCalibrationProfileBytes) + 1);
    if (file.error() != QFileDevice::NoError) return {ProfileLoadState::Rejected, {}, "Profile read failed: " + file.errorString().toStdString()};
    auto outcome = parseCalibrationProfile(std::string_view{bytes.constData(), static_cast<std::size_t>(bytes.size())});
    if (outcome.result) outcome.error = calibrationCompatibilityError(*outcome.result, id, width, height);
    if (!outcome.result || !outcome.error.empty()) return {ProfileLoadState::Rejected, {}, outcome.error};
    return {ProfileLoadState::Loaded, std::move(outcome.result), {}};
}
std::string QtCalibrationProfileStore::save(const IntrinsicCalibration& calibration)
{
    try {
        const auto text = serializeCalibrationProfile(calibration);
        if (!QDir{}.mkpath(directory_)) return "Cannot create calibration profile directory.";
        QSaveFile file{path(calibration.sourceId, calibration.width, calibration.height)};
        // No direct-write fallback: atomic replacement or an actionable failure.
        file.setDirectWriteFallback(false);
        if (!file.open(QIODevice::WriteOnly)) return "Cannot save profile: " + file.errorString().toStdString();
        if (file.write(text.data(), static_cast<qint64>(text.size())) != static_cast<qint64>(text.size()))
            return "Profile write failed: " + file.errorString().toStdString();
        if (!file.commit()) return "Profile commit failed: " + file.errorString().toStdString();
        return {};
    } catch (const std::exception& error) { return error.what(); }
}
} // namespace micro_robot
