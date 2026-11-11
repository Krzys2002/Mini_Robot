#include "calibration/QtWorkspaceProfileStore.hpp"
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QLockFile>

namespace micro_robot {
QtWorkspaceProfileStore::QtWorkspaceProfileStore(QString directory)
    : directory_(QDir(directory).absolutePath())
{
}

QString QtWorkspaceProfileStore::path(std::string_view workspace, const IntrinsicCalibration& active) const
{
    QByteArray key;
    for (auto id : {workspace, std::string_view{active.sourceId}}) {
        key += QByteArray::number(static_cast<qulonglong>(id.size())) + ':';
        key.append(id.data(), static_cast<qsizetype>(id.size()));
    }
    key += ':' + QByteArray::number(active.width) + 'x' + QByteArray::number(active.height);
    return QDir(directory_)
        .filePath(QString::fromLatin1(QCryptographicHash::hash(key, QCryptographicHash::Sha256).toHex()) +
                  ".workspace");
}

QString QtWorkspaceProfileStore::acquisitionPath(const WorkspaceCalibrationProfile& profile) const
{
    const auto id = QByteArray::fromStdString(profile.acquisitionId);
    return path(profile.workspaceId,profile.intrinsics) + '.'
        + QString::fromLatin1(QCryptographicHash::hash(id,QCryptographicHash::Sha256).toHex()) + ".acquisition";
}
WorkspaceAcquisitionLoadOutcome QtWorkspaceProfileStore::readAcquisition(const WorkspaceCalibrationProfile& profile) const
{
    QFile file{acquisitionPath(profile)};
    if (!file.open(QIODevice::ReadOnly)) return {{},"Linked acquisition record missing/unreadable: "+file.errorString().toStdString()};
    const auto bytes = file.read(static_cast<qint64>(MaximumWorkspaceAcquisitionBytes)+1);
    if (file.error()!=QFileDevice::NoError) return {{},file.errorString().toStdString()};
    auto result = parseWorkspaceAcquisition({bytes.constData(),static_cast<std::size_t>(bytes.size())});
    if (result.record && serializeWorkspaceProfile(result.record->profile)!=serializeWorkspaceProfile(profile))
        return {{},"Linked acquisition belongs to another calibration/frame/optical binding."};
    return result;
}

WorkspaceProfileLoadOutcome QtWorkspaceProfileStore::load(std::string_view workspace,
                                                          const IntrinsicCalibration& active)
{
    if (const auto error = validateIntrinsicCalibration(active); !error.empty())
        return {ProfileLoadState::Rejected, {}, error};
    const auto name = path(workspace, active);
    if (!QFileInfo::exists(name))
        return {};
    QFile file{name};
    if (!file.open(QIODevice::ReadOnly))
        return {ProfileLoadState::Rejected, {}, file.errorString().toStdString()};
    const auto bytes = file.read(static_cast<qint64>(MaximumWorkspaceProfileBytes) + 1);
    if (file.error() != QFileDevice::NoError)
        return {ProfileLoadState::Rejected, {}, file.errorString().toStdString()};
    auto result = parseWorkspaceProfile({bytes.constData(), static_cast<std::size_t>(bytes.size())});
    if (result.profile) {
        result.error = workspaceCalibrationCompatibilityError(*result.profile, active);
        if (result.profile->workspaceId != workspace)
            result.error = "Workspace identity mismatch.";
        if (result.error.empty() && !result.profile->acquisitionId.empty()) {
            auto acquisition = readAcquisition(*result.profile);
            result.error = acquisition.error;
            result.acquisition = std::move(acquisition.record);
        }
        if (!result.error.empty()) {
            result.acquisition.reset();
            result.profile.reset();
            result.state = ProfileLoadState::Rejected;
        }
    }
    return result;
}

std::string QtWorkspaceProfileStore::save(const WorkspaceCalibrationProfile& profile)
{
    try {
        const auto text = serializeWorkspaceProfile(profile);
        if (!profile.acquisitionId.empty()) {
            const auto acquisition = readAcquisition(profile);
            if (!acquisition.record) return acquisition.error;
        }
        if (!QDir{}.mkpath(directory_))
            return "Cannot create workspace profile directory.";
        QSaveFile file{path(profile.workspaceId, profile.intrinsics)};
        file.setDirectWriteFallback(false);
        if (!file.open(QIODevice::WriteOnly))
            return file.errorString().toStdString();
        if (file.write(text.data(), static_cast<qint64>(text.size())) != static_cast<qint64>(text.size()))
            return file.errorString().toStdString();
        if (!file.commit())
            return file.errorString().toStdString();
        return {};
    } catch (const std::exception& e) {
        return e.what();
    }
}
std::string QtWorkspaceProfileStore::saveAcquisition(const WorkspaceCalibrationProfile& profile,
                                                    const WorkspaceAcquisitionRecord& record)
{
    try {
        if (serializeWorkspaceProfile(record.profile)!=serializeWorkspaceProfile(profile))
            return "Acquisition snapshot/profile mismatch.";
        const auto text = serializeWorkspaceAcquisition(record);
        if (!QDir{}.mkpath(directory_)) return "Cannot create workspace acquisition directory.";
        const auto name = acquisitionPath(profile);
        QLockFile lock{name+".lock"};
        if (!lock.tryLock(StoreLockWaitMilliseconds)) return "Acquisition snapshot is being saved by another writer.";
        if (QFileInfo::exists(name)) {
            QFile previous{name};
            if (!previous.open(QIODevice::ReadOnly)) return previous.errorString().toStdString();
            const auto bytes = previous.read(static_cast<qint64>(MaximumWorkspaceAcquisitionBytes)+1);
            if (previous.error()!=QFileDevice::NoError || bytes!=QByteArray::fromStdString(text))
                return "Immutable acquisition identity already contains different data. Generate a new snapshot identity.";
        } else {
            QSaveFile file{name}; file.setDirectWriteFallback(false);
            if (!file.open(QIODevice::WriteOnly)) return file.errorString().toStdString();
            if (file.write(text.data(),static_cast<qint64>(text.size()))!=static_cast<qint64>(text.size()) || !file.commit())
                return file.errorString().toStdString();
        }
        // A failure here preserves the old profile and its still-immutable record.
        // Unreferenced snapshots are retained as evidence; no automatic deletion.
        return save(profile);
    } catch (const std::exception& error) { return error.what(); }
}
} // namespace micro_robot
