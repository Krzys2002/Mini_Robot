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

QString QtWorkspaceProfileStore::acquisitionPath(const QString& file, const WorkspaceCalibrationProfile& profile)
{
    const auto id = QByteArray::fromStdString(profile.acquisitionId);
    return file + '.'
        + QString::fromLatin1(QCryptographicHash::hash(id,QCryptographicHash::Sha256).toHex()) + ".acquisition";
}
WorkspaceAcquisitionLoadOutcome QtWorkspaceProfileStore::readAcquisition(const QString& name,
    const WorkspaceCalibrationProfile& profile)
{
    QFile file{acquisitionPath(name, profile)};
    if (!file.open(QIODevice::ReadOnly)) return {{},"Linked acquisition record missing/unreadable: "+file.errorString().toStdString()};
    const auto bytes = file.read(static_cast<qint64>(MaximumWorkspaceAcquisitionBytes)+1);
    if (file.error()!=QFileDevice::NoError) return {{},file.errorString().toStdString()};
    auto result = parseWorkspaceAcquisition({bytes.constData(),static_cast<std::size_t>(bytes.size())});
    if (result.record && tabletopProfileBinding(result.record->profile)!=tabletopProfileBinding(profile))
        return {{},"Linked acquisition belongs to another calibration/frame/optical binding."};
    return result;
}

WorkspaceProfileLoadOutcome QtWorkspaceProfileStore::load(std::string_view workspace,
                                                          const IntrinsicCalibration& active)
{
    return readProfile(path(workspace, active), active, workspace);
}
WorkspaceProfileLoadOutcome QtWorkspaceProfileStore::loadFile(std::string_view file, const IntrinsicCalibration& active)
{
    return readProfile(QString::fromUtf8(file.data(), static_cast<qsizetype>(file.size())), active, std::nullopt);
}
WorkspaceProfileLoadOutcome QtWorkspaceProfileStore::readProfile(const QString& name, const IntrinsicCalibration& active,
                                                                std::optional<std::string_view> workspace)
{
    if (const auto error = validateIntrinsicCalibration(active); !error.empty())
        return {ProfileLoadState::Rejected, {}, error};
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
        if (workspace && result.profile->workspaceId != *workspace)
            result.error = "Workspace identity mismatch.";
        if (result.error.empty() && !result.profile->acquisitionId.empty()) {
            auto acquisition = readAcquisition(name, *result.profile);
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
    return writeProfile(path(profile.workspaceId, profile.intrinsics), profile);
}

std::string QtWorkspaceProfileStore::saveFile(std::string_view file, const WorkspaceCalibrationProfile& profile,
                                              const WorkspaceAcquisitionRecord* record)
{
    const QFileInfo destination{QString::fromUtf8(file.data(), static_cast<qsizetype>(file.size()))};
    return record ? writeAcquisition(destination.absoluteFilePath(), profile, *record)
                  : writeProfile(destination.absoluteFilePath(), profile);
}

std::string QtWorkspaceProfileStore::writeProfile(const QString& name, const WorkspaceCalibrationProfile& profile)
{
    try {
        const auto text = serializeWorkspaceProfile(profile);
        if (!profile.acquisitionId.empty()) {
            const auto acquisition = readAcquisition(name, profile);
            if (!acquisition.record) return acquisition.error;
        }
        if (!QDir{}.mkpath(QFileInfo(name).absolutePath()))
            return "Cannot create workspace profile directory.";
        QSaveFile file{name};
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
    return writeAcquisition(path(profile.workspaceId, profile.intrinsics), profile, record);
}
std::string QtWorkspaceProfileStore::writeAcquisition(const QString& profileName, const WorkspaceCalibrationProfile& profile,
                                                      const WorkspaceAcquisitionRecord& record)
{
    try {
        if (tabletopProfileBinding(record.profile)!=tabletopProfileBinding(profile))
            return "Acquisition snapshot/profile mismatch.";
        const auto text = serializeWorkspaceAcquisition(record);
        const auto name = acquisitionPath(profileName, profile);
        if (!QDir{}.mkpath(QFileInfo(name).absolutePath())) return "Cannot create workspace acquisition directory.";
        QLockFile lock{name+".lock"};
        if (!lock.tryLock(StoreLockWaitMilliseconds)) return "Acquisition snapshot is being saved by another writer.";
        if (QFileInfo::exists(name)) {
            QFile previous{name};
            if (!previous.open(QIODevice::ReadOnly)) return previous.errorString().toStdString();
            const auto bytes = previous.read(static_cast<qint64>(MaximumWorkspaceAcquisitionBytes)+1);
            if (previous.error()!=QFileDevice::NoError)
                return previous.errorString().toStdString();
            if (bytes!=QByteArray::fromStdString(text)) {
                // Legacy v1 sidecars have no elevation fields. Compare their
                // validated canonical meaning, but never replace the evidence bytes.
                const auto existing=parseWorkspaceAcquisition({bytes.constData(),static_cast<std::size_t>(bytes.size())});
                if (!existing.record || serializeWorkspaceAcquisition(*existing.record)!=text)
                    return "Immutable acquisition identity already contains different data. Generate a new snapshot identity.";
            }
        } else {
            QSaveFile file{name}; file.setDirectWriteFallback(false);
            if (!file.open(QIODevice::WriteOnly)) return file.errorString().toStdString();
            if (file.write(text.data(),static_cast<qint64>(text.size()))!=static_cast<qint64>(text.size()) || !file.commit())
                return file.errorString().toStdString();
        }
        // A failure here preserves the old profile and its still-immutable record.
        // Unreferenced snapshots are retained as evidence; no automatic deletion.
        return writeProfile(profileName, profile);
    } catch (const std::exception& error) { return error.what(); }
}
} // namespace micro_robot
