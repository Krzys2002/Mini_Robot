#include "calibration/QtWorkspaceProfileStore.hpp"
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>

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
        if (!result.error.empty()) {
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
} // namespace micro_robot
