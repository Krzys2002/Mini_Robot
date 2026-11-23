#pragma once

#include <QFileDialog>

namespace micro_robot {

enum class ProfileFileKind { Camera, Workspace };

inline QString chooseProfileFile(QWidget* parent, ProfileFileKind kind, QFileDialog::AcceptMode mode)
{
    const bool camera = kind == ProfileFileKind::Camera;
    const auto type = camera ? QStringLiteral("camera profile") : QStringLiteral("workspace profile");
    const auto suffix = camera ? QStringLiteral("intrinsics") : QStringLiteral("workspace");
    const bool saving = mode == QFileDialog::AcceptSave;
    QFileDialog dialog(parent, saving ? QStringLiteral("Save %1 as").arg(type)
                                      : QStringLiteral("Load %1 from file").arg(type));
    dialog.setAcceptMode(mode);
    dialog.setFileMode(saving ? QFileDialog::AnyFile : QFileDialog::ExistingFile);
    const auto filter = QStringLiteral("%1 (*.%2)").arg(type, suffix);
    dialog.setNameFilter(saving ? filter : filter + QStringLiteral(";;All files (*)"));
    if (saving) dialog.setDefaultSuffix(suffix);
    if (dialog.exec() != QDialog::Accepted) return {};
    return dialog.selectedFiles().value(0);
}

} // namespace micro_robot
