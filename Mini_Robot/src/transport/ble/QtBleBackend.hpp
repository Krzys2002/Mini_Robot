#pragma once
#include "transport/ble/IBleBackend.hpp"
#include <memory>

namespace micro_robot {
// Composition-root factory. Native Qt Bluetooth types stay in the .cpp file.
[[nodiscard]] std::unique_ptr<IBleBackend> makeQtBleBackend();
// Copy the Qt backend makes of every received notification before decoding: at most
// ble_gatt::notificationCopyLimit (21) bytes, one beyond a default-MTU payload, so
// malformed lengths stay rejectable without copying an arbitrarily large payload.
[[nodiscard]] QByteArray boundBleNotificationCopy(const QByteArray& bytes);
} // namespace micro_robot
