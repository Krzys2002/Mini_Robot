#pragma once
#include "transport/ble/IBleBackend.hpp"
#include <memory>

namespace micro_robot {
// Composition-root factory. Native Qt Bluetooth types stay in the .cpp file.
[[nodiscard]] std::unique_ptr<IBleBackend> makeQtBleBackend();
} // namespace micro_robot
