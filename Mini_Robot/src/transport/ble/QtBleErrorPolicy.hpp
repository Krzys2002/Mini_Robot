#pragma once
#include "transport/TransportTypes.hpp"

#include <QLowEnergyController>
#include <QMetaEnum>
#include <QString>

namespace micro_robot {
// What QtBleBackend does with one QLowEnergyController::errorOccurred (#71).
enum class QtBleErrorAction {
    Ignore,   // Not a link problem (for example an RSSI read we never request).
    LinkLoss, // The established peer link is gone: recoverable, reconnect explicitly.
    Failure,  // A local or protocol failure the operator must act on: transport fault.
};
struct QtBleErrorDisposition {
    QtBleErrorAction action{QtBleErrorAction::Failure};
    TransportErrorCode code{TransportErrorCode::BackendFailure}; // Meaningful for Failure only.
};

// Classifies a controller error by the Qt error and whether the link finished setup
// (`established`: Robot Info validated and both CCCDs enabled). Only errors in which Qt
// reports the remote link itself as closed or dropped count as link loss, and only on an
// established link; setup errors stay failures because the link never became usable.
// Adapter, permission and authorization errors are local and actionable, so they are
// Unavailable failures even on an established link and never pass as a vanished peer.
[[nodiscard]] inline QtBleErrorDisposition classifyQtBleControllerError(QLowEnergyController::Error error,
                                                                       bool established) noexcept
{
    using Error = QLowEnergyController::Error;
    switch (error) {
    case Error::NoError:
        return {QtBleErrorAction::Ignore, TransportErrorCode::BackendFailure};
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
    case Error::RssiReadError:
        return {QtBleErrorAction::Ignore, TransportErrorCode::BackendFailure};
#endif
    case Error::RemoteHostClosedError:
    case Error::ConnectionError:
        return established ? QtBleErrorDisposition{QtBleErrorAction::LinkLoss, TransportErrorCode::Unavailable}
                           : QtBleErrorDisposition{QtBleErrorAction::Failure, TransportErrorCode::BackendFailure};
    case Error::InvalidBluetoothAdapterError:
    case Error::AuthorizationError:
#if QT_VERSION >= QT_VERSION_CHECK(6, 4, 0)
    case Error::MissingPermissionsError:
#endif
        return {QtBleErrorAction::Failure, TransportErrorCode::Unavailable};
    default: // UnknownError, UnknownRemoteDeviceError, NetworkError, AdvertisingError.
        return {QtBleErrorAction::Failure, TransportErrorCode::BackendFailure};
    }
}

// Native diagnostic kept with the classified event, e.g.
// "Qt RemoteHostClosedError: Remote device closed the connection".
[[nodiscard]] inline QString describeQtBleControllerError(QLowEnergyController::Error error, const QString& native)
{
    const auto* key = QMetaEnum::fromType<QLowEnergyController::Error>().valueToKey(static_cast<int>(error));
    auto text = QStringLiteral("Qt %1").arg(key ? QString::fromLatin1(key) : QString::number(static_cast<int>(error)));
    if (!native.isEmpty()) text += QStringLiteral(": ") + native;
    return text;
}
} // namespace micro_robot
