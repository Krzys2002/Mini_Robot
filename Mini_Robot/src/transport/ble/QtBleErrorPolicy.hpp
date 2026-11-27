#pragma once
#include "transport/TransportTypes.hpp"

#include <QLowEnergyController>
#include <QMetaEnum>
#include <QString>

#include <optional>
#include <utility>

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

// The one terminal event QtBleBackend reports for a link: `lost` or `failed`.
struct QtBleLinkEnd {
    QtBleErrorAction action{QtBleErrorAction::Failure}; // LinkLoss or Failure, never Ignore.
    TransportErrorCode code{TransportErrorCode::BackendFailure};
    QString diagnostic;
};

// Orders the native teardown events of one link so that exactly one terminal event is
// reported and the most specific cause wins. Qt's BlueZ backend closes a link by
// invalidating its services first, then emitting the controller error, if any, then
// `disconnected`, all in one call. Service invalidation only blocks writes and records
// a fallback cause; a controller error, if one follows, decides. The fallback is
// reported on `disconnected` or, when Qt sends neither, by `settle()`, which the
// backend queues behind the native call. A link loss chosen here is still checked
// against the local adapter (confirmQtBleLinkLoss) before it is reported.
// Not thread-safe; owned by one link on its Qt thread.
class QtBleLinkTeardown {
public:
    static constexpr auto invalidServiceDiagnostic = "Robot Control service became invalid";

    // False once the service is invalidated or a terminal event is reported.
    [[nodiscard]] bool writable() const noexcept { return !invalidated_ && !ended_; }
    [[nodiscard]] bool ended() const noexcept { return ended_; }

    // Blocks writes and records the fallback cause; reports nothing yet.
    void serviceInvalidated(bool established)
    {
        if (ended_ || invalidated_) return;
        invalidated_ = true;
        fallback_ = established
            ? QtBleLinkEnd{QtBleErrorAction::LinkLoss, TransportErrorCode::Unavailable,
                           QString::fromLatin1(invalidServiceDiagnostic)}
            : QtBleLinkEnd{QtBleErrorAction::Failure, TransportErrorCode::BackendFailure,
                           QString::fromLatin1(invalidServiceDiagnostic)};
    }
    // A classified controller error takes precedence over a recorded fallback.
    [[nodiscard]] std::optional<QtBleLinkEnd> controllerError(QLowEnergyController::Error error, bool established,
                                                              const QString& native)
    {
        const auto disposition = classifyQtBleControllerError(error, established);
        if (disposition.action == QtBleErrorAction::Ignore) return {};
        return end({disposition.action, disposition.code, describeQtBleControllerError(error, native)});
    }
    // Any other terminal cause the backend finds (setup, protocol or write failure).
    [[nodiscard]] std::optional<QtBleLinkEnd> failure(TransportErrorCode code, const QString& message)
    {
        return end({QtBleErrorAction::Failure, code, message});
    }
    // An unrequested `disconnected` without invalidation or a controller error.
    [[nodiscard]] std::optional<QtBleLinkEnd> unexpectedClose()
    {
        return end({QtBleErrorAction::LinkLoss, TransportErrorCode::Unavailable, {}});
    }
    // Reports the recorded fallback, if one is pending. Without one, `disconnected` is a
    // plain close that the caller reports itself (or unexpectedClose when unrequested).
    [[nodiscard]] std::optional<QtBleLinkEnd> disconnected() { return settle(); }
    [[nodiscard]] std::optional<QtBleLinkEnd> settle()
    {
        if (!fallback_) return {};
        return end(*fallback_);
    }

private:
    std::optional<QtBleLinkEnd> end(QtBleLinkEnd event)
    {
        if (ended_) return {};
        ended_ = true;
        fallback_.reset();
        return event;
    }
    bool invalidated_{false};
    bool ended_{false};
    std::optional<QtBleLinkEnd> fallback_;
};

// State of the local adapter that carried the link, read after the link ended.
enum class QtBleLocalAdapter { Available, PoweredOff, Missing };

// Qt's BlueZ backend (6.10 and 6.12) reports a removed or powered-off local adapter as a
// plain disconnect: BlueZ first marks the device disconnected, Qt closes the link with
// NoError and stops watching the adapter, so InvalidBluetoothAdapterError never arrives.
// A link loss is therefore only reported as recoverable while the local adapter is still
// present and powered; otherwise it is a local Unavailable failure that keeps the native
// diagnostic. Failures pass through unchanged.
[[nodiscard]] inline QtBleLinkEnd confirmQtBleLinkLoss(QtBleLinkEnd end, QtBleLocalAdapter adapter)
{
    if (end.action != QtBleErrorAction::LinkLoss || adapter == QtBleLocalAdapter::Available) return end;
    auto diagnostic = adapter == QtBleLocalAdapter::Missing
        ? QStringLiteral("Local Bluetooth adapter unavailable (removed)")
        : QStringLiteral("Local Bluetooth adapter unavailable (powered off)");
    if (!end.diagnostic.isEmpty()) diagnostic += QStringLiteral("; native: ") + end.diagnostic;
    return {QtBleErrorAction::Failure, TransportErrorCode::Unavailable, std::move(diagnostic)};
}
} // namespace micro_robot
