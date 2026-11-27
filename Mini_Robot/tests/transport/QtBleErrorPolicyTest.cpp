#include "transport/ble/QtBleErrorPolicy.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <tuple>
#include <vector>

using namespace micro_robot;
using Error = QLowEnergyController::Error;

namespace {
struct Expected {
    Error error;
    QtBleErrorAction established;
    QtBleErrorAction duringSetup;
    TransportErrorCode failureCode; // Code whenever the action is Failure.
};
} // namespace

TEST(QtBleErrorPolicyTest, ClassifiesEveryControllerErrorByLinkState)
{
    const std::vector<Expected> table{
        {Error::NoError, QtBleErrorAction::Ignore, QtBleErrorAction::Ignore, TransportErrorCode::BackendFailure},
        {Error::RemoteHostClosedError, QtBleErrorAction::LinkLoss, QtBleErrorAction::Failure,
         TransportErrorCode::BackendFailure},
        {Error::ConnectionError, QtBleErrorAction::LinkLoss, QtBleErrorAction::Failure,
         TransportErrorCode::BackendFailure},
        {Error::InvalidBluetoothAdapterError, QtBleErrorAction::Failure, QtBleErrorAction::Failure,
         TransportErrorCode::Unavailable},
        {Error::AuthorizationError, QtBleErrorAction::Failure, QtBleErrorAction::Failure,
         TransportErrorCode::Unavailable},
#if QT_VERSION >= QT_VERSION_CHECK(6, 4, 0)
        {Error::MissingPermissionsError, QtBleErrorAction::Failure, QtBleErrorAction::Failure,
         TransportErrorCode::Unavailable},
#endif
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
        {Error::RssiReadError, QtBleErrorAction::Ignore, QtBleErrorAction::Ignore, TransportErrorCode::BackendFailure},
#endif
        {Error::UnknownError, QtBleErrorAction::Failure, QtBleErrorAction::Failure, TransportErrorCode::BackendFailure},
        {Error::UnknownRemoteDeviceError, QtBleErrorAction::Failure, QtBleErrorAction::Failure,
         TransportErrorCode::BackendFailure},
        {Error::NetworkError, QtBleErrorAction::Failure, QtBleErrorAction::Failure, TransportErrorCode::BackendFailure},
        {Error::AdvertisingError, QtBleErrorAction::Failure, QtBleErrorAction::Failure,
         TransportErrorCode::BackendFailure},
    };
    for (const auto& row : table) {
        SCOPED_TRACE(describeQtBleControllerError(row.error, {}).toStdString());
        const auto established = classifyQtBleControllerError(row.error, true);
        const auto setup = classifyQtBleControllerError(row.error, false);
        EXPECT_EQ(established.action, row.established);
        EXPECT_EQ(setup.action, row.duringSetup);
        if (established.action == QtBleErrorAction::Failure) {
            EXPECT_EQ(established.code, row.failureCode);
        }
        if (setup.action == QtBleErrorAction::Failure) {
            EXPECT_EQ(setup.code, row.failureCode);
        }
        if (established.action == QtBleErrorAction::LinkLoss) {
            EXPECT_EQ(established.code, TransportErrorCode::Unavailable);
        }
    }
}

TEST(QtBleErrorPolicyTest, LocalAdapterAndPermissionErrorsNeverPassAsAVanishedPeer)
{
    for (const auto error : {Error::InvalidBluetoothAdapterError, Error::AuthorizationError}) {
        const auto disposition = classifyQtBleControllerError(error, true);
        EXPECT_EQ(disposition.action, QtBleErrorAction::Failure);
        EXPECT_EQ(disposition.code, TransportErrorCode::Unavailable);
    }
}

TEST(QtBleErrorPolicyTest, DiagnosticNamesTheQtErrorAndKeepsTheNativeText)
{
    EXPECT_EQ(describeQtBleControllerError(Error::RemoteHostClosedError,
                                           QStringLiteral("Remote device closed the connection")),
              QStringLiteral("Qt RemoteHostClosedError: Remote device closed the connection"));
    EXPECT_EQ(describeQtBleControllerError(Error::AuthorizationError, {}), QStringLiteral("Qt AuthorizationError"));
}

namespace {
// Counts terminal events like QtBleBackend::report: each engaged result is one emit.
struct Recorder {
    std::vector<QtBleLinkEnd> ends;
    void take(std::optional<QtBleLinkEnd> end)
    {
        if (end) ends.push_back(std::move(*end));
    }
};
constexpr auto adapterText = "Bluetooth adapter is no longer available";
} // namespace

TEST(QtBleLinkTeardownTest, BlueZAdapterRemovalAfterServiceInvalidationStaysAnAdapterFault)
{
    // Qt 6.12 BlueZ executeClose(InvalidBluetoothAdapterError): services invalidated,
    // then the controller error, then `disconnected`, then the queued settle step.
    QtBleLinkTeardown teardown;
    Recorder recorder;
    teardown.serviceInvalidated(true);
    EXPECT_FALSE(teardown.writable()) << "writes are inhibited at invalidation";
    EXPECT_FALSE(teardown.ended()) << "invalidation alone reports nothing yet";
    recorder.take(teardown.controllerError(Error::InvalidBluetoothAdapterError, true, QString::fromLatin1(adapterText)));
    recorder.take(teardown.disconnected());
    recorder.take(teardown.settle());
    ASSERT_EQ(recorder.ends.size(), 1U) << "exactly one terminal event";
    EXPECT_EQ(recorder.ends[0].action, QtBleErrorAction::Failure);
    EXPECT_EQ(recorder.ends[0].code, TransportErrorCode::Unavailable);
    EXPECT_EQ(recorder.ends[0].diagnostic,
              QStringLiteral("Qt InvalidBluetoothAdapterError: Bluetooth adapter is no longer available"));
}

TEST(QtBleLinkTeardownTest, SpecificErrorDuringSetupInvalidationWinsOverTheGenericFallback)
{
    QtBleLinkTeardown teardown;
    Recorder recorder;
    teardown.serviceInvalidated(false);
    recorder.take(teardown.controllerError(Error::InvalidBluetoothAdapterError, false, {}));
    recorder.take(teardown.disconnected());
    ASSERT_EQ(recorder.ends.size(), 1U);
    EXPECT_EQ(recorder.ends[0].action, QtBleErrorAction::Failure);
    EXPECT_EQ(recorder.ends[0].code, TransportErrorCode::Unavailable);
}

TEST(QtBleLinkTeardownTest, PeerCloseAfterServiceInvalidationIsOneLinkLossWithTheQtDiagnostic)
{
    QtBleLinkTeardown teardown;
    Recorder recorder;
    teardown.serviceInvalidated(true);
    recorder.take(teardown.controllerError(Error::RemoteHostClosedError, true,
                                           QStringLiteral("Remote device closed the connection")));
    recorder.take(teardown.disconnected());
    recorder.take(teardown.settle());
    ASSERT_EQ(recorder.ends.size(), 1U);
    EXPECT_EQ(recorder.ends[0].action, QtBleErrorAction::LinkLoss);
    EXPECT_EQ(recorder.ends[0].diagnostic, QStringLiteral("Qt RemoteHostClosedError: Remote device closed the connection"));
}

TEST(QtBleLinkTeardownTest, UnexplainedInvalidationEndsOnDisconnectedOrTheQueuedSettleStep)
{
    for (const bool viaDisconnected : {true, false}) {
        SCOPED_TRACE(viaDisconnected ? "disconnected" : "settle");
        QtBleLinkTeardown teardown;
        Recorder recorder;
        teardown.serviceInvalidated(true);
        recorder.take(teardown.controllerError(Error::NoError, true, {})); // Ignored, not a cause.
        if (viaDisconnected) recorder.take(teardown.disconnected());
        recorder.take(teardown.settle());
        recorder.take(teardown.disconnected());
        ASSERT_EQ(recorder.ends.size(), 1U);
        EXPECT_EQ(recorder.ends[0].action, QtBleErrorAction::LinkLoss);
        EXPECT_EQ(recorder.ends[0].diagnostic, QString::fromLatin1(QtBleLinkTeardown::invalidServiceDiagnostic));
    }
    QtBleLinkTeardown setup;
    setup.serviceInvalidated(false);
    const auto end = setup.settle();
    ASSERT_TRUE(end);
    EXPECT_EQ(end->action, QtBleErrorAction::Failure) << "setup invalidation stays a failure";
    EXPECT_EQ(end->code, TransportErrorCode::BackendFailure);
}

TEST(QtBleLinkTeardownTest, PlainDisconnectWithoutInvalidationIsLeftToTheCallerAndLaterEventsAreIgnored)
{
    QtBleLinkTeardown teardown;
    EXPECT_TRUE(teardown.writable());
    EXPECT_FALSE(teardown.disconnected()) << "no fallback: caller reports a plain close";
    EXPECT_FALSE(teardown.ended());

    const auto first = teardown.failure(TransportErrorCode::BackendFailure, QStringLiteral("write failed"));
    ASSERT_TRUE(first);
    EXPECT_TRUE(teardown.ended());
    EXPECT_FALSE(teardown.writable());
    teardown.serviceInvalidated(true);
    EXPECT_FALSE(teardown.controllerError(Error::InvalidBluetoothAdapterError, true, {}));
    EXPECT_FALSE(teardown.failure(TransportErrorCode::Unavailable, QStringLiteral("late")));
    EXPECT_FALSE(teardown.disconnected());
    EXPECT_FALSE(teardown.settle());
}

TEST(QtBleLinkTeardownTest, BlueZAdapterUnbindReportedAsAPlainDisconnectFaultsOnTheAdapterCheck)
{
    // Bench, Qt 6.10.2 + BlueZ 5.85, btusb unbind while connected: BlueZ marks the device
    // disconnected first, Qt closes with NoError (no controller error), services become
    // invalid, then `disconnected`. Only the local adapter check can name the cause.
    QtBleLinkTeardown teardown;
    Recorder recorder;
    teardown.serviceInvalidated(true);
    recorder.take(teardown.disconnected());
    recorder.take(teardown.settle());
    ASSERT_EQ(recorder.ends.size(), 1U);
    ASSERT_EQ(recorder.ends[0].action, QtBleErrorAction::LinkLoss);

    const auto removed = confirmQtBleLinkLoss(recorder.ends[0], QtBleLocalAdapter::Missing);
    EXPECT_EQ(removed.action, QtBleErrorAction::Failure);
    EXPECT_EQ(removed.code, TransportErrorCode::Unavailable);
    EXPECT_EQ(removed.diagnostic, QStringLiteral(
        "Local Bluetooth adapter unavailable (removed); native: Robot Control service became invalid"));

    const auto poweredOff = confirmQtBleLinkLoss(recorder.ends[0], QtBleLocalAdapter::PoweredOff);
    EXPECT_EQ(poweredOff.action, QtBleErrorAction::Failure);
    EXPECT_EQ(poweredOff.code, TransportErrorCode::Unavailable);
    EXPECT_TRUE(poweredOff.diagnostic.startsWith(QStringLiteral("Local Bluetooth adapter unavailable (powered off)")));

    const auto peer = confirmQtBleLinkLoss(recorder.ends[0], QtBleLocalAdapter::Available);
    EXPECT_EQ(peer.action, QtBleErrorAction::LinkLoss) << "robot reset with a working adapter stays recoverable";
    EXPECT_EQ(peer.diagnostic, recorder.ends[0].diagnostic);
}

TEST(QtBleLinkTeardownTest, UnexpectedPlainCloseIsLinkLossAndTheAdapterCheckLeavesFailuresUnchanged)
{
    QtBleLinkTeardown teardown;
    const auto close = teardown.unexpectedClose();
    ASSERT_TRUE(close);
    EXPECT_EQ(close->action, QtBleErrorAction::LinkLoss);
    EXPECT_TRUE(close->diagnostic.isEmpty());
    EXPECT_FALSE(teardown.unexpectedClose()) << "one terminal event";
    EXPECT_EQ(confirmQtBleLinkLoss(*close, QtBleLocalAdapter::Missing).diagnostic,
              QStringLiteral("Local Bluetooth adapter unavailable (removed)"));

    const QtBleLinkEnd failure{QtBleErrorAction::Failure, TransportErrorCode::BackendFailure, QStringLiteral("x")};
    for (const auto adapter : {QtBleLocalAdapter::Available, QtBleLocalAdapter::PoweredOff, QtBleLocalAdapter::Missing}) {
        const auto unchanged = confirmQtBleLinkLoss(failure, adapter);
        EXPECT_EQ(unchanged.action, QtBleErrorAction::Failure);
        EXPECT_EQ(unchanged.code, TransportErrorCode::BackendFailure);
        EXPECT_EQ(unchanged.diagnostic, QStringLiteral("x"));
    }
}
