#include "transport/ble/QtBleErrorPolicy.hpp"

#include <gtest/gtest.h>

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
