#pragma once
// Per-session command validation for protocol version 1 (#161). It turns one written
// frame into at most one call on the single motion-command authority
// (runtime::IMotionCommandSink, #158) and exactly one CommandResult, in the specified
// validation order:
//
//   1-4. version, direction/type and exact length (V1Codec decodeCommand);
//   5.   sequence, then state, then validity, then range.
//
// The authority keeps every motion decision: it checks state atomically with the
// actuator gate. A Velocity whose validity or range is invalid never reaches it; the
// session then reports the rejection in v1 precedence order (a state rejection first),
// using the runtime status at that moment. A concurrent transition can therefore change
// only which rejection code is reported, never motion. A rejected frame changes neither
// motion, lastAccepted nor the deadline.
//
// Portable and allocation-free: no BLE, GATT, motor or clock calls. Not thread-safe:
// the BLE adapter (#163) owns one instance per connection, calls it from its single
// callback context, stamps `receipt` at callback entry on the robot clock and notifies the
// returned bytes. It calls open() when the session becomes ready (both CCCDs enabled) and
// end() when it stops being ready (#162): the session then holds the authority's
// SessionToken, and only its Velocity and ClearEmergencyStop can be admitted. Before
// open() and after end(), Stop and EmergencyStop still take effect (v1); Velocity and Clear
// are refused (RobotFault), and per v1 the adapter sends no result while not ready.
// Detecting readiness and scheduling telemetry are #163/#164.
#include "protocol/V1Codec.hpp"
#include "protocol/V1Sequence.hpp"
#include "runtime/ActuatorGate.hpp"
#include "runtime/RuntimePorts.hpp"
#include <chrono>
#include <cstdint>
#include <optional>
#include <span>

namespace codecell::protocol::v1 {

// Authorization of the gated operations, Velocity and ClearEmergencyStop (#157 review, #163).
// Asked after a frame passed steps 1-5's sequence check and before the authority sees it.
// Stop and EmergencyStop are never asked: they only remove motion. v1 has no "unauthorized"
// result code and its frames are frozen, so a refusal is reported as RobotFault (the
// rejection the desktop already handles) and changes neither motion nor lastAccepted.
class ICommandAuthorizer {
public:
    virtual ~ICommandAuthorizer() = default;
    [[nodiscard]] virtual bool authorize(MessageType operation) noexcept = 0;
};

// Read-only view of the runtime status (runtime::FirmwareRuntime::status() in production).
class IRuntimeStatusView {
public:
    virtual ~IRuntimeStatusView() = default;
    [[nodiscard]] virtual runtime::RuntimeStatus status() const noexcept = 0;
};

// Authority outcome to v1 result code. v1 has no "not ready" code: a runtime that has
// not completed start-up (or entered maintenance) refuses motion for self-protection,
// which the robot reports as RobotFault, a rejection the desktop already handles.
[[nodiscard]] constexpr ResultCode resultCodeFor(runtime::CommandOutcome outcome) noexcept
{
    switch (outcome) {
    case runtime::CommandOutcome::Accepted: return ResultCode::Accepted;
    case runtime::CommandOutcome::RejectedNotReady: return ResultCode::RobotFault;
    case runtime::CommandOutcome::RejectedEmergencyStopLatched: return ResultCode::EmergencyStopLatched;
    case runtime::CommandOutcome::RejectedFault: return ResultCode::RobotFault;
    case runtime::CommandOutcome::RejectedInvalid: return ResultCode::InvalidValidity;
    case runtime::CommandOutcome::RejectedOutOfRange: return ResultCode::OutOfRange;
    case runtime::CommandOutcome::RejectedSessionNotReady: return ResultCode::RobotFault;
    }
    return ResultCode::RobotFault;  // Unknown outcome: a rejection, never an acceptance.
}

// The state checks of step 5 as the authority applies them (SafetyState::admitVelocity):
// Fault, then the E-stop latch, then start-up readiness. Accepted when none applies. Session
// readiness, the last state check, is the session's own (CommandSession::isOpen()).
[[nodiscard]] constexpr ResultCode velocityStateRejection(const runtime::RuntimeStatus& status) noexcept
{
    if (status.state == runtime::MotionState::Fault) return ResultCode::RobotFault;
    if (status.state == runtime::MotionState::EmergencyStop) return ResultCode::EmergencyStopLatched;
    if (!status.ready) return ResultCode::RobotFault;
    return ResultCode::Accepted;
}

class CommandSession {
public:
    CommandSession(runtime::IMotionCommandSink& commands, const IRuntimeStatusView& runtime,
                   const CommandLimits& limits) noexcept
        : commands_(commands), runtime_(runtime), limits_(limits)
    {
    }
    CommandSession(const CommandSession&) = delete;
    CommandSession& operator=(const CommandSession&) = delete;

    // A write to the Command characteristic. Returns the one CommandResult for it.
    // `authorizer` (nullable) decides Velocity and ClearEmergencyStop before dispatch.
    [[nodiscard]] CommandResultBytes onCommandWrite(std::span<const std::uint8_t> bytes, runtime::Instant receipt,
                                                    ICommandAuthorizer* authorizer = nullptr) noexcept
    {
        const auto decoded = decodeCommand(bytes);
        if (!decoded.frame) return result(decoded.echoSequence, decoded.echoType, decoded.rejection);
        const CommandFrame& frame = *decoded.frame;
        const auto type = static_cast<std::uint8_t>(frame.type);
        if (!sequence_.admits(frame.type, frame.sequence))
            return result(frame.sequence, type, ResultCode::StaleSequence);
        const bool gated = frame.type == MessageType::Velocity || frame.type == MessageType::ClearEmergencyStop;
        if (gated && authorizer && !authorizer->authorize(frame.type))
            return result(frame.sequence, type, ResultCode::RobotFault);
        const ResultCode code = dispatch(frame, receipt);
        if (code == ResultCode::Accepted) sequence_.record(frame.type, frame.sequence);
        return result(frame.sequence, type, code);
    }

    // A Command write that session security refused before it was parsed (#157 review): it
    // has no effect; the one CommandResult rejects it (RobotFault) with whatever sequence
    // and type are readable.
    [[nodiscard]] CommandResultBytes refuse(std::span<const std::uint8_t> bytes) noexcept
    {
        const auto decoded = decodeCommand(bytes);
        const auto sequence = decoded.frame ? decoded.frame->sequence : decoded.echoSequence;
        const auto type = decoded.frame ? static_cast<std::uint8_t>(decoded.frame->type) : decoded.echoType;
        return result(sequence, type, ResultCode::RobotFault);
    }

    // A write to the Emergency Stop characteristic: any length or content latches before
    // anything is parsed. A well-formed v1 EmergencyStop echoes its sequence; any other
    // payload echoes sequence 0. Both are Accepted, because the latch was applied.
    [[nodiscard]] CommandResultBytes onEmergencyStopWrite(std::span<const std::uint8_t> bytes) noexcept
    {
        (void)commands_.emergencyStop();  // Never rejected (v1); the drive is off on return.
        const auto decoded = decodeCommand(bytes);
        const bool wellFormed = decoded.frame && decoded.frame->type == MessageType::EmergencyStop;
        return result(wellFormed ? decoded.frame->sequence : kUnreadableSequence,
                      static_cast<std::uint8_t>(MessageType::EmergencyStop), ResultCode::Accepted);
    }

    // One Telemetry frame on the shared robot counter. The session owns the safety fields:
    // `state` and `commandExpired` come from the runtime status at this moment (#162) and
    // `lastAcceptedSequence` from this session; the caller supplies only uptime and
    // measurements. nullopt (and no sequence used) when a value is not representable.
    [[nodiscard]] std::optional<TelemetryBytes> telemetry(TelemetryValues values) noexcept
    {
        const auto status = runtime_.status();
        values.state = status.state;
        values.commandExpired = status.commandExpired;
        values.lastAcceptedSequence = sequence_.lastAccepted();
        const auto bytes = encodeTelemetry(transmit_.peek(), values);
        if (bytes) (void)transmit_.take();
        return bytes;
    }

    [[nodiscard]] std::optional<std::uint16_t> lastAcceptedSequence() const noexcept
    {
        return sequence_.lastAccepted();
    }
    [[nodiscard]] std::uint16_t nextTransmitSequence() const noexcept { return transmit_.peek(); }

    [[nodiscard]] bool isOpen() const noexcept { return session_.has_value(); }

    // The session became ready: both counters restart and the authority issues the token.
    // Opening again replaces the previous session (the authority ends it as for Stop).
    void open() noexcept
    {
        resetCounters();
        session_ = commands_.openSession();
    }

    // The session stopped being ready (disconnect, notification disable, overload): the
    // authority stops as for Stop and keeps the latch; both counters restart. No-op when
    // not open.
    void end(runtime::SessionEndReason reason) noexcept
    {
        resetCounters();
        if (!session_) return;
        commands_.sessionEnded(*session_, reason);
        session_.reset();
    }

private:
    ResultCode dispatch(const CommandFrame& frame, runtime::Instant receipt) noexcept
    {
        switch (frame.type) {
        case MessageType::Stop: return resultCodeFor(commands_.stop());
        case MessageType::EmergencyStop: return resultCodeFor(commands_.emergencyStop());
        case MessageType::ClearEmergencyStop: return resultCodeFor(commands_.clearEmergencyStop(token()));
        case MessageType::Velocity: return velocity(frame.velocity, receipt);
        case MessageType::Telemetry:
        case MessageType::CommandResult: break;
        }
        return ResultCode::UnknownMessageType;  // Unreachable: decodeCommand admits commands only.
    }

    ResultCode velocity(const VelocityFields& fields, runtime::Instant receipt) noexcept
    {
        if (const auto invalid = checkVelocityFields(fields, limits_); invalid != ResultCode::Accepted) {
            auto state = velocityStateRejection(runtime_.status());
            if (state == ResultCode::Accepted && !isOpen()) state = ResultCode::RobotFault;
            return state != ResultCode::Accepted ? state : invalid;
        }
        return resultCodeFor(commands_.velocity(token(), toBodyVelocity(fields),
                                                std::chrono::milliseconds{fields.validForMilliseconds}, receipt));
    }

    // The default token is never current, so the authority refuses it.
    [[nodiscard]] runtime::SessionToken token() const noexcept { return session_.value_or(runtime::SessionToken{}); }

    void resetCounters() noexcept
    {
        sequence_.reset();
        transmit_.reset();
    }

    CommandResultBytes result(std::uint16_t commandSequence, std::uint8_t commandType, ResultCode code) noexcept
    {
        return encodeCommandResult(transmit_.take(), {commandSequence, commandType, code, runtime_.status().state});
    }

    runtime::IMotionCommandSink& commands_;
    const IRuntimeStatusView& runtime_;
    CommandLimits limits_;
    CommandSequenceState sequence_{};
    TransmitSequence transmit_{};
    std::optional<runtime::SessionToken> session_{};
};
} // namespace codecell::protocol::v1
