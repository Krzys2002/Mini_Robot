// Host tests of the #161 uint16 serial arithmetic and per-session sequence state:
// equal, older, newer, wraparound and the half-range ambiguity; the first-command,
// stale-Stop and E-stop exceptions; and the shared robot transmit counter.
#include "protocol/V1Sequence.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>

using namespace codecell::protocol::v1;

TEST(ProtocolSequenceTest, NewerThanIsSerialComparisonOverTheWholeSpace)
{
    // Exhaustive over every distance for a set of bases, including both wrap points.
    for (const std::uint32_t base : {0U, 1U, 0x7FFFU, 0x8000U, 0xFFFEU, 0xFFFFU, 12345U}) {
        for (std::uint32_t distance = 0; distance <= 0xFFFF; ++distance) {
            const auto b = static_cast<std::uint16_t>(base);
            const auto a = static_cast<std::uint16_t>(base + distance);
            const bool expected = distance >= 1 && distance <= 0x7FFF;
            ASSERT_EQ(isNewer(a, b), expected) << base << " + " << distance;
        }
    }
}

TEST(ProtocolSequenceTest, EqualOlderWraparoundAndHalfRangeCases)
{
    EXPECT_FALSE(isNewer(258, 258));       // Equal.
    EXPECT_FALSE(isNewer(257, 258));       // Older.
    EXPECT_TRUE(isNewer(259, 258));
    EXPECT_TRUE(isNewer(0, 0xFFFF));       // Wraps forward.
    EXPECT_FALSE(isNewer(0xFFFF, 0));      // Wraps backward: older.
    EXPECT_TRUE(isNewer(0x7FFF, 0));       // Largest newer distance.
    EXPECT_FALSE(isNewer(0x8000, 0));      // Exactly half the range: ambiguous, not newer...
    EXPECT_FALSE(isNewer(0, 0x8000));      // ...in either direction.
    EXPECT_FALSE(isNewer(0x8001, 1));
    EXPECT_FALSE(isNewer(1, 0x8001));
}

TEST(ProtocolSequenceTest, FirstAcceptedCommandOfASessionNeedsNoComparison)
{
    for (const auto type : {MessageType::Stop, MessageType::Velocity, MessageType::ClearEmergencyStop}) {
        for (const std::uint16_t sequence : {std::uint16_t{0}, std::uint16_t{0x8000}, std::uint16_t{0xFFFF}}) {
            CommandSequenceState state;
            EXPECT_FALSE(state.lastAccepted());
            EXPECT_TRUE(state.admits(type, sequence));
            state.record(type, sequence);
            EXPECT_EQ(state.lastAccepted(), sequence);
        }
    }
}

TEST(ProtocolSequenceTest, VelocityAndClearMustBeNewerThanLastAccepted)
{
    for (const auto type : {MessageType::Velocity, MessageType::ClearEmergencyStop}) {
        CommandSequenceState state;
        state.record(MessageType::Velocity, 0xFFFE);
        EXPECT_FALSE(state.admits(type, 0xFFFE));  // Equal.
        EXPECT_FALSE(state.admits(type, 0xFFFD));  // Older.
        EXPECT_FALSE(state.admits(type, 0x7FFE));  // Half the range ahead: ambiguous.
        EXPECT_TRUE(state.admits(type, 0xFFFF));
        EXPECT_TRUE(state.admits(type, 0x0000));   // Across the wrap.
        EXPECT_TRUE(state.admits(type, 0x7FFD));   // Largest newer distance.
    }
}

TEST(ProtocolSequenceTest, StaleStopIsAdmittedButNeverMovesLastAcceptedBackward)
{
    CommandSequenceState state;
    state.record(MessageType::Velocity, 100);
    for (const std::uint16_t stale : {std::uint16_t{100}, std::uint16_t{99}, std::uint16_t{100 + 0x8000}}) {
        EXPECT_TRUE(state.admits(MessageType::Stop, stale));
        state.record(MessageType::Stop, stale);
        EXPECT_EQ(state.lastAccepted(), 100);
    }
    state.record(MessageType::Stop, 101);  // A newer Stop advances it.
    EXPECT_EQ(state.lastAccepted(), 101);
    EXPECT_FALSE(state.admits(MessageType::Velocity, 101));
}

TEST(ProtocolSequenceTest, EmergencyStopIsNeverCheckedAndNeverRecorded)
{
    CommandSequenceState state;
    EXPECT_TRUE(state.admits(MessageType::EmergencyStop, 5));
    state.record(MessageType::EmergencyStop, 5);
    EXPECT_FALSE(state.lastAccepted());  // Still the first command of the session afterwards.
    state.record(MessageType::Velocity, 10);
    for (const std::uint16_t any : {std::uint16_t{0}, std::uint16_t{10}, std::uint16_t{11}, std::uint16_t{0x800A}}) {
        EXPECT_TRUE(state.admits(MessageType::EmergencyStop, any));
        state.record(MessageType::EmergencyStop, any);
        EXPECT_EQ(state.lastAccepted(), 10);
    }
}

TEST(ProtocolSequenceTest, RobotFramesAreNeverAdmittedAsCommands)
{
    const CommandSequenceState state;
    EXPECT_FALSE(state.admits(MessageType::Telemetry, 1));
    EXPECT_FALSE(state.admits(MessageType::CommandResult, 1));
}

TEST(ProtocolSequenceTest, SessionResetForgetsLastAccepted)
{
    CommandSequenceState state;
    state.record(MessageType::Velocity, 500);
    state.reset();
    EXPECT_FALSE(state.lastAccepted());
    EXPECT_TRUE(state.admits(MessageType::Velocity, 10));  // First of the new session.
}

TEST(ProtocolSequenceTest, TransmitCounterStartsAtZeroAndWrapsModulo65536)
{
    TransmitSequence counter;
    EXPECT_EQ(counter.peek(), 0);
    EXPECT_EQ(counter.take(), 0);
    EXPECT_EQ(counter.take(), 1);
    for (std::uint32_t i = 2; i < 0xFFFF; ++i) (void)counter.take();
    EXPECT_EQ(counter.take(), 0xFFFF);
    EXPECT_EQ(counter.take(), 0);  // Wrapped.
    counter.reset();
    EXPECT_EQ(counter.peek(), 0);
}
