// Unit tests of the #158 runtime components: configuration validation, the pure safety
// state machine and the depth-1 mailbox.
#include "runtime/RuntimeConfig.hpp"
#include "runtime/SafetyState.hpp"
#include "runtime/SetpointMailbox.hpp"
#include "RuntimeFakes.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace codecell::runtime;
using namespace std::chrono_literals;
namespace timing = codecell::timing;

TEST(RuntimeConfigTest, BudgetDefaultsScheduleButCannotArmTheDrive)
{
    const RuntimeConfig config;
    EXPECT_EQ(validateScheduling(config), ConfigError::None);
    EXPECT_EQ(validate(config), ConfigError::DriveProfileInvalid) << "no drive profile is invented (#160)";
    EXPECT_EQ(validate(fakes::validRuntimeConfig()), ConfigError::None);
    EXPECT_EQ(config.safetyCheckPeriod, timing::kSafetyCheckPeriod);
    EXPECT_EQ(config.controlPeriod, timing::kControlPeriod);
    EXPECT_EQ(config.controlStallLimit, timing::kControlStallLimit);
    EXPECT_EQ(config.commandValidityCeiling, timing::kCommandValidityCeiling);
}

TEST(RuntimeConfigTest, EachRuleRejectsItsViolation)
{
    auto with = [](auto mutate) {
        RuntimeConfig config = fakes::validRuntimeConfig();
        mutate(config);
        return validate(config);
    };
    EXPECT_EQ(with([](auto& c) { c.safetyCheckPeriod = 0ms; }), ConfigError::PeriodNotPositive);
    EXPECT_EQ(with([](auto& c) { c.controlPeriod = -1ms; }), ConfigError::PeriodNotPositive);
    EXPECT_EQ(with([](auto& c) { c.safetyCheckPeriod = timing::kSafetyCheckPeriod * 2; }),
              ConfigError::PeriodAboveBudget);
    EXPECT_EQ(with([](auto& c) { c.controlPeriod = timing::kControlPeriod + 1ms; }), ConfigError::PeriodAboveBudget);
    EXPECT_EQ(with([](auto& c) { c.controlStallLimit = c.controlPeriod - 1ms; }), ConfigError::StallLimitOutOfRange);
    EXPECT_EQ(with([](auto& c) { c.controlStallLimit = timing::kControlStallLimit + 1ms; }),
              ConfigError::StallLimitOutOfRange);
    EXPECT_EQ(with([](auto& c) { c.commandValidityCeiling = 0ms; }), ConfigError::ValidityCeilingOutOfRange);
    EXPECT_EQ(with([](auto& c) { c.commandValidityCeiling = timing::kCommandValidityCeiling + 1ms; }),
              ConfigError::ValidityCeilingOutOfRange);
    EXPECT_EQ(with([](auto& c) { c.safetyTask.priority = timing::kNimbleHostPriority; }),
              ConfigError::PriorityOrderViolated);
    EXPECT_EQ(with([](auto& c) { c.safetyTask.priority = timing::kBleControllerPriority; }),
              ConfigError::PriorityOrderViolated);
    EXPECT_EQ(with([](auto& c) { c.controlTask.priority = timing::kLogDrainPriority; }),
              ConfigError::PriorityOrderViolated);
    EXPECT_EQ(with([](auto& c) { c.controlTask.stackBytes = 0; }), ConfigError::StackSizeZero);
    EXPECT_EQ(with([](auto& c) { c.driveProfile.trackWidthMetres = 0.0; }), ConfigError::DriveProfileInvalid);
}

TEST(SafetyStateTest, OutOfRangeIsCheckedAfterStateAndValidityAndChangesNothing)
{
    SafetyState safety;
    EXPECT_EQ(safety.admitVelocity(0us, 100ms, 250ms, false), CommandOutcome::RejectedNotReady);
    safety.markReady();
    EXPECT_EQ(safety.admitVelocity(0us, 0ms, 250ms, false), CommandOutcome::RejectedInvalid);
    const auto generation = safety.generation();
    EXPECT_EQ(safety.admitVelocity(0us, 100ms, 250ms, false), CommandOutcome::RejectedOutOfRange);
    EXPECT_EQ(safety.state(), MotionState::Idle);
    EXPECT_EQ(safety.generation(), generation) << "a rejected command never replaces intent";
    EXPECT_FALSE(safety.deadline());
    safety.latchEmergencyStop();
    EXPECT_EQ(safety.admitVelocity(0us, 100ms, 250ms, false), CommandOutcome::RejectedEmergencyStopLatched);
}

TEST(SafetyStateTest, StartsIdleNotReadyAndRefusesMotion)
{
    SafetyState safety;
    EXPECT_EQ(safety.state(), MotionState::Idle);
    EXPECT_FALSE(safety.ready());
    EXPECT_EQ(safety.admitVelocity(0us, 100ms, 250ms, true), CommandOutcome::RejectedNotReady);
    EXPECT_FALSE(safety.permits(safety.generation(), 0us));
}

TEST(SafetyStateTest, DeadlineIsReceiptRelativeCappedAndExpiresExactlyAtIt)
{
    SafetyState safety;
    safety.markReady();
    ASSERT_EQ(safety.admitVelocity(1000us, 900ms, 250ms, true), CommandOutcome::Accepted);
    ASSERT_TRUE(safety.deadline());
    EXPECT_EQ(*safety.deadline(), 1000us + 250ms);
    const auto generation = safety.generation();
    EXPECT_TRUE(safety.permits(generation, 1000us + 250ms - 1us));
    EXPECT_FALSE(safety.permits(generation, 1000us + 250ms));
    EXPECT_FALSE(safety.expireIfDue(1000us + 250ms - 1us));
    EXPECT_TRUE(safety.expireIfDue(1000us + 250ms));
    EXPECT_EQ(safety.state(), MotionState::Idle);
    EXPECT_FALSE(safety.permits(generation, 0us));
}

TEST(SafetyStateTest, EveryMotionEndingTransitionAdvancesTheGeneration)
{
    SafetyState safety;
    safety.markReady();
    ASSERT_EQ(safety.admitVelocity(0us, 100ms, 250ms, true), CommandOutcome::Accepted);
    auto previous = safety.generation();
    auto advanced = [&] {
        const bool changed = safety.generation() != previous;
        previous = safety.generation();
        return changed;
    };
    safety.stop();
    EXPECT_TRUE(advanced());
    ASSERT_EQ(safety.admitVelocity(0us, 100ms, 250ms, true), CommandOutcome::Accepted);
    EXPECT_TRUE(advanced()) << "a new Velocity supersedes the previous one";
    safety.latchEmergencyStop();
    EXPECT_TRUE(advanced());
    EXPECT_EQ(safety.clearEmergencyStop(), CommandOutcome::Accepted);
    EXPECT_TRUE(advanced());
    safety.enterFault(FaultCode::ControlStalled);
    EXPECT_TRUE(advanced());
}

TEST(SafetyStateTest, PermissionNeedsTheExactAdmittedGeneration)
{
    SafetyState safety;
    safety.markReady();
    ASSERT_EQ(safety.admitVelocity(0us, 100ms, 250ms, true), CommandOutcome::Accepted);
    const auto admitted = safety.generation();
    EXPECT_TRUE(safety.permits(admitted, 1us));
    // Only equality is used, so unsigned wrap of the counter cannot grant permission.
    EXPECT_FALSE(safety.permits(admitted - 1, 1us));
    EXPECT_FALSE(safety.permits(admitted + 1, 1us));
}

TEST(SafetyStateTest, FaultIsStickyKeepsTheFirstCodeAndOutranksTheLatch)
{
    SafetyState safety;
    safety.markReady();
    safety.enterFault(FaultCode::ControlStalled);
    safety.enterFault(FaultCode::DriveWriteFailed);
    EXPECT_EQ(safety.fault(), FaultCode::ControlStalled);
    safety.latchEmergencyStop();
    EXPECT_EQ(safety.state(), MotionState::Fault);
    EXPECT_EQ(safety.clearEmergencyStop(), CommandOutcome::RejectedFault);
    safety.markReady();
    EXPECT_FALSE(safety.ready());
    EXPECT_EQ(safety.admitVelocity(0us, 100ms, 250ms, true), CommandOutcome::RejectedFault);
}

TEST(SafetyStateTest, LatchRejectsVelocityUntilExplicitClear)
{
    SafetyState safety;
    safety.markReady();
    safety.latchEmergencyStop();
    EXPECT_EQ(safety.admitVelocity(0us, 100ms, 250ms, true), CommandOutcome::RejectedEmergencyStopLatched);
    safety.stop();
    EXPECT_EQ(safety.state(), MotionState::EmergencyStop) << "Stop has no effect while latched";
    EXPECT_EQ(safety.clearEmergencyStop(), CommandOutcome::Accepted);
    EXPECT_EQ(safety.state(), MotionState::Idle);
    EXPECT_FALSE(safety.deadline());
}

TEST(SetpointMailboxTest, NewestWinsReadsDoNotConsumeAndOverwritesAreReported)
{
    fakes::FakeCriticalSection section;
    SetpointMailbox mailbox;
    mailbox.bind(section);
    EXPECT_FALSE(mailbox.latest());
    EXPECT_FALSE(mailbox.publish({{0.01, 0.0}, 1}));
    EXPECT_TRUE(mailbox.publish({{0.02, 0.0}, 2})) << "the unread setpoint was replaced";
    auto first = mailbox.latest();
    ASSERT_TRUE(first);
    EXPECT_EQ(first->generation, 2u);
    auto again = mailbox.latest();
    ASSERT_TRUE(again);
    EXPECT_EQ(again->generation, 2u);
    EXPECT_FALSE(mailbox.publish({{0.03, 0.0}, 3})) << "a read setpoint is not an overwrite";
    EXPECT_EQ(section.depth, 0);
    EXPECT_EQ(section.maxDepth, 1);
}
