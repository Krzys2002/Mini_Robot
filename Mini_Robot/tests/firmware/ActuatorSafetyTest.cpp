#include "ActuatorSafety.hpp"
#include "controller.h"
#include <gtest/gtest.h>
#include <limits>

CodeCell myCodeCell;
extern SemaphoreHandle_t commandMutex;
extern RobotCommand commandBuffer;
extern MotorOutput motorBuffer;
void writeMotorOutput(int16_t, int16_t, uint64_t);

namespace {
TEST(ActuatorSafetyTest, NoOutputWithoutIntentAndDeadlineIsExclusive) {
    prototype::ActuatorSafety gate{300};
    EXPECT_FALSE(gate.permits(1, 0));
    const auto ticket = gate.accept(1000);
    EXPECT_TRUE(gate.permits(ticket, 1299));
    EXPECT_FALSE(gate.permits(ticket, 1300));
    EXPECT_FALSE(gate.permits(ticket, 1301));
    // Once expired it cannot become fresh after a complete timer wrap.
    EXPECT_FALSE(gate.permits(ticket, 1000));
}
TEST(ActuatorSafetyTest, WrapSafeExpiryAndOldGenerationCannotRevive) {
    prototype::ActuatorSafety gate{300};
    const auto at = std::numeric_limits<std::uint32_t>::max() - 100;
    const auto old = gate.accept(at);
    EXPECT_TRUE(gate.permits(old, 198));
    EXPECT_FALSE(gate.permits(old, 199));
    const auto next = gate.accept(200);
    EXPECT_FALSE(gate.permits(old, 201));
    EXPECT_TRUE(gate.permits(next, 201));
}
TEST(ActuatorSafetyTest, ClearRequiresNewIntentAndFaultCannotBeCleared) {
    prototype::ActuatorSafety gate{300};
    const auto before = gate.accept(1);
    gate.stop();
    EXPECT_EQ(gate.accept(2), 0U);
    EXPECT_FALSE(gate.permits(before, 2));
    EXPECT_TRUE(gate.clear());
    EXPECT_FALSE(gate.permits(before, 3));
    const auto after = gate.accept(4);
    EXPECT_TRUE(gate.permits(after, 5));
    gate.fault();
    EXPECT_FALSE(gate.clear());
    EXPECT_EQ(gate.accept(6), 0U);
    EXPECT_FALSE(gate.permits(after, 6));
}
void cycle(void (*task)(void*)) {
    try { task(nullptr); FAIL() << "Fake task must return through its first scheduler yield"; }
    catch (const fake::EndTask&) {}
}
TEST(PrototypeControllerTest, ActualActuatorLoopRejectsStalledControlAndPreClearSnapshots) {
    ASSERT_TRUE(initController());
    fake::now = 1000;
    ASSERT_TRUE(setDirectControl(50, 0));
    cycle(controlTask);
    cycle(motorTask);
    ASSERT_GT(myCodeCell.Drive1.outputs.back().speed, 0);
    const auto oldGeneration = commandBuffer.safetyGeneration;
    // Control work stops. The independent actuator loop still checks age.
    fake::now = 1300;
    cycle(motorTask);
    EXPECT_EQ(myCodeCell.Drive1.outputs.back().speed, 0);
    // A stalled computation publishing its old result cannot extend validity.
    writeMotorOutput(70, 70, oldGeneration);
    cycle(motorTask);
    EXPECT_EQ(myCodeCell.Drive2.outputs.back().speed, 0);
    fake::now = 1400;
    ASSERT_TRUE(setDirectControl(50, 0)); cycle(controlTask);
    const auto preStop = commandBuffer.safetyGeneration;
    emergencyStopNow();
    EXPECT_FALSE(setDirectControl(100, 0));
    ASSERT_TRUE(clearEmergencyStop());
    writeMotorOutput(90, 90, preStop); cycle(motorTask);
    EXPECT_EQ(myCodeCell.Drive1.outputs.back().speed, 0);
    ASSERT_TRUE(setDirectControl(20, 0));
    // New traffic does not make a previous computed buffer current.
    writeMotorOutput(90, 90, preStop); cycle(motorTask);
    EXPECT_EQ(myCodeCell.Drive2.outputs.back().speed, 0);
    cycle(controlTask); cycle(motorTask);
    EXPECT_GT(myCodeCell.Drive1.outputs.back().speed, 0);
    // Mutex contention is fail-closed, never an indefinite actuator wait.
    commandMutex->held = true; cycle(motorTask); commandMutex->held = false;
    EXPECT_EQ(myCodeCell.Drive1.outputs.back().speed, 0);
    emergencyStopNow();
    // A concurrent new Stop arriving after clear starts cannot be erased by it.
    fake::afterTake = [] { emergencyStopNow(); };
    EXPECT_FALSE(clearEmergencyStop());
    EXPECT_TRUE(emergencyStop.load());
    ASSERT_TRUE(clearEmergencyStop());
    cycle(motorTask); EXPECT_EQ(myCodeCell.Drive1.outputs.back().speed, 0);
    controllerFaultNow();
    EXPECT_FALSE(clearEmergencyStop()); EXPECT_FALSE(setDirectControl(50, 0));
    cycle(motorTask); EXPECT_EQ(myCodeCell.Drive1.outputs.back().speed, 0);
    fake::failAllocation = true;
    EXPECT_FALSE(initController());
    cycle(motorTask); EXPECT_EQ(myCodeCell.Drive2.outputs.back().speed, 0);
}
} // namespace
