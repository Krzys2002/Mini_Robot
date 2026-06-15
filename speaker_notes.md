# Speaker Notes

## Slide 1: Title

Introduce the project as a compact differential-drive robot built around the CodeCell C6 Drive board. Mention that the presentation covers the mechanical layout, electronics, control software, wireless interface, and safety mechanisms.

## Slide 2: Outline

Briefly walk through the structure: project goals, hardware, mechanical design, software architecture, movement control, wireless communication, browser interface, safety, and future work. Keep this slide short and use it only as orientation.

## Slide 3: Project Goals

Explain that the main idea was to build a small mobile robot with minimal external electronics. Emphasize that the CodeCell C6 Drive acts as the core module: motor driver, wireless interface, and sensor platform. Point out the two modes: direct joystick control and autonomous target movement.

## Slide 4: CodeCell C6 Drive - Board Overview

Describe the board as the central hardware element. Highlight the ESP32-C6 for computation and wireless communication, the BNO085 IMU for heading estimation, the motor drivers for the two wheels, and the LiPo/USB-C power path. Mention that using one compact board simplifies wiring and makes the robot easier to reproduce.

## Slide 5: Mechanical Design

Explain the differential-drive principle: two independently driven wheels create straight motion, turns, and rotation in place. Mention that the chassis is 3D printed and designed to hold the board, battery, and motors compactly. Use the three diagrams to show equal speed, unequal speed, and opposite wheel directions.

## Slide 6: FreeRTOS Task Architecture

Explain that the firmware is split into three FreeRTOS tasks. The communication task handles HTTP commands, the control task interprets commands and computes motor targets, and the motor task drives the H-bridges. Point out that shared buffers are protected by mutexes so tasks do not write over each other.

## Slide 7: Differential-Drive Motor Mixing

Describe how joystick input is converted into left and right motor commands. The forward/backward value is combined with the rotation value. A positive rotation increases one motor and decreases the other, creating a turn. The example shows a forward-right curve where one wheel moves faster than the other.

## Slide 8: Heading-Stabilised Control - PID on Yaw

Explain that the BNO085 gives a fused yaw angle, which is more useful than raw accelerometer or gyro data alone. The controller compares the desired heading with the measured heading. The PID constants were chosen conservatively so the robot corrects drift without oscillating strongly.

## Slide 9: PID Yaw Feedback Loop

Walk through the feedback loop from left to right. The set heading is compared with measured heading, PID produces a correction, the correction is clamped, and the motor mixer applies it to the wheels. The robot motion changes the measured yaw, closing the loop through the IMU.

## Slide 10: Control Modes - Direct vs. Target

Compare the two control modes. Direct mode is for manual driving from the browser joystick and continuously sends movement commands. Target mode is more autonomous: the user gives a distance and angle, and the robot executes the motion in phases.

## Slide 11: Target Mode - Two-Phase Execution

Explain that target mode is implemented as a simple state machine. First, the robot rotates until the heading error is small enough. Then it drives forward while the PID keeps the heading stable. After the distance is reached, motors stop and the robot returns to idle.

## Slide 12: Start Ramp & Motor Deadband Compensation

Explain why the start ramp is needed: small DC motors may not move reliably at low PWM because of static friction. The algorithm briefly applies a higher boost and then smoothly decays to the requested speed. This makes starts more reliable without keeping the motor overpowered.

## Slide 13: Wi-Fi Access Point & REST API

Describe the communication setup. The robot creates its own Wi-Fi access point, so no external router is required. The browser sends simple HTTP GET requests to control movement, request status, and trigger or clear the emergency stop.

## Slide 14: Communication Flow

Use this slide to explain the message sequence. The browser sends movement commands frequently, status is polled less often, target commands start autonomous motion, and emergency stop has a dedicated endpoint. Emphasize that the API is simple enough to test from any browser.

## Slide 15: Web-Based Control Interface

Explain that the control application is a single-page web interface. It works on a phone or laptop and supports both mouse and touch input. Mention the main controls: connection, joystick, target command panel, emergency stop, and live status.

## Slide 16: Safety Mechanisms

Present safety as a core part of the design. Emergency stop overrides normal motion, command timeout stops the robot if communication is lost, mutexes protect shared state, and joystick auto-centering prevents the robot from continuing to move after input is released.

## Slide 17: Summary

Summarize the project as a self-contained differential-drive robot built from a compact board with integrated wireless control, motor driving, and IMU feedback. Reinforce the main technical achievements: FreeRTOS architecture, PID heading control, browser control, and safety mechanisms.

## Slide 18: Future Work

Mention practical extensions. Obstacle avoidance could use the proximity sensor, odometry could improve distance estimation, multi-waypoint navigation would make target mode more useful, and BLE/Zigbee or a smartphone app could improve the user experience.

## Slide 19: Thank You

Close by inviting questions. Be ready to answer about PID tuning, communication reliability, mechanical design choices, and how the robot estimates distance in target mode.
