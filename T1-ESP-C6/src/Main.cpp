#include <Arduino.h>
#include <CodeCell.h>

#include "controller.h"
#include "robotWebServer.h"

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);
  delay(500);

  const bool initialized = initController();

  myCodeCell.Init(
    LIGHT +
    MOTION_ROTATION +
    MOTION_LINEAR_ACC
  );

  myCodeCell.Drive1.Init();
  myCodeCell.Drive2.Init();

  myCodeCell.Drive1.Drive(true, 0);
  myCodeCell.Drive2.Drive(true, 0);
  if (!initialized) { Serial.println("Controller initialization failed; drives disabled"); return; }

  if (xTaskCreate(motorTask, "MotorTask", 2048, NULL, 3, NULL) != pdPASS
      || xTaskCreate(controlTask, "ControlTask", 4096, NULL, 2, NULL) != pdPASS
      || xTaskCreate(communicationTask, "CommunicationTask", 8192, NULL, 1, NULL) != pdPASS) {
    controllerFaultNow();
    Serial.println("Task creation failed; controller fault latched, drives disabled");
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}