#include <Arduino.h>
#include <CodeCell.h>

#include "controller.h"
#include "robotWebServer.h"

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);
  delay(500);

  initController();

  myCodeCell.Init(
    LIGHT +
    MOTION_ROTATION +
    MOTION_LINEAR_ACC
  );

  myCodeCell.Drive1.Init();
  myCodeCell.Drive2.Init();

  setDirectControl(0, 0);

  xTaskCreate(motorTask, "MotorTask", 2048, NULL, 3, NULL);
  xTaskCreate(controlTask, "ControlTask", 4096, NULL, 2, NULL);
  xTaskCreate(communicationTask, "CommunicationTask", 8192, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}