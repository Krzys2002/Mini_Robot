#include <Arduino.h>
#include <CodeCell.h>
#include <WiFi.h>
#include <WebServer.h>

CodeCell myCodeCell;

#define PROXIMITY_RANGE 1000U
#define MOTOR_DEADZONE  30U
#define MOTOR_MAXSPEED  90U

// Hotspot settings
const char* AP_SSID = "CodeCell-C6-Drive";
const char* AP_PASS = "12345678";   // minimum 8 chars

WebServer server(80);

// ---------------- Shared command buffer ----------------

struct MotorCommand
{
    bool m1Forward;
    bool m2Forward;
    uint8_t m1Speed;
    uint8_t m2Speed;
    uint32_t sequence;
    bool changed;
};

MotorCommand commandBuffer =
{
    true,
    true,
    0,
    0,
    0,
    false
};

SemaphoreHandle_t commandMutex;

// ---------------- Helpers ----------------

uint8_t clampSpeed(int value)
{
    if (value < 0) return 0;
    if (value > 100) return 100;
    return (uint8_t)value;
}

void writeCommand(bool m1Forward, uint8_t m1Speed,
                  bool m2Forward, uint8_t m2Speed)
{

    if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE)
    {
        commandBuffer.m1Forward = m1Forward;
        commandBuffer.m1Speed = m1Speed;
        commandBuffer.m2Forward = m2Forward;
        commandBuffer.m2Speed = m2Speed;
        commandBuffer.sequence++;
        commandBuffer.changed = true;
        xSemaphoreGive(commandMutex);
    }

}

MotorCommand readCommand()
{
    MotorCommand copy;
    if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE)
    {
        copy = commandBuffer;
        commandBuffer.changed = false;
        xSemaphoreGive(commandMutex);
    }
    return copy;
}

uint8_t squaredRampSpeed(uint32_t elapsedMs,
                         uint8_t startSpeed,
                         uint8_t endSpeed,
                         uint32_t rampTimeMs)
{
    if (elapsedMs >= rampTimeMs) return endSpeed;
    float t = (float)elapsedMs / (float)rampTimeMs;
    float curve = t * t;   // squared time curve
    float speed = startSpeed + ((float)endSpeed - startSpeed) * curve;

    return (uint8_t)round(speed);
}

// ---------------- HTTP communication ----------------

//

// Example command from phone/browser:

// http://192.168.4.1/set?m1=60&m2=60

// http://192.168.4.1/set?m1=-50&m2=75

//

// Negative value = reverse

// Positive value = forward

// 0 = stop

void handleSetCommand()
{
    if (!server.hasArg("m1") || !server.hasArg("m2"))
    {
      server.send(400, "text/plain",
                  "ERROR: use /set?m1=-100..100&m2=-100..100");
      return;
    }

    int rawM1 = server.arg("m1").toInt();
    int rawM2 = server.arg("m2").toInt();

    rawM1 = constrain(rawM1, -100, 100);
    rawM2 = constrain(rawM2, -100, 100);

    bool m1Forward = rawM1 >= 0;
    bool m2Forward = rawM2 >= 0;

    uint8_t m1Speed = clampSpeed(abs(rawM1));
    uint8_t m2Speed = clampSpeed(abs(rawM2));

    writeCommand(m1Forward, m1Speed, m2Forward, m2Speed);
    MotorCommand current = readCommand();

    String reply;

    reply += "OK\n";
    reply += "Command accepted\n";
    reply += "SEQ=" + String(current.sequence) + "\n";
    reply += "M1=" + String(current.m1Forward ? "FWD" : "REV");
    reply += " SPEED=" + String(current.m1Speed) + "\n";
    reply += "M2=" + String(current.m2Forward ? "FWD" : "REV");
    reply += " SPEED=" + String(current.m2Speed) + "\n";
    server.send(200, "text/plain", reply);

}

void handleStatus()
{
    MotorCommand current = readCommand();
    String reply;

    reply += "SEQ=" + String(current.sequence) + "\n";
    reply += "M1=" + String(current.m1Forward ? "FWD" : "REV");
    reply += " SPEED=" + String(current.m1Speed) + "\n";
    reply += "M2=" + String(current.m2Forward ? "FWD" : "REV");
    reply += " SPEED=" + String(current.m2Speed) + "\n";
    server.send(200, "text/plain", reply);
}

// ---------------- FreeRTOS tasks ----------------

void motorTask(void* parameter)
{
    uint32_t rampStartTime = millis();

    const uint8_t START_SPEED = 70;
    const uint8_t END_SPEED = 30;
    const uint32_t RAMP_TIME_MS = 2000;

    while (true)
    {
        if (myCodeCell.Run(20))
        {
            uint32_t elapsed = millis() - rampStartTime;
            uint8_t rampSpeed = squaredRampSpeed(
              elapsed,
              START_SPEED,
              END_SPEED,
              RAMP_TIME_MS
            );

            // Motors run forward with ramped speed
            myCodeCell.Drive1.Drive(true, rampSpeed);
            myCodeCell.Drive2.Drive(true, rampSpeed);

            Serial.printf(
              "Ramp time: %lu ms | Motor speed: %u%%\n",
              elapsed,
              rampSpeed
            );

        }

        vTaskDelay(pdMS_TO_TICKS(5));

    }

}

void communicationTask(void* parameter)
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);

    Serial.println("Hotspot started");
    Serial.print("SSID: ");
    Serial.println(AP_SSID);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/set", HTTP_GET, handleSetCommand);
    server.on("/status", HTTP_GET, handleStatus);
    server.begin();

    while (true)
    {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------- Arduino setup / loop ----------------

void setup()
{
    Serial.begin(115200);
    delay(500);
    commandMutex = xSemaphoreCreateMutex();

    if (commandMutex == NULL)
    {
      Serial.println("ERROR: mutex creation failed");
      while (true)
      {
        delay(1000);
      }
    }


    myCodeCell.Init(LIGHT);
    myCodeCell.Drive1.Init();
    myCodeCell.Drive2.Init();
    writeCommand(true, 0, true, 0);

    xTaskCreate(
        motorTask,
        "MotorTask",
        4096,
        NULL,
        2,
        NULL
    );

    xTaskCreate(
        communicationTask,
        "CommunicationTask",
        8192,
        NULL,
        1,
        NULL
    );

}

void loop()
{
    // Empty. FreeRTOS tasks do the work.
    vTaskDelay(pdMS_TO_TICKS(1000));
}