//
// Created by Krzysztof Sawicki on 24/05/2026.
//

#include "robotWebServer.h"
#include "controller.h"

#include <WiFi.h>
#include <WebServer.h>

const char* AP_SSID = "CodeCell-C6-Drive";
const char* AP_PASS = "12345678";

WebServer server(80);

void sendJson(int code, const String& json) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(code, "application/json", json);
}

void handleCorsOptions() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

void handleEmergencyStop() {
  emergencyStopNow();

  sendJson(
    200,
    "{\"ok\":true,\"emergencyStop\":true,\"message\":\"EMERGENCY STOP ACTIVATED\"}"
  );
}

void handleEmergencyClear() {
  clearEmergencyStop();

  sendJson(
    200,
    "{\"ok\":true,\"emergencyStop\":false,\"message\":\"EMERGENCY STOP CLEARED\"}"
  );
}

void handleMoveCommand() {
  if (!server.hasArg("move") || !server.hasArg("rotation")) {
    sendJson(
      400,
      "{\"ok\":false,\"error\":\"Use /move?move=-100..100&rotation=-100..100\"}"
    );
    return;
  }

  int move = constrain(server.arg("move").toInt(), -100, 100);
  int rotation = constrain(server.arg("rotation").toInt(), -100, 100);

  setDirectControl(move, rotation);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"mode\":\"direct\",";
  json += "\"move\":" + String(move) + ",";
  json += "\"rotation\":" + String(rotation);
  json += "}";

  sendJson(200, json);
}

void handleTargetCommand() {
  if (!server.hasArg("distance") || !server.hasArg("angle")) {
    sendJson(
      400,
      "{\"ok\":false,\"error\":\"Use /target?distance=1.0&angle=90&imu=0\"}"
    );
    return;
  }

  float distanceM = server.arg("distance").toFloat();
  float angleDeg = server.arg("angle").toFloat();
  bool useImu = server.hasArg("imu") && server.arg("imu").toInt() == 1;

  setTargetControl(distanceM, angleDeg, useImu);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"mode\":\"target\",";
  json += "\"distanceM\":" + String(distanceM, 3) + ",";
  json += "\"angleDeg\":" + String(angleDeg, 1) + ",";
  json += "\"useImu\":" + String(useImu ? "true" : "false");
  json += "}";

  sendJson(200, json);
}

void handleStatus() {
  RobotStatus s = readRobotStatus();

  String json = "{";
  json += "\"ok\":true,";
  json += "\"batteryMv\":" + String(s.batteryMv) + ",";
  json += "\"batteryPercent\":" + String(s.batteryPercent) + ",";
  json += "\"uptimeS\":" + String(s.uptimeMs / 1000.0f, 1) + ",";
  json += "\"headingDeg\":" + String(s.headingDeg, 1) + ",";
  json += "\"estimatedDistanceM\":" + String(s.estimatedDistanceM, 3) + ",";
  json += "\"move\":" + String(s.move) + ",";
  json += "\"rotation\":" + String(s.rotation) + ",";
  json += "\"m1\":" + String(s.m1Signed) + ",";
  json += "\"m2\":" + String(s.m2Signed);
  json += "}";

  sendJson(200, json);
}

void communicationTask(void* parameter) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println("Hotspot started");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/move", HTTP_GET, handleMoveCommand);
  server.on("/target", HTTP_GET, handleTargetCommand);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/estop", HTTP_GET, handleEmergencyStop);
  server.on("/clear_estop", HTTP_GET, handleEmergencyClear);

  server.on("/move", HTTP_OPTIONS, handleCorsOptions);
  server.on("/target", HTTP_OPTIONS, handleCorsOptions);
  server.on("/status", HTTP_OPTIONS, handleCorsOptions);
  server.on("/estop", HTTP_OPTIONS, handleCorsOptions);
  server.on("/clear_estop", HTTP_OPTIONS, handleCorsOptions);

  server.begin();

  while (true) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}