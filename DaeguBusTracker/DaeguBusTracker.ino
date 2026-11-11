#include <Arduino.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <TinyGPS++.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>
#include "route_map.h"

// Local credentials. Never print these values to Serial.
const char WIFI_SSID[] = "";
const char WIFI_PASSWORD[] = "";
const char DAEGU_API_KEY[] = "";

void trackerSetup();
void trackerLoop();
void setup() { trackerSetup(); }
void loop() { trackerLoop(); }
