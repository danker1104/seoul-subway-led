// Runtime for the UNO R4 WiFi. The large, generated lookup table is in route_map.h.
namespace {
constexpr uint8_t PIN_ROTATION = A0, PIN_BUTTON = 2, PIN_BUZZER = 5;
constexpr uint32_t GPS_BAUD = 9600, WIFI_RETRY_MS = 10000;
constexpr uint32_t POSITION_REFRESH_MS = 60000, ETA_REFRESH_MS = 120000;
constexpr uint32_t STALE_MS = 180000, HTTP_TIMEOUT_MS = 10000;
constexpr uint16_t SESSION_LIMIT = 600;
constexpr bool BUZZER_ENABLED = false; // Enable after checking buzzer voltage/current and driver.

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
TinyGPSPlus gps;
enum class Screen : uint8_t { WIFI, GPS, FIND_STOP, ROUTES, DIRECTIONS, VEHICLES, TRACK, ERROR };
Screen screen = Screen::WIFI, resumeScreen = Screen::GPS;
char errorText[25] = "";
uint32_t wifiAttemptMs = 0, displayMs = 0, gpsSampleMs = 0;
uint32_t buttonChangedMs = 0, buttonDownMs = 0, nextRetryMs = 0;
uint32_t posPollMs = 0, etaPollMs = 0, posGoodMs = 0;
uint16_t apiCalls = 0, knob = 0;
uint8_t gpsCount = 0, menuIndex = 0, routeCount = 0, vehicleCount = 0;
bool buttonRaw = HIGH, buttonStable = HIGH, longHandled = false;
bool gpsReady = false, etaAvailable = false, buzzerPlayed = false, vehiclePresent = false;
bool vehicleListLoaded = false;
bool apiQuotaExceeded = false;
double gpsLat[5], gpsLon[5], latitude = 0, longitude = 0;
uint64_t stopId = 0;
char stopName[42] = "", stopIdText[16] = "";
uint16_t stopIndex = 0, routeChoices[64], chosenRoute = 0;
char routeId[12] = "", routeNo[16] = "";
int targetSeq[2] = {-1, -1}, startSeq[2] = {-1, -1};
int routeSequences[2][256];
uint16_t routeSequenceCount[2] = {0, 0};
uint8_t direction = 0;
struct Vehicle { char id[20]; int seq; };
Vehicle vehicles[24];
char selectedVehicle[20] = "";
int selectedSeq = -1, etaSeconds = -1;

void copyText(char* destination, const char* source, size_t capacity) {
  if (!capacity) return;
  strncpy(destination, source ? source : "", capacity - 1);
  destination[capacity - 1] = 0;
}

void setError(const char* message, Screen resume) {
  strncpy(errorText, message, sizeof(errorText) - 1);
  errorText[sizeof(errorText) - 1] = 0;
  resumeScreen = resume;
  screen = Screen::ERROR;
  nextRetryMs = millis() + 30000;
  Serial.print(F("Tracker error: ")); Serial.println(errorText);
}

String encodePart(const char* value) {
  String out;
  while (*value) {
    uint8_t c = static_cast<uint8_t>(*value++);
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') out += char(c);
    else { char hex[4]; snprintf(hex, sizeof(hex), "%%%02X", c); out += hex; }
  }
  return out;
}

// This reader exposes only HTTP body bytes, including dechunking when required.
class ApiReader {
 public:
  WiFiSSLClient client;
  bool chunked = false, ended = false;
  long remaining = -1, chunkLeft = 0;
  uint32_t lastByteMs = 0;
  int status = 0;
  uint32_t bytes = 0;
  bool complete = false;

  int raw() {
    uint32_t started = millis();
    while (!client.available()) {
      if (!client.connected() || (uint32_t)(millis() - started) >= HTTP_TIMEOUT_MS) return -1;
      delay(1);
    }
    return client.read();
  }
  String rawLine() {
    String line;
    while (line.length() < 160) {
      int c = raw();
      if (c < 0) return String();
      if (c == '\n') return line;
      if (c != '\r') line += char(c);
    }
    return String();
  }
  bool begin(const char* endpoint, const char* paramName = nullptr, const char* paramValue = nullptr) {
    if (WiFi.status() != WL_CONNECTED || !DAEGU_API_KEY[0] ||
        apiCalls >= SESSION_LIMIT || apiQuotaExceeded) return false;
    client.setTimeout(HTTP_TIMEOUT_MS);
    if (!client.connect("apis.data.go.kr", 443)) return false;
    String path = String("/6270000/dbmsapi02/") + endpoint + "?serviceKey=" + encodePart(DAEGU_API_KEY);
    if (paramName) path += String("&") + paramName + "=" + encodePart(paramValue);
    ++apiCalls; // Count before transmission, including failed responses.
    client.print(String("GET ") + path + " HTTP/1.1\r\nHost: apis.data.go.kr\r\nConnection: close\r\nAccept: application/json\r\nAccept-Encoding: identity\r\n\r\n");
    // Never print path: it contains the API key.
    String line = rawLine();
    if (!line.startsWith("HTTP/1.")) { client.stop(); return false; }
    status = line.substring(9, 12).toInt();
    for (uint8_t n = 0; n < 40; ++n) {
      line = rawLine();
      if (line.length() == 0) break;
      line.toLowerCase();
      if (line.startsWith("content-length:")) remaining = line.substring(15).toInt();
      if (line.startsWith("transfer-encoding:") && line.indexOf("chunked") >= 0) chunked = true;
      if (line.startsWith("content-encoding:") && line.indexOf("gzip") >= 0) { client.stop(); return false; }
    }
    if (status == 429) apiQuotaExceeded = true;
    if (status != 200) { client.stop(); return false; }
    return true;
  }
  int get() {
    if (ended || remaining == 0) { if (remaining == 0) complete = true; return -1; }
    if (chunked && chunkLeft == 0) {
      String line = rawLine();
      if (!line.length()) { ended = true; return -1; }
      chunkLeft = strtol(line.c_str(), nullptr, 16);
      if (chunkLeft <= 0) { ended = true; complete = true; return -1; }
    }
    int c = raw();
    if (c < 0) { ended = true; return -1; }
    ++bytes;
    if (remaining > 0) --remaining;
    if (chunked && --chunkLeft == 0) { if (raw() != '\r' || raw() != '\n') ended = true; }
    return c;
  }
  void close() { client.stop(); }
};

// Finds a JSON array key. Quoted text inside JSON strings contains escapes, so
// the exact unescaped key pattern cannot be mistaken for text in a value.
bool findArray(ApiReader& stream, const char* key) {
  char token[20];
  snprintf(token, sizeof(token), "\"%s\"", key);
  uint8_t match = 0;
  const uint8_t length = strlen(token);
  char prefix[350]; size_t prefixUsed = 0;
  int c;
  while ((c = stream.get()) >= 0) {
    if (prefixUsed + 1 < sizeof(prefix)) prefix[prefixUsed++] = char(c);
    if (c == token[match]) ++match;
    else match = (c == token[0]) ? 1 : 0;
    if (match == length) {
      do { c = stream.get(); } while (c == ' ' || c == '\r' || c == '\n');
      if (c != ':') { match = 0; continue; }
      do { c = stream.get(); } while (c == ' ' || c == '\r' || c == '\n');
      if (c == '[') return true;
      match = 0;
    }
  }
  prefix[prefixUsed] = 0;
  if (strstr(prefix, "\"resultCode\":\"22\"") ||
      strstr(prefix, "LIMITED_NUMBER_OF_SERVICE_REQUESTS_EXCEEDS_ERROR")) apiQuotaExceeded = true;
  return false;
}

// Returns 1 for an object, 0 at the array end, -1 for malformed/oversized data.
int nextObject(ApiReader& stream, char* out, size_t capacity) {
  int c;
  do { c = stream.get(); } while (c == ',' || c == ' ' || c == '\n' || c == '\r');
  if (c == ']') return 0;
  if (c != '{') return -1;
  size_t used = 0;
  int depth = 0;
  bool inString = false, escape = false;
  do {
    if (used + 1 >= capacity) return -1;
    out[used++] = char(c);
    if (inString) {
      if (escape) escape = false;
      else if (c == '\\') escape = true;
      else if (c == '"') inString = false;
    } else {
      if (c == '"') inString = true;
      else if (c == '{') ++depth;
      else if (c == '}') --depth;
    }
    if (depth == 0) { out[used] = 0; return 1; }
    c = stream.get();
  } while (c >= 0);
  return -1;
}

bool nearestStop() {
  ApiReader stream;
  if (!stream.begin("getBasic02")) return false;
  bool found = findArray(stream, "bs");
  double bestM = 500.0;
  uint64_t bestId = 0;
  char bestName[42] = "";
  char object[400];
  int result = -1;
  if (found) while ((result = nextObject(stream, object, sizeof(object))) == 1) {
    JsonDocument doc;
    if (deserializeJson(doc, object)) { found = false; break; }
    const char* id = doc["bsId"] | "";
    double lon = doc["xPos"] | 0.0, lat = doc["yPos"] | 0.0;
    if (!*id || lon < 125 || lon > 131 || lat < 33 || lat > 39) continue;
    double meters = TinyGPSPlus::distanceBetween(latitude, longitude, lat, lon);
    if (meters < bestM) {
      bestM = meters; bestId = strtoull(id, nullptr, 10);
      copyText(bestName, doc["bsNm"] | "", sizeof(bestName));
    }
  }
  // The base response has more arrays after bs. Consume them to verify the
  // complete 3 MB download before accepting a nearest stop.
  int lastNonWhitespace = -1;
  if (found && result == 0) {
    int c;
    while ((c = stream.get()) >= 0) if (c != ' ' && c != '\r' && c != '\n') lastNonWhitespace = c;
  }
  if (!found || result != 0 || lastNonWhitespace != '}' || !stream.complete) {
    stream.close(); return false;
  }
  stream.close();
  if (!bestId) { setError("No stop within 500m", Screen::GPS); return true; }
  for (uint16_t i = 0; i < STOP_COUNT; ++i) if (STOP_IDS[i] == bestId) {
    stopIndex = i; stopId = bestId;
    copyText(stopName, bestName, sizeof(stopName));
    snprintf(stopIdText, sizeof(stopIdText), "%llu", (unsigned long long)bestId);
    routeCount = 0;
    for (uint16_t r = 0; r < ROUTE_COUNT; ++r) {
      for (uint16_t n = ROUTE_OFFSETS[r]; n < ROUTE_OFFSETS[r + 1]; ++n) {
        if (ROUTE_STOPS[n] != stopIndex) continue;
        if (routeCount >= 64) { setError("Too many routes", Screen::FIND_STOP); return true; }
        routeChoices[routeCount++] = r;
        break;
      }
    }
    if (!routeCount) { setError("No routes for stop", Screen::FIND_STOP); return true; }
    menuIndex = 0; screen = Screen::ROUTES;
    Serial.print(F("Nearest stop: ")); Serial.print(stopName);
    Serial.print(F(" routes=")); Serial.println(routeCount);
    return true;
  }
  setError("Stop map out of date", Screen::GPS);
  return true;
}

bool loadDirections() {
  ApiReader stream;
  if (!stream.begin("getBs02", "routeId", routeId)) return false;
  if (!findArray(stream, "items")) { stream.close(); return false; }
  targetSeq[0] = targetSeq[1] = -1;
  startSeq[0] = startSeq[1] = -1;
  routeSequenceCount[0] = routeSequenceCount[1] = 0;
  char object[400]; int result;
  while ((result = nextObject(stream, object, sizeof(object))) == 1) {
    JsonDocument doc;
    if (deserializeJson(doc, object)) { stream.close(); return false; }
    int d = atoi(doc["moveDir"] | "-1");
    if (d < 0 || d > 1) continue;
    int seq = atoi(doc["seq"] | "-1");
    if (seq < 0) continue;
    if (routeSequenceCount[d] >= 256) { stream.close(); return false; }
    routeSequences[d][routeSequenceCount[d]++] = seq;
    if (startSeq[d] < 0 || seq < startSeq[d]) startSeq[d] = seq;
    const char* bs = doc["bsId"] | "";
    if (strtoull(bs, nullptr, 10) == stopId) targetSeq[d] = seq;
  }
  stream.close();
  if (result != 0) return false;
  if (targetSeq[0] < 0 && targetSeq[1] < 0) return false;
  if (targetSeq[0] >= 0 && targetSeq[1] >= 0) { menuIndex = 0; screen = Screen::DIRECTIONS; }
  else { direction = targetSeq[0] >= 0 ? 0 : 1; screen = Screen::VEHICLES; vehicleCount = 0; vehicleListLoaded = false; }
  return true;
}

bool loadVehicles(bool initial) {
  ApiReader stream;
  if (!stream.begin("getPos02", "routeId", routeId)) return false;
  if (!findArray(stream, "items")) { stream.close(); return false; }
  char object[500]; int result;
  uint8_t count = 0;
  bool foundSelected = false;
  int newSeq = selectedSeq;
  while ((result = nextObject(stream, object, sizeof(object))) == 1) {
    JsonDocument doc;
    if (deserializeJson(doc, object)) { stream.close(); return false; }
    int d = atoi(doc["moveDir"] | "-1");
    if (d != direction) continue;
    const char* id = doc["vhcNo2"] | "";
    int seq = doc["seq"] | -1;
    if (!*id || seq < 0 || seq > targetSeq[direction]) continue;
    if (initial) {
      if (count >= 24) { stream.close(); setError("Too many vehicles", Screen::ROUTES); return true; }
      copyText(vehicles[count].id, id, sizeof(vehicles[count].id));
      vehicles[count].seq = seq;
      ++count;
    } else if (strcmp(id, selectedVehicle) == 0) {
      foundSelected = true; newSeq = seq;
    }
  }
  stream.close();
  if (result != 0) return false;
  if (initial) { vehicleCount = count; menuIndex = 0; vehicleListLoaded = true; }
  else {
    vehiclePresent = foundSelected;
    if (foundSelected) { selectedSeq = newSeq; posGoodMs = millis(); }
  }
  posPollMs = millis();
  return true;
}

bool loadEta() {
  ApiReader stream;
  if (!stream.begin("getRealtime02", "bsId", stopIdText)) return false;
  if (!findArray(stream, "items")) { stream.close(); return false; }
  char object[3000]; int result;
  bool matched = false;
  int seconds = -1;
  while ((result = nextObject(stream, object, sizeof(object))) == 1) {
    JsonDocument doc;
    if (deserializeJson(doc, object)) { stream.close(); return false; }
    const char* number = doc["routeNo"] | "";
    if (strcmp(number, routeNo) != 0) continue;
    JsonVariant arr = doc["arrList"];
    if (arr.is<JsonArray>()) for (JsonVariant entry : arr.as<JsonArray>()) {
      const char* id = entry["vhcNo2"] | "";
      if (strcmp(id, selectedVehicle) == 0) {
        seconds = entry["arrTime"] | -1;
        matched = seconds > 0;
      }
    }
    else if (arr.is<JsonObject>()) {
      const char* id = arr["vhcNo2"] | "";
      if (strcmp(id, selectedVehicle) == 0) {
        seconds = arr["arrTime"] | -1;
        matched = seconds > 0;
      }
    }
  }
  stream.close();
  if (result != 0) return false;
  etaAvailable = matched;
  etaSeconds = matched ? seconds : -1;
  etaPollMs = millis();
  if (BUZZER_ENABLED && matched && seconds <= 180 && !buzzerPlayed) {
    tone(PIN_BUZZER, 2200, 180);
    buzzerPlayed = true;
  }
  return true;
}

uint8_t choiceFromKnob(uint8_t count) {
  if (!count) return 0;
  uint32_t value = uint32_t(knob) * count / 1024U;
  return value >= count ? count - 1 : value;
}

void drawBus(uint8_t x, uint8_t y) {
  oled.drawFrame(x + 2, y, 18, 10);
  oled.drawBox(x + 5, y + 2, 4, 3);
  oled.drawBox(x + 12, y + 2, 4, 3);
  oled.drawDisc(x + 6, y + 10, 2);
  oled.drawDisc(x + 16, y + 10, 2);
}
void drawStop() {
  oled.drawVLine(111, 35, 21);
  oled.drawFrame(107, 35, 9, 8);
  oled.drawHLine(105, 55, 14);
}

void drawScreen() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  if (screen == Screen::TRACK) {
    oled.setCursor(2, 10); oled.print(routeNo); oled.print(" BUS ");
    size_t len = strlen(selectedVehicle);
    oled.print(len > 4 ? selectedVehicle + len - 4 : selectedVehicle);
    oled.drawHLine(0, 13, 128);
    oled.setCursor(2, 27);
    if (etaAvailable) { oled.print("ETA "); oled.print((etaSeconds + 59) / 60); oled.print("m"); }
    else oled.print("ETA --");
    oled.setCursor(79, 27); oled.print("STOP");
    oled.drawHLine(6, 46, 114);
    drawStop();
    if (vehiclePresent && selectedSeq >= 0 && targetSeq[direction] > startSeq[direction]) {
      int num = selectedSeq - startSeq[direction];
      int den = targetSeq[direction] - startSeq[direction];
      int x = 4 + constrain(num * 91 / den, 0, 91);
      drawBus(x, 34);
      uint16_t remainingStops = 0;
      for (uint16_t i = 0; i < routeSequenceCount[direction]; ++i)
        if (routeSequences[direction][i] > selectedSeq &&
            routeSequences[direction][i] <= targetSeq[direction]) ++remainingStops;
      oled.setCursor(2, 63); oled.print(remainingStops); oled.print(" STOPS AWAY");
    } else { oled.setCursor(2, 63); oled.print("BUS POSITION --"); }
    if (!vehiclePresent) { oled.setCursor(2, 39); oled.print("BUS NOT FOUND"); }
    else if ((uint32_t)(millis() - posGoodMs) > STALE_MS) {
      oled.setCursor(2, 39); oled.print("DATA STALE");
    }
  } else {
    oled.drawStr(0, 10, "Daegu Bus Tracker"); oled.drawHLine(0, 13, 128);
    switch (screen) {
      case Screen::WIFI: oled.drawStr(0, 30, "Wi-Fi connecting"); break;
      case Screen::GPS:
        oled.drawStr(0, 30, "Waiting for GPS");
        oled.setCursor(0, 47); oled.print("Samples "); oled.print(gpsCount); oled.print("/5"); break;
      case Screen::FIND_STOP: oled.drawStr(0, 30, "Finding nearest stop"); break;
      case Screen::ROUTES:
        oled.setCursor(0, 26); oled.print("STOP ");
        if (strlen(stopIdText) > 4) oled.print(stopIdText + strlen(stopIdText) - 4);
        else oled.print(stopIdText);
        oled.drawStr(0, 38, "Select route (A0/D2)");
        if (routeCount) {
          char label[16]; copyText(label, ROUTE_NUMBERS[routeChoices[menuIndex]], sizeof(label));
          oled.setCursor(0, 51); oled.print("> "); oled.print(label);
          oled.setCursor(0, 60); oled.print(menuIndex + 1); oled.print("/"); oled.print(routeCount);
        } break;
      case Screen::DIRECTIONS:
        oled.drawStr(0, 28, "Select direction");
        oled.setCursor(0, 48); oled.print("> "); oled.print(menuIndex == 0 ? "Forward" : "Reverse"); break;
      case Screen::VEHICLES:
        oled.drawStr(0, 27, "Select bus (A0/D2)");
        if (vehicleCount) { oled.setCursor(0, 45); oled.print("> "); oled.print(vehicles[menuIndex].id); }
        else oled.drawStr(0, 47, "No active buses"); break;
      case Screen::ERROR:
        oled.drawStr(0, 30, errorText);
        oled.drawStr(0, 47, "Retrying shortly"); break;
      default: break;
    }
  }
  oled.sendBuffer();
}

void shortPress() {
  if (screen == Screen::ROUTES && routeCount) {
    chosenRoute = routeChoices[menuIndex];
    copyText(routeId, ROUTE_IDS[chosenRoute], sizeof(routeId));
    copyText(routeNo, ROUTE_NUMBERS[chosenRoute], sizeof(routeNo));
    if (!loadDirections()) setError("Route stops failed", Screen::ROUTES);
  } else if (screen == Screen::DIRECTIONS) {
    direction = menuIndex; screen = Screen::VEHICLES; vehicleListLoaded = false;
    if (!loadVehicles(true)) setError("Bus list failed", Screen::DIRECTIONS);
  } else if (screen == Screen::VEHICLES && vehicleCount) {
    copyText(selectedVehicle, vehicles[menuIndex].id, sizeof(selectedVehicle));
    selectedSeq = vehicles[menuIndex].seq;
    vehiclePresent = true;
    posGoodMs = posPollMs = millis();
    etaAvailable = false; etaSeconds = -1; buzzerPlayed = false;
    screen = Screen::TRACK;
    if (!loadEta()) Serial.println(F("ETA lookup failed; tracking position only."));
  }
}

void longPress() {
  if (screen == Screen::TRACK) { screen = Screen::VEHICLES; etaAvailable = false; }
  else if (screen == Screen::VEHICLES || screen == Screen::DIRECTIONS) screen = Screen::ROUTES;
}

void updateButton(uint32_t now) {
  bool raw = digitalRead(PIN_BUTTON);
  if (raw != buttonRaw) { buttonRaw = raw; buttonChangedMs = now; }
  if ((uint32_t)(now - buttonChangedMs) >= 40 && raw != buttonStable) {
    buttonStable = raw;
    if (raw == LOW) { buttonDownMs = now; longHandled = false; }
    else if (!longHandled) shortPress();
  }
  if (buttonStable == LOW && !longHandled && (uint32_t)(now - buttonDownMs) >= 1000) {
    longHandled = true; longPress();
  }
}

void updateGps(uint32_t now) {
  while (Serial1.available()) gps.encode(Serial1.read());
  if (screen != Screen::GPS || !gps.location.isUpdated() || !gps.location.isValid() ||
      gps.location.age() > 3000) return;
  if (gpsSampleMs && (uint32_t)(now - gpsSampleMs) < 1000) return;
  gpsSampleMs = now ? now : 1;
  gpsLat[gpsCount] = gps.location.lat(); gpsLon[gpsCount] = gps.location.lng();
  if (++gpsCount < 5) return;
  double lat = 0, lon = 0;
  for (uint8_t i = 0; i < 5; ++i) { lat += gpsLat[i]; lon += gpsLon[i]; }
  lat /= 5; lon /= 5;
  for (uint8_t i = 0; i < 5; ++i) {
    if (TinyGPSPlus::distanceBetween(lat, lon, gpsLat[i], gpsLon[i]) > 30) { gpsCount = 0; return; }
  }
  latitude = lat; longitude = lon; gpsReady = true; screen = Screen::FIND_STOP;
}

void updateWifi(uint32_t now) {
  if (WiFi.status() == WL_CONNECTED) {
    if (screen == Screen::WIFI) screen = resumeScreen;
    return;
  }
  if (screen != Screen::WIFI) { resumeScreen = screen; screen = Screen::WIFI; }
  if (wifiAttemptMs && (uint32_t)(now - wifiAttemptMs) < WIFI_RETRY_MS) return;
  wifiAttemptMs = now ? now : 1;
  if (WIFI_SSID[0]) WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}
} // namespace

void trackerSetup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  Serial.begin(115200);
  Serial1.begin(GPS_BAUD);
  Wire.begin();
  oled.begin();
  knob = analogRead(PIN_ROTATION);
  drawScreen();
}

void trackerLoop() {
  uint32_t now = millis();
  updateGps(now);
  updateButton(now);
  knob = (uint32_t(knob) * 7 + analogRead(PIN_ROTATION)) / 8;
  if (screen == Screen::ROUTES) menuIndex = choiceFromKnob(routeCount);
  if (screen == Screen::DIRECTIONS) menuIndex = choiceFromKnob(2);
  if (screen == Screen::VEHICLES) menuIndex = choiceFromKnob(vehicleCount);
  updateWifi(now);
  if (screen == Screen::FIND_STOP) {
    if (!nearestStop() && screen == Screen::FIND_STOP) setError("Stop lookup failed", Screen::FIND_STOP);
  }
  if (screen == Screen::VEHICLES && !vehicleListLoaded && chosenRoute < ROUTE_COUNT) {
    if (!loadVehicles(true)) setError("Bus list failed", Screen::VEHICLES);
  }
  if (screen == Screen::TRACK) {
    if ((uint32_t)(now - posPollMs) >= POSITION_REFRESH_MS) {
      posPollMs = now;
      if (!loadVehicles(false)) Serial.println(F("Position refresh failed."));
    }
    if (etaAvailable && (uint32_t)(now - etaPollMs) >= ETA_REFRESH_MS) {
      etaPollMs = now;
      if (!loadEta()) { etaAvailable = false; etaSeconds = -1; }
    }
  }
  if (screen == Screen::ERROR && !apiQuotaExceeded &&
      (int32_t)(now - nextRetryMs) >= 0) screen = resumeScreen;
  if ((uint32_t)(now - displayMs) >= 200) { displayMs = now; drawScreen(); }
}
