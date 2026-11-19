#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_system.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
constexpr char WIFI_SSID[] = "cybernetics-rpi";
constexpr char WIFI_PASSWORD[] = "";
constexpr char SCARA_TOKEN[] = "";
#endif

struct Axis {
  const char* label;
  uint8_t stepPin, dirPin, limitPin, positiveLevel;
  int homeSign;                         // +1 or -1 in API coordinates
  int32_t minSteps, maxSteps, seekSteps, releaseSteps, backoffSteps;
};

#if __has_include("machine_config.h")
#include "machine_config.h"
#else
#include "machine_config.example.h"
#endif

#ifndef SCARA_ENABLE_PREHOME_NUDGE
#define SCARA_ENABLE_PREHOME_NUDGE 0
#endif
#ifndef SCARA_MIMIC_Z_MAX_STEPS
#define SCARA_MIMIC_Z_MAX_STEPS 0
#endif
constexpr int32_t MIMIC_Z_MAX = SCARA_MIMIC_Z_MAX_STEPS;
static_assert(MIMIC_Z_MAX >= 0 && MIMIC_Z_MAX <= 136062, "Invalid bottom-referenced Z limit");
constexpr int32_t MIMIC_Z_CHUNK = 30, MIMIC_J1_CHUNK = 8, MIMIC_SWITCH_RELEASE_STEPS = 400;
constexpr uint32_t MIMIC_LEASE_MS = 250;
constexpr int32_t NUDGE_STEPS = 10, NUDGE_BUDGET = 30;

enum State { IDLE, HOMING, MOVING, HOLD, FAULT, ESTOP };
enum HomePhase { SEEK, RELEASE, LATCH, BACKOFF };
constexpr uint32_t LEASE_MS = 2000;
constexpr uint32_t DRILL_LEASE_MS = 2500;  // heartbeat must renew while the drill is on
constexpr uint32_t HOME_FEED_US = 5000, LATCH_FEED_US = 9000;
constexpr uint32_t MIN_FEED_US = 1000, MAX_FEED_US = 20000;
constexpr uint32_t DEBOUNCE_MS = 8;
constexpr uint8_t SPINDLE_PIN = 23;     // active HIGH; off on boot, stop, fault and link loss
constexpr int Z = 0;
constexpr int J1 = 1, J2 = 2;

struct Motion {
  State state = IDLE;
  bool homed = false, pulse = false, debouncing = false;
  bool zReferenced = false, zMustRezero = false, mimicChunk = false, zPressed = false;
  uint32_t zPressedSince = 0;
  int axis = 0, sign = 0;
  int32_t pos[3] = {}, target[3] = {}, remaining = 0;
  uint32_t command = 0, lease = 0, nextEdge = 0, feed = HOME_FEED_US, changedAt = 0;
  HomePhase phase = SEEK;
  bool homeFault = false;
  int32_t nudgeSpent = 0;
  const char* fault = nullptr;
};

WebServer server(80);
portMUX_TYPE motionLock = portMUX_INITIALIZER_UNLOCKED;
Motion motion;
char bootId[17];
bool motionTaskReady = false;
bool profileValid = false;
bool serving = false;
bool drillOn = false;
uint32_t drillLease = 0;
wl_status_t lastNetStatus = WL_IDLE_STATUS;
IPAddress lastIp;
uint32_t lastLinkReport = 0, lastStatusChange = 0;

// Keep Arduino's sketch preprocessor from generating prototypes before these enum types.
const char* stateName(State state);
const char* phaseName(HomePhase phase);
const char* netName(wl_status_t status);
void halt(State state, const char* fault);
void homePhase(HomePhase phase);
void reply(int code, JsonDocument& doc);

const char* stateName(State state) {
  switch (state) {
    case IDLE: return "IDLE";
    case HOMING: return "HOMING";
    case MOVING: return "MOVING";
    case HOLD: return "HOLD";
    case FAULT: return "FAULT";
    case ESTOP: return "ESTOP";
  }
  return "FAULT";
}

const char* phaseName(HomePhase phase) {
  switch (phase) {
    case SEEK: return "SEEK";
    case RELEASE: return "RELEASE";
    case LATCH: return "LATCH";
    case BACKOFF: return "BACKOFF";
  }
  return nullptr;
}

const char* netName(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "starting";
    case WL_NO_SSID_AVAIL: return "SSID not found";
    case WL_SCAN_COMPLETED: return "scan complete";
    case WL_CONNECTED: return "connected";
    case WL_CONNECT_FAILED: return "authentication failed";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_DISCONNECTED: return "disconnected, retrying";
    default: return "unknown";
  }
}

bool validProfile() {
  const Axis& z = AXES[Z];
  // Z has no homing phase and no upper switch. It is an unreferenced, bounded
  // step axis; pos.z = 0 is the boot datum, not a measured height.
  if (!MOTION_COMMISSIONED ||
      z.homeSign < -1 || z.homeSign > 1 || z.positiveLevel > HIGH ||
      z.minSteps >= z.maxSteps || z.minSteps > 0 || z.maxSteps < 0 ||
      z.minSteps < -1000000 || z.maxSteps > 1000000 ||
      z.seekSteps != 0 || z.releaseSteps != 0 || z.backoffSteps != 0) return false;
  for (int i = J1; i <= J2; ++i) {
    const Axis& axis = AXES[i];
    if ((axis.homeSign != 1 && axis.homeSign != -1) ||
        axis.positiveLevel > HIGH || axis.minSteps > 0 || axis.maxSteps < 0 ||
        axis.minSteps < -1000000 || axis.maxSteps > 1000000 ||
        axis.seekSteps <= 0 || axis.seekSteps > 1000000 ||
        axis.backoffSteps <= 0 || axis.releaseSteps < axis.backoffSteps ||
        axis.releaseSteps > axis.seekSteps) return false;
  }
  return true;
}

bool active() { return motion.state == HOMING || motion.state == MOVING; }
bool triggered(int axis) { return digitalRead(AXES[axis].limitPin) == LOW; }

// All functions below that mutate motion run under motionLock.
void halt(State state, const char* fault) {
  motion.homeFault = state == FAULT && motion.state == HOMING;
  for (int i = 0; i < 3; ++i) {
    digitalWrite(AXES[i].stepPin, LOW);
    motion.target[i] = motion.pos[i];
  }
  digitalWrite(SPINDLE_PIN, LOW);
  drillOn = false;
  drillLease = 0;
  motion.state = state;
  motion.fault = fault;
  motion.homed = false;
  if (motion.zReferenced) {
    // The old boot-relative datum was overwritten; require a new zero instead
    // of falling back to legacy unreferenced targets with a stale count.
    motion.zReferenced = false;
    motion.zMustRezero = true;
  }
  motion.mimicChunk = false;
  motion.pulse = false;
  ++motion.command;
}

void direction(int sign, uint32_t feed) {
  const Axis& axis = AXES[motion.axis];
  motion.sign = sign;
  motion.feed = feed;
  digitalWrite(axis.dirPin, sign > 0 ? axis.positiveLevel : !axis.positiveLevel);
  motion.nextEdge = micros() + feed;
}

void homePhase(HomePhase phase) {
  const Axis& axis = AXES[motion.axis];
  motion.phase = phase;
  motion.debouncing = false;
  switch (phase) {
    case SEEK:    motion.remaining = axis.seekSteps;    direction(axis.homeSign, HOME_FEED_US); break;
    case RELEASE: motion.remaining = axis.releaseSteps; direction(-axis.homeSign, LATCH_FEED_US); break;
    case LATCH:   motion.remaining = axis.releaseSteps; direction(axis.homeSign, LATCH_FEED_US); break;
    case BACKOFF: motion.remaining = axis.backoffSteps; direction(-axis.homeSign, LATCH_FEED_US); break;
  }
}

void startHomeAxis(int axis) {
  motion.axis = axis;
  homePhase(triggered(axis) ? RELEASE : SEEK);
}

bool stable(int axis, bool expected, uint32_t now) {
  if (triggered(axis) != expected) {
    motion.debouncing = false;
    return false;
  }
  if (!motion.debouncing) {
    motion.debouncing = true;
    motion.changedAt = now;
  }
  return now - motion.changedAt >= DEBOUNCE_MS;
}

void tick(bool connected) {
  const uint32_t now = millis();
  const bool bottom = triggered(Z);
  if (bottom && !motion.zPressed) motion.zPressedSince = now;
  motion.zPressed = bottom;
  if (!active()) return;
  if (static_cast<int32_t>(now - motion.lease) >= 0 || !connected) {
    halt(FAULT, "HEARTBEAT");  // No uncommanded retraction in this spindle-off release.
    return;
  }
  // The bottom contact stops downward Z only. Upward Z may leave a pressed
  // switch so the tool can be raised; there is no upper switch.
  if (motion.state == HOMING) {
    if (bottom) {
      halt(FAULT, "Z_BOTTOM_LIMIT");
      return;
    }
  } else if (motion.axis == Z) {
    if (bottom && motion.sign < 0) {
      if (motion.zReferenced && motion.pos[Z] != 0) {
        halt(FAULT, "Z_REFERENCE_LOST");
        return;
      }
      // The bottom contact ends a downward Z move without invalidating the
      // rotary datum. The operator can then jog Z+ to lift off the switch.
      digitalWrite(AXES[Z].stepPin, LOW);
      motion.pulse = false;
      motion.target[Z] = motion.pos[Z];
      motion.state = IDLE;
      motion.mimicChunk = false;
      motion.fault = "Z_BOTTOM_LIMIT";
      return;
    }
    if (bottom && motion.zReferenced && motion.sign > 0 &&
        motion.pos[Z] >= MIMIC_SWITCH_RELEASE_STEPS) {
      halt(FAULT, "Z_SWITCH_NOT_RELEASED");
      return;
    }
  } else if (bottom) {
    halt(FAULT, "Z_BOTTOM_LIMIT");
    return;
  }
  if (motion.state == MOVING && motion.axis != Z) {
    for (int i = J1; i <= J2; ++i) {
      if (triggered(i)) {
        halt(FAULT, i == J1 ? "LIMIT_J1" : "LIMIT_J2");
        return;
      }
    }
  } else if (motion.axis == J2 && triggered(J1)) {
    halt(FAULT, "LIMIT_J1");
    return;
  }

  int i = motion.axis;
  if (i != Z && i != J1 && i != J2) {
    halt(FAULT, "INVALID_MOTION_AXIS");
    return;
  }
  uint32_t edge = micros();
  if (static_cast<int32_t>(edge - motion.nextEdge) < 0) return;
  if (motion.pulse) {
    digitalWrite(AXES[i].stepPin, LOW);
    motion.pulse = false;
    motion.nextEdge = edge + motion.feed;
    return;
  }
  if (motion.state == MOVING && motion.pos[i] == motion.target[i]) {
    motion.state = IDLE;
    motion.mimicChunk = false;
    return;
  }
  if (motion.state == HOMING) {
    if (motion.phase == SEEK || motion.phase == LATCH) {
      // Stop emitting new pulses as soon as raw contact is seen.
      if (stable(i, true, now)) {
        homePhase(motion.phase == SEEK ? RELEASE : BACKOFF);
        return;
      }
      if (triggered(i)) return;
    } else if (motion.phase == RELEASE) {
      if (stable(i, false, now)) {
        homePhase(LATCH);
        return;
      }
      if (!triggered(i)) return;
    } else if (motion.remaining == 0) {
      if (triggered(i)) { halt(FAULT, "HOME_SWITCH_STUCK"); return; }
      if (!stable(i, false, now)) return;
      motion.pos[i] = motion.target[i] = 0;
      if (i == J1) startHomeAxis(J2);
      else { motion.state = IDLE; motion.homed = true; }
      return;
    }
    if (motion.remaining == 0) {
      static const char* const timeouts[][3] = {
        {"HOME_TIMEOUT_J1_SEEK", "HOME_TIMEOUT_J1_RELEASE", "HOME_TIMEOUT_J1_LATCH"},
        {"HOME_TIMEOUT_J2_SEEK", "HOME_TIMEOUT_J2_RELEASE", "HOME_TIMEOUT_J2_LATCH"},
      };
      halt(FAULT, timeouts[i - J1][motion.phase]);
      return;
    }
    --motion.remaining;
  }
  digitalWrite(AXES[i].stepPin, HIGH);
  motion.pos[i] += motion.sign;
  motion.pulse = true;
  motion.nextEdge = edge + motion.feed;
}

void tickDrill(bool connected) {
  if (!drillOn) return;
  if (!connected || static_cast<int32_t>(millis() - drillLease) >= 0) {
    digitalWrite(SPINDLE_PIN, LOW);
    drillOn = false;
    drillLease = 0;
  }
}

void motorTask(void*) {
  for (;;) {
    bool connected = WiFi.status() == WL_CONNECTED;
    portENTER_CRITICAL(&motionLock);
    tickDrill(connected);
    tick(connected);
    portEXIT_CRITICAL(&motionLock);
    vTaskDelay(1);
  }
}

void reply(int code, JsonDocument& doc) {
  String result;
  serializeJson(doc, result);
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", result);
}

void failure(int code, const char* error) {
  JsonDocument doc;
  doc["error"] = error;
  reply(code, doc);
}

bool authorized() {
  if (strlen(SCARA_TOKEN) >= 16 && server.header("X-SCARA-Token") == SCARA_TOKEN) return true;
  failure(403, "UNAUTHORIZED");
  return false;
}

void status() {
  if (!authorized()) return;
  portENTER_CRITICAL(&motionLock);
  Motion current = motion;
  const bool currentDrill = drillOn;
  portEXIT_CRITICAL(&motionLock);
  JsonDocument doc;
  doc["fw"] = "scara-slave/1.0";
  doc["boot"] = bootId;
  doc["command_id"] = current.command;
  doc["state"] = stateName(current.state);
  doc["homed"] = current.homed;
  doc["spindle"] = currentDrill;
  doc["spindle_gpio"] = SPINDLE_PIN;
  doc["drill_lease_ms"] = DRILL_LEASE_MS;
  doc["z_mode"] = "bounded";
  doc["mimicry"]["enabled"] = MIMIC_Z_MAX > 0 && profileValid && motionTaskReady;
  doc["mimicry"]["z_referenced"] = current.zReferenced;
  doc["mimicry"]["z_max_steps"] = MIMIC_Z_MAX;
  doc["commissioned"] = profileValid && motionTaskReady;
  doc["prehome_nudge"]["enabled"] = bool(SCARA_ENABLE_PREHOME_NUDGE) && profileValid && motionTaskReady;
  doc["prehome_nudge"]["step"] = NUDGE_STEPS;
  doc["prehome_nudge"]["remaining"] = NUDGE_BUDGET - current.nudgeSpent;
  doc["simulator"] = false;
  doc["fault"] = current.fault;
  if (current.state == HOMING || current.homeFault) {
    doc["home_axis"] = AXES[current.axis].label;
    doc["home_phase"] = phaseName(current.phase);
  } else {
    doc["home_axis"] = nullptr;
    doc["home_phase"] = nullptr;
  }
  doc["rssi"] = WiFi.RSSI();
  doc["ip"] = WiFi.localIP().toString();
  for (int i = 0; i < 3; ++i) {
    const char* name = AXES[i].label;
    doc["pos"][name] = current.pos[i];
    doc["target"][name] = current.target[i];
    doc["limits"][name] = triggered(i) ? 0 : 1;
    doc["bounds"][name][0] = i == Z && current.zReferenced ? 0 : AXES[i].minSteps;
    doc["bounds"][name][1] = i == Z && current.zReferenced ? MIMIC_Z_MAX : AXES[i].maxSteps;
  }
  reply(200, doc);
}

void command() {
  if (!authorized()) return;
  String path = server.uri(), raw = server.arg("plain");
  JsonDocument input;
  if (raw.length() == 0 || raw.length() > 512 || deserializeJson(input, raw) ||
      !input.is<JsonObject>()) {
    failure(400, "INVALID_JSON");
    return;
  }
  int axis = -1;
  const char* label = input["axis"] | "";
  for (int i = 0; i < 3; ++i)
    if (strcmp(label, AXES[i].label) == 0) axis = i;
  int32_t target = input["to"].as<int32_t>();
  uint32_t feed = input["feed"].as<uint32_t>();
  const bool nudge = path == "/nudge";
  const bool mimic = path == "/mimic-jog", zero = path == "/z-zero";
  const int32_t delta = input["delta"].as<int32_t>();
  const bool validNudge = (axis == J1 || axis == J2) && input["delta"].is<int32_t>() &&
      (delta == NUDGE_STEPS || delta == -NUDGE_STEPS) &&
      input["clearance_confirmed"].is<bool>() && input["clearance_confirmed"].as<bool>();
  const bool validMove = (axis == Z || axis == J1 || axis == J2) && input["to"].is<int32_t>() &&
       input["feed"].is<uint32_t>() && feed >= MIN_FEED_US && feed <= MAX_FEED_US;
  const int32_t chunk = axis == Z ? MIMIC_Z_CHUNK : MIMIC_J1_CHUNK;
  const bool validMimic = (axis == Z || axis == J1) && input["delta"].is<int32_t>() &&
      delta != 0 && delta >= -chunk && delta <= chunk;
  const bool matchingBoot = strcmp(input["boot"] | "", bootId) == 0;
  const bool validHeartbeat = input["command_id"].is<uint32_t>();
  uint32_t expected = input["command_id"].as<uint32_t>();
  const char* error = nullptr;
  int code = 200;

  portENTER_CRITICAL(&motionLock);
  if (path == "/estop") {
    halt(ESTOP, "ESTOP");
  } else if (path == "/stop" && motion.state != ESTOP && motion.state != FAULT) {
    halt(HOLD, "STOP");
  } else if (path == "/reset" && (motion.state == ESTOP || motion.state == FAULT || motion.state == HOLD)) {
    halt(IDLE, nullptr);
  } else if (path == "/heartbeat") {
    if (!validHeartbeat || !matchingBoot || motion.command != expected) error = "COMMAND_CHANGED";
    else if (active() && static_cast<int32_t>(millis() - motion.lease) >= 0) {
      halt(FAULT, "HEARTBEAT");
      error = "HEARTBEAT_EXPIRED";
    } else {
      if (active() && !motion.mimicChunk) motion.lease = millis() + LEASE_MS;
      if (drillOn) drillLease = millis() + DRILL_LEASE_MS;
    }
  } else if (path == "/drill" && input["on"].is<bool>()) {
    if (!input["on"].as<bool>()) {
      digitalWrite(SPINDLE_PIN, LOW);
      drillOn = false;
      drillLease = 0;
    } else if (!matchingBoot) error = "COMMAND_CHANGED";
    else if (motion.state != IDLE) error = "INVALID_STATE_OR_COMMAND";
    else {
      digitalWrite(SPINDLE_PIN, HIGH);
      drillOn = true;
      drillLease = millis() + DRILL_LEASE_MS;
    }
  } else if (path == "/spindle" && input["on"].is<bool>()) {
    digitalWrite(SPINDLE_PIN, LOW);
    if (input["on"].as<bool>()) error = "SPINDLE_DISABLED";
  } else if ((path == "/home" || path == "/move" || nudge || mimic || zero) && motion.state == IDLE) {
    if (!profileValid || !motionTaskReady) error = "NOT_COMMISSIONED";
    else if (!validHeartbeat || !matchingBoot || motion.command != expected) error = "COMMAND_CHANGED";
    else if (mimic || zero) {
      if (MIMIC_Z_MAX == 0) error = "MIMICRY_DISABLED";
      else if (drillOn) error = "DRILL_ACTIVE";
      else if (zero) {
        if (!triggered(Z) || !motion.zPressed || millis() - motion.zPressedSince < DEBOUNCE_MS)
          error = "Z_BOTTOM_SWITCH_NOT_ACTIVE";
        else {
          motion.pos[Z] = motion.target[Z] = 0;
          motion.zReferenced = true;
          motion.zMustRezero = false;
          motion.fault = nullptr;
          ++motion.command;
        }
      } else if (!validMimic) { error = "INVALID_MIMIC_JOG"; code = 400; }
      else if (!motion.zReferenced) error = "Z_NOT_REFERENCED";
      else {
        const bool bottom = triggered(Z);
        target = motion.pos[axis] + delta;
        const int32_t low = axis == Z ? 0 : AXES[J1].minSteps;
        const int32_t high = axis == Z ? MIMIC_Z_MAX : AXES[J1].maxSteps;
        if (target < low || target > high) error = "MIMIC_TRAVEL_LIMIT";
        else if (axis == Z && bottom && delta < 0) error = "Z_BOTTOM_LIMIT";
        else if (axis == J1 && (!motion.homed || bottom)) error = "NOT_HOMED_OR_SAFE_Z";
        else if (axis == J1 && (triggered(J1) || triggered(J2))) error = "JOINT_LIMIT_ACTIVE";
        else {
          motion.axis = axis;
          motion.target[axis] = target;
          motion.state = MOVING;
          motion.mimicChunk = true;
          motion.fault = nullptr;
          direction(delta > 0 ? 1 : -1, axis == Z ? 1000 : 3000);
          code = 202;
        }
      }
    } else if (path == "/home") {
      if (triggered(Z)) error = "Z_BOTTOM_LIMIT";
      else {
        digitalWrite(SPINDLE_PIN, LOW);
        drillOn = false;
        drillLease = 0;
        motion.fault = nullptr;
        motion.state = HOMING;
        motion.homed = false;
        startHomeAxis(J1);
        code = 202;
      }
    } else if (nudge) {
      if (triggered(Z)) error = "Z_BOTTOM_LIMIT";
      else if (!SCARA_ENABLE_PREHOME_NUDGE) error = "PREHOME_NUDGE_DISABLED";
      else if (!validNudge) { error = "INVALID_NUDGE_OR_CLEARANCE"; code = 400; }
      else if (motion.homed) error = "ALREADY_HOMED";
      else if (motion.nudgeSpent + NUDGE_STEPS > NUDGE_BUDGET) error = "NUDGE_BUDGET_EXHAUSTED";
      else if (triggered(J1) || triggered(J2)) error = "JOINT_LIMIT_ACTIVE";
      else {
        motion.nudgeSpent += NUDGE_STEPS;
        digitalWrite(SPINDLE_PIN, LOW);
        drillOn = false;
        drillLease = 0;
        motion.fault = nullptr;
        motion.axis = axis;
        motion.target[axis] = motion.pos[axis] + delta;
        motion.state = MOVING;
        direction(delta > 0 ? 1 : -1, LATCH_FEED_US);
        code = 202;
      }
    } else if (!validMove || target < (axis == Z && motion.zReferenced ? 0 : AXES[axis].minSteps) ||
               target > (axis == Z && motion.zReferenced ? MIMIC_Z_MAX : AXES[axis].maxSteps)) {
      error = "INVALID_TARGET_OR_FEED"; code = 400;
    }
    else if (axis != Z && !motion.homed) error = "NOT_HOMED";
    else if (axis == Z) {
      if (motion.zMustRezero) error = "Z_NOT_REFERENCED";
      else if (triggered(Z) && target < motion.pos[Z]) error = "Z_BOTTOM_LIMIT";
      else {
        digitalWrite(SPINDLE_PIN, LOW);
        drillOn = false;
        drillLease = 0;
        motion.fault = nullptr;
        motion.axis = axis;
        motion.target[axis] = target;
        motion.state = MOVING;
        direction(target >= motion.pos[axis] ? 1 : -1, feed);
        code = 202;
      }
    } else if (triggered(Z)) error = "Z_BOTTOM_LIMIT";
    else if (triggered(J1) || triggered(J2)) error = "JOINT_LIMIT_ACTIVE";
    else {
      digitalWrite(SPINDLE_PIN, LOW);
      drillOn = false;
      drillLease = 0;
      motion.fault = nullptr;
      motion.axis = axis;
      motion.target[axis] = target;
      motion.state = MOVING;
      direction(target >= motion.pos[axis] ? 1 : -1, feed);
      code = 202;
    }
    if (code == 202) {
      ++motion.command;
      motion.lease = millis() + (motion.mimicChunk ? MIMIC_LEASE_MS : LEASE_MS);
    }
  } else {
    error = "INVALID_STATE_OR_COMMAND";
  }
  uint32_t commandId = motion.command;
  portEXIT_CRITICAL(&motionLock);
  if (error) { failure(code == 200 ? 409 : code, error); return; }
  JsonDocument out;
  out["boot"] = bootId;
  out["command_id"] = commandId;
  reply(code, out);
}

void setup() {
  pinMode(SPINDLE_PIN, OUTPUT);
  digitalWrite(SPINDLE_PIN, LOW);
  for (const Axis& axis : AXES) {
    pinMode(axis.stepPin, OUTPUT);
    digitalWrite(axis.stepPin, LOW);
    pinMode(axis.dirPin, OUTPUT);
    pinMode(axis.limitPin, INPUT_PULLUP);
  }
  Serial.begin(115200);
  snprintf(bootId, sizeof(bootId), "%08lx%08lx",
           static_cast<unsigned long>(esp_random()), static_cast<unsigned long>(esp_random()));
  profileValid = validProfile();
  motionTaskReady = xTaskCreate(motorTask, "scara_motion", 4096, nullptr, 2, nullptr) == pdPASS;
  Serial.printf("\nSCARA %s (" __DATE__ " " __TIME__ ") / bounded unreferenced Z\n",
                profileValid && motionTaskReady ? "DRY-RUN" : "STATUS-ONLY");
  Serial.printf("Profile: commission=%d z[%ld,%ld] j1[%ld,%ld] j2[%ld,%ld]\n",
                profileValid ? 1 : 0,
                static_cast<long>(AXES[0].minSteps), static_cast<long>(AXES[0].maxSteps),
                static_cast<long>(AXES[1].minSteps), static_cast<long>(AXES[1].maxSteps),
                static_cast<long>(AXES[2].minSteps), static_cast<long>(AXES[2].maxSteps));
  Serial.printf("WiFi: ssid=%s token=%u chars\n", WIFI_SSID,
                static_cast<unsigned>(strlen(SCARA_TOKEN)));
  if (!strlen(WIFI_PASSWORD) || strlen(SCARA_TOKEN) < 16) {
    Serial.println("Supply Wi-Fi credentials and a controller token in secrets.h.");
    return;
  }
  static const char* headers[] = {"X-SCARA-Token"};
  server.collectHeaders(headers, 1);
  server.on("/status", HTTP_GET, status);
  const char* commands[] = {"/home", "/move", "/nudge", "/heartbeat", "/stop", "/estop", "/reset",
                            "/spindle", "/drill", "/z-zero", "/mimic-jog"};
  for (const char* path : commands) server.on(path, HTTP_POST, command);
  server.onNotFound([]() { failure(404, "UNKNOWN_ENDPOINT"); });
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("scara");
  WiFi.setAutoReconnect(true);
  // Modem sleep adds >1 s of latency per exchange and breaks short HTTP timeouts.
  WiFi.setSleep(false);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastStatusChange = millis();
}

void loop() {
  const wl_status_t status = WiFi.status();
  const IPAddress ip = status == WL_CONNECTED ? WiFi.localIP() : IPAddress();
  const uint32_t now = millis();

  if (status != lastNetStatus || ip != lastIp) {
    lastNetStatus = status;
    lastIp = ip;
    lastStatusChange = now;
    if (status == WL_CONNECTED) {
      Serial.printf("WiFi: connected ip=%s rssi=%d dBm\n",
                    ip.toString().c_str(), WiFi.RSSI());
    } else {
      Serial.printf("WiFi: %s (ssid=%s)\n", netName(status), WIFI_SSID);
    }
    lastLinkReport = now;
  }

  if (status == WL_CONNECTED) {
    if (!serving) {
      MDNS.begin("scara");
      server.begin();
      serving = true;
      Serial.printf("HTTP: ready at http://%s/ (X-SCARA-Token required)\n", ip.toString().c_str());
    }
    server.handleClient();
    if (now - lastLinkReport >= 30000) {
      lastLinkReport = now;
      Serial.printf("Link: ip=%s rssi=%d dBm heap=%u\n", ip.toString().c_str(),
                    WiFi.RSSI(), static_cast<unsigned>(ESP.getFreeHeap()));
    }
  } else if (now - lastStatusChange >= 30000) {
    // Association can stall after an AP restart; nudge it without dropping the lease.
    lastStatusChange = now;
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
  delay(2);
}
