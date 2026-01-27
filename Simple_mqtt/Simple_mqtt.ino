#include <RTCZero.h>
#include <MKRNB.h>
#include <ArduinoMqttClient.h>

// ---------- USER SETTINGS ----------
static const char APN[]      = "gigsky-02";
static const char PINNUMBER[] = "";          // SIM PIN if needed, else ""
static const char MQTT_HOST[] = "mqtt.eu.thingsboard.cloud";
static const int  MQTT_PORT   = 1883;        // 8883 for TLS (more power + memory)
static const char MQTT_TOPIC[] = "v1/devices/me/telemetry";

// How often to publish:
static const uint32_t PUBLISH_PERIOD_SEC = 10; // 2 minutes

// Connection time limits (keep short to save power):
static const uint32_t NET_TIMEOUT_MS  = 90UL * 1000UL;
static const uint32_t MQTT_TIMEOUT_MS = 30UL * 1000UL;

// ---------- GLOBALS ----------
RTCZero rtc;

NB nbAccess;
GPRS gprs;
NBClient nbClient;
MqttClient mqtt(nbClient);

volatile bool rtcFired = false;

static void rtcWakeISR() {
  // Keep ISR tiny: just set a flag.
  rtcFired = true;
}

// Simple millis()-based timeout helper
static bool waitUntil(bool (*predicate)(), uint32_t timeoutMs) {
  uint32_t start = millis();
  while (!predicate()) {
    if (millis() - start >= timeoutMs) return false;
    delay(50);
  }
  return true;
}

static void armNextAlarmFromNow(uint32_t secondsFromNow) {
  // One-shot alarm: now + 600s
  uint32_t now = rtc.getEpoch();
  rtc.setAlarmEpoch(now + secondsFromNow);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}

static bool connectNetwork() {
  int bs = 0;
  int gp = 0;
  Serial.println("[NET] ConnectNetwork"); 
  // Bring up modem + register + data context
  // NOTE: begin() can take time; keep retries low for power.
  if (bs = nbAccess.begin(PINNUMBER, APN));
  Serial.print("[NET] nbAccess -> ");
  Serial.println(bs); 
  // Attach PDP context (GPRS class used by MKRNB for data)
  if (gp = gprs.attachGPRS(APN) != GPRS_READY) return false;
  Serial.print("[NET] gprs -> ");
  Serial.println(gp); 
  return true;
}

static bool connectMqtt() {
  String mqttToken = "vxl9ZxrJ5J6soedld65D";
  String mqttDeviceID = "HPT02";
  String mqttID = mqttDeviceID + "-" + String((uint32_t)millis(), HEX);

  Serial.println("[MQTT] Connecting..."); 

  mqtt.setId(mqttID);
  mqtt.setUsernamePassword(mqttToken.c_str(), "");
  // Optional keepalive small, but we disconnect after publish anyway:
  mqtt.setKeepAliveInterval(30);

  uint32_t start = millis();
  while (!mqtt.connect(MQTT_HOST, MQTT_PORT)) {
    if (millis() - start >= MQTT_TIMEOUT_MS) return false;
    delay(250);
  }
  return true;
}

static void disconnectAll() {
  Serial.println("[NET] Disconnecting..."); 
  if (mqtt.connected()) mqtt.stop();
  // Power modem off to minimize current between wakeups:
  nbAccess.shutdown();  // documented API :contentReference[oaicite:3]{index=3}
}

static void publishOnce() {
  // 1) Network
  Serial.println("[NET] publishOnce"); 
  uint32_t start = millis();
  while (!connectNetwork()) {
    if (millis() - start >= NET_TIMEOUT_MS) return;
    // If something half-started, shut down and retry cleanly
//    disconnectAll();
    delay(1000);
  }

  // 2) MQTT
  if (!connectMqtt()) {
    disconnectAll();
    return;
  }

  // 3) Publish
  // Keep payload short to save data + time-on-air.
  char payload[64] = "Test";
  uint32_t now = rtc.getEpoch();
  snprintf(payload, sizeof(payload), "{\"t\":%lu,\"ok\":1}", (unsigned long)now);
  mqtt.beginMessage(MQTT_TOPIC);
  mqtt.print(payload);
  mqtt.endMessage();

  // 4) Disconnect ASAP
  disconnectAll();
}

void setup() {
  // If you’re on battery, avoid leaving Serial active (USB costs power).
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  Serial.println("\n=== MIN STABLE BOOT ===");

  rtc.begin(); // 24h init (RTCZero uses 24h internally)

  // You MUST set RTC time at least once (from compile time, GNSS, network, etc.)
  // For demo: start at epoch 0. Replace with a real time source if needed.
  rtc.setEpoch(0);

  rtc.attachInterrupt(rtcWakeISR);

  // Arm first wake 10 minutes from now
  armNextAlarmFromNow(PUBLISH_PERIOD_SEC);

  // Clear any pending flag
  rtcFired = false;
}

void loop() {
  if (rtcFired) {
    // Acknowledge the wake
    rtcFired = false;
    Serial.println("[RTC] Fired!");  
    // Do the work
    publishOnce();

    // Re-arm the next one-shot alarm for +10 minutes
    armNextAlarmFromNow(PUBLISH_PERIOD_SEC);
  }

  // Deep sleep until RTC alarm interrupt wakes us
  // This is the lowest-power standby mode on SAMD21 via RTCZero.
//  rtc.standbyMode();
}

