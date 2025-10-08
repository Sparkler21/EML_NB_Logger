/*
  MKR NB 1500 dual-mode (Cat-M1/NB-IoT) data logger
  - 1 analog (via ADS1115)
  - 1 contact-closure (rain gauge) on D7 (interrupt)
  - 10 minute MQTT publish, then deep sleep
*/

#include <MKRNB.h>
#include <ArduinoMqttClient.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ---------- User config ----------
#define DEVICE_ID   "EML_HPT_001"
#define APN         "your.apn.here"      // e.g. Vodafone LPWA APN, or what your SIM provider gives you
#define APN_USER    ""
#define APN_PASS    ""
#define USE_NBIOT   false                // true = NB-IoT, false = LTE-M (if supported by your core)

const char* MQTT_HOST  = "mqtt.eu.thingsboard.cloud";
const int   MQTT_PORT  = 1883;                    // 8883 if you later use TLS
const char* MQTT_TOPIC = "v1/devices/me/telemetry";
const char* TB_TOKEN = "lxQiBuxXNwTQttZ8Aody";    // <- your TB device token

const char* PINNUMBER = "";              // SIM PIN if you have one; else ""

// Cadence
const uint32_t SAMPLE_INTERVAL_SEC  = 60;   // 1 min sample
const uint32_t PUBLISH_INTERVAL_SEC = 600;  // 10 minute publish
// ---------------------------------

// Pins
const int PIN_RAIN = 7;  // contact-closure input (to GND)

// Globals
volatile uint32_t rainCount = 0;
volatile uint32_t lastPulseUs = 0;

RTCZero rtc;
Adafruit_ADS1115 ads;

NB nbAccess;
GPRS gprs;
NBUDP udp;
NBClient nbClient;
MqttClient mqtt(nbClient);

// --- ISRs (no ESP32 attributes on SAMD21) ---
void rainISR() {
  uint32_t t = micros();
  if (t - lastPulseUs > 5000) { // ~5 ms debounce
    rainCount++;
    lastPulseUs = t;
  }
}
void wakeupISR() { /* empty, just to satisfy attachInterruptWakeup */ }

// Simple ring buffer for samples between publishes
struct Sample { uint32_t ts; int16_t analog; uint16_t tips; };
const size_t BUF_MAX = 300;
Sample buf[BUF_MAX];
size_t headIdx = 0, tailIdx = 0;

// Fwds
void takeSample();
bool connectNetwork();
bool connectMQTT();
void publishBuffer();
void sleepFor(uint32_t seconds);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  // I2C + ADC
  Wire.begin();
  ads.setGain(GAIN_ONE); // ±4.096V; tune to your sensor range
  if (!ads.begin()) {
    Serial.println("ADS1115 not found!");
  }

  // Rain input
  pinMode(PIN_RAIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RAIN), rainISR, FALLING);

  // RTC + low power
  rtc.begin();
  rtc.setEpoch(0);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), wakeupISR, FALLING);

  // Optional RAT select (only if your MKRNB version exposes it)
  #ifdef RADIO_ACCESS_TECHNOLOGY_NBIOT
    nbAccess.setRadioAccessTechnology(
      USE_NBIOT ? RADIO_ACCESS_TECHNOLOGY_NBIOT
                : RADIO_ACCESS_TECHNOLOGY_EUTRAN
    );
  #endif

  // First sample immediately so first publish has data
  takeSample();
}

void loop() {
  static uint32_t lastSample = 0;
  uint32_t now = rtc.getEpoch();

  if ((now - lastSample) >= SAMPLE_INTERVAL_SEC || lastSample == 0) {
    takeSample();
    lastSample = now;
  }

  bool timeToPublish = (lastSample != 0) &&
                       ((now - lastSample) < 2) &&     // just after sampling
                       ((now == 0) || ((now - (uint32_t)0) % PUBLISH_INTERVAL_SEC == 0)); // simple cadence gate

  // More robust: track lastPublishTs; here we just do hourly after boot.
  static uint32_t lastPublishTs = 0;
  if (lastPublishTs == 0 || (now - lastPublishTs) >= PUBLISH_INTERVAL_SEC) {
    timeToPublish = true;
  }

  if (timeToPublish) {
    if (connectNetwork()) {
      if (connectMQTT()) {
        publishBuffer();
        mqtt.stop();
      }
      // Cleanly detach and end to save power
      gprs.detachGPRS();
      nbAccess.shutdown();
    }
    lastPublishTs = rtc.getEpoch();
  }

  // Sleep; RTC alarm or rain interrupt will wake us
  sleepFor(1);
}

// ---- Helpers ----
void takeSample() {
  int16_t adc = ads.readADC_SingleEnded(0);
  uint16_t tips = rainCount;  // snapshot since last sample
  rainCount = 0;

  uint32_t ts = rtc.getEpoch();
  size_t nextHead = (headIdx + 1) % BUF_MAX;
  if (nextHead == tailIdx) {         // buffer full -> drop oldest
    tailIdx = (tailIdx + 1) % BUF_MAX;
  }
  buf[headIdx] = { ts, adc, tips };
  headIdx = nextHead;

  Serial.print("Sample t="); Serial.print(ts);
  Serial.print(" adc="); Serial.print(adc);
  Serial.print(" tips="); Serial.println(tips);
}

unsigned long ntpUnix() {
  const char* host = "pool.ntp.org";
  const int NTP_PORT = 123;
  byte pkt[48] = {0};
  pkt[0] = 0b11100011; // LI, Version, Mode
  if (udp.begin(2390) != 1) return 0;
  if (!udp.beginPacket(host, NTP_PORT)) return 0;
  udp.write(pkt, 48);
  udp.endPacket();

  uint32_t t0 = millis();
  while (!udp.parsePacket()) {
    if (millis() - t0 > 1500) return 0; // timeout ~1.5s
  }
  udp.read(pkt, 48);
  udp.stop();
  // NTP time starts 1900-01-01; UNIX starts 1970-01-01
  const unsigned long seventyYears = 2208988800UL;
  unsigned long secsSince1900 =
      (unsigned long)pkt[40] << 24 | (unsigned long)pkt[41] << 16 |
      (unsigned long)pkt[42] << 8  | (unsigned long)pkt[43];
  return secsSince1900 - seventyYears; // UNIX epoch seconds
}

// Helper: append a uint64_t to an Arduino String
static inline void appendUint64(String &s, uint64_t v) {
  char buf[21];                          // enough for 20 digits + NUL
  snprintf(buf, sizeof(buf), "%llu", (unsigned long long)v);
  s += buf;
}

bool connectNetwork() {
  Serial.println("Attaching cellular...");
  // Pass APN credentials here (correct for MKRNB)
  for (int i = 0; i < 3; i++) {
    if (nbAccess.begin(PINNUMBER, APN, APN_USER, APN_PASS) == NB_READY) {
      if (gprs.attachGPRS() == GPRS_READY) {
        Serial.println("Cellular attached.");

        unsigned long unixNow = ntpUnix();
        if (unixNow) {
          rtc.setEpoch(unixNow);  // now rtc.getEpoch() is real UTC seconds
        }
        return true;
      }
    }
    delay(2000);
  }
  Serial.println("Cellular attach failed.");
  return false;
}

bool connectMQTT() {
  Serial.println("Connecting MQTT (ThingsBoard)...");
  mqtt.setId(DEVICE_ID);                  // optional clientId
  mqtt.setUsernamePassword(TB_TOKEN, ""); // TB: username = token, password blank
  if (!mqtt.connect(MQTT_HOST, MQTT_PORT)) {
    Serial.print("MQTT connect failed, code=");
    Serial.println(mqtt.connectError());
    return false;
  }
  Serial.println("MQTT connected.");
  return true;
}


void publishBuffer() {
  if (headIdx == tailIdx) {
    Serial.println("No data to send.");
    return;
  }

  // (Optional) quick sanity guard: if RTC isn't synced, fall back to server time.
  // Remove this block if you're sure RTC is always synced.
  if (rtc.getEpoch() < 1700000000UL) { // ~Nov 2023
    Serial.println("RTC not synced; sending without explicit timestamps (server time).");
    String payload = "[";
    size_t i = tailIdx;
    bool first = true;
    while (i != headIdx) {
      if (!first) payload += ',';
      payload += "{\"values\":{";
      payload += "\"a\":"; payload += buf[i].analog;
      payload += ",\"c\":"; payload += buf[i].tips;
      payload += "}}";
      first = false;
      i = (i + 1) % BUF_MAX;
    }
    payload += "]";

    Serial.print("Publishing "); Serial.print(payload.length()); Serial.println(" bytes (server-timed)");
    if (mqtt.beginMessage(MQTT_TOPIC, false, 1)) {
      mqtt.print(payload);
      mqtt.endMessage();
      tailIdx = headIdx;
      Serial.println("Publish OK");
    } else {
      Serial.println("Publish beginMessage() failed");
    }
    return;
  }

  // Build TB array payload with per-sample timestamps
  String payload;
  payload.reserve(1024);            // adjust if you batch a lot of samples
  payload += "[";

  size_t i = tailIdx;
  bool first = true;
  while (i != headIdx) {
    uint64_t ts_ms = (uint64_t)buf[i].ts * 1000ULL;   // ms since epoch

    if (!first) payload += ',';
    payload += "{\"ts\":";
    appendUint64(payload, ts_ms);
    payload += ",\"values\":{";
    payload += "\"a\":"; payload += buf[i].analog;     // analog reading
    payload += ",\"c\":"; payload += buf[i].tips;      // rain tips in interval
    payload += "}}";
    first = false;

    i = (i + 1) % BUF_MAX;
  }
  payload += "]";

  Serial.print("Publishing "); Serial.print(payload.length()); Serial.println(" bytes (TB timestamps)");
  if (mqtt.beginMessage("v1/devices/me/telemetry", false, 1)) { // retained=false, QoS 1
    mqtt.print(payload);
    mqtt.endMessage();
    tailIdx = headIdx;              // advance buffer on success
    Serial.println("Publish OK");
  } else {
    Serial.println("Publish beginMessage() failed");
  }
}


void sleepFor(uint32_t seconds) {
  if (seconds == 0) return;

  uint32_t now = rtc.getEpoch();
  uint32_t target = now + seconds;
  uint8_t h = (target / 3600) % 24;
  uint8_t m = (target / 60) % 60;
  uint8_t s = target % 60;

  rtc.setAlarmTime(h, m, s);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  LowPower.deepSleep();   // wakes on RTC alarm or rain interrupt
}
