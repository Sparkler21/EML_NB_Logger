/*
  MKR NB 1500 dual-mode (Cat-M1/NB-IoT) data logger
  - 1 analog (via ADS1115)
  - 1 contact-closure (rain gauge) on D7 (interrupt)
  - Hourly MQTT publish, then deep sleep
*/

#include <MKRNB.h>
#include <ArduinoMqttClient.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ---------- User config ----------
#define DEVICE_ID   "DL-01"
#define APN         "your.apn.here"      // e.g. "lpwa.vodafone.com" or as given by your provider
#define APN_USER    ""
#define APN_PASS    ""
#define USE_NBIOT   false                // true to prefer NB-IoT; false for Cat-M1 (LTE-M)

const char* MQTT_HOST = "broker.example.com";
const int   MQTT_PORT = 1883;            // use 8883 if you add TLS later
const char* MQTT_TOPIC = "sensors/dl01";

// Cadence
const uint32_t SAMPLE_INTERVAL_SEC  = 300;   // 5 min sample
const uint32_t PUBLISH_INTERVAL_SEC = 3600;  // 1 hour publish
// ---------------------------------

// Pins
const int PIN_RAIN = 7;  // contact-closure input (to GND)

// Globals
volatile uint32_t rainCount = 0;
volatile uint32_t lastPulseUs = 0;

RTCZero rtc;
Adafruit_ADS1115 ads;

NB nbAccess;
GPRS gprs;                       // yes, still named GPRS in MKRNB for PDP context handling
NBClient nbClient;
MqttClient mqtt(nbClient);

uint32_t bootEpoch = 0;          // optional: keep zero if you don't care about absolute epoch
uint32_t lastPublishTs = 0;

// Simple ring buffer for samples between publishes
struct Sample { uint32_t ts; int16_t analog; uint16_t tips; };
const size_t BUF_MAX = 300;
Sample buf[BUF_MAX];
size_t headIdx = 0, tailIdx = 0;

void IRAM_ATTR rainISR() {
  uint32_t t = micros();
  if (t - lastPulseUs > 5000) { // ~5 ms debounce
    rainCount++;
    lastPulseUs = t;
  }
}

void setup() {
  // Serial for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  // I2C + ADC
  Wire.begin();
  ads.setGain(GAIN_ONE); // ±4.096V; pick a better gain for your sensor range
  if (!ads.begin()) {
    Serial.println("ADS1115 not found!");
  }

  // Rain input
  pinMode(PIN_RAIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RAIN), rainISR, FALLING);

  // RTC for timed wake
  rtc.begin();
  rtc.setEpoch(0);

  // Low power: allow wake on RTC alarm and external interrupt
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), []{}, FALLING);

  // Optional: set radio access technology
  // MKRNB exposes: nbAccess.setRadioAccessTechnology(RADIO_ACCESS_TECHNOLOGY_EUTRAN) for LTE-M,
  // and RADIO_ACCESS_TECHNOLOGY_NBIOT for NB-IoT on some versions.
  // Not all core versions have this call; if unavailable, skip and let network decide.
  #ifdef RADIO_ACCESS_TECHNOLOGY_NBIOT
  nbAccess.setRadioAccessTechnology(USE_NBIOT ? RADIO_ACCESS_TECHNOLOGY_NBIOT
                                              : RADIO_ACCESS_TECHNOLOGY_EUTRAN);
  #endif

  // First sample immediately so we have data on first publish
  takeSample();
}

void loop() {
  // Sample on cadence
  static uint32_t lastSample = 0;
  uint32_t now = rtc.getEpoch();
  if ((now - lastSample) >= SAMPLE_INTERVAL_SEC || lastSample == 0) {
    takeSample();
    lastSample = now;
  }

  bool timeToPublish = (lastPublishTs == 0) || ((now - lastPublishTs) >= PUBLISH_INTERVAL_SEC);
  if (timeToPublish) {
    if (connectNetwork()) {
      if (connectMQTT()) {
        publishBuffer();
        mqtt.stop();
      }
      // Power down modem cleanly to save energy
      nbAccess.shutdown();  // if your core lacks shutdown(), use nbAccess.end()
    }
    lastPublishTs = rtc.getEpoch();
  }

  // Sleep until the next interesting thing (next 1-second tick); RTC alarm handles longer waits.
  sleepFor(1);
}

// ---- Helpers ----
void takeSample() {
  int16_t adc = ads.readADC_SingleEnded(0);
  uint16_t tips = rainCount;  // snapshot
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

bool connectNetwork() {
  Serial.println("Attaching cellular...");

  // NB-IoT/LTE-M attach; the MKRNB library uses SIM PIN if needed.
  // Try a few times; rural sites can take longer.
  for (int i = 0; i < 3; i++) {
    if (nbAccess.begin() == NB_READY) {
      if (gprs.attachGPRS(APN, APN_USER, APN_PASS)) {
        Serial.println("Cellular attached.");
        return true;
      }
    }
    delay(2000);
  }
  Serial.println("Cellular attach failed.");
  return false;
}

bool connectMQTT() {
  Serial.println("Connecting MQTT...");
  mqtt.setId(DEVICE_ID);
  // Optional: set username/password
  // mqtt.setUsernamePassword("user", "pass");

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

  // Build a compact JSON batch: {"dev":"DL-01","int":300,"n":K,"t0":ts0,"a":[...],"c":[...]}
  String payload;
  payload.reserve(512);
  size_t from = tailIdx, to = headIdx;
  uint32_t interval = SAMPLE_INTERVAL_SEC;
  uint32_t t0 = buf[from].ts;

  payload += "{\"dev\":\""; payload += DEVICE_ID;
  payload += "\",\"int\":"; payload += interval;
  payload += ",\"n\":"; payload += (to >= from ? (to - from) : (BUF_MAX - from + to));
  payload += ",\"t0\":"; payload += t0;
  payload += ",\"a\":[";
  bool first = true;
  size_t i = from;
  while (i != to) { if (!first) payload += ','; payload += buf[i].analog; first = false; i = (i + 1) % BUF_MAX; }
  payload += "],\"c\":[";
  first = true; i = from;
  while (i != to) { if (!first) payload += ','; payload += buf[i].tips;   first = false; i = (i + 1) % BUF_MAX; }
  payload += "]}";

  Serial.print("Publishing "); Serial.print(payload.length()); Serial.println(" bytes");

  if (mqtt.beginMessage(MQTT_TOPIC, false, 1)) { // retained=false, QoS 1
    mqtt.print(payload);
    mqtt.endMessage();
    // On success, advance tail
    tailIdx = to;
    Serial.println("Publish OK");
  } else {
    Serial.println("Publish beginMessage() failed");
  }
}

void sleepFor(uint32_t seconds) {
  if (seconds == 0) return;
  // Program RTC alarm for seconds from now and deep sleep
  uint32_t now = rtc.getEpoch();
  uint32_t target = now + seconds;

  // Convert to h:m:s for RTCZero alarm
  uint8_t h = (target / 3600) % 24;
  uint8_t m = (target / 60) % 60;
  uint8_t s = target % 60;

  rtc.setAlarmTime(h, m, s);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  LowPower.deepSleep();   // wakes on RTC alarm or rain interrupt
}
