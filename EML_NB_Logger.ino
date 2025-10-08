/*
  MKR NB 1500 river/rain logger → ThingsBoard (EU)
  - ADS1115 (AIN0) for river level
  - Rain gauge on D7 (contact to GND), debounced in ISR
  - Sample every 60 s, publish every 10 min
  - MQTT TB-native timestamps (per-sample {"ts", "values"})
  - NTP time sync (no RTCZero getEpoch to avoid stalls)
*/

#include <MKRNB.h>
#include <ArduinoMqttClient.h>
#include <ArduinoLowPower.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <NBUDP.h>

// ---------- User config ----------
#define DEVICE_ID   "EML_HPT_001"
#define APN         "gigsky-02"
#define APN_USER    ""
#define APN_PASS    ""
#define USE_NBIOT   false     // true = NB-IoT, false = LTE-M (if core supports selection)

const char* MQTT_HOST  = "mqtt.eu.thingsboard.cloud";
const int   MQTT_PORT  = 1883;                      // 8883 if you add TLS later
const char* TB_TOPIC   = "v1/devices/me/telemetry"; // TB telemetry topic
const char* TB_TOKEN   = "lxQiBuxXNwTQttZ8Aody";    // ThingsBoard device access token

const char* PINNUMBER  = "";                        // SIM PIN if any

// Cadence (seconds)
const uint32_t SAMPLE_INTERVAL_SEC  = 60;   // 1 minute
const uint32_t PUBLISH_INTERVAL_SEC = 600;  // 10 minutes
// ----------------------------------

// Pins
const int PIN_RAIN = 7;   // contact-closure input (to GND)

// Globals
volatile uint32_t rainTipsCounter = 0;
volatile uint32_t lastPulseUs = 0;

Adafruit_ADS1115 ads;
NB nbAccess;
GPRS gprs;
NBClient nbClient;
MqttClient mqtt(nbClient);
NBUDP udp;

// ---- Monotonic, NTP-anchored clock (no RTCZero stalls) ----
static uint32_t unix_base = 0;   // UTC seconds at last sync
static uint32_t ms_base   = 0;   // millis() at last sync
static bool     time_synced = false;

static inline void setUnixTime(uint32_t unix_now) {
  if (unix_now == 0) return;
  unix_base = unix_now;
  ms_base   = millis();
  time_synced = true;
}

static inline uint32_t nowUnix() {
  if (!time_synced) return millis() / 1000UL; // relative since boot (non-blocking)
  return unix_base + (millis() - ms_base) / 1000UL;
}

// NTP (UDP) → UNIX seconds; 0 on failure
unsigned long ntpUnix() {
  const char* host = "pool.ntp.org";
  const int NTP_PORT = 123;
  uint8_t pkt[48] = {0};
  pkt[0] = 0b11100011; // LI=3, VN=4, Mode=3 (client)

  if (udp.begin(2390) != 1) return 0;
  if (!udp.beginPacket(host, NTP_PORT)) { udp.stop(); return 0; }
  udp.write(pkt, 48);
  udp.endPacket();

  unsigned long t0 = millis();
  while (!udp.parsePacket()) {
    if (millis() - t0 > 1500) { udp.stop(); return 0; }
  }
  udp.read(pkt, 48);
  udp.stop();

  const unsigned long seventyYears = 2208988800UL; // NTP(1900)->UNIX(1970)
  unsigned long secs1900 =
    ((unsigned long)pkt[40] << 24) |
    ((unsigned long)pkt[41] << 16) |
    ((unsigned long)pkt[42] <<  8) |
     (unsigned long)pkt[43];
  return secs1900 - seventyYears;
}

// Ring buffer of minute samples (enough for ~1 day if you like)
struct Sample { uint32_t ts; int16_t a; uint16_t c; };
const size_t BUF_MAX = 512;
Sample buf[BUF_MAX];
size_t headIdx = 0, tailIdx = 0;

// Helper: append uint64_t to Arduino String
static inline void appendUint64(String &s, uint64_t v) {
  char b[21];
  snprintf(b, sizeof(b), "%llu", (unsigned long long)v);
  s += b;
}

// ISR for rain tips (debounced)
void rainISR() {
  uint32_t t = micros();
  if (t - lastPulseUs > 5000) { // ~5 ms debounce
    rainTipsCounter++;
    lastPulseUs = t;
  }
}

// ---- Protos ----
void takeSample();
bool connectNetwork();
bool connectMQTT();
void publishBuffer();
void sleepFor(uint32_t seconds);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println("MKR NB 1500 → TB logger");

  // ADS1115
  Wire.begin();
  if (!ads.begin()) {
    Serial.println("ADS1115 not found!");
  } else {
    // Pick a gain to match your sensor range; GAIN_ONE = ±4.096 V FS (LSB ~125 uV)
    ads.setGain(GAIN_ONE);
  }

  // Rain input
  pinMode(PIN_RAIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RAIN), rainISR, FALLING);

  // Optional RAT selection (not all core versions expose this)
  #ifdef RADIO_ACCESS_TECHNOLOGY_NBIOT
    nbAccess.setRadioAccessTechnology(
      USE_NBIOT ? RADIO_ACCESS_TECHNOLOGY_NBIOT
                : RADIO_ACCESS_TECHNOLOGY_EUTRAN
    );
  #endif

  // Take an immediate sample so first publish has data
  takeSample();
}

void loop() {
  static uint32_t lastSample = 0;
  static uint32_t lastPublishTs = 0;

  uint32_t now = nowUnix();

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
      // Detach for power saving
      gprs.detachGPRS();
      nbAccess.shutdown();
    }
    lastPublishTs = nowUnix();
  }

  // Sleep until next second tick (saves a lot on idle)
  sleepFor(1);
}

// ---- Take one sample (every minute) ----
void takeSample() {
  // Read analog (river level) on ADS1115 AIN0
  int16_t adc = ads.readADC_SingleEnded(0);

  // Snapshot rain tips since last sample
  uint16_t tips = rainTipsCounter;
  rainTipsCounter = 0;

  uint32_t ts = nowUnix();

  // Push to ring buffer
  size_t nextHead = (headIdx + 1) % BUF_MAX;
  if (nextHead == tailIdx) { // buffer full -> drop oldest
    tailIdx = (tailIdx + 1) % BUF_MAX;
  }
  buf[headIdx] = { ts, adc, tips };
  headIdx = nextHead;

  Serial.print("Sample t="); Serial.print(ts);
  Serial.print(" adc="); Serial.print(adc);
  Serial.print(" tips="); Serial.println(tips);
}

// ---- Network attach + one-shot NTP sync if needed ----
bool connectNetwork() {
  Serial.println("Attaching cellular...");
  for (int i = 0; i < 3; i++) {
    if (nbAccess.begin(PINNUMBER, APN, APN_USER, APN_PASS) == NB_READY) {
      if (gprs.attachGPRS() == GPRS_READY) {
        Serial.println("Cellular attached.");
        if (!time_synced) {
          unsigned long t = ntpUnix();
          if (t) {
            setUnixTime(t);
            Serial.print("NTP synced: "); Serial.println((unsigned long)t);
          } else {
            Serial.println("NTP sync failed (will use relative time for now).");
          }
        }
        return true;
      }
    }
    delay(2000);
  }
  Serial.println("Cellular attach failed.");
  return false;
}

// ---- MQTT connect (TB: username=token, password blank) ----
bool connectMQTT() {
  Serial.println("Connecting MQTT (ThingsBoard)...");
  mqtt.setId(DEVICE_ID);
  mqtt.setUsernamePassword(TB_TOKEN, "");
  if (!mqtt.connect(MQTT_HOST, MQTT_PORT)) {
    Serial.print("MQTT connect failed, code=");
    Serial.println(mqtt.connectError());
    return false;
  }
  Serial.println("MQTT connected.");
  return true;
}

// ---- Publish TB-native timestamp array ----
void publishBuffer() {
  if (headIdx == tailIdx) {
    Serial.println("No data to send.");
    return;
  }

  String payload;
  payload.reserve(2048); // adjust if your bursts are large
  payload += "[";

  size_t i = tailIdx;
  bool first = true;
  while (i != headIdx) {
    uint64_t ts_ms = (uint64_t)buf[i].ts * 1000ULL; // per-sample ms

    if (!first) payload += ',';
    payload += "{\"ts\":";
    appendUint64(payload, ts_ms);
    payload += ",\"values\":{";
    payload += "\"a\":"; payload += buf[i].a;     // raw ADC counts
    payload += ",\"c\":"; payload += buf[i].c;     // rain tips during that minute
    payload += "}}";

    first = false;
    i = (i + 1) % BUF_MAX;
  }
  payload += "]";

  Serial.print("Publishing "); Serial.print(payload.length()); Serial.println(" bytes (TB format)");

  if (mqtt.beginMessage(TB_TOPIC, false, 1)) { // retained=false, QoS=1
    mqtt.print(payload);
    mqtt.endMessage();
    tailIdx = headIdx;   // advance on success
    Serial.println("Publish OK");
  } else {
    Serial.println("Publish beginMessage() failed");
  }
}

// ---- Sleep helper (ms granularity) ----
void sleepFor(uint32_t seconds) {
  if (seconds == 0) return;
  LowPower.deepSleep(seconds * 1000UL);
}
