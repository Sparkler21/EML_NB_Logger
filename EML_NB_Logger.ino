/*
  MKR NB 1500 river/rain logger → ThingsBoard (EU)
  - ADS1115 (AIN0) for river level
  - Rain gauge on D7 (contact to GND), debounced in ISR
  - Sample every 60 s, publish every 10 min
  - MQTT over IP (TCP:1883) — NO DNS, uses known-good broker IPs
  - NTP sync once after attach; server-time used until synced
*/

#include <MKRNB.h>
#include <ArduinoMqttClient.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <NBUDP.h>
#include <RTCZero.h>

// ===================== User config =====================
#define DEVICE_ID   "EML_HPT_001"

// Your APN for LTE-M
#define APN         "gigsky-02"
#define APN_USER    ""
#define APN_PASS    ""
#define PINNUMBER   ""          // SIM PIN if any

// ThingsBoard MQTT (EU) — using IPs to skip DNS on LPWA
const char* MQTT_IPS[] = { "3.127.14.137", "18.196.252.195", "63.176.17.51" };
const uint8_t N_MQTT_IPS = sizeof(MQTT_IPS)/sizeof(MQTT_IPS[0]);
const int     MQTT_PORT  = 1883;                 // plain MQTT
const char*   TB_TOPIC   = "v1/devices/me/telemetry";
const char*   TB_TOKEN   = "uo423fza9leqz4pry7cb"; // username for MQTT

// Cadence (seconds)
const uint32_t SAMPLE_INTERVAL_SEC  = 60;        // 1 minute
const uint32_t PUBLISH_INTERVAL_SEC = 60;       // 10 minutes
// =======================================================

// Pins
const int PIN_RAIN = 7;  // contact-closure input (to GND)

// Globals
volatile uint32_t rainTipsCounter = 0;
volatile uint32_t lastPulseUs = 0;
bool cellAttached = false;   // remember attach state across publishes

// Devices
Adafruit_ADS1115 ads;
NB nbAccess;
GPRS gprs;
NBClient nbClient;           // non-TLS socket for MQTT :1883
MqttClient mqtt(nbClient);
NBUDP udp;
RTCZero rtc;

// ---- Monotonic, NTP-anchored clock (no RTC stalls) ----
static uint32_t unix_base = 0;   // UTC seconds at last sync
static uint32_t ms_base   = 0;   // millis() at last sync
static bool     time_synced = false;
volatile bool rtc_ready = false;   // becomes true after first NTP sync

static inline void setUnixTime(uint32_t unix_now) {
  if (unix_now == 0) return;
  unix_base = unix_now;
  ms_base   = millis();
  time_synced = true;
}

static inline uint32_t nowUnix() {
  if (rtc_ready) {
    return rtc.getEpoch();          // UTC seconds from RTC once set
  } else {
    // pre-sync fallback so code never blocks
    return millis() / 1000UL;       // relative seconds since boot
  }
}

// Robust NTP (UDP) → UNIX seconds; 0 on failure
// - Tries multiple anycast NTP IPs (no DNS)
// - Two attempts per server, with LPWA-friendly timeouts
// - Random local UDP port; clean open/close to avoid socket clashes
unsigned long ntpUnix() {
  // Known-good anycast NTPs (IPv4)
  static const char* NTP_IPS[] = {
    "162.159.200.1",   // time.cloudflare.com
    "216.239.35.0",    // time.google.com anycast
    "216.239.35.4",    // time.google.com anycast
    "129.250.35.250"   // NTT pool
  };
  static const uint8_t N_NTP_IPS = sizeof(NTP_IPS)/sizeof(NTP_IPS[0]);

  const uint16_t NTP_PORT = 123;
  const unsigned long RECV_TIMEOUT_MS = 4000;   // LPWA: allow up to 4s for a reply
  const unsigned long BETWEEN_TRIES_MS = 200;

  // NTP request packet
  uint8_t pkt[48] = {0};
  pkt[0] = 0b11100011; // LI=3 (unsync), VN=4, Mode=3 (client)

  // Ensure a clean UDP socket and choose a random-ish local port
  udp.stop();
  uint16_t localPort = 20000 + (millis() % 20000);
  if (udp.begin(localPort) != 1) {
    Serial.println("NTP: UDP begin failed");
    return 0;
  }

  for (uint8_t s = 0; s < N_NTP_IPS; ++s) {
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
      // Start a fresh packet to the current server
      if (!udp.beginPacket(NTP_IPS[s], NTP_PORT)) {
        delay(BETWEEN_TRIES_MS);
        continue;
      }
      udp.write(pkt, sizeof(pkt));
      udp.endPacket();

      // Wait for response (single packet is enough)
      unsigned long t0 = millis();
      while (!udp.parsePacket()) {
        if (millis() - t0 > RECV_TIMEOUT_MS) break;
      }

      if (udp.available() >= 48) {
        uint8_t rb[48];
        int n = udp.read(rb, sizeof(rb));
        // Close ASAP to free socket
        udp.stop();

        if (n >= 48) {
          // Transmit Timestamp (seconds since 1900) at bytes 40..43
          unsigned long secs1900 =
            ((unsigned long)rb[40] << 24) |
            ((unsigned long)rb[41] << 16) |
            ((unsigned long)rb[42] <<  8) |
             (unsigned long)rb[43];

          const unsigned long NTP_TO_UNIX = 2208988800UL; // 1900→1970
          unsigned long unix = secs1900 - NTP_TO_UNIX;

          // Sanity check: > 2005-01-01
          if (unix > 1104537600UL) {
            Serial.print("NTP OK from "); Serial.print(NTP_IPS[s]);
            Serial.print(" → "); Serial.println(unix);
            return unix;
          }
        }

        // If we got here the packet was bad; reopen for next try
        if (udp.begin(localPort) != 1) {
          Serial.println("NTP: UDP re-begin failed");
          return 0;
        }
      }

      delay(BETWEEN_TRIES_MS);
    }
  }

  // All attempts failed; close socket and report 0
  udp.stop();
  return 0;
}


// Ring buffer of minute samples
struct Sample { uint32_t ts; int16_t riverLevel; uint16_t rain; };
const size_t BUF_MAX = 512;
Sample buf[BUF_MAX];
size_t headIdx = 0, tailIdx = 0;

// Helper: append uint64_t to Arduino String
static inline void appendUint64(String &s, uint64_t v) {
  char b[21];
  snprintf(b, sizeof(b), "%lu", (unsigned long long)v);
  s += b;
}

// ---- Coverage / registration helpers (use AT passthrough when available) ----

// Parse +CSQ format (0..31 good; 99 = unknown/no signal).
// Rough mapping: dBm ≈ -113 + 2*CSQ (legacy RSSI heuristic)
static bool modemGetCSQ(int &csq, int &dbm) {
  csq = 99; dbm = -999;
#if defined(MODEM_H)
  MODEM.begin();
  // Clear any stale bytes
  while (MODEM.available()) MODEM.read();

  MODEM.println("AT+CSQ");
  String resp;
  unsigned long t0 = millis();
  while (millis() - t0 < 600) {
    while (MODEM.available()) resp += (char)MODEM.read();
  }
  int p = resp.indexOf("+CSQ:");
  if (p >= 0) {
    int c, b;
    if (sscanf(resp.c_str() + p, "+CSQ: %d,%d", &c, &b) == 2) {
      csq = c;
      if (csq >= 0 && csq <= 31) dbm = -113 + (csq * 2);
      return true;
    }
  }
#endif
  return false;
}

// Read LTE registration: +CEREG: <n>,<stat> ...  stat: 1=registered (home), 5=registered(roaming)
static bool modemGetCEREG(int &stat) {
  stat = 0;
#if defined(MODEM_H)
  MODEM.begin();
  while (MODEM.available()) MODEM.read();

  MODEM.println("AT+CEREG?");
  String resp;
  unsigned long t0 = millis();
  while (millis() - t0 < 600) {
    while (MODEM.available()) resp += (char)MODEM.read();
  }
  int p = resp.indexOf("+CEREG:");
  if (p >= 0) {
    // Handle formats like: +CEREG: 0,1  or +CEREG: 2,5,"...", ...
    int n, s;
    if (sscanf(resp.c_str() + p, "+CEREG: %d,%d", &n, &s) == 2) {
      stat = s;
      return true;
    }
  }
#endif
  return false;
}

// Wait for usable signal and/or registration before trying to attach.
// minCSQ: e.g. 7 (~ -99 dBm) is a practical floor for NB/LTE-M.
// If AT passthrough isn’t available, we just return true (can’t test).
static bool waitForCoverage(uint8_t minCSQ, uint32_t timeoutMs) {
#if !defined(MODEM_H)
  // No AT passthrough available in this core build; skip gating.
  return true;
#else
  unsigned long start = millis();
  int lastDbm = -999, lastCsq = 99, reg = 0;
  while (millis() - start < timeoutMs) {
    modemGetCSQ(lastCsq, lastDbm);
    modemGetCEREG(reg);

    // Show a heartbeat every ~2s
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000) {
      Serial.print("Signal CSQ="); Serial.print(lastCsq);
      Serial.print(" ("); Serial.print(lastDbm); Serial.print(" dBm)");
      Serial.print("  CEREG="); Serial.println(reg);
      lastPrint = millis();
    }

    // Accept if registered and CSQ meets threshold (or CSQ unavailable but registered)
    bool registered = (reg == 1 || reg == 5);
    bool okSignal   = (lastCsq != 99 && lastCsq >= (int)minCSQ);
    if (registered && okSignal) return true;

    delay(500);
  }
  Serial.println("Coverage check: insufficient signal/registration this cycle.");
  return false;
#endif
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
bool connectMQTT_IP();
void publishOnceSimple();
void publishBuffer();
void sleepFor(uint32_t seconds);

// ===================== Setup / Loop =====================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.println("MKR NB 1500 → TB logger (MQTT by IP)");

  // Faster socket ops so connect() doesn't hang forever
  nbClient.setTimeout(30000); // 30 s

  // ADS1115
  Wire.begin();
  if (!ads.begin()) {
    Serial.println("ADS1115 not found!");
  } else {
    // Pick gain to match your sensor; GAIN_ONE = ±4.096 V FS (≈125 µV/LSB)
    ads.setGain(GAIN_ONE);
  }

  // Rain input
  pinMode(PIN_RAIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RAIN), rainISR, FALLING);

  // Optional RAT selection (if core exposes it)
  #ifdef RADIO_ACCESS_TECHNOLOGY_NBIOT
    nbAccess.setRadioAccessTechnology(RADIO_ACCESS_TECHNOLOGY_EUTRAN); // LTE-M
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
      if (connectMQTT_IP()) {
        if (!time_synced) {
          // First cycle (or after loss of time): send simple object
          publishOnceSimple();
        } else {
          // Normal: send full timestamped batch
          publishBuffer();
        }
        mqtt.stop();
      }
      // Stay attached for reliability/power — do NOT detach/shutdown here
      // gprs.detachGPRS();
      // nbAccess.shutdown();
    }
    lastPublishTs = nowUnix();
  }
  static uint32_t lastNtp = 0;
  if (rtc_ready && (now - lastNtp) >= 86400UL) {   // every 24h
    unsigned long t = ntpUnix();
    if (t) {
      rtc.setEpoch(t);
      Serial.print("RTC re-synced: "); Serial.println(t);
    }
  lastNtp = now;
}
  // Light sleep
  // sleepFor(1);
  delay(1000);
}

// ===================== Helpers =====================

// Take one sample (every minute)
void takeSample() {
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
  Serial.print(" riverLevel="); Serial.print(adc);
  Serial.print(" rain="); Serial.println(tips);
}

// Network attach + one-shot NTP sync if needed
bool connectNetwork() {
  // If we already have an IP, don't re-attach
  if (cellAttached) {
    IPAddress ip = gprs.getIPAddress();
    if (ip != IPAddress(0,0,0,0)) {
      Serial.print("Already attached. PDP IP: "); Serial.println(ip);
      return true;
    } else {
      Serial.println("Attached flag set but no IP; will reattach.");
      cellAttached = false; // force reattach
    }
  }

  Serial.println("Attaching cellular...");
  const uint32_t ATTACH_TIMEOUT = 60000UL; // ~60 s for LTE-M
  unsigned long start = millis();

// Quick pre-check: don’t even try to attach if there’s no signal/registration.
// minCSQ 7 ≈ ~ -99 dBm (heuristic); adjust to taste (6..9). Timeout e.g. 30 s.
if (!waitForCoverage(/*minCSQ*/7, /*timeoutMs*/30000)) {
  // Skip this publish window; we’ll try again next loop.
  return false;
}

  while (millis() - start < ATTACH_TIMEOUT) {
    NB_NetworkStatus_t s = nbAccess.begin(PINNUMBER, APN, APN_USER, APN_PASS);
    Serial.print("nbAccess.begin() -> "); Serial.println((int)s);
    if (s == NB_READY) {
      NB_NetworkStatus_t g = gprs.attachGPRS();
      Serial.print("gprs.attachGPRS() -> "); Serial.println((int)g);
      if (g == GPRS_READY) {
        IPAddress ip = gprs.getIPAddress();
        Serial.print("PDP IP: "); Serial.println(ip);
        if (ip == IPAddress(0,0,0,0)) {
          Serial.println("No PDP IP yet, retrying...");
        } else {
        if (!rtc_ready) {
          unsigned long t = ntpUnix();
          if (t) {
            // 1) keep your monotonic anchors (optional, if you still use them elsewhere)
            setUnixTime(t);

            // 2) start and set the RTC in UTC
            rtc.begin();            // safe to call once after boot
            rtc.setEpoch(t);        // set UTC seconds
            rtc_ready = true;

            Serial.print("NTP synced & RTC set: "); Serial.println(t);
          } else {
            Serial.println("NTP sync failed (using server-time until next try).");
          }
        }

          cellAttached = true;
          Serial.println("Cellular attached.");
          return true;
        }
      }
    }
    delay(2000);
  }

  int csq, dbm, reg;
  if (modemGetCSQ(csq, dbm) && modemGetCEREG(reg)) {
    Serial.print("Pre-attach signal OK: CSQ="); Serial.print(csq);
    Serial.print(" ("); Serial.print(dbm); Serial.print(" dBm), CEREG=");
    Serial.println(reg);
  }


  Serial.println("Cellular attach failed.");
  return false;
}

// MQTT connect (by IP, with retries)
bool connectMQTT_IP() {
  Serial.println("Connecting MQTT (1883) by IP...");
  mqtt.stop();
  mqtt.setId(DEVICE_ID);
  mqtt.setUsernamePassword(TB_TOKEN, "");
  mqtt.setKeepAliveInterval(60);
  nbClient.setTimeout(30000);

  for (uint8_t i = 0; i < N_MQTT_IPS; ++i) {
    Serial.print("MQTT try IP: "); Serial.println(MQTT_IPS[i]);
    unsigned long t0 = millis();
    if (mqtt.connect(MQTT_IPS[i], MQTT_PORT)) {
      Serial.print("MQTT connected in "); Serial.print(millis() - t0); Serial.println(" ms");
      return true;
    }
    Serial.print("connect failed, code="); Serial.println(mqtt.connectError());
    delay(3000);
  }
  return false;
}

// One-off simple publish (TB server timestamps)
void publishOnceSimple() {
  if (headIdx == tailIdx) return;
  size_t last = (headIdx + BUF_MAX - 1) % BUF_MAX;

  String msg = "{\"riverLevel\":";
  msg += buf[last].riverLevel;
  msg += ",\"rain\":";
  msg += buf[last].rain;
  msg += "}";

  Serial.println("MQTT payload (simple):");
  Serial.println(msg);

  if (mqtt.beginMessage(TB_TOPIC, false, 0)) { // retained=false, QoS0
    mqtt.print(msg);
    mqtt.endMessage();
    Serial.println("Simple publish OK");
    // keep buffer (we'll still batch them later)
  } else {
    Serial.println("Simple publish failed");
  }
}
/*
void publishBuffer() {
  if (headIdx == tailIdx) return;
  size_t last = (headIdx + BUF_MAX - 1) % BUF_MAX;

  if (!rtc_ready) {
    Serial.println("RTC not set yet; sending simple server-timestamped object.");
    publishOnceSimple();   // your existing function
    return;
  }
  uint64_t ts_ms = (uint64_t)buf[i].ts * 1000ULL;

  String msg = "[{\"ts\":";
  appendUint64(msg, ts_ms);
  msg += ",\"values\":{";
  msg += "\"riverLevel\":";
  msg += buf[last].riverLevel;
  msg += ",\"rain\":";
  msg += buf[last].rain;
  msg += "}}]";

  Serial.println("MQTT payload (simple):");
  Serial.println(msg);

  if (mqtt.beginMessage(TB_TOPIC, false, 0)) { // retained=false, QoS0
    mqtt.print(msg);
    mqtt.endMessage();
    Serial.println("Simple publish OK");
    // keep buffer (we'll still batch them later)
  } else {
    Serial.println("Simple publish failed");
  }  
}
*/
// Publish TB-native timestamped array
void publishBuffer() {
  if (!rtc_ready) {
    Serial.println("RTC not set yet; sending simple server-timestamped object.");
    publishOnceSimple();   // your existing function
    return;
  }

  if (headIdx == tailIdx) {
    Serial.println("No data to send.");
    return;
  }

  String payload;
  payload.reserve(2048);
  payload += "[";

  size_t i = tailIdx;
  bool first = true;
  while (i != headIdx) {
    uint64_t ts_ms = (uint64_t)buf[i].ts * 1000ULL;

    if (!first) payload += ',';
    payload += "{\"ts\":";
    appendUint64(payload, ts_ms);
    payload += ",\"values\":{";
    payload += "\"riverLevel\":"; payload += buf[i].riverLevel;
    payload += ",\"rain\":";       payload += buf[i].rain;
    payload += "}}";

    first = false;
    i = (i + 1) % BUF_MAX;
  }
  payload += "]";

  Serial.print("Publishing "); Serial.print(payload.length()); Serial.println(" bytes (TB format)");
  Serial.println("MQTT payload:");
  Serial.println(payload);

  if (mqtt.beginMessage(TB_TOPIC, false, 1)) { // retained=false, QoS1
    mqtt.print(payload);
    mqtt.endMessage();
    tailIdx = headIdx;   // advance on success
    Serial.println("Publish OK (batch)");
  } else {
    Serial.println("Publish beginMessage() failed");
  }
}

// Low-power helper (optional)
void sleepFor(uint32_t seconds) {
  if (seconds == 0) return;
  delay(seconds * 1000UL); // swap for LowPower.deepSleep() later if needed
}
