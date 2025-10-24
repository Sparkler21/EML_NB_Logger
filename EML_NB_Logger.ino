/*
  MKR NB 1500 river/rain logger → ThingsBoard (EU)
  - ADS1115 (AIN0) for river level
  - Rain gauge on D7 (contact to GND), debounced in ISR
  - Sample every 60 s, publish every 10 min
*/

#include <MKRNB.h>
#include <ArduinoMqttClient.h>
#include <Wire.h>
//#include <Adafruit_ADS1X15.h>
#include <NBUDP.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>

/**********************************************
 * Enables debug support. To disable this
 * feature set to 0.
 **********************************************/
#define ENABLE_DEBUG                       1

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
const uint32_t PUBLISH_INTERVAL_SEC = 600;       // 10 minutes
// =======================================================

// Pins
const int PIN_RAIN = 7;  // contact-closure input (to GND)

// Globals
volatile uint32_t rainTipsCounter = 0;
volatile uint32_t lastPulseUs = 0;
bool cellAttached = false;   // remember attach state across publishes

// Devices
//Adafruit_ADS1115 ads;
NB nbAccess;
GPRS gprs;
NBClient nbClient;           // non-TLS socket for MQTT :1883
MqttClient mqtt(nbClient);
NBUDP udp;
RTCZero rtc;

// Ring buffer of minute samples
struct Sample { 
    uint16_t sampleNo = 0;
    uint32_t ts; 
    int16_t riverLevelTotal; 
    int16_t riverLevelMax; 
    int16_t riverLevelMin; 
    uint16_t rainInterval;
    uint16_t rain24hr; 
  };
const size_t BUF_MAX = 512;
Sample buf[BUF_MAX];
size_t headIdx = 0, tailIdx = 0;

void setup() {

  #if ENABLE_DEBUG
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("MKR NB 1500 → TB logger (MQTT by IP)");
  #endif

  // Start RTC (needed for timed wake)
  rtc.begin();
  rtc.enableAlarm(rtc.MATCH_SS); 

// Rain input
  pinMode(PIN_RAIN, INPUT_PULLUP);
  // Allow wake on rain contact (D7 -> FALLING to GND)
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), rainWakeISR, FALLING);
  // Wake on RTC alarm (minute boundary)
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, rtcWakeISR, CHANGE);

}

void loop() {
  // If first (boot) run or "Recovery Needed" flag 
      // run the modem connection (loop several times to try)
      // if connected....
        // NTP Time Sync
        // RTC Setup and alarms
      // else if not connected...
        // Set "Not Connected" Flag
        // Use internal clock system
  // If SAMPLE_INTERVAL_SEC == TRUE
    // Sample Sensors
    Sample.sampleNo++;
    takeSamples(Sample.sampleNo);
    // Store/calculate sensor values (SD CARD?)
  // If PUBLISH_INTERVAL_SEC == TRUE
    // Create Json Payload report string
    // Send and store report strings
    // If "Not Connected" Flag is TRUE
      // Increment recovery counter (maybe run 6 times and 10 mins)
      // If recovery counter is TRUE set "Recovery Counter" = TRUE
  // #if !ENABLE_DEBUG
    // Sleep!
  // #endif 
}
// Modem Connect Function
// NTP Time Sync Function
// RTC Set-up and Alarms
// Internal Clock System Function
// Sample and process Sensors Function (inc Rain Reset if day change?)
void takeSamples(uint16_t no_of_samples) {
  // Read River Level ADC(0)
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

  buf[headIdx] = { 
      Sample.sampleNo,
      Sample.ts,
      Sample.riverLevelTotal,
      Sample.RiverLevelMax,
      Sample.RiverLevelMin,
      Sample.rainInterval,
      Sample.rain24hr 
    };
  headIdx = nextHead;
  #if ENABLE_DEBUG
    Serial.print("Sample t="); Serial.print(ts);
    Serial.print(" riverLevel="); Serial.print(adc);
    Serial.print(" rain="); Serial.println(tips);
  #endif
}
// Store Data Function (SD Card)

// Sleep Function
void sleepBetweenSamples() {
  LowPower.deepSleep();  // sleeps until RTC alarm or rain interrupt
}

// RTC Interrupt function
void rtcWakeISR() {}  // empty ISR; just wakes the MCU

// Rain Interrupt function
// ISR for rain tips (debounced)
void rainWakeISR() {
  uint32_t t = micros();
  if (t - lastPulseUs > 5000) { // ~5 ms debounce
    rainTipsCounter++;
    lastPulseUs = t;
  }
}