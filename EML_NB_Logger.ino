/*
  MKR NB 1500 river/rain logger → ThingsBoard (EU)
  - ADS1115 (AIN0) for river level
  - Rain gauge on D7 (contact to GND), debounced in ISR
  - Sample every 60 s, publish every 10 min
*/

#include <MKRNB.h>
#include <ArduinoMqttClient.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <NBUDP.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>
#include <sam.h>

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
volatile uint32_t rain24hrTipsCounter = 0;
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

// Ring buffer of minute samples
struct Sample { 
    uint16_t sampleNo = 0;
    uint32_t ts; 
    int16_t riverLevel; 
    uint16_t rainInterval;
    uint16_t rain24hr; 
  };

const size_t BUF_MAX = 10;
Sample samples[BUF_MAX];
bool rtcWakeFlag = false;
uint16_t currentSampleNo = 0;
uint16_t logIntervalMinutes = 10;  //  Number of minutes till we send the message
bool logIntervalFlag = false;

uint32_t tsSendValue = 0;; 
uint16_t riverLevelAveSendValue = 0;
uint16_t riverLevelMaxSendValue = 0;    //  This equals the minimum river level in mm.
uint16_t riverLevelMinSendValue = 5000; //  This equals the maximum river level in mm.
uint16_t rainIntervalSendValue = 0;
uint16_t rain24hrSendValue = 0;

//  Helpers

static inline uint32_t nextMinuteBoundary(uint32_t t) {
  uint32_t next = ((t / 60) + 1) * 60;   // round up to :00
  if (next <= t) next += 60;             // absolute safety
  return next;
}

void armNextTopOfMinute() {
  uint32_t now  = rtc.getEpoch();
  uint32_t next = nextMinuteBoundary(now);
  rtc.setAlarmEpoch(next);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}

void setup() {

  #if ENABLE_DEBUG
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("MKR NB 1500 → TB logger (MQTT by IP)");
  #endif

  // Start peripherals you actually use
  Wire.begin();
  ads.begin(); 

  // Start RTC (needed for timed wake)
  rtc.begin();
//  rtc.setTime(0, 0, 0);                   // hh, mm, ss (until you have NTP)
//  rtc.setAlarmSeconds(0);                 // fire when seconds == 0
//  rtc.enableAlarm(rtc.MATCH_SS);          // once per minute at :00
  // Until you NTP-sync, set any valid epoch so the RTC ticks forward.
  // Replace this with rtc.setEpoch(ntpEpoch) once you have network time.
  rtc.setEpoch(1730000000UL);

  // Wake on RTC alarm (minute boundary)
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, rtcWakeISR, CHANGE);
// Rain input
  pinMode(PIN_RAIN, INPUT_PULLUP);
  // Allow wake on rain contact (D7 -> FALLING to GND)
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), rainWakeISR, FALLING);

  // First wake at the next :00
  armNextTopOfMinute();

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
  if (rtcWakeFlag) {
    rtcWakeFlag = false;
    #if ENABLE_DEBUG
      Serial.println("Sampling...");
    #endif
    // Sample Sensors
    takeSamples(currentSampleNo);
    // Store/calculate sensor values (SD CARD?)
    storeSamples(currentSampleNo);
    rtcWakeFlag = false;  //Reset Flag

    //Increment currentSampleNo
    currentSampleNo++;  // Increment sample counter
  }

  // Re-arm for the NEXT :00 (recomputed to avoid drift)
  armNextTopOfMinute();
  // Optional: visual proof you’re on the minute
  // (epoch % 60 should be ~0 when you print)
  #if ENABLE_DEBUG
    Serial.print("Woke at epoch=");
    Serial.println(rtc.getEpoch());
  #endif

  // If it is time to send report/payload message
  if ((currentSampleNo + 1) >= logIntervalMinutes){  //  Note: currentSampleNo starts at 0 so need to add 1.  10min interval.
    logIntervalFlag = true;
    #if ENABLE_DEBUG
      Serial.println("Report and Sending...");
    #endif
    calcLogSamples(currentSampleNo);
    // Create Json Payload report string and Send
    createJsonMsg();

    // If "Not Connected" Flag is TRUE
      // Increment recovery counter (maybe run 6 times and 10 mins)
      // If recovery counter is TRUE set "Recovery Counter" = TRUE
  }
  if(logIntervalFlag == true){
    currentSampleNo = 0;  //reset sample counter
  }
  logIntervalFlag = false;  //  Reset Flag

  // #if !ENABLE_DEBUG
    // Sleep!
    LowPower.idle();
    //LowPower.deepSleep();                   // <— put MCU in standby
  // #endif 
}

/****************************************************************/
//                        FUNCTIONS                             //
/****************************************************************/
// Modem Connect Function
// NTP Time Sync Function
// RTC Set-up and Alarms
// Internal Clock System Function

// Sample Function
void takeSamples(uint16_t no_of_samples) {
  // Read River Level ADC(0)
  int16_t adc = ads.readADC_SingleEnded(0);
  int16_t currentRiverLevel = adc;  //  Need multipliers and offsets?

  // Snapshot rain tips since last sample
  samples[no_of_samples].rainInterval = rainTipsCounter;
  rainTipsCounter = 0;
  samples[no_of_samples].rain24hr = rain24hrTipsCounter;  //  Need to reset rain24hrTipsCounter every midnight!

//  uint32_t ts = nowUnix();


  #if ENABLE_DEBUG
    Serial.print("Sample No.="); Serial.println(samples[no_of_samples].sampleNo);
    Serial.print("ts="); Serial.println(samples[no_of_samples].ts);
    Serial.print("riverLevel="); Serial.println(samples[no_of_samples].riverLevel);
    Serial.print("rainInterval="); Serial.println(samples[no_of_samples].rainInterval);
    Serial.print("rain24hr="); Serial.println(samples[no_of_samples].rain24hr);
  #endif
}
// Process Sensors Function (inc Rain Reset if day change?)
void calcLogSamples(uint16_t no_of_samples){
  uint16_t i = 0;
  while (i < no_of_samples){
    riverLevelAveSendValue = riverLevelAveSendValue + samples[i].riverLevel;  //  Create a Total of the river level sampled values
    if(samples[i].riverLevel > riverLevelMaxSendValue){  // New Max value?
      riverLevelMaxSendValue = samples[i].riverLevel;
    }
    if(samples[i].riverLevel < riverLevelMinSendValue){  // New Min value?
      riverLevelMinSendValue = samples[i].riverLevel;
    }
    rainIntervalSendValue = rainIntervalSendValue + samples[i].rainInterval;  //  Totalise the rain values
    i++;
  }
  tsSendValue = samples[no_of_samples].ts;  // Take the final Timpstamp value
  riverLevelAveSendValue = riverLevelAveSendValue/(no_of_samples + 1);  //  Create the Average river level
  rain24hrSendValue = samples[no_of_samples].rain24hr;  // Take the final value for the rolling 24hr value
}
// Store Samples Data Function (SD Card)
void storeSamples(uint16_t no_of_samples){
    //samples[no_of_samples].sampleNo);
    //samples[no_of_samples].ts);
    //samples[no_of_samples].riverLevel);
    //samples[no_of_samples].rainInterval);
    //samples[no_of_samples].rain24hr);
}

// Create Json message to send
void createJsonMsg(){
  String payload;
  payload.reserve(2048);

  payload += "[";
  payload += "{\"ts\":";
  payload += tsSendValue;   // uint32_t seconds
  payload += "000";       // ms
  payload += ",\"values\":{";
  payload = "{\"riverLevelAve\":";
  payload += riverLevelAveSendValue;
  payload += ",\"riverLevelMax\":";
  payload += riverLevelMaxSendValue;
  payload += ",\"riverLevelMin\":";
  payload += riverLevelMinSendValue;
  payload += ",\"rainInt\":";
  payload += rainIntervalSendValue;
  payload += ",\"rain24hr\":";
  payload += rain24hrSendValue;
  payload += "}}";
  payload += "]";

  #if ENABLE_DEBUG
    Serial.println(payload);
  #endif
}

// Sleep Function
void sleepBetweenSamples() {
  LowPower.deepSleep();  // sleeps until RTC alarm or rain interrupt
}

// RTC Interrupt function
void rtcWakeISR() {
  // 1) Stop further RTC alarm interrupts until we re-arm
  rtc.disableAlarm();

  // 2) Clear the hardware alarm flag (this is the crucial bit)
  RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0;

  // 3) Latch once-per-minute work
  rtcWakeFlag = true;
}

// Rain Interrupt function
// ISR for rain tips (debounced)
void rainWakeISR() {
  uint32_t t = micros();
  if (t - lastPulseUs > 5000) { // ~5 ms debounce
    rainTipsCounter++;
    rain24hrTipsCounter++;
    lastPulseUs = t;
  }
}