/*
  MKR NB 1500 river/rain logger → ThingsBoard (EU)
  - ADS1115 (AIN0) for river level
  - Rain gauge on D7 (contact to GND), debounced in ISR
  - Sample every 60 s, publish every 10 min

PINS:
D0 = GPS Power Pin
D1 = LED EXT
D2 = 
D3 = River Level Enable
D4 = SD Chip Select
D5 = Flash Chip Select
D6 =
D7 = Rain Gauge Input
D8 = MOSI
D9 = SCK
D10 = MISO
D11 = SDA
D12 = SCL
D13 = ->Rx
D14 = <-Tx
A2 = River Level Input

Notes on Ultrasonic Level Sensor:  XL-MaxSonar-WR/WRC Pin Out 

Pin 1- Leave open (or high) for serial output on the Pin 5 output. When Pin 1 is held low the Pin 5 output sends a pulse 
(instead of serial data), suitable for low noise chaining. 

Pin 2- This pin outputs a pulse-width representation of range. To calculate the distance, use a scale factor of 58uS per cm. 
(MB7051, MB7052, MB7053, MB7054, MB7060, MB7062, MB7066) 
This pin outputs the analog voltage envelope of the acoustic waveform. For the MB7070 series and MB7092 sensors, this 
is a real-time always-active output (MB7070, MB7072, MB7076, MB7092, MB7150, MB7155) 

Pin 3- AN-This pin outputs analog voltage with a scaling factor of (Vcc/1024) per cm. A supply of 5V yields ~4.9mV/
cm., and 3.3V yields ~3.2mV/cm. Hardware limits the maximum reported range on this output to ~700 cm at 5V and ~600 
cm at 3.3V. The output is buffered and corresponds to the most recent range data. 
For the 10-meter sensors (MB7051, MB7053, MB7054, MB7066, MB7076) Pin 3 outputs an analog voltage with a 
scaling of (Vcc/1024) per 2-cm. A supply of 5V yields ~4.9mV/2-cm., and 3.3V yields ~3.2mV/2-cm. This Analog 
Voltage output steps in 2-cm increments. 

Pin 4- RX- This pin is internally pulled high. If Pin-4 is left unconnected or held high, the sensor will continually measure 
the range. If Pin-4 is held low the sensor will stop ranging. Bring high 20uS or more to command a range reading.

Pin 5- TX- When Pin 1 is open or held high, the Pin 5 output delivers asynchronous serial data in an RS232* format, 
except the voltages are 0-Vcc. The output is an ASCII capital “R”, followed by ASCII character digits representing the 
range in centimeters up to a maximum of 765 (select models) or 1068 (select models), followed by a carriage return 
(ASCII 13). The baud rate is 9600, 8 bits, no parity, with one stop bit. Although the voltages of 0V to Vcc are outside the 
RS232* standard, most RS232* devices have sufficient margin to read the 0V to Vcc serial data. If standard voltage level 
RS232* is desired, invert, and connect an RS23* converter such as a MAX232.When Pin 1 is held low, the Pin 5 output 
sends a single pulse, suitable for low noise chaining (no serial data). 

*The MB7150 & MB7155 are TTL output format (inverted RS232) and follow the same ASCII data structure 

V+ Operates on 3V - 5.5V. The average (and peak) current draw for 3.3V operation is 2.1mA (50mA peak) and 5V 
operation is 3.4mA (100mA peak) respectively. Peak current is used during sonar pulse transmit. Please reference page 13 
for minimum operating voltage verses temperature information.

GND-Return for the DC power supply. GND (& V+) must be ripple and noise free for best operation.

*/
#include <MKRNB.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>
#include <TimeLib.h>
#include <ArduinoMqttClient.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_SleepyDog.h>
#include <TinyGPS.h>
#include "NetKick.h"

/************************************************************
 * Enables debug support. To disable this feature set to 0.
 ***********************************************************/
#define ENABLE_DEBUG          1
//#define VBAT_PIN              A6
#define DEFAULT_INT_VALUE     0
#define DEFAULT_BOOL_VALUE    false
#define DEFAULT_STRING_VALUE  "default"
#define DEFAULT_BYTE_VALUE    0x00

/* Initialise Library instances */
RTCZero rtc;
GPRS gprs;
NB nbAccess;
NBScanner scanner;
NBClient nbClient;
NBClient probeClient; 
MqttClient mqttClient(nbClient);
NBUDP Udp;
TinyGPS gps;
NBModem modem;

File    configFile;

static IPAddress tbIp;
static bool tbIpValid = false;

// Configuration settings (from SD Card)
struct parameters {
  String serialNo;
  String deviceID;
  String apn;
  String apnUser;
  String apnPass;
  String pin;
  String tbHostString;
  uint16_t tbPortInt;
  String tbTopicString;
  String tbTokenString;
  int sensorsMode;
  int rtcAlarmSecond;
  int samplingInterval;
  String riverLevelRange;
  String rainGaugeCF;
} settings;

// -------------------- New Settings --------------------
static const char* APNS[] = { "gigsky-02", "flolive.net", "ukapn" };
static const int APN_COUNT = sizeof(APNS) / sizeof(APNS[0]);

static const uint32_t REG_TIMEOUT_MS  = 45000;
static const uint32_t PDP_TIMEOUT_MS  = 30000;
static const uint32_t TCP_TIMEOUT_MS  = 8000;

static const uint32_t SLEEP_SECONDS = 60;
// ----------------------------------------

float riverLevelRange_float;
float rainGaugeCF_float;
const int LED_Ext = 1;
const int SDchipSelect = 4;
const int FLASHchipSelect = 5;
const String logFile = "log.txt";
char    configChar;
// =======================================================
IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
unsigned int localPort = 2390;      // local port to listen for UDP packets
// --- Watchdog + network recovery state ---
const int WDT_TIMEOUT_MS = 16000;          // ~16 s (max on SAMD21)
uint8_t netRecoverFailures = 0;
const uint8_t NET_RECOVER_REBOOT_LIMIT = 3;
// Flags
bool NTP_updatedFlag = false;
volatile bool rtcWakeFlag = false;
volatile bool rainWakeFlag = false;
bool sendMsgFlag = false;
bool goToSleepFlag = false;
bool nbAttachAPN_Flag = false;
bool mqttConnectFlag = false;
bool ntpUpdateFlag = false;
bool gpsUpdateFlag = false;
bool gpsRetryFlag = false;
// --- Time sync state ---
bool useServerTimestamp = true;                 // start with TB server time until we sync
uint32_t nextTimeSyncMs = 0;                    // millis() when we may try again
uint32_t timeSyncBackoffMs = 5UL*60UL*1000UL;   // start with 5 minutes
// Pins
const int PIN_RAIN = 7;  // contact-closure input (to GND)
const int PIN_RL_EN = 3;  // River Level Enable
const int GPSpowerPin = 0;
// Globals
volatile uint32_t rainTipsCounter = 0;
volatile uint32_t rain24hrTipsCounter = 0;
volatile uint32_t lastPulseUs = 0;
String previousSDfilename = "";
String currentSDfilename = "";
volatile bool rainEnabled = false;
// minute samples
float batteryVolts = 0;
uint8_t rssi = 0;
uint8_t pctSignalStrength = 0;
uint8_t sampleNo = 0;
uint16_t riverLevel = 0;
float currentRiverLevel = 0;
float riverLevelTotal = 0;
float riverLevelMax = 0;
float riverLevelMin = 0;
uint8_t rainInterval = 0;
uint16_t rain24hr = 0; 
uint16_t currentSampleNo = 0;
uint32_t tsSendValue = 0;
float riverLevelAveSendValue = 0;
float riverLevelMaxSendValue = 0;    //  This equals the minimum river level in cm.
float riverLevelMinSendValue = 0; //  This equals the maximum river level in cm.
float rainIntervalSendValue = 0;
float rain24hrSendValue = 0;
uint8_t sampleYear;
uint8_t sampleMonth;
uint8_t sampleDay;
uint8_t sampleHour;
uint8_t sampleMinute;
uint8_t sampleSecond;  

float GPSlat;  // GPS Latitude
float GPSlon;  // GPS Longditude
uint16_t GPSupdateMins = 10080;  //  To set the number on minutes to elapse before updating GPS Date/Time/Location.  Currently weekly.
uint16_t GPScounter = 0;
bool GPStimeReceived = false;

///////////////////////////////////////////////////////////
//  NEW HELPERS
///////////////////////////////////////////////////////////
// -------------------- Watchdog helpers (SleepyDog) --------------------
static void wdtArmForMqttConnect() {
  int actual = Watchdog.enable(16000);
  Serial.print("[WDT] enabled, timeout = ");
  Serial.print(actual);
  Serial.println(" ms");
}

static void wdtDisarm() {
  Watchdog.disable();
  Serial.println("[WDT] disabled");
}

static void wdtFeed() {
  // Only call reset() if watchdog is enabled; harmless if disabled in most builds,
  // but we'll avoid calling wdtFeed() when disabled anyway.
  Watchdog.reset();
}

static void wdtArmLongOp(uint32_t ms) {
  int actual = Watchdog.enable(ms);
  Serial.print("[WDT] enabled, timeout = ");
  Serial.print(actual);
  Serial.println(" ms");
}

static void wdtDoneLongOp() {
  Watchdog.reset();
  Watchdog.disable();
  Serial.println("[WDT] disabled");
}

// -------------------- LED helpers --------------------
static void ledOff() { digitalWrite(LED_Ext, LOW); }
static void ledOn()  { digitalWrite(LED_Ext, HIGH); }
static void ledBlink(uint8_t n, uint16_t onMs = 80, uint16_t offMs = 140) {
  for (uint8_t i = 0; i < n; i++) { ledOn(); delay(onMs); ledOff(); delay(offMs); }
}
static void stage(uint8_t code) { ledOff(); delay(150); ledBlink(code); delay(250); }

// -------------------- Network helpers --------------------
static bool waitSimAndModemReady(uint32_t timeoutMs) {
  Serial.println("[NET] Wait modem+SIM ready...");
  uint32_t t0 = millis();

  while (millis() - t0 < timeoutMs) {
    // Basic AT
    String at = saraAT("AT", 1000);
    if (at.indexOf("OK") < 0) { delay(300); continue; }

    // SIM ready?
    String cpin = saraAT("AT+CPIN?", 1000);
    if (cpin.indexOf("READY") < 0) { delay(500); continue; }

    // Full functionality?
    String cfun = saraAT("AT+CFUN?", 1000);
    if (cfun.indexOf("+CFUN: 1") < 0) { delay(500); continue; }

    Serial.println("[NET] Modem+SIM ready");
    return true;
  }

  Serial.println("[NET] Modem+SIM ready timeout");
  return false;
}

static bool waitForIp(uint32_t timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    wdtFeed();  // <<< WDT FEED HERE (loop can run a long time)
    IPAddress ip = gprs.getIPAddress();
    if ((ip[0] | ip[1] | ip[2] | ip[3]) != 0) {
      Serial.print("[NET] IP: "); Serial.println(ip);
      return true;
    }
    delay(500);
  }
  return false;
}

static bool ensureRegisteredWithKicks(uint32_t regTimeoutMs) {
  Serial.println("[NET] Wait for registration (CEREG)...");
  saraAT("AT+CEREG=2", 2000);
  saraAT("AT+COPS=0", 10000);

  uint32_t t0 = millis();
  uint32_t lastKickMs = 0;

  while (millis() - t0 < regTimeoutMs) {
    wdtFeed();  // <<< WDT FEED HERE (this loop can run 45s)
    String r = saraAT("AT+CEREG?", 2000);
    int stat = parseCeregStat(r);
    Serial.print("[NET] "); Serial.println(r);

    if (stat == 1 || stat == 5) {
      Serial.println("[NET] Registered");
      return true;
    }
    if (stat == 3) {
      Serial.println("[NET] Registration denied");
      return false;
    }

    // If we’re “searching” too long, kick the modem
    // Kick every ~20s if not progressing
    if (millis() - lastKickMs > 20000UL) {
      lastKickMs = millis();
      Serial.println("[NET] Stalled; kick: CFUN=0/1 + COPS=0");
      saraAT("AT+CFUN=0", 8000);
      delay(400);
      saraAT("AT+CFUN=1", 8000);
      delay(1200);
      saraAT("AT+COPS=0", 10000);
      delay(400);
    }

    delay(1500);
  }

  Serial.println("[NET] Registration timeout/failed");
  return false;
}

static bool attachPdpWithApn(const char* apn, uint32_t ipTimeoutMs) {
  bool ip_done = false;
  Serial.print("[NET] Trying APN: "); Serial.println(apn);

  // Clean socket state first
  mqttClient.stop();
  nbClient.stop();
  delay(150);

  // PDP down (fresh context)
  gprs.detachGPRS();
  delay(600);

  // Make APN take effect
  saraAT("AT+CGACT=0,1", 8000);
  delay(150);

  String cmd = String("AT+CGDCONT=1,\"IP\",\"") + apn + "\"";
  Serial.print("[NET] "); Serial.println(cmd);
  Serial.println(saraAT(cmd.c_str(), 3000));
  Serial.println(saraAT("AT+CGDCONT?", 3000));

  Serial.println("[NET] attachGPRS(true)...");
  NB_NetworkStatus_t st = gprs.attachGPRS(true);
  Serial.print("[NET] attachGPRS -> "); Serial.println((int)st);

  uint32_t t0 = millis();
  while (millis() - t0 < ipTimeoutMs) {
    wdtFeed();  // <<< WDT FEED HERE (can run up to 30s)
    IPAddress ip = gprs.getIPAddress();
    if ((ip[0] | ip[1] | ip[2] | ip[3]) != 0) {
      Serial.print("[NET] IP: "); Serial.println(ip);
      Serial.println(saraAT("AT+CGATT?", 2000));
      Serial.println(saraAT("AT+CGPADDR=1", 3000));
      ip_done = true;
      return true;
    }
    delay(500);
  }

  Serial.println("[NET] PDP/IP timeout");
  return false;
}

static bool bringUpNetworkStable() {
  // If SARA serial is wedged, recover it first
  if (!saraPing()) {
    Serial.println("[NET] SerialSARA not responding; CFUN toggle");
    saraAT("AT+CFUN=0", 8000);
    delay(500);
    saraAT("AT+CFUN=1", 8000);
    delay(1500);
  }

  wdtFeed();  // <<< WDT FEED HERE (before big operations)

  if (!ensureRegisteredWithKicks(REG_TIMEOUT_MS)) return false;

  // Try APNs without re-running nbAccess.begin()
  for (int i = 0; i < APN_COUNT; i++) {
    if (attachPdpWithApn(APNS[i], PDP_TIMEOUT_MS)) {
      Serial.println("[NET] Network OK");
      
      // Give NB/LTE-M a moment after IP assignment
      delay(5000);

      // DNS + TCP warmup: helps a lot with intermittent CONNACK=3 cases
      if (!dnsResolveTB()) {
        Serial.println("[DNS] Resolve failed (will still try MQTT)");
      }

//      if (!tcpProbeTB(12000)) {
//        Serial.println("[TCP] Broker unreachable right now");
//        // Optional: treat as network failure so APN loop can retry
//        return false;
//      }
      return true;
    }
  }
  Serial.println("[NET] All APNs failed");
  return false;
}

static bool dnsResolveTB() {
  String cmd = String("AT+UDNSRN=0,\"") + settings.tbHostString.c_str() + "\"";
  String r = saraAT(cmd.c_str(), 12000);

  Serial.print("[DNS] "); Serial.println(cmd);
  Serial.println(r);

  // Success if it contains "+UDNSRN:" AND "OK"
  bool ok = (r.indexOf("+UDNSRN:") >= 0) && (r.indexOf("\nOK") >= 0 || r.endsWith("OK\r\n") || r.indexOf("OK") >= 0);

  Serial.print("[DNS] ok="); Serial.println(ok ? "true" : "false");
  return ok;
}



static bool tcpProbeTB(uint32_t timeoutMs) {
  Serial.println("[TCP] Probe broker...");
  probeClient.setTimeout(timeoutMs);

  bool ok;
  if (tbIpValid) ok = probeClient.connect(tbIp, settings.tbPortInt);
  else          ok = probeClient.connect(tbIp, settings.tbPortInt);

  if (!ok) {
    Serial.println("[TCP] Probe failed");
    probeClient.stop();
    return false;
  }

  Serial.println("[TCP] Probe OK");
  probeClient.stop();
  delay(200);
  return true;
}


// -------------------- MQTT helpers --------------------
static void mqttConfigure() {
  nbClient.setTimeout(TCP_TIMEOUT_MS);

  String cid = settings.deviceID + "-" + String((uint32_t)millis(), HEX);
  mqttClient.setId(cid);

  mqttClient.setUsernamePassword(settings.tbTokenString.c_str(), "");
  mqttClient.setKeepAliveInterval(45 * 1000);
}

static bool mqttConnectOnce() {
  mqttConfigure();
  Serial.print("[MQTT] Connecting to "); Serial.print(settings.tbHostString.c_str()); Serial.print(":"); Serial.println(settings.tbPortInt);

  mqttClient.stop();
  nbClient.stop();
  delay(1000);
  // light socket close attempt (ignore failures)
  saraAT("AT+USOCL=0", 2000);
  saraAT("AT+USOCL=1", 2000);
  delay(200);

  // If this *still* blocks, we’ll handle it next by guarding with WDT.
  bool ok = mqttClient.connect(settings.tbHostString.c_str(), settings.tbPortInt);

  if (!ok) {
    int e = mqttClient.connectError();
    Serial.print("[MQTT] connectError="); Serial.println(e);
    return false;
  }
  Serial.println("[MQTT] Connected");
  return true;
}

static bool mqttConnectWithRetries(int attempts) {
  mqttConfigure();
  nbClient.setTimeout(15000);
  mqttClient.setConnectionTimeout(15000);

  bool didPdpRebuild = false;

  for (int i = 1; i <= attempts; i++) {
    Serial.print("[MQTT] Connect attempt "); Serial.println(i);

    // Clean socket state + give the network/NAT a moment
    mqttClient.stop();
    nbClient.stop();
    delay(2000);

    Serial.println("[WDT] Arm watchdog for MQTT connect");
    wdtArmForMqttConnect();                 // 16s

    Serial.println("[MQTT] Calling mqttClient.connect()");
    bool ok = mqttClient.connect(settings.tbHostString.c_str(), settings.tbPortInt);

    Serial.println("[WDT] MQTT connect returned; feeding watchdog");
    wdtFeed();
    Watchdog.disable();
    Serial.println("[WDT] disabled");

    if (ok) {
      Serial.println("[MQTT] Connected");
      return true;
    }

    int e = mqttClient.connectError();
    Serial.print("[MQTT] connectError="); Serial.println(e);

    // Cleanup after failure
    mqttClient.stop();
    nbClient.stop();
    delay(1000);

    // Only do the PDP rebuild ONCE, and only for CONNACK=3
    if (e == 3 && !didPdpRebuild) {
      didPdpRebuild = true;
      Serial.println("[MQTT] e=3: rebuild PDP then retry once");

      Serial.println("[PDP] detachGPRS...");
      wdtArmLongOp(30000);                 // 30s for PDP operations
      gprs.detachGPRS();
      wdtDoneLongOp();
      Serial.println("[PDP] detachGPRS done");

      delay(2500);

      Serial.println("[PDP] attachGPRS(true)...");
      wdtArmLongOp(30000);
      NB_NetworkStatus_t st2 = gprs.attachGPRS(true);
      wdtDoneLongOp();
      Serial.print("[PDP] attachGPRS -> "); Serial.println((int)st2);

      Serial.println("[PDP] waitForIp...");
      bool gotIp = waitForIp(15000);
      Serial.print("[PDP] waitForIp -> "); Serial.println(gotIp ? "OK" : "TIMEOUT");

      // Give routing/NAT time to settle after reattach
      delay(5000);

      // Retry (loop continues to next attempt)
      continue;
    }

    // If we reach here:
    // - error wasn't 3, OR
    // - error was 3 but we've already rebuilt PDP once
    Serial.println("[MQTT] Not retrying further this cycle");
    return false;
  }

  return false;
}


static void modemPowerDown() {
  Serial.println("[PWR] stop MQTT/TCP");
  mqttClient.stop();
  nbClient.stop();
  delay(500);

  Serial.println("[PWR] detach GPRS");
  gprs.detachGPRS();
  delay(500);

  Serial.println("[PWR] nbAccess.shutdown()");
  nbAccess.shutdown();
  delay(500);
}



///////////////////////////////////////////////////////////
//  Helper Functions
///////////////////////////////////////////////////////////


static void printResetCause() {
  uint8_t rc = PM->RCAUSE.reg;

  Serial.print("[RST] RCAUSE=0x");
  Serial.println(rc, HEX);

  if (rc & PM_RCAUSE_POR)   Serial.println("[RST] Power-on reset");
  if (rc & PM_RCAUSE_BOD12) Serial.println("[RST] Brown-out 1.2V");
  if (rc & PM_RCAUSE_BOD33) Serial.println("[RST] Brown-out 3.3V");
  if (rc & PM_RCAUSE_EXT)   Serial.println("[RST] External reset");
  if (rc & PM_RCAUSE_WDT)   Serial.println("[RST] Watchdog reset");
  if (rc & PM_RCAUSE_SYST)  Serial.println("[RST] System reset");
}



static void setApnContextFromSettings() {
  // Ensure CGDCONT matches SD card settings
  String cmd = "AT+CGDCONT=1,\"IP\",\"" + settings.apn + "\"";
  Serial.print("[NET] Set APN (CGDCONT): ");
  Serial.println(cmd);
  Serial.println(saraAT(cmd.c_str(), 3000));
  Serial.println(saraAT("AT+CGDCONT?", 3000));
}

static bool attachWithApn(const String& apn, uint32_t ipWaitMs = 20000) {
  gprs.detachGPRS();
  delay(600);

  Serial.println(saraAT("AT+CGACT=0,1", 5000)); // make CGDCONT change take effect
  delay(300);

  String cmd = "AT+CGDCONT=1,\"IP\",\"" + apn + "\"";
  Serial.print("[NET] Set CGDCONT APN -> ");
  Serial.println(apn);
  Serial.println(saraAT(cmd.c_str(), 3000));
  Serial.println(saraAT("AT+CGDCONT?", 3000));

  Serial.println("[NET] gprs.attachGPRS(true) ...");
  NB_NetworkStatus_t st = gprs.attachGPRS(true);
  Serial.print("[NET] attachGPRS status -> ");
  Serial.println((int)st);

  unsigned long t0 = millis();
  while (millis() - t0 < ipWaitMs) {
    IPAddress ip = gprs.getIPAddress();
    if ((ip[0] | ip[1] | ip[2] | ip[3]) != 0) {
      Serial.print("[NET] IP acquired: ");
      Serial.println(ip);
      Serial.println(saraAT("AT+CGATT?", 2000));
      Serial.println(saraAT("AT+CGPADDR=1", 3000));
      return true;
    }
    delay(500);
  }

  Serial.println("[NET] No IP after attach attempt");
  return false;
}

static void modemPowerDownForSleep(bool printDiag = true) {
  Serial.println("[PWR] Modem power-down...");

  // Stop MQTT/client first (avoids weird hangs)
  mqttClient.stop();
  nbClient.stop();
  delay(100);


  // 0) Stop MQTT/TCP first (clean socket close)
  if (mqttClient.connected()) {
    Serial.println("[PWR] Stopping MQTT...");
    mqttClient.stop();
  }
  nbClient.stop();
  delay(100);

  // 1) Optional diagnostics while modem is still awake
  if (printDiag) {
    Serial.println("[PWR] Diagnostics (before RF off)");
    Serial.println(saraAT("AT+CFUN?", 2000));    // expect 1
    Serial.println(saraAT("AT+CPAS", 2000));
    Serial.println(saraAT("AT+CGATT?", 2000));  // expect 1
    Serial.println(saraAT("AT+CGACT?", 2000));  // see PDP state
  }

  // 2) Deactivate PDP context (saves power + avoids stuck sockets)
  Serial.println("[PWR] PDP deactivate (CGACT=0,1)...");
  Serial.println(saraAT("AT+CGACT=0,1", 8000));
  delay(100);

  // 3) RF off (big current drop)
  Serial.println("[PWR] RF off (CFUN=0)...");
  Serial.println(saraAT("AT+CFUN=0", 8000));
  delay(200);

  // 4) Verify (optional)
  if (printDiag) {
    Serial.println("[PWR] Diagnostics (after RF off)");
    Serial.println(saraAT("AT+CFUN?", 2000));    // expect 0
    Serial.println(saraAT("AT+CGATT?", 2000));  // expect 0 or ERROR
  }

  // 5) Full modem shutdown (cuts deeper)
  Serial.println("[PWR] Modem shutdown (nbAccess.shutdown())");
  nbAccess.shutdown();
  delay(200);

  Serial.println("[PWR] Modem is down");
}

static void prepareForSleep() {
  // Kill LED
  ledOff();

  // Put modem into lowest state you can (PDP down + RF off + shutdown)
  modemPowerDownForSleep(false);

  // Optional: stop Serial USB to reduce a little (if you can live without USB logs)
  // Serial.end();
}


void resetAlarms(){
    rtc.setAlarmSeconds(settings.rtcAlarmSecond);
    rtc.enableAlarm(rtc.MATCH_SS);
}

//  Get sampling time from RTC
void sampleTimeandDateFromRTC(){
  sampleSecond = rtc.getSeconds();  
  sampleMinute = rtc.getMinutes();
  sampleHour = rtc.getHours();
  sampleYear = rtc.getYear();
  sampleMonth = rtc.getMonth();
  sampleDay = rtc.getDay();
}

static bool modemPowerUp(bool printDiag = true) {
  Serial.println("[PWR] Modem power-up...");

  // Power the SARA back on (PWRKEY pulse)
  modem.begin();

  // Give the modem time to boot firmware
  delay(1000);

  // Wait until AT interface responds
  unsigned long t0 = millis();
  while (millis() - t0 < 15000) {
    if (saraAT("AT", 1000).indexOf("OK") >= 0) {
      break;
    }
    delay(300);
  }

  // Basic sanity config
  saraAT("ATE0", 2000);      // echo off
  saraAT("AT+CMEE=2", 2000);

  // Re-enable RF stack (you shut it down earlier)
  Serial.println("[PWR] RF on (CFUN=1)");
  Serial.println(saraAT("AT+CFUN=1", 10000));

  if (printDiag) {
    Serial.println("[PWR] Diagnostics after wake");
    Serial.println(saraAT("AT+CFUN?", 2000));
    Serial.println(saraAT("AT+CPIN?", 2000));
    Serial.println(saraAT("AT+CGATT?", 2000));
  }

  return saraAT("AT", 1000).indexOf("OK") >= 0;
}


static bool modemWakeAndInit(uint32_t readyTimeoutMs = 30000) {
  // Start MKRNB state machine
  int bs = 0;
  while(bs <= 1){
    Serial.println("[NET] nbAccess.begin()...");
    nbAccess.begin();
    Serial.print("[NET] nbAccess.begin() -> "); Serial.println(bs);
  }
  // Even if bs looks OK, after a shutdown() the modem may still be booting internally.
  // So wait until AT works + SIM ready + CFUN=1.
  if (!waitSimAndModemReady(readyTimeoutMs)) {
    Serial.println("[NET] modemWakeAndInit: modem not ready");
    return false;
  }

  // Ensure full RF on (in case you left it CFUN=0)
  saraAT("AT+CFUN=1", 8000);
  delay(1200);

  return true;
}


String createSDfilename(){

  String SDfilename = "";
  SDfilename += sampleYear; SDfilename += sampleMonth; SDfilename += sampleDay; SDfilename += ".csv";
  Serial.print("SDfilename: ");
  Serial.println(SDfilename);
  return SDfilename;
}

void writeSDcard(String SDfilename, String dataString){
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(SDchipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(SDfilename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.print("error opening ");
    Serial.println(SDfilename);
  }
  delay(1000);
}

bool getGPSinfo(){

  //TURN ON GPS DEVICE
  digitalWrite(GPSpowerPin, HIGH);
  Serial1.begin(9600);
  delay(1000);

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  uint8_t gpsAttemptsCounter = 0;
  uint8_t GPSattempts = 100;  //  How many times (Seconds) to try to connect to GPS
  
  Serial.println("[GPS] Searching...");

  while(!GPStimeReceived && (gpsAttemptsCounter <= GPSattempts)){
    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      ledBlink(1);
      while (Serial1.available())
      {
        char c = Serial1.read();
        // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        if (gps.encode(c)) // Did a new valid sentence come in?
          newData = true;
      }
    }

    if (newData)
    {
      unsigned long age; 
      int GPSyear;
      byte GPSmonth, GPSday, GPShour, GPSminute, GPSsecond, GPShundredths;
      gps.f_get_position(&GPSlat, &GPSlon, &age);
      Serial.print("LAT=");
      Serial.print(GPSlat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : GPSlat, 6);
      Serial.print(" LON=");
      Serial.print(GPSlon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : GPSlon, 6);
      Serial.print(" SAT=");
      Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      Serial.print(" PREC=");
      Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

      gps.crack_datetime(&GPSyear, &GPSmonth, &GPSday, &GPShour, &GPSminute, &GPSsecond, &GPShundredths, &age);
      Serial.print(" Year=");
      Serial.print(GPSyear);
      Serial.print(" month=");
      Serial.print(GPSmonth);
      Serial.print(" Day=");
      Serial.print(GPSday);
      Serial.print(" Hour=");
      Serial.print(GPShour);
      Serial.print(" Minute=");
      Serial.print(GPSminute);
      Serial.print(" Second=");
      Serial.print(GPSsecond);

      if(GPSyear > 2024){
        rtc.setTime(GPShour, GPSminute, GPSsecond);
        rtc.setDate(GPSday, GPSmonth, (GPSyear-2000));
        GPStimeReceived = true;  //  Got our Date and Time
      }

    }
    
    unsigned short numberSats = gps.satellites();

    gps.stats(&chars, &sentences, &failed);
    Serial.print(" CHARS=");
    Serial.print(chars);
    Serial.print(" SENTENCES=");
    Serial.print(sentences);
    Serial.print(" CSUM ERR=");
    Serial.print(failed);
    Serial.print(" Sats=");
    Serial.println(numberSats);
    if (chars == 0)
      Serial.println("** No characters received from GPS: check wiring **"); 

    gpsAttemptsCounter++;
  }
    //TURN OFF GPS DEVICE
    digitalWrite(GPSpowerPin, LOW);
    // Cleanly stop GPS UART to avoid any lingering serial activity
    Serial1.end();
    if(GPStimeReceived){

      return true;
    }
    else{

      return false;
    }
}

void writeSDcardLog(String dataString){
  String dataStringLog = "";

  dataStringLog += rtc.getDay();
  dataStringLog += "-";
  dataStringLog += rtc.getMonth();
  dataStringLog += "-";
  dataStringLog += rtc.getYear();  
  dataStringLog += " ";
  dataStringLog += rtc.getHours();
  dataStringLog += ":";
  dataStringLog += rtc.getMinutes();
  dataStringLog += ":";
  dataStringLog += rtc.getSeconds();
  dataStringLog += " ";
  dataStringLog += dataString;
//  dataStringLog += "\n"; 
  writeSDcard(logFile, dataStringLog);
}

bool publishTelemetryJSON(const char* topic, const String& json) {

if (!mqttClient.connected()) {
  Serial.println("[PUB] MQTT not connected; skip publish");
  return false;
}
  if (!mqttClient.beginMessage(topic)) {
    Serial.println(F("[PUB] beginMessage failed -> sleep and retry next wake"));
    writeSDcardLog("[PUB] beginMessage failed -> sleep");

    mqttClient.stop();
    nbClient.stop();
    delay(100);

//    modemPowerDownForSleep();
//    goToSleepFlag = true;
    return false;
  }

  mqttClient.print(json);
  bool ok = mqttClient.endMessage();
  Serial.print(F("[PUB] endMessage() -> "));
  Serial.println(ok ? F("OK") : F("FAIL"));

  unsigned long t0 = millis();
  while (millis() - t0 < 800) { mqttClient.poll(); delay(20); }

  if (!ok) {
    Serial.println(F("[PUB] Send failed -> sleep and retry next wake"));
    writeSDcardLog("[PUB] Send failed -> sleep");

    mqttClient.stop();
    nbClient.stop();
    delay(100);

//    modemPowerDownForSleep();
//    goToSleepFlag = true;
    return false;
  }

  return true;
}

void readBatteryVoltage() {
  float v_adc = 0.0;
  uint32_t adc = 0;
  const int N = 10;

  for (int i = 0; i < N; i++) {
    adc = adc + analogRead(ADC_BATTERY);   // 0–4095 for 12-bit ADC
    delay(50); // small delay between samples
  }
  v_adc = ((adc/N) / 4095.0) * 3.3f; //
  batteryVolts = v_adc * 1.275f;  // Reverse the voltage divider (1.2 MOhm and 330 kOhm)
  #if ENABLE_DEBUG
    Serial.print("Battery Volts = "); Serial.print(batteryVolts); Serial.print(", ");
  #endif
}
void readRSSI() {
  rssi = scanner.getSignalStrength().toInt();
  if (rssi == 99){
    pctSignalStrength = 0;
  }
  else{
    pctSignalStrength = rssi * 3;
  }
    #if ENABLE_DEBUG
    Serial.print("SignalStr = "); Serial.print(pctSignalStrength); Serial.print(", ");
  #endif
}

void takeRiverLevelSamples(uint16_t no_of_samples) {
  // Read River Level ADC(0)
  uint32_t adc = 0;
  const int N = 10;

  //POWER UP SENSOR
  digitalWrite(PIN_RL_EN, HIGH);
  //SMALL DELAY of MORE THAN 20uS
  delay(50); // small delay between samples  

  for (int i = 0; i < N; i++) {
    //adc = ads.readADC_SingleEnded(0);
    adc = adc + analogRead(2);
    delay(50); // small delay between samples
  }
  //SHUT DOWN SENSOR
  digitalWrite(PIN_RL_EN, LOW);
  currentRiverLevel = adc/N;  //  Need multipliers and offsets?
  //currentRiverLevel = (currentRiverLevel/1024) * riverLevelRange;
  currentRiverLevel = ((currentRiverLevel/4095.0) * 3300)/3.2;
  adc = 0;  // reset
  riverLevelTotal = riverLevelTotal + currentRiverLevel;  //  Totalise the river level samples
  #if ENABLE_MAX_MIN_RL
    if(riverLevelMax < currentRiverLevel){
      riverLevelMax = currentRiverLevel;  //  New Max in the samples
    }
    if(riverLevelMin > currentRiverLevel){
      riverLevelMin = currentRiverLevel;  //  New Min in the samples
    }
  #endif

  #if ENABLE_DEBUG
    Serial.print(sampleYear+2000); Serial.print("-"); Serial.print(sampleMonth); Serial.print("-"); Serial.print(sampleDay);
    Serial.print(", ");
    Serial.print(sampleHour); Serial.print(":"); Serial.print(sampleMinute); Serial.print(":"); Serial.print(sampleSecond); Serial.print(", ");
    Serial.print("Sample No = "); Serial.print(no_of_samples); Serial.print(", ");
    Serial.print("RiverLevel-Current = "); Serial.print(currentRiverLevel); Serial.print(", ");
    Serial.print("RiverLevel-Total = "); Serial.print(riverLevelTotal); Serial.print(", ");
    #if ENABLE_MAX_MIN_RL
      Serial.print("RiverLevel-Max = "); Serial.print(riverLevelMax); Serial.print(", ");
      Serial.print("RiverLevel-Min = "); Serial.println(riverLevelMin);
    #endif
  #endif
}

void takeRainSamples(uint16_t no_of_samples) {

  #if ENABLE_DEBUG
    Serial.print(sampleYear+2000); Serial.print("-"); Serial.print(sampleMonth); Serial.print("-"); Serial.print(sampleDay);
    Serial.print(", ");
    Serial.print(sampleHour); Serial.print(":"); Serial.print(sampleMinute); Serial.print(":"); Serial.print(sampleSecond); Serial.print(", ");
    Serial.print("Sample No = "); Serial.print(no_of_samples); Serial.print(", ");
    Serial.print("rainInterval = "); Serial.print(rainTipsCounter); Serial.print(", ");
    Serial.print("rain24hr = "); Serial.println(rain24hrTipsCounter);
  #endif
}

// Process Sensors Function (inc Rain Reset if day change?)
void calcSamples(uint16_t no_of_samples){
Serial.println("calcSamples");
  if(settings.sensorsMode == 0 || settings.sensorsMode == 2){  //River Level
    riverLevelAveSendValue = riverLevelTotal/no_of_samples;  //  Calculate the average river level during the sampling period
    #if ENABLE_MAX_MIN_RL
      riverLevelMaxSendValue = riverLevelMax;
      riverLevelMinSendValue = riverLevelMin;
      riverLevelMax = 0;
      riverLevelMin = riverLevelRange_float;
    #endif
    riverLevelTotal = 0;
  }

  if(settings.sensorsMode == 0 || settings.sensorsMode == 1){  //Rain
    rainIntervalSendValue = rainTipsCounter * rainGaugeCF_float;
    rain24hrSendValue = rain24hrTipsCounter * rainGaugeCF_float;
    rainTipsCounter = 0;
  }
  currentSampleNo = 0;
}

// Create Json message to send
void createAndSendJsonMsg(){
  String payload;
  payload.reserve(2048);

  //if (!useServerTimestamp) {  //  Changed from NTP to GPS so changed flag
    if (GPStimeReceived){
    // use device timestamp (array with ts + values)
//    uint32_t ts = rtc.getEpoch();     // seconds since 1970
    if (settings.sensorsMode == 0) {
      payload += "[{\"ts\":"; payload += tsSendValue; payload += "000";
      payload += ",\"values\":{";
      payload += "\"device\":\""; payload += settings.deviceID;           payload += "\",";
      payload += "\"batV\":";     payload += batteryVolts;                payload += ",";
      payload += "\"signalStr\":";     payload += pctSignalStrength;      payload += ",";
      payload += "\"rlAve\":";    payload += riverLevelAveSendValue;      payload += ",";
      #if ENABLE_MAX_MIN_RL
      payload += "\"rlMax\":";    payload += riverLevelMaxSendValue;      payload += ",";
      payload += "\"rlMin\":";    payload += riverLevelMinSendValue;      payload += ",";
      #endif
      payload += "\"rainInt\":";  payload += rainIntervalSendValue;       payload += ",";
      payload += "\"rain24hr\":"; payload += rain24hrSendValue;
      payload += "}}]";
    } else if (settings.sensorsMode == 1) {
      payload += "[{\"ts\":";     payload += tsSendValue; payload += "000";
      payload += ",\"values\":{";
      payload += "\"device\":\""; payload += settings.deviceID;           payload += "\",";
      payload += "\"batV\":";     payload += batteryVolts;                payload += ",";
      payload += "\"signalStr\":";     payload += pctSignalStrength;           payload += ",";
      payload += "\"rainInt\":";  payload += rainIntervalSendValue;       payload += ",";
      payload += "\"rain24hr\":"; payload += rain24hrSendValue;
      payload += "}}]";
    } else { // 2
      payload += "[{\"ts\":";     payload += tsSendValue; payload += "000";
      payload += ",\"values\":{";
      payload += "\"device\":\""; payload += settings.deviceID;           payload += "\",";
      payload += "\"batV\":";     payload += batteryVolts;                payload += ",";
      payload += "\"signalStr\":";     payload += pctSignalStrength;           payload += ",";
      payload += "\"riverLevelAve\":"; payload += riverLevelAveSendValue; 
      #if ENABLE_MAX_MIN_RL
      payload += ",";
      payload += "\"riverLevelMax\":"; payload += riverLevelMaxSendValue; payload += ",";
      payload += "\"riverLevelMin\":"; payload += riverLevelMinSendValue;
      #endif
      payload += "}}]";
    }
  } else {
    // server timestamp: plain object (no ts, no "values" wrapper)
    if (settings.sensorsMode == 0) {
      payload += "{";
      payload += "\"device\":\""; payload += settings.deviceID;           payload += "\",";
      payload += "\"batV\":";    payload += batteryVolts;                 payload += ",";
      payload += "\"signalStr\":";     payload += pctSignalStrength;           payload += ",";
      payload += "\"rlAve\":";   payload += riverLevelAveSendValue;       payload += ",";
      #if ENABLE_MAX_MIN_RL
      payload += "\"rlMax\":";   payload += riverLevelMaxSendValue;       payload += ",";
      payload += "\"rlMin\":";   payload += riverLevelMinSendValue;       payload += ",";
      #endif
      payload += "\"rainInt\":"; payload += rainIntervalSendValue;        payload += ",";
      payload += "\"rain24hr\":";payload += rain24hrSendValue;
      payload += "}";
    } else if (settings.sensorsMode == 1) {
      payload += "{";
      payload += "\"device\":\""; payload += settings.deviceID;           payload += "\",";
      payload += "\"batV\":";    payload += batteryVolts;                 payload += ",";
      payload += "\"signalStr\":";     payload += pctSignalStrength;           payload += ",";
      payload += "\"rainInt\":";  payload += rainIntervalSendValue;       payload += ",";
      payload += "\"rain24hr\":"; payload += rain24hrSendValue;
      payload += "}";
    } else {
      payload += "{";
      payload += "\"device\":\""; payload += settings.deviceID;           payload += "\",";
      payload += "\"batV\":";    payload += batteryVolts;                 payload += ",";
      payload += "\"signalStr\":";     payload += pctSignalStrength;           payload += ",";
      payload += "\"riverLevelAve\":"; payload += riverLevelAveSendValue; 
      #if ENABLE_MAX_MIN_RL
      payload += ",";
      payload += "\"riverLevelMax\":"; payload += riverLevelMaxSendValue; payload += ",";
      payload += "\"riverLevelMin\":"; payload += riverLevelMinSendValue;
      #endif
      payload += "}";
    }
  }

  // Hardened publish you already added:
bool ok = publishTelemetryJSON(settings.tbTopicString.c_str(), payload);
#if ENABLE_DEBUG
  Serial.print("Sent: "); Serial.println(payload);
  if (!ok) Serial.println("[PUB] Telemetry send failed");
#endif
}

// Create Json message to send
void createAndSendGPSJsonMsg(){
  String payload;
  payload.reserve(2048);

  uint32_t ts = rtc.getEpoch();

  payload += "[{\"ts\":"; payload += ts; payload += "000";
  payload += ",\"values\":{";
  payload += "\"GPSlat\":"; payload += String(GPSlat, 6); payload += ",";
  payload += "\"GPSlon\":"; payload += String(GPSlon, 6);
  payload += "}}]";


  // Hardened publish you already added:
bool ok = publishTelemetryJSON(settings.tbTopicString.c_str(), payload);
#if ENABLE_DEBUG
  Serial.print("Sent: "); Serial.println(payload);
  if (!ok) Serial.println("[PUB] Telemetry send failed");
#endif
}

// Create message to SD save
String createSDcardPayloadHeader(){
  String payloadHeader;
    //Headers
    payloadHeader += ("Date"); payloadHeader += (",");
    payloadHeader += ("Time"); payloadHeader += (",");
    payloadHeader += ("Device"); payloadHeader += (",");
    payloadHeader += ("BatteryVolts"); payloadHeader += (",");
    payloadHeader += ("SignalStr"); payloadHeader += (",");
    if (settings.sensorsMode == 0){  //  Both RL and Rain
      payloadHeader += ("RiverLevel-Ave"); payloadHeader += (",");
      #if ENABLE_MAX_MIN_RL
        payloadHeader += ("RiverLevel-Max"); payloadHeader += (",");
        payloadHeader += ("RiverLevel-Min"); payloadHeader += (",");
      #endif
      payloadHeader += ("rainInterval"); payloadHeader += (",");
      payloadHeader += ("rain24hr"); payloadHeader += (",");
    }
    else if (settings.sensorsMode == 1){  //  Rainfall Only
      payloadHeader += ("rainInterval"); payloadHeader += (",");
      payloadHeader += ("rain24hr"); payloadHeader += (",");
    }
    else if (settings.sensorsMode == 2){  // River Level Only
      payloadHeader += ("RiverLevel-Current"); payloadHeader += (",");
      #if ENABLE_MAX_MIN_RL
        payloadHeader += ("RiverLevel-Max"); payloadHeader += (",");
        payloadHeader += ("RiverLevel-Min"); payloadHeader += (",");
      #endif    
    }
    payloadHeader += ("\n");
    //Units
    payloadHeader += ("DD/MM/YY"); payloadHeader += (",");
    payloadHeader += ("hh:mm:ss"); payloadHeader += (",");
    payloadHeader += ("Name"); payloadHeader += (",");
    payloadHeader += ("V"); payloadHeader += (",");
    payloadHeader += ("%"); payloadHeader += (",");  // 0-31... 31 being the strongest.  99
    if (settings.sensorsMode == 0){  //  Both RL and Rain
      payloadHeader += ("cm"); payloadHeader += (",");
      #if ENABLE_MAX_MIN_RL
        payloadHeader += ("cm"); payloadHeader += (",");
        payloadHeader += ("cm"); payloadHeader += (",");
      #endif
      payloadHeader += ("mm"); payloadHeader += (",");
      payloadHeader += ("mm"); payloadHeader += (",");
    }
    else if (settings.sensorsMode == 1){  //  Rainfall Only
      payloadHeader += ("mm"); payloadHeader += (",");
      payloadHeader += ("mm"); payloadHeader += (",");
    }
    else if (settings.sensorsMode == 2){  // River Level Only
      payloadHeader += ("cm"); payloadHeader += (",");
      #if ENABLE_MAX_MIN_RL
        payloadHeader += ("cm"); payloadHeader += (",");
        payloadHeader += ("cm"); payloadHeader += (",");
      #endif    
    }

    return payloadHeader;  
}

// Create message to SD save
String createSDcardPayload(){
  String payload;

    payload += (sampleYear+2000); payload += ("-"); payload += (sampleMonth); payload += ("-"); payload += (sampleDay); payload += (",");
    payload += (sampleHour); payload += (":"); payload += (sampleMinute); payload += (":"); payload += (sampleSecond); payload += (",");
    payload += (settings.deviceID); payload += (",");
    payload += (batteryVolts); payload += (",");
    payload += (pctSignalStrength); payload += (",");
    if (settings.sensorsMode == 0){  //  Both RL and Rain
      payload += (riverLevelAveSendValue); payload += (","); 
      #if ENABLE_MAX_MIN_RL
        payload += (riverLevelMaxSendValue); payload += (",");
        payload += (riverLevelMinSendValue); payload += (",");
      #endif
      payload += (rainIntervalSendValue); payload += (",");
      payload += (rain24hrSendValue); payload += (",");
    }
    else if (settings.sensorsMode == 1){  //  Rainfall Only
      payload += (rainIntervalSendValue); payload += (",");
      payload += (rain24hrSendValue); payload += (",");
    }
    else if (settings.sensorsMode == 2){  // River Level Only
      payload += (riverLevelAveSendValue); payload += (","); 
      #if ENABLE_MAX_MIN_RL
        payload += (riverLevelMaxSendValue); payload += (",");
        payload += (riverLevelMinSendValue); payload += (",");
      #endif    
    }
    return payload;
}

////////////////////////////////////////
//  Get the Configs from the SD Card  //
////////////////////////////////////////
boolean getSettings()
{
 // Open the settings file for reading:
  String  description = "";

  Serial.print("Reading SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(SDchipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  configFile = SD.open("SETTINGS.TXT");
  if (!configFile) {
    Serial.println("Settings not opened");
    return false;
  }

  // read from the file until there's nothing else in it:
  while (configFile.available()) 
  {
    configChar = configFile.read();

    if (configChar == '#')           
    { // Comment - ignore this line
      skipLine();

    } 
    else if (isalnum(configChar))  
    { // Add a configChar to the description
      description.concat(configChar);
    } 
    else if (configChar =='=')     
    { // start checking the value for possible results

      // First going to trim out all trailing white spaces
      do 
      {
        configChar = configFile.read();
      } 
      while(configChar == ' ');

      // Property list
      if (description == "serialNo") 
      {
        settings.serialNo = getStringSetting(DEFAULT_STRING_VALUE);
        settings.serialNo.trim();
      }
      else if (description == "deviceID") 
      {
        settings.deviceID = getStringSetting(DEFAULT_STRING_VALUE);
        settings.deviceID.trim();
      }
      else if (description == "apn") 
      {
        settings.apn = getStringSetting(DEFAULT_STRING_VALUE);
        settings.apn.trim();
      }
      else if (description == "apnUser") 
      {
        settings.apnUser = getStringSetting(DEFAULT_STRING_VALUE);
        settings.apnUser.trim();
      }
      else if (description == "apnPass") 
      {
        settings.apnPass = getStringSetting(DEFAULT_STRING_VALUE);
        settings.apnPass.trim();
      }
      else if (description == "pin") 
      {
        settings.pin = getStringSetting(DEFAULT_STRING_VALUE);
        settings.pin.trim();
      }
      else if (description == "tbHostString") 
      {
        settings.tbHostString = getStringSetting(DEFAULT_STRING_VALUE);
        settings.tbHostString.trim();
      }
      else if (description == "tbPortInt") 
      {
        settings.tbPortInt = getIntSetting(DEFAULT_INT_VALUE);
      }
      else if (description == "tbTopicString") 
      {
        settings.tbTopicString = getStringSetting(DEFAULT_STRING_VALUE);
        settings.tbTopicString.trim();
      }
      else if (description == "tbTokenString") 
      {
        settings.tbTokenString = getStringSetting(DEFAULT_STRING_VALUE);
        settings.tbTokenString.trim();
      }
      else if (description == "sensorsMode") 
      {
        settings.sensorsMode = getIntSetting(DEFAULT_INT_VALUE);
      }
      else if (description == "rtcAlarmSecond") 
      {
        settings.rtcAlarmSecond = getIntSetting(DEFAULT_INT_VALUE);
      } 
      else if (description == "samplingInterval") 
      {
        settings.samplingInterval = getIntSetting(DEFAULT_INT_VALUE);
      } 
      else if (description == "riverLevelRange") 
      {
        settings.riverLevelRange = getStringSetting(DEFAULT_STRING_VALUE);
        settings.riverLevelRange.trim();
      }
      else if (description == "rainGaugeCF") 
      {
        settings.rainGaugeCF = getStringSetting(DEFAULT_STRING_VALUE);
        settings.rainGaugeCF.trim();
      }
      else 
      { // Unknown parameter - ignore this line
        skipLine();
      }
      description = "";
    } 
    else 
    {
      // Ignore this configChar (could be space, tab, newline, carriage return or something else)
    }
  }
  // close the file:
  configFile.close();

  riverLevelRange_float = settings.riverLevelRange.toFloat();   //  Convert from Strings to Floats
  rainGaugeCF_float = settings.rainGaugeCF.toFloat();

  return true;
}

////////////////////////////////////////////
//  Skipline - Part of Configs functions  //
////////////////////////////////////////////
void skipLine()
{
  do 
  {
    configChar = configFile.read();
  } 
  while (configChar != '\n');
}
////////////////////////////////////////////////////
//  Get Int Settings - Part of Configs functions  //
////////////////////////////////////////////////////
int getIntSetting(int defaultValue)
{
  String  value = "";
  boolean valid = true;

  while (configChar != '\n') 
  {
    if (isdigit(configChar) || configChar == '-') 
    {
      value.concat(configChar);
    } 
//    else if (configChar != '\n') 
//    { // Use of invalid values
//      valid = false;
//    }
    configChar = configFile.read();            
  }
  
  if (valid) { 
    // Convert string to integer
    char charBuf[value.length()+1];
    value.toCharArray(charBuf,value.length()+1);
    return atoi(charBuf);
  } else {
    // revert to default value for invalid entry in settings
    return defaultValue;
  }
}
/////////////////////////////////////////////////////
//  Get Bool Settings - Part of Configs functions  //
/////////////////////////////////////////////////////
bool getBoolSetting(bool defaultValue)
{
  if (configChar == '1') 
  {
    return true;
  } 
  else if (configChar == '0') 
  {
    return false;
  } 
  else 
  {
    return defaultValue;
  }
}
///////////////////////////////////////////////////////
//  Get String Settings - Part of Configs functions  //
///////////////////////////////////////////////////////
String getStringSetting(String defaultValue)
{
  String value = "";
  do 
  {
    value.concat(configChar);
    configChar = configFile.read();
  } 
  while(configChar != '\n');
  
  if (value != "") 
  {
    return value;
  } 
  else 
  {
    return defaultValue;
  }

}

//////////////////////////////////////
//  Print locally the Configs       //
//////////////////////////////////////
void serialPrintSettings()
{
  Serial.println(F("-------------------------------------------------"));
  Serial.println(F("Loading configuration settings from SD Card......"));
  Serial.println(F("-------------------------------------------------"));
  Serial.print(F("serialNo: "));
  Serial.println(settings.serialNo);
  Serial.print(F("deviceID: "));
  Serial.println(settings.deviceID);
  Serial.print(F("apn: "));
  Serial.println(settings.apn);  
  Serial.print(F("apnUser: "));
  Serial.println(settings.apnUser);  
  Serial.print(F("apnPass: "));
  Serial.println(settings.apnPass);  
  Serial.print(F("pin: "));
  Serial.println(settings.pin);  
  Serial.print(F("tbHostString: "));
  Serial.println(settings.tbHostString);  
  Serial.print(F("tbPortInt: "));
  Serial.println(settings.tbPortInt);  
  Serial.print(F("tbTopicString: "));
  Serial.println(settings.tbTopicString);  
  Serial.print(F("tbTokenString: "));
  Serial.println(settings.tbTokenString);  
  Serial.print(F("sensorsMode: "));
  Serial.println(settings.sensorsMode);  
  Serial.print(F("rtcAlarmSecond: "));
  Serial.println(settings.rtcAlarmSecond);  
  Serial.print(F("samplingInterval(min): "));
  Serial.println(settings.samplingInterval);  
  Serial.print(F("riverLevelRange: "));
  Serial.println(settings.riverLevelRange);  
  Serial.print(F("rainGaugeCF: "));
  Serial.println(settings.rainGaugeCF);  
  Serial.println(F("-------------------------------------------------"));
}

static void scheduleNext10MinAlarm() {
  uint8_t h = rtc.getHours();
  uint8_t m = rtc.getMinutes();

  uint8_t nextM = (uint8_t)((m / 10) * 10 + 10);
  uint8_t nextH = h;

  if (nextM >= 60) {
    nextM = 0;
    nextH = (uint8_t)((h + 1) % 24);
  }

  rtc.setAlarmHours(nextH);
  rtc.setAlarmMinutes(nextM);
  rtc.setAlarmSeconds(0);

  rtc.enableAlarm(rtc.MATCH_HHMMSS);
}


///////////////////////////////////////////////////////////
//  SETUP
///////////////////////////////////////////////////////////
void setup() {
  //Pins
  pinMode(LED_Ext, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GPSpowerPin, OUTPUT);
  pinMode(PIN_RAIN, INPUT_PULLUP);  // Rain input
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), rainWakeISR, FALLING);
  digitalWrite(GPSpowerPin, LOW);  
  digitalWrite(LED_BUILTIN, LOW);

  //unused pins:
  pinMode(2, INPUT_PULLUP);  // 
  pinMode(6, INPUT_PULLUP);  // 

  analogReadResolution(12);
  analogReference(AR_DEFAULT);  // 3.3 V ref (default), explicit for clarity

  ledOn();//  Put LED On to signal booting up.

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  Serial.println("\n=== MIN STABLE BOOT ===");


  if(getSettings())  //  Read SlingShot Configs
  {
    Serial.println("Get Settings - Success");
    serialPrintSettings();  //  Print configs to screen
  }
  else
  {
    Serial.println("Get Settings - Failed");
    writeSDcardLog("[SD] Get Settings - Failed");
  }

  rainEnabled = (settings.sensorsMode == 0 || settings.sensorsMode == 1);


  // RTC Setup
  rtc.begin(); // initialize RTC 24H format
//  rtc.setAlarmSeconds(0);
//  rtc.setAlarmSeconds(settings.rtcAlarmSecond);
//  rtc.enableAlarm(rtc.MATCH_SS);

  //Intialise limits from uploaded settings
  riverLevelMin = riverLevelRange_float;
  riverLevelMinSendValue = riverLevelRange_float;
  ledOff();//  Put LED Off to signal start of connection
  delay(3000);

  Serial.println("[NET] nbAccess.begin()...");            //char : 0 if asynchronous. If synchronous, returns status : 0=ERROR, 1=IDLE, 2=CONNECTING, 3=NB_READY, 4=GPRS_READY, 5=TRANSPARENT_CONNECTED
  int bs = nbAccess.begin();
  Serial.print("[NET] nbAccess.begin() -> "); Serial.println(bs);

  if(bs <= 1){  //Reboot and try again
    Watchdog.enable(1000);
  }

  stage(1);  //  nbAccess complete, now GPS

  //  Time and alarm settings
  gpsUpdateFlag = getGPSinfo();  //  Flash LED during this
  rtc.attachInterrupt(rtcWakeISR);
  scheduleNext10MinAlarm();

  // after PDP is up, before MQTT
  delay(5000);                   // let routing/NAT settle
  saraAT("AT+CGATT?", 2000);      // quick sanity
  saraAT("AT+CGPADDR=1", 3000);   // quick sanity

  stage(2);  //  GPS Complete, Now MQTT

  nbClient.setTimeout(8000);

  if (!mqttConnectWithRetries(3)) {
    // Last-resort: one modem kick then try again
    Serial.println("[MQTT] retry failed; CFUN kick then final try");
    saraAT("AT+CFUN=0", 8000);
    delay(500);
    saraAT("AT+CFUN=1", 8000);
    delay(2000);

    if (!mqttConnectWithRetries(2)) {
      Serial.println("[BOOT] MQTT failed");
    }
  }

  stage(3);  //  MQTT Complete, now send initial GPS message

//  if(nbAttachAPN_Flag && mqttConnectFlag && gpsUpdateFlag){
      #if ENABLE_DEBUG
        Serial.println("createAndSendGPSJsonMsg");
      #endif
    createAndSendGPSJsonMsg();  // Send initial Date/Time, Latitude and Longditude data from GPS to TB

      stage(4);  //  send initial GPS message complete, now loop

  unsigned long t1 = millis();
  while (millis() - t1 < 800) { 
    wdtFeed();  // <<< optional
    mqttClient.poll(); delay(20); 
  }
  goToSleepFlag = true;
}

///////////////////////////////////////////////////////////
//  START OF LOOP
///////////////////////////////////////////////////////////
void loop()
{

  // If woke due to rain tip only: go straight back to sleep
  if (rainWakeFlag && !rtcWakeFlag) {
    rainWakeFlag = false;
    goToSleepFlag = true;
    #if ENABLE_DEBUG
      Serial.print("rainWakeISR!, ");
      Serial.print("rainCount: "); Serial.print(rainTipsCounter);
      Serial.print(", rainCount24hr: "); Serial.println(rain24hrTipsCounter);
    #endif
  }


    // If woke due to 10-min alarm: do the expensive cycle
    if (rtcWakeFlag) {
      rtcWakeFlag = false;
    #if ENABLE_DEBUG
      Serial.println("rtcWakeISR!, ");
    #endif
      // Timestamp + sample time
      tsSendValue = rtc.getEpoch();
      sampleTimeandDateFromRTC();

      // Read battery + RSSI (RSSI requires modem typically; if scanner needs modem, do it later)
      readBatteryVoltage();

      // River level: do it here (only once per 10 min)
      currentSampleNo = 1;                  // since you now sample once per interval
      riverLevelTotal = 0;
      #if ENABLE_MAX_MIN_RL
        riverLevelMax = 0;
        riverLevelMin = riverLevelRange_float;
      #endif
      if (settings.sensorsMode == 0 || settings.sensorsMode == 2) {
        takeRiverLevelSamples(1);
        riverLevelAveSendValue = currentRiverLevel;   // single sample => average = sample
        #if ENABLE_MAX_MIN_RL
          riverLevelMaxSendValue = riverLevelMax;
          riverLevelMinSendValue = riverLevelMin;
        #endif
        riverLevelTotal = 0;
      }

      // Rain values: use counters accumulated during sleep
      if (settings.sensorsMode == 0 || settings.sensorsMode == 1) {
        // atomically copy & clear
        noInterrupts();
        uint32_t tipsInterval = rainTipsCounter;
        rainTipsCounter = 0;
        uint32_t tips24 = rain24hrTipsCounter;
        interrupts();

        rainIntervalSendValue = tipsInterval * rainGaugeCF_float;
        rain24hrSendValue     = tips24        * rainGaugeCF_float;
      }
    #if ENABLE_DEBUG
      Serial.println("[PWR] Modem Power Up");
    #endif
      // Bring modem up, connect, read RSSI if you want it, send
      modemPowerUp(false);
      modemWakeAndInit(30000);

      // (Optional) now RSSI (if it needs modem alive)
      readRSSI();

      bringUpNetworkStable();          // if you want your “stable network bring-up”
      mqttConnectWithRetries(3);
      createAndSendJsonMsg();
    #if ENABLE_DEBUG
      Serial.println("[PWR] Modem Power Down");
    #endif
      modemPowerDownForSleep(false);

      // SD logging
      currentSDfilename = createSDfilename();
      if (currentSDfilename != previousSDfilename) {
        if (!SD.exists(currentSDfilename)) writeSDcard(currentSDfilename, createSDcardPayloadHeader());
        previousSDfilename = currentSDfilename;
      }
      writeSDcard(currentSDfilename, createSDcardPayload());

      // Midnight reset (still works)
      if (sampleHour == 23 && sampleMinute == 59) {
        noInterrupts();
        rain24hrTipsCounter = 0;
        interrupts();
      }

      // Schedule next 10-min alarm and sleep
      scheduleNext10MinAlarm();
      goToSleepFlag = true;
    }

    if (goToSleepFlag) {
      goToSleepFlag = false;
      #if ENABLE_DEBUG
        Serial.println("[PWR] Sleep");
      #endif
//      prepareForSleep();
//      LowPower.deepSleep();   // <-- don’t forget to actually sleep
    }
  }

///////////////////////////////////////////////////////////
//  END OF LOOP
///////////////////////////////////////////////////////////

// Interrupt Functions
void rtcWakeISR()
{
  rtcWakeFlag = true;
}

// Rain Interrupt function
void rainWakeISR() {
  if (!rainEnabled) return;

  uint32_t t = micros();
  if (t - lastPulseUs > 50000) { // 50 ms debounce (adjust if needed)
    rainTipsCounter++;
    rain24hrTipsCounter++;
    lastPulseUs = t;
  }
  rainWakeFlag = true; // just to know why we woke (optional)
}