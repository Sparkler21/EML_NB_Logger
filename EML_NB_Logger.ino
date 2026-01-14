/*
  MKR NB 1500 river/rain logger → ThingsBoard (EU)
  - ADS1115 (AIN0) for river level
  - Rain gauge on D7 (contact to GND), debounced in ISR
  - Sample every 60 s, publish every 10 min

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
//static const char* TB_HOST  = "mqtt.eu.thingsboard.cloud";
//static const int   TB_PORT  = 1883;
//static const char* TB_TOPIC = "v1/devices/me/telemetry";
//static const char* TB_TOKEN = "vxl9ZxrJ5J6soedld65D";
//static const char* DEVICE_ID = "HPT02";

static const char* APNS[] = { "gigsky-02", "flolive.net", "ukapn" };
static const int APN_COUNT = sizeof(APNS) / sizeof(APNS[0]);

static const uint32_t REG_TIMEOUT_MS  = 45000;
static const uint32_t PDP_TIMEOUT_MS  = 30000;
static const uint32_t TCP_TIMEOUT_MS  = 8000;

static const uint32_t SLEEP_SECONDS = 60;
// ----------------------------------------

float riverLevelRange_float;
float rainGaugeCF_float;
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
// minute samples
float batteryVolts = 0;
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
uint16_t GPSupdateMins = 1440;  //  To set the number on minutes to elapse before updating GPS Date/Time/Location.
uint16_t GPScounter = 0;
bool GPStimeReceived = false;

void forceReboot(const char* reason);
bool recoverNetworkAndMqtt();
void bootSequence();

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
static void ledOff() { digitalWrite(LED_BUILTIN, LOW); }
static void ledOn()  { digitalWrite(LED_BUILTIN, HIGH); }
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
  while (millis() - t0 < ipTimeoutMs || ip_done == true) {
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
  stage(5);
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
      stage(5);
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

  stage(7);
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
      ledBlink(5);   // PDP/IP OK
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
//      ledLong(2);    // GPS fix/time OK
      return true;
    }
    else{
      ledBlink(2);   // GPS fail
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

void forceReboot(const char* reason) {
  Serial.print("[REBOOT] ");
  Serial.println(reason);
  writeSDcardLog(String("[REBOOT] ") + reason);
  delay(200);  // let serial/SD flush
  NVIC_SystemReset();  // SAMD21 full reset
}

bool nbAttachAPN(uint32_t regDeadlineMs = 45000, uint32_t pdpDeadlineMs = 30000) {

if (!saraPing()) {
  Serial.println("[NET] SerialSARA not responding; toggling CFUN...");
  saraAT("AT+CFUN=0", 8000);
  delay(500);
  saraAT("AT+CFUN=1", 8000);
  delay(1500);
}

Serial.println("[NET] Ensuring modem is registered (AT+CEREG?)...");

unsigned long t0 = millis();
bool regOK = false;

while (millis() - t0 < regDeadlineMs) {
  String r = saraAT("AT+CEREG?", 2000);
  int stat = parseCeregStat(r);
  Serial.print("[NET] ");
  Serial.println(r);
  if (stat == 1 || stat == 5) { regOK = true; break; }
  if (stat == 3) { Serial.println("[NET] Registration denied"); return false; }
  delay(1500);
}

if (!regOK) {
  Serial.println("[NET] Registration timeout (AT+CEREG?)");
  return false;
}

  // Signal check (non-blocking)
  unsigned long csqStart = millis();
  String csq = scanner.getSignalStrength();
  while (csq == "99" && (millis() - csqStart) < 15000UL) {
    delay(250);
    csq = scanner.getSignalStrength();
  }
  Serial.print("[NET] CSQ: ");
  Serial.println(csq);

  // Try APNs in order: SD first, then provider-recommended fallbacks
  const uint32_t ipWait = pdpDeadlineMs;
bool ok = false;

if (settings.apn.length() > 0) {
  ok = attachWithApn(settings.apn, ipWait);
}
if (!ok && settings.apn != "flolive.net") {
  ok = attachWithApn("flolive.net", ipWait);
}
if (!ok && settings.apn != "gigsky-02") {
  ok = attachWithApn("gigsky-02", ipWait);
}
if (!ok && settings.apn != "ukapn") {
  ok = attachWithApn("ukapn", ipWait);
}

if (ok) return true;

Serial.println("[NET] PDP/IP timeout on all APNs");
return false;

}


bool recoverNetworkAndMqtt() {
  Serial.println(F("[NET] Recovery: NB + MQTT"));

//  gprs.detachGPRS();
//  delay(600);

  if (!nbAttachAPN(20000, 20000)) {
    Serial.println(F("[NET] nbAttachAPN() failed in recovery"));
    netRecoverFailures++;
    if (netRecoverFailures >= NET_RECOVER_REBOOT_LIMIT) {
      forceReboot("[NET] nbAttachAPN() failed repeatedly");
    }
    return false;
  }

  if (!ensureMqttConnected(settings.tbHostString.c_str(), settings.tbPortInt)) {
    Serial.println(F("[NET] MQTT connect failed in recovery"));
    netRecoverFailures++;
    if (netRecoverFailures >= NET_RECOVER_REBOOT_LIMIT) {
      forceReboot("[NET] MQTT connect failed repeatedly");
    }
    return false;
  }

  // Success – clear error counter
  netRecoverFailures = 0;
  Serial.println(F("[NET] Recovery successful"));
  return true;
}


void printMqttErr(int e) {
  Serial.print("[MQTT] connectError="); Serial.println(e);
  switch (e) {
    case -1: Serial.println("[MQTT] Timeout waiting for CONNACK"); break;
    case -2: Serial.println("[MQTT] TCP connect failed (PDP/DNS/firewall)"); break;
    case -3: Serial.println("[MQTT] Protocol/broker rejected"); break;
    case -4: Serial.println("[MQTT] Auth/token rejected (check device token)"); break;
    default: Serial.println("[MQTT] Unknown error"); break;
  }
}

// --- MQTT helpers ---
void setupMqttClient() {
  String cid = settings.deviceID + "-" + String((uint32_t)millis(), HEX);
  mqttClient.setId(cid);

  mqttClient.setUsernamePassword(settings.tbTokenString.c_str(), "");
  mqttClient.setKeepAliveInterval(45 * 1000);

  static bool willSet = false;
  if (!willSet) {
    mqttClient.beginWill("v1/devices/me/attributes", false, 0);
    mqttClient.print("{\"status\":\"offline\"}");
    mqttClient.endWill();
    willSet = true;
  }
}


bool ensureMqttConnected(const char* host, int port) {
  if (mqttClient.connected()) return true;

  // Hard reset client/socket state (fast)
  mqttClient.stop();
  nbClient.stop();
  delay(200);

  setupMqttClient();  // re-apply clientId/token/will

  // Preflight TCP using the SAME nbClient MQTT uses
  Serial.print("[MQTT] TCP preflight (nbClient)... ");
  nbClient.stop();
  delay(50);

  if (!nbClient.connect(host, port)) {
    Serial.println("FAIL");
    nbClient.stop();
    return false;     // fail fast (caller will sleep)
  }
  Serial.println("OK");
  nbClient.stop();
  delay(80);

  // Optional: close a few stale sockets (helps after bad wakes)
  saraAT("AT+USOCL=0", 1200);
  saraAT("AT+USOCL=1", 1200);
  saraAT("AT+USOCL=2", 1200);
  saraAT("AT+USOCL=3", 1200);
  delay(30);

  Serial.print("[MQTT] Connecting to "); Serial.print(host); Serial.print(":"); Serial.println(port);

  // IMPORTANT for Option A:
  // Do NOT arm watchdog here. If connect hangs, WDT would reboot (which you don’t want).
  bool ok = mqttClient.connect(host, port);

  if (ok) {
    Serial.println("[MQTT] Connected.");
    unsigned long t0 = millis();
    while (millis() - t0 < 400) { mqttClient.poll(); delay(20); }
    return true;
  }

  Serial.print("[MQTT] connectError="); Serial.println(mqttClient.connectError());
  mqttClient.stop();
  nbClient.stop();
  delay(200);
  return false;
}







bool publishTelemetryJSON(const char* topic, const String& json) {
/*  if (!mqttConnectWithRetries(3)) {
 // if (!ensureMqttConnected(settings.tbHostString.c_str(), settings.tbPortInt)) {
    Serial.println(F("[PUB] MQTT not connected -> sleep and retry next wake"));
    writeSDcardLog("[PUB] MQTT not connected -> sleep");

//    modemPowerDownForSleep();
//    goToSleepFlag = true;
    return false;
  }
*/
  if (!mqttClient.connected()){
    mqttConnectOnce();
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



void scheduleNextTimeSync(bool success) {
  if (success) {
    // after success, relax retries (e.g., 6h; change to 24h if you prefer)
    timeSyncBackoffMs = 6UL * 60UL * 60UL * 1000UL;
  } else {
    // exponential backoff up to 6h
    if (timeSyncBackoffMs < 6UL * 60UL * 60UL * 1000UL)
      timeSyncBackoffMs *= 2;
  }
  // jitter ±20% to avoid “sync storms”
  uint32_t j = timeSyncBackoffMs / 5;
  nextTimeSyncMs = millis() + (timeSyncBackoffMs - j + (rand() % (2*j + 1)));
  Serial.print("[TIME] Next sync in ~");
  Serial.print(timeSyncBackoffMs / 60000);
  Serial.println(" min");
}

// Return true on success, sets TimeLib + RTCZero
bool syncTimeViaNTP_UDP(uint32_t overallMs = 15000) {
  Serial.println("[NTP] Start");
  NBUDP ntpUdp;  // fresh socket avoids stale state

  // patient UDP open on ephemeral port
  auto openUDP = [&](uint32_t totalMs)->bool {
    unsigned long t0 = millis();
    uint16_t attempt = 0;
    while (millis() - t0 < totalMs) {
      if (ntpUdp.begin(localPort)) return true;   // ephemeral local port
      ntpUdp.stop();
      delay(min<uint16_t>(150 + attempt*150, 800));
      attempt++;
    }
    return false;
  };

  if (!openUDP(8000)) {
    Serial.println("[NTP] UDP open failed");
    ntpUdp.stop();
    return false;
  }

  // NTP packet
  const int NTP_PACKET_SIZE = 48;
  static uint8_t pkt[NTP_PACKET_SIZE];
  memset(pkt, 0, NTP_PACKET_SIZE);
  pkt[0] = 0b11100011;   // LI, Version, Mode
  pkt[2] = 6;            // Polling
  pkt[3] = 0xEC;         // Precision
  pkt[12]=49; pkt[13]=0x4E; pkt[14]=49; pkt[15]=52;

  // DNS-free anycast/time hosts
  const IPAddress servers[] = {
    IPAddress(129, 6, 15, 28),   // time.nist.gov
    IPAddress(216, 239, 35, 0),  // time.google.com
    IPAddress(162, 159, 200, 1)  // time.cloudflare.com
  };

  auto tryOne = [&](const IPAddress& ip)->bool {
    if (!ntpUdp.beginPacket(ip, 123)) return false;
    ntpUdp.write(pkt, NTP_PACKET_SIZE);
    if (!ntpUdp.endPacket()) return false;

    unsigned long t0 = millis();
    while (millis() - t0 < 2500) {
      int sz = ntpUdp.parsePacket();
      if (sz >= NTP_PACKET_SIZE) {
        ntpUdp.read(pkt, NTP_PACKET_SIZE);
        uint32_t secs1900 =
          (uint32_t(pkt[40])<<24)|(uint32_t(pkt[41])<<16)|(uint32_t(pkt[42])<<8)|uint32_t(pkt[43]);
        const uint32_t seventyYears = 2208988800UL;
        if (secs1900 > seventyYears) {
          time_t epoch = secs1900 - seventyYears;
          setTime(epoch);
          rtc.setTime(hour(), minute(), second());
          rtc.setDate(day(), month(), (year()-2000));
          Serial.print("[NTP] OK epoch "); Serial.println(epoch);
          #if ENABLE_DEBUG
            Serial.print("[NTP] UTC: ");
            Serial.print(year());
            Serial.print("-");
            Serial.print(month());
            Serial.print("-");
            Serial.print(day());
            Serial.print(" ");
            Serial.print(hour());
            Serial.print(":");
            Serial.print(minute());
            Serial.print(":");
            Serial.println(second());
          #endif

          return true;
        }
      }
      delay(50);
    }
    return false;
  };

  unsigned long start = millis();
  bool ok = false;
  for (const auto& ip : servers) {
    if (millis() - start > overallMs) break;
    if (tryOne(ip)) { ok = true; break; }
  }
  ntpUdp.stop();
  if (!ok) Serial.println("[NTP] No reply");
  return ok;
}

// Return true on success, sets TimeLib + RTCZero
bool syncTimeViaHTTPDate_IP(uint32_t connectMs = 6000) {
  Serial.println("[HTTP-Date] Start");
  NBClient http;
  IPAddress hostIP(93, 184, 216, 34); // example.com

  if (!http.connect(hostIP, 80)) {
    Serial.println("[HTTP-Date] TCP connect failed");
    http.stop();
    return false;
  }

  http.print(
    "GET / HTTP/1.1\r\n"
    "Host: example.com\r\n"
    "Connection: close\r\n\r\n"
  );

  unsigned long t0 = millis();
  while (!http.available() && millis() - t0 < connectMs) delay(50);

  String line, dateLine;
  while (http.connected()) {
    int c = http.read();
    if (c < 0) { delay(5); continue; }
    if (c == '\r') continue;
    if (c == '\n') {
      if (line.length() == 0) break;          // end headers
      if (line.startsWith("Date:")) dateLine = line;
      line = "";
    } else line += char(c);
  }
  http.stop();

  if (dateLine.length() == 0) { Serial.println("[HTTP-Date] No Date header"); return false; }

  // "Date: Wed, 12 Nov 2025 15:22:12 GMT"
  int comma = dateLine.indexOf(',');
  int sp1 = dateLine.indexOf(' ', comma+1);
  if (comma < 0 || sp1 < 0) return false;
  String rest = dateLine.substring(sp1+1); rest.trim();  // "12 Nov 2025 15:22:12 GMT"

  int s1 = rest.indexOf(' ');
  int s2 = rest.indexOf(' ', s1+1);
  int s3 = rest.indexOf(' ', s2+1);
  if (s1<0 || s2<0 || s3<0) return false;

  int d = rest.substring(0, s1).toInt();
  String mon = rest.substring(s1+1, s2);
  int yy = rest.substring(s2+1, s3).toInt();
  String hms = rest.substring(s3+1);
  int spGMT = hms.indexOf(' '); if (spGMT > 0) hms = hms.substring(0, spGMT);
  int c1 = hms.indexOf(':'), c2 = hms.indexOf(':', c1+1);
  if (c1<0 || c2<0) return false;
  int hh = hms.substring(0, c1).toInt();
  int mm = hms.substring(c1+1, c2).toInt();
  int ss = hms.substring(c2+1).toInt();

  int monthNum =
    (mon=="Jan")?1:(mon=="Feb")?2:(mon=="Mar")?3:(mon=="Apr")?4:(mon=="May")?5:
    (mon=="Jun")?6:(mon=="Jul")?7:(mon=="Aug")?8:(mon=="Sep")?9:(mon=="Oct")?10:
    (mon=="Nov")?11:(mon=="Dec")?12:0;
  if (!monthNum) return false;

  tmElements_t tme;
  tme.Second = ss; tme.Minute = mm; tme.Hour = hh;
  tme.Day = d; tme.Month = monthNum; tme.Year = yy - 1970;
  time_t epoch = makeTime(tme);
  if (epoch < 1609459200UL) return false; // sanity >= 2021

  setTime(epoch);
  rtc.setTime(hour(), minute(), second());
  rtc.setDate(day(), month(), (year()-2000));
  Serial.print("[HTTP-Date] OK epoch "); Serial.println(epoch);
  #if ENABLE_DEBUG
  Serial.print("[HTTP] UTC: ");
  Serial.print(year());
  Serial.print("-");
  Serial.print(month());
  Serial.print("-");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.println(second());
#endif
  return true;
}

// Try once now (UDP first, then HTTP). Update flags & schedule next.
bool timeSyncOnce() {
  bool ok = syncTimeViaNTP_UDP(15000);
  if (!ok) ok = syncTimeViaHTTPDate_IP(6000);

  useServerTimestamp = !ok;
  scheduleNextTimeSync(ok);
  Serial.println(ok ? "[TIME] Sync OK (device timestamps enabled)"
                    : "[TIME] Sync FAILED (using TB server timestamps)");
  return ok;
}

// call periodically (e.g., in loop): only runs when due
void timeSyncTick() {
  if ((int32_t)(millis() - nextTimeSyncMs) >= 0) {
    timeSyncOnce();
  }
}

void bootSequence(){
  Serial.println("\n=== BOOT ===");
  nbAttachAPN_Flag = nbAttachAPN();
  if(nbAttachAPN_Flag){
    Serial.println(saraAT("AT+CGATT?", 2000));
    Serial.println(saraAT("AT+CGPADDR=1", 3000));
  }
  else{
    Serial.println("[BOOT] Attach failed");
    writeSDcardLog("[BOOT] Attach failed");
    return;
  }

  setupMqttClient();                       // your working helper
  mqttConnectFlag = ensureMqttConnected(settings.tbHostString.c_str(), settings.tbPortInt);
  if (!mqttConnectFlag) {
    // one short PDP refresh + single retry
//    gprs.detachGPRS(); delay(600);
    if (nbAttachAPN(20000, 20000)) {
      mqttConnectFlag = ensureMqttConnected(settings.tbHostString.c_str(), settings.tbPortInt);
    }
    if(!mqttConnectFlag){
      Serial.println("[MQQT] Connection failed");
      writeSDcardLog("[MQQT] Connection failed");
    }
  }

  mqttConnectFlag = ensureMqttConnected(settings.tbHostString.c_str(), settings.tbPortInt);
  if (!mqttConnectFlag) {
    Serial.println("[BOOT] MQTT failed -> sleeping and retry next wake");
    writeSDcardLog("[BOOT] MQTT failed -> sleep");

//    modemPowerDownForSleep();   // important: drop modem current
//    goToSleepFlag = true;       // your loop will deepSleep
    return;
  }

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
    Serial.print("Battery Volts = "); Serial.println(batteryVolts);
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
      payload += "\"device\":\""; payload += settings.deviceID; payload += "\",";
      payload += "\"batV\":";    payload += batteryVolts;              payload += ",";
      payload += "\"rlAve\":";   payload += riverLevelAveSendValue;    payload += ",";
      #if ENABLE_MAX_MIN_RL
      payload += "\"rlMax\":";   payload += riverLevelMaxSendValue;    payload += ",";
      payload += "\"rlMin\":";   payload += riverLevelMinSendValue;    payload += ",";
      #endif
      payload += "\"rainInt\":"; payload += rainIntervalSendValue;     payload += ",";
      payload += "\"rain24hr\":";payload += rain24hrSendValue;
      payload += "}}]";
    } else if (settings.sensorsMode == 1) {
      payload += "[{\"ts\":"; payload += tsSendValue; payload += "000";
      payload += ",\"values\":{";
      payload += "\"device\":\""; payload += settings.deviceID; payload += "\",";
      payload += "\"batV\":";    payload += batteryVolts;              payload += ",";
      payload += "\"rainInt\":";  payload += rainIntervalSendValue;    payload += ",";
      payload += "\"rain24hr\":"; payload += rain24hrSendValue;
      payload += "}}]";
    } else { // 2
      payload += "[{\"ts\":"; payload += tsSendValue; payload += "000";
      payload += ",\"values\":{";
      payload += "\"device\":\""; payload += settings.deviceID; payload += "\",";
      payload += "\"batV\":";    payload += batteryVolts;              payload += ",";
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
      payload += "\"device\":\""; payload += settings.deviceID; payload += "\",";
      payload += "\"batV\":";    payload += batteryVolts;              payload += ",";
      payload += "\"rlAve\":";   payload += riverLevelAveSendValue;    payload += ",";
      #if ENABLE_MAX_MIN_RL
      payload += "\"rlMax\":";   payload += riverLevelMaxSendValue;    payload += ",";
      payload += "\"rlMin\":";   payload += riverLevelMinSendValue;    payload += ",";
      #endif
      payload += "\"rainInt\":"; payload += rainIntervalSendValue;     payload += ",";
      payload += "\"rain24hr\":";payload += rain24hrSendValue;
      payload += "}";
    } else if (settings.sensorsMode == 1) {
      payload += "{";
      payload += "\"device\":\""; payload += settings.deviceID; payload += "\",";
      payload += "\"batV\":";    payload += batteryVolts;              payload += ",";
      payload += "\"rainInt\":";  payload += rainIntervalSendValue;    payload += ",";
      payload += "\"rain24hr\":"; payload += rain24hrSendValue;
      payload += "}";
    } else {
      payload += "{";
      payload += "\"device\":\""; payload += settings.deviceID; payload += "\",";
      payload += "\"batV\":";    payload += batteryVolts;              payload += ",";
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


///////////////////////////////////////////////////////////
//  SETUP
///////////////////////////////////////////////////////////
void setup() {
  //Pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  //  Put LED On to signal booting up.
  pinMode(GPSpowerPin, OUTPUT);
  digitalWrite(GPSpowerPin, LOW);
  // Rain input
  pinMode(PIN_RAIN, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), rainWakeISR, FALLING);
  
  analogReadResolution(12);
  analogReference(AR_DEFAULT);  // 3.3 V ref (default), explicit for clarity

  ledOn();

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  Serial.println("\n=== MIN STABLE BOOT ===");


  if(getSettings())  //  Read SlingShot Configs
  {
    Serial.println("Get Settings - Success");
    serialPrintSettings();  //  Print configs to screen
    ledBlink(1);   // SD OK
  }
  else
  {
    Serial.println("Get Settings - Failed");
    writeSDcardLog("[SD] Get Settings - Failed");
  }

  //Intialise limits from uploaded settings
  riverLevelMin = riverLevelRange_float;
  riverLevelMinSendValue = riverLevelRange_float;

  // RTC Setup
  rtc.begin(); // initialize RTC 24H format
  rtc.setAlarmSeconds(0);
  rtc.setAlarmSeconds(settings.rtcAlarmSecond);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(rtcWakeISR);

  stage(1);

  delay(3000);
  gpsUpdateFlag = getGPSinfo();

//  waitSimAndModemReady(15000);   // 15s on cold boot is reasonable
  Serial.println("[NET] nbAccess.begin()...");            //char : 0 if asynchronous. If synchronous, returns status : 0=ERROR, 1=IDLE, 2=CONNECTING, 3=NB_READY, 4=GPRS_READY, 5=TRANSPARENT_CONNECTED
  int bs = nbAccess.begin();
  Serial.print("[NET] nbAccess.begin() -> "); Serial.println(bs);


  if(bs == 0){  //Reboot and try again
    Watchdog.enable(1000);
  }


  // IMPORTANT: do NOT call nbAccess.begin() again per-APN
//  if (!bringUpNetworkStable()) {
//    Serial.println("[BOOT] Network failed");
//    // sleep path...
//  }

  // after PDP is up, before MQTT
  delay(5000);                   // let routing/NAT settle
  saraAT("AT+CGATT?", 2000);      // quick sanity
  saraAT("AT+CGPADDR=1", 3000);   // quick sanity

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
      stage(9);
//      modemPowerDown();
      ledOff();
//      LowPower.deepSleep(SLEEP_SECONDS * 1000);
    }
  }


//  if(nbAttachAPN_Flag && mqttConnectFlag && gpsUpdateFlag){
      #if ENABLE_DEBUG
        Serial.println("createAndSendGPSJsonMsg");
      #endif
    createAndSendGPSJsonMsg();  // Send initial Date/Time, Latitude and Longditude data from GPS to TB
//  }
//  else{
//    gpsRetryFlag = true;
//  }

/*
  String payload = "{\"hello\":\"world\",\"rssi\":\"" + scanner.getSignalStrength() + "\"}";
  Serial.print("[PUB] "); Serial.println(payload);

  if (mqttClient.beginMessage(settings.tbTokenString.c_str())) {
    mqttClient.print(payload);
    bool ok = mqttClient.endMessage();
    Serial.print("[PUB] endMessage -> "); Serial.println(ok ? "OK" : "FAIL");
    stage(ok ? 6 : 10);
  } else {
    Serial.println("[PUB] beginMessage failed");
  }
*/
  unsigned long t1 = millis();
  while (millis() - t1 < 800) { 
    wdtFeed();  // <<< optional
    mqttClient.poll(); delay(20); 
  }

//  modemPowerDown();
  ledOff();
//  Serial.println("[SLEEP] deepSleep");
  delay(50);
//  LowPower.deepSleep(SLEEP_SECONDS * 1000);
}

///////////////////////////////////////////////////////////
//  SETUP
///////////////////////////////////////////////////////////
/*void setup() {
//  Watchdog.disable();   // make sure a previous run didn't leave it ticking
  //Pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  //  Put LED On to signal booting up.
  pinMode(GPSpowerPin, OUTPUT);
  digitalWrite(GPSpowerPin, LOW);
  // Rain input
  pinMode(PIN_RAIN, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), rainWakeISR, FALLING);
  
  analogReadResolution(12);
  analogReference(AR_DEFAULT);  // 3.3 V ref (default), explicit for clarity

  //Serial Comms
  Serial.begin(115200);
  unsigned long t0 = millis(); while (!Serial && millis()-t0 < 4000) {}

  printResetCause();

  // Enable watchdog during boot sequences
//  int wdtMs = Watchdog.enable(WDT_TIMEOUT_MS);
//  Serial.print(F("[WDT] Boot watchdog enabled, timeout ~"));
//  Serial.print(wdtMs);
//  Serial.println(F(" ms"));

  if(getSettings())  //  Read SlingShot Configs
  {
    Serial.println("Get Settings - Success");
    serialPrintSettings();  //  Print configs to screen
    ledBlink(1);   // SD OK
  }
  else
  {
    Serial.println("Get Settings - Failed");
    writeSDcardLog("[SD] Get Settings - Failed");
  }

  //Intialise limits from uploaded settings
  riverLevelMin = riverLevelRange_float;
  riverLevelMinSendValue = riverLevelRange_float;

  // RTC Setup
  rtc.begin(); // initialize RTC 24H format
  rtc.setAlarmSeconds(0);
  rtc.setAlarmSeconds(settings.rtcAlarmSecond);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(rtcWakeISR);
  ledBlink(2);   // RTC OK

  // -------------------------------------------------
  // Fix #2: Run NetKick BEFORE bootSequence/MQTT so the
  // MKRNB library isn't fighting modem state changes.
  // -------------------------------------------------
  Serial.println("[NET] Powering modem (nbAccess.begin())...");
  int beginSt = nbAccess.begin();
  Serial.print("[NET] nbAccess.begin() -> ");
  Serial.println(beginSt);
  ledBlink(3);   // modem powered


  bool netOK = ensureNetworkReadyAnyLocation();
  if (!netOK) {
    Serial.println("[BOOT] Network attach failed (NetKick)");
  }
  if (netOK) ledBlink(4);  // registered
  else       ledBlink(8);  // network fail (one-time)


  bootSequence();

  delay(3000);
  gpsUpdateFlag = getGPSinfo();

  // Give the modem ~1s to flush
  unsigned long t1 = millis();
  while (millis() - t1 < 1000) { mqttClient.poll(); delay(20); }

  if(nbAttachAPN_Flag && mqttConnectFlag && gpsUpdateFlag){
      #if ENABLE_DEBUG
        Serial.println("createAndSendGPSJsonMsg");
      #endif
    createAndSendGPSJsonMsg();  // Send initial Date/Time, Latitude and Longditude data from GPS to TB
  }
  else{
    gpsRetryFlag = true;
  }

  // We’ll re-enable the watchdog inside loop() on each wake
  Watchdog.disable();
      #if ENABLE_DEBUG
        Serial.println("exitSetup");
      #endif
}
*/
///////////////////////////////////////////////////////////
//  START OF LOOP
///////////////////////////////////////////////////////////
void loop()
{
  // Enable watchdog while we are awake and doing work
//  Watchdog.enable(WDT_TIMEOUT_MS);
  mqttClient.poll(); // keepalive
//  Watchdog.reset();  // we’re alive here

  if(rtcWakeFlag){  //  RTC Alarm (1min)
    tsSendValue = rtc.getEpoch();  //  Time for JsonMsg if needed
    sampleTimeandDateFromRTC();
    rtcWakeFlag = false;
    #if ENABLE_DEBUG
      Serial.println("rtcWakeISR!");
    #endif
    //Read the LiPo battery volts level
    readBatteryVoltage();
    // Sample Sensors
    if(settings.sensorsMode == 0 || settings.sensorsMode == 2){  //River Level
      takeRiverLevelSamples(currentSampleNo);
    }
    if(settings.sensorsMode == 0 || settings.sensorsMode == 1){  //Rain
      takeRainSamples(currentSampleNo);
    }
    //See if 10minute period has arrived?
    int modTest = (sampleMinute+1)%settings.samplingInterval;  //  Need to add 1 as now sampling just before the minute change on 59secs
    if(modTest == 0){
      sendMsgFlag = true;
      #if ENABLE_DEBUG
        Serial.println("Modulus!");
      #endif
    }
    //Increment currentSampleNo and GPS update counter
    currentSampleNo++;  // Increment sample counter
    GPScounter++;

    // Update GPS if counter time is up!
    if((GPSupdateMins <= GPScounter) || gpsRetryFlag == true){
      gpsUpdateFlag = getGPSinfo();
      if(gpsUpdateFlag){
        createAndSendGPSJsonMsg();  // Send Date/Time, Latitude and Longditude data from GPS to TB
        gpsUpdateFlag = false;
        gpsRetryFlag = false;
      }
      GPScounter = 0;
    }

    resetAlarms();
    goToSleepFlag = true;
  //  Watchdog.reset();   // finished handling the minute tick
  }

  if(settings.sensorsMode == 0 || settings.sensorsMode == 1){  //Rain
    if(rainWakeFlag){  // Rain Sensor Alarm
      rainWakeFlag = false;
      #if ENABLE_DEBUG
        Serial.print("rainWakeISR!, ");
        Serial.print("rainCount: "); Serial.print(rainTipsCounter);
        Serial.print(", rainCount24hr: "); Serial.println(rain24hrTipsCounter);
      #endif
      resetAlarms();
      goToSleepFlag = true;
    }
  }

  if(sendMsgFlag){  //  Send Message
    sendMsgFlag = false;  //clear flag
    #if ENABLE_DEBUG
      Serial.println("sendMessage!");
    #endif 

    if (!nbAttachAPN_Flag){
      bootSequence(); //Try to connect again if boot failed
      //recoverNetworkAndMqtt();
    } 

    //  This is where we do the sample averaging and message creation!
    calcSamples(currentSampleNo);
    createAndSendJsonMsg(); 

    //SD Card
    currentSDfilename = createSDfilename();
    if (currentSDfilename != previousSDfilename){  //New file so new header needed
      if(!SD.exists(currentSDfilename)){
        Serial.println("[SD] New Header");
        writeSDcard(currentSDfilename, createSDcardPayloadHeader());  //  Write new header on new file only
      }
      previousSDfilename = currentSDfilename;  //  Update previous filename so we can move on...
    }
    writeSDcard(currentSDfilename, createSDcardPayload());  //  Write data

    // after publish + drain
//    if (!mqttClient.connected()) {
//      timeSyncTick();
//    }

  //  Watchdog.reset();   // we successfully made it through send & SD

    //if this is midnight we need to clear the 24hr counter!!!  But after the midnight sample and sendMsg has been done!
    if(sampleHour == 23 && sampleMinute == 59){
      rain24hrTipsCounter = 0;
    }
  }

  // Sleep!
  if(goToSleepFlag){
//    #if ENABLE_DEBUG
//      Serial.println("Sleep!");
//      delay(100);
//    #endif
//    goToSleepFlag = false;


    // Put peripherals into low-power before sleeping
//    Serial1.end();        // GPS UART

    // SD card: release bus & CS
//    SD.end();
//    pinMode(SDchipSelect, INPUT_PULLUP);

    // Make sure GPS is off (you already do it in getGPSinfo(), but belt & braces)
    digitalWrite(GPSpowerPin, LOW);
  //  Watchdog.reset();   // finished handling the minute tick  
    // Disable WDT while we’re sleeping for ~60s
  //  Watchdog.disable();

    delay(100);
  //  LowPower.idle();
    digitalWrite(LED_BUILTIN, LOW);  //  Turn LED off.
//    mqttClient.stop();
//    nbClient.stop();
    delay(100);
 //   modemPowerDownForSleep(true);
 //   LowPower.deepSleep();
  //  LowPower.sleep();
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
// ISR for rain tips (debounced)
void rainWakeISR() {
  if(settings.sensorsMode == 0 || settings.sensorsMode == 1){  //Rain
    uint32_t t = micros();
    rainWakeFlag = true;
    if (t - lastPulseUs > 10000) { // ~10 ms debounce
      rainTipsCounter++;
      rain24hrTipsCounter++;
      lastPulseUs = t;
    }
  }
}