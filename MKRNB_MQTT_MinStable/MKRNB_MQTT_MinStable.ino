/*
MKR NB 1500 river/rain logger → ThingsBoard (EU)
Mark Dutton (EML)
21st January 2026

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
#include <ArduinoMqttClient.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>
#include <Adafruit_SleepyDog.h>
#include <TimeLib.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include "NetKick.h"
/************************************************************
 * Enables debug support. To disable this feature set to 0.
 ***********************************************************/
#define ENABLE_DEBUG          1
#define DEFAULT_INT_VALUE     0
#define DEFAULT_BOOL_VALUE    false
#define DEFAULT_STRING_VALUE  "default"
#define DEFAULT_BYTE_VALUE    0x00
// -------------------- Pins ------------------------
const int LED_Ext = 1;

// -------------------- Settings --------------------
static const char* TB_HOST  = "mqtt.eu.thingsboard.cloud";
static const int   TB_PORT  = 1883;
static const char* TB_TOPIC = "v1/devices/me/telemetry";
static const char* TB_TOKEN = "vxl9ZxrJ5J6soedld65D";
static const char* DEVICE_ID = "HPT02";

static const char* APNS[] = { "gigsky-02", "flolive.net", "ukapn" };
static const int APN_COUNT = sizeof(APNS) / sizeof(APNS[0]);

static const uint32_t REG_TIMEOUT_MS  = 45000;
static const uint32_t PDP_TIMEOUT_MS  = 30000;
static const uint32_t TCP_TIMEOUT_MS  = 8000;

static const uint32_t SLEEP_SECONDS = 60;

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

// -------------------- Initialise Library instances --------------------
NB     nbAccess;
GPRS   gprs;
NBScanner scanner;
NBClient nbClient;
NBClient probeClient; 
MqttClient mqttClient(nbClient);
RTCZero rtc;
NBUDP Udp;
TinyGPS gps;

static IPAddress tbIp;
static bool tbIpValid = false;

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
  String cmd = String("AT+UDNSRN=0,\"") + TB_HOST + "\"";
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
  if (tbIpValid) ok = probeClient.connect(tbIp, TB_PORT);
  else          ok = probeClient.connect(TB_HOST, TB_PORT);

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

  String cid = String(DEVICE_ID) + "-" + String((uint32_t)millis(), HEX);
  mqttClient.setId(cid);

  mqttClient.setUsernamePassword(TB_TOKEN, "");
  mqttClient.setKeepAliveInterval(45 * 1000);
}

static bool mqttConnectOnce() {
  mqttConfigure();
  Serial.print("[MQTT] Connecting to "); Serial.print(TB_HOST); Serial.print(":"); Serial.println(TB_PORT);

  mqttClient.stop();
  nbClient.stop();
  delay(1000);
  // light socket close attempt (ignore failures)
  saraAT("AT+USOCL=0", 2000);
  saraAT("AT+USOCL=1", 2000);
  delay(200);

  // If this *still* blocks, we’ll handle it next by guarding with WDT.
  bool ok = mqttClient.connect(TB_HOST, TB_PORT);

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
    bool ok = mqttClient.connect(TB_HOST, TB_PORT);

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

// -------------------- Setup --------------------
void setup() {
  pinMode(LED_Ext, OUTPUT);
  ledOn();

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  Serial.println("\n=== MIN STABLE BOOT ===");

  stage(1);

  rtc.begin();

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
      modemPowerDown();
      ledOff();
      LowPower.deepSleep(SLEEP_SECONDS * 1000);
    }
  }

  String payload = "{\"hello\":\"world\",\"rssi\":\"" + scanner.getSignalStrength() + "\"}";
  Serial.print("[PUB] "); Serial.println(payload);

  if (mqttClient.beginMessage(TB_TOPIC)) {
    mqttClient.print(payload);
    bool ok = mqttClient.endMessage();
    Serial.print("[PUB] endMessage -> "); Serial.println(ok ? "OK" : "FAIL");
    stage(ok ? 6 : 10);
  } else {
    Serial.println("[PUB] beginMessage failed");
  }

  unsigned long t1 = millis();
  while (millis() - t1 < 800) { 
    wdtFeed();  // <<< optional
    mqttClient.poll(); delay(20); 
  }

  modemPowerDown();
  ledOff();
  Serial.println("[SLEEP] deepSleep");
  delay(50);
  LowPower.deepSleep(SLEEP_SECONDS * 1000);
}

void loop() {}
