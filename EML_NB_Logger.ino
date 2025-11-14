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
/************************************************************
 * Enables debug support. To disable this feature set to 0.
 ***********************************************************/
#define ENABLE_DEBUG          1
#define VBAT_PIN              A6
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
MqttClient mqttClient(nbClient);
NBUDP Udp;

File    configFile;

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
// Flags
bool NTP_updatedFlag = false;
volatile bool rtcWakeFlag = false;
volatile bool rainWakeFlag = false;
bool sendMsgFlag = false;
bool goToSleepFlag = false;
bool nbAttachAPN_Flag = false;
bool mqttConnectFlag = false;
bool ntpUpdateFlag = false;
// --- Time sync state ---
bool useServerTimestamp = true;                 // start with TB server time until we sync
uint32_t nextTimeSyncMs = 0;                    // millis() when we may try again
uint32_t timeSyncBackoffMs = 5UL*60UL*1000UL;   // start with 5 minutes
// Pins
const int PIN_RAIN = 7;  // contact-closure input (to GND)
const int PIN_RL_EN = 6;  // River Level Enable
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

///////////////////////////////////////////////////////////
//  Helper Functions
///////////////////////////////////////////////////////////
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
  dataStringLog += "\n"; 
  writeSDcard(logFile, dataStringLog);
}

bool nbAttachAPN(uint32_t beginDeadlineMs = 5000, uint32_t regDeadlineMs = 45000, uint32_t pdpDeadlineMs = 30000) {
  int st = 0;
  unsigned long t0 = millis();
  Serial.println("[NET] NB.begin(PIN+APN)...");
  while(st != 3  || (millis() - t0 < beginDeadlineMs)){
    st = nbAccess.begin(settings.pin.c_str(), settings.apn.c_str(), settings.apnUser.c_str(), settings.apnPass.c_str());
  }
  if(st == 3){
    Serial.print("[NET] NB.begin(APN) -> "); Serial.println(st);
  }
  else{
    Serial.print("[NET] NB.begin(APN) FAIL -> "); Serial.println(st);
    return false;
  }

  // --- Wait for network registration (CEREG 1=home, 5=roaming) ---
  t0 = millis();
  int lastReg = -1;
  while (millis() - t0 < regDeadlineMs) {
    int reg = nbAccess.isAccessAlive();   // 0 not, 1 home, 2 searching, 3 denied, 4 unknown, 5 roaming
    if (reg != lastReg) {
      Serial.print("[NET] CEREG: "); Serial.println(reg);
      lastReg = reg;
    }
    if (reg == 1 || reg == 5) break;           // registered
    if (reg == 3) {                             // registration denied
      Serial.println("[NET] Registration denied (CEREG=3) — check SIM/profile/PLMN");
      return false;
    }
    delay(500);
  }
  if (!(lastReg == 1 || lastReg == 5)) {
    Serial.println("[NET] Registration timeout — no network attach");
    return false;
  }

  // Optional: show signal once
  while (scanner.getSignalStrength() == "99"){
    //  No signal - Stay in loop!!!  TIMEOUT NEEDED??????
  }
  Serial.print("[NET] CSQ: "); Serial.println(scanner.getSignalStrength());
  // --- Bring up PDP (get IP) with timed poll ---
  Serial.println("[NET] Starting PDP bring-up...");
  t0 = millis();
  while (millis() - t0 < pdpDeadlineMs) {
    (void)gprs.attachGPRS();                 // quick nudge; harmless if already up
    IPAddress ip = gprs.getIPAddress();
    Serial.print("[NET] IP poll: "); Serial.println(ip);
    if ((ip[0] | ip[1] | ip[2] | ip[3]) != 0) {
      delay(750);                             // settle sockets a bit
      Serial.print("[NET] PDP IP: "); Serial.println(ip);
      return true;
    }
    delay(500);
  }
  // One clean refresh if PDP didn’t appear
  Serial.println("[NET] PDP timed out; refreshing modem once...");
  nbAccess.shutdown(); delay(1500);
  st = nbAccess.begin(settings.pin.c_str(), settings.apn.c_str(), settings.apnUser.c_str(), settings.apnPass.c_str());
  Serial.print("[NET] NB.begin(APN) -> "); Serial.println(st);

  // Re-check registration briefly
  t0 = millis();
  while (millis() - t0 < 10000) {
    int reg = nbAccess.isAccessAlive();
    if (reg == 1 || reg == 5) break;
    if (reg == 3) { Serial.println("[NET] Registration denied after refresh"); return false; }
    delay(500);
  }

  // Try PDP again (short window)
  t0 = millis();
  while (millis() - t0 < 15000) {
    (void)gprs.attachGPRS();
    IPAddress ip = gprs.getIPAddress();
    Serial.print("[NET] IP poll (retry): "); Serial.println(ip);
    if ((ip[0] | ip[1] | ip[2] | ip[3]) != 0) {
      delay(750);
      Serial.print("[NET] PDP IP (retry): "); Serial.println(ip);
      return true;
    }
    delay(500);
  }

  Serial.println("[NET] PDP attach failed after refresh");
  return false;
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
  mqttClient.setId(settings.deviceID);                 // non-empty clientId (yours)
  mqttClient.setUsernamePassword(settings.tbTokenString.c_str(), ""); // TB token as username, blank password
  mqttClient.setKeepAliveInterval(45 * 1000);   // NB/LTE-M friendly
  mqttClient.beginWill("v1/devices/me/attributes", false, 0);
  mqttClient.print("{\"status\":\"offline\"}");
  mqttClient.endWill();
}

// Try connect if needed (no infinite wait)
bool ensureMqttConnected(const char* host, int port) {
  if (mqttClient.connected()) return true;
  Serial.print("[MQTT] Connecting to "); Serial.print(host); Serial.print(":"); Serial.println(port);
  if (!mqttClient.connect(host, port)) {
    Serial.print("[MQTT] connectError="); Serial.println(mqttClient.connectError());
    return false;
  }
  Serial.println("[MQTT] Connected.");
  return true;
}

// Publish JSON and give the modem time to flush
bool publishTelemetryJSON(const char* topic, const String& json) {
  if (!ensureMqttConnected(settings.tbHostString.c_str(), settings.tbPortInt)) return false;
  if (!mqttClient.beginMessage(topic)) {
    Serial.println("[PUB] beginMessage() failed");
    return false;
  }
  mqttClient.print(json);
  bool ok = mqttClient.endMessage();
  Serial.print("[PUB] endMessage() -> "); Serial.println(ok ? "OK" : "FAIL");
  unsigned long t0 = millis();
  while (millis() - t0 < 800) { mqttClient.poll(); delay(20); }  // drain ~0.8s
  return ok;
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

void readBatteryVoltage() {
  int raw = analogRead(VBAT_PIN);   // 0–4095 for 12-bit ADC
  batteryVolts = (raw / 4095.0) * 3.3 * 2; // *2 because of onboard divider
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

  if (!useServerTimestamp) {
    // use device timestamp (array with ts + values)
    uint32_t ts = rtc.getEpoch();     // seconds since 1970
    if (settings.sensorsMode == 0) {
      payload += "[{\"ts\":"; payload += ts; payload += "000";
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
      payload += "[{\"ts\":"; payload += ts; payload += "000";
      payload += ",\"values\":{";
      payload += "\"device\":\""; payload += settings.deviceID; payload += "\",";
      payload += "\"rainInt\":";  payload += rainIntervalSendValue;    payload += ",";
      payload += "\"rain24hr\":"; payload += rain24hrSendValue;
      payload += "}}]";
    } else { // 2
      payload += "[{\"ts\":"; payload += ts; payload += "000";
      payload += ",\"values\":{";
      payload += "\"device\":\""; payload += settings.deviceID; payload += "\",";
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
      payload += "\"rainInt\":";  payload += rainIntervalSendValue;    payload += ",";
      payload += "\"rain24hr\":"; payload += rain24hrSendValue;
      payload += "}";
    } else {
      payload += "{";
      payload += "\"device\":\""; payload += settings.deviceID; payload += "\",";
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

void flashLED(uint8_t no_of_flashes, uint16_t delay_time)
{
  for (int i = 0; i < no_of_flashes; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delay_time);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delay_time);
  }
}

///////////////////////////////////////////////////////////
//  SETUP
///////////////////////////////////////////////////////////
void setup() {
  //Pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // Rain input
  pinMode(PIN_RAIN, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), rainWakeISR, FALLING);
  
  analogReadResolution(12);

  //Serial Comms
  Serial.begin(115200);
  unsigned long t0 = millis(); while (!Serial && millis()-t0 < 4000) {}

  if(getSettings())  //  Read SlingShot Configs
  {
    Serial.println("Get Settings - Success");
    serialPrintSettings();  //  Print configs to screen
  }
  else
  {
    Serial.println("Get Settings - Failed");
    writeSDcardLog("[SD] Get Settings - Failed");
    flashLED(5, 200);
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

  Serial.println("\n=== BOOT ===");
  nbAttachAPN_Flag = nbAttachAPN();
  if (!nbAttachAPN_Flag) {
    Serial.println("[BOOT] Attach failed — not proceeding.");
    writeSDcardLog("[BOOT] Attach failed — not proceeding");
    flashLED(5, 200);
    return;
  }

setupMqttClient();                       // your working helper
mqttConnectFlag = ensureMqttConnected(settings.tbHostString.c_str(), settings.tbPortInt);
if (!mqttConnectFlag) {
  // one short PDP refresh + single retry
  gprs.detachGPRS(); delay(600);
  if (nbAttachAPN(20000, 20000, 20000)) {
    mqttConnectFlag = ensureMqttConnected(settings.tbHostString.c_str(), settings.tbPortInt);
  }
  if(!mqttConnectFlag){
    Serial.println("[MQQT] Connection failed — not proceeding.");
    writeSDcardLog("[MQQT] Connection failed — not proceeding.");
    flashLED(5, 200);
  }
}

  ntpUpdateFlag = timeSyncOnce();

  if(ntpUpdateFlag == false){
    writeSDcardLog("[NTP] NTP Clock Update Failed");
    flashLED(10, 200);
  }
  if(nbAttachAPN_Flag == true && mqttConnectFlag == true && ntpUpdateFlag == true){
    Serial.println("[BOOT] Connected.");
    writeSDcardLog("[BOOT] Success");
    flashLED(1, 2000);
  }
  // Give the modem ~1s to flush
  unsigned long t1 = millis();
  while (millis() - t1 < 1000) { mqttClient.poll(); delay(20); }
  

}

///////////////////////////////////////////////////////////
//  START OF LOOP
///////////////////////////////////////////////////////////
void loop()
{
  mqttClient.poll(); // keepalive

  if(rtcWakeFlag){  //  RTC Alarm (1min)
    flashLED(2, 100);
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
    int modTest = (rtc.getMinutes()+1)%settings.samplingInterval;  //  Need to add 1 as now sampling just before the minute change on 59secs
    if(modTest == 0){
      sendMsgFlag = true;
      #if ENABLE_DEBUG
        Serial.println("Modulus!");
      #endif
    }
    //Increment currentSampleNo
    currentSampleNo++;  // Increment sample counter

    resetAlarms();
    goToSleepFlag = true;
  }

  if(settings.sensorsMode == 0 || settings.sensorsMode == 1){  //Rain
    if(rainWakeFlag){  // Rain Sensor Alarm
      rainWakeFlag = false;
      flashLED(1, 100);
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
    tsSendValue = rtc.getEpoch();  //  Time for JsonMsg
    sendMsgFlag = false;  //clear flag
    #if ENABLE_DEBUG
      Serial.println("sendMessage!");
    #endif  
    flashLED(3, 100);
    //  This is where we do the sample averaging and message creation!
    calcSamples(currentSampleNo);
    createAndSendJsonMsg(); 

    //SD Card
    currentSDfilename = createSDfilename();
    if (currentSDfilename != previousSDfilename){  //New file so new header needed
      writeSDcard(currentSDfilename, createSDcardPayloadHeader());  //  Write new header on new file
      previousSDfilename = currentSDfilename;  //  Update previous filename so we can move on...
    }
    writeSDcard(currentSDfilename, createSDcardPayload());  //  Write data

    // after publish + drain
    if (!mqttClient.connected()) {
      timeSyncTick();
    }

    //if this is midnight we need to clear the 24hr counter!!!  But after the midnight sample and sendMsg has been done!
    if(sampleHour == 23 && sampleMinute == 59){
      rain24hrTipsCounter = 0;
    }
  }

  // Sleep!
  if(goToSleepFlag){
    #if ENABLE_DEBUG
      Serial.println("Sleep!");
      delay(100);
    #endif
    goToSleepFlag = false;  
    LowPower.idle();
    //LowPower.deepSleep();
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