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
//#include <Adafruit_ADS1X15.h>
#include <ArduinoMqttClient.h>

#include <ArduinoBearSSL.h>  // (not used for TLS here, just keep includes harmless)
#include <Client.h>

//////////////////////
//ERROR CHECKING
//////////////////////
enum class NetErr : int {
  OK = 0,
  SIM_NOT_READY,
  BAD_APN,
  NO_SIGNAL,
  NOT_REGISTERED,    // CEREG not 1 or 5
  PDP_FAIL,
  DNS_FAIL,
  UDP_OPEN_FAIL,
  UDP_SEND_FAIL,
  UDP_RECV_TIMEOUT,
  NTP_BAD_REPLY,
  MODEM_TIMEOUT,
  UNKNOWN
};

const char* netErrStr(NetErr e) {
  switch (e) {
    case NetErr::OK: return "OK";
    case NetErr::SIM_NOT_READY: return "SIM not ready";
    case NetErr::BAD_APN: return "APN/auth fail";
    case NetErr::NO_SIGNAL: return "No/low signal";
    case NetErr::NOT_REGISTERED: return "Not registered (CEREG)";
    case NetErr::PDP_FAIL: return "PDP activation failed";
    case NetErr::DNS_FAIL: return "DNS resolution failed";
    case NetErr::UDP_OPEN_FAIL: return "UDP open failed";
    case NetErr::UDP_SEND_FAIL: return "UDP send failed";
    case NetErr::UDP_RECV_TIMEOUT: return "UDP recv timeout";
    case NetErr::NTP_BAD_REPLY: return "NTP bad reply";
    case NetErr::MODEM_TIMEOUT: return "Modem timeout";
    default: return "Unknown";
  }
}

struct AttachResult {
  NetErr err;
  int csq;      // 0..31, 99 unknown
  int cereg;    // 0=not, 1=home, 5=roaming, 2=searching, 3=denied, 4=unknown
};



// --- forward declarations (add after your AttachResult struct) ---
void rtcWakeISR();
void rainWakeISR();

void getNTP();
void sendNTPpacket(IPAddress& address);
void showError(NetErr e);

// Ensure the compiler knows the signature and the AttachResult type here:
AttachResult nbAttach(const char* apn, const char* user, const char* pass, uint32_t deadlineMs);
NetErr syncTimeViaNTP(uint32_t totalDeadlineMs = 15000);

/************************************************************
 * Enables debug support. To disable this feature set to 0.
 ***********************************************************/
#define ENABLE_DEBUG                        1
#define ENABLE_MAX_MIN_RL                   0
// ===================== User config =====================
#define DEVICE_ID   "EML_HPT_001"
// Your APN for LTE-M
//#define APN         "gigsky-02"
const char* APN = "gigsky-02";
const char* APN_USER = "";
const char* APN_PASS = "";
//#define APN_USER    ""
//#define APN_PASS    ""
#define PINNUMBER   ""          // SIM PIN if any
#define VBAT_PIN A6
#define SENSORS_MODE 0  // 0 = Rain and River Level, 1 = Rain only, 2 = River Level only
#define RTC_ALARM_SECOND 58  //  The second the alarm is triggered
#define SAMPLING_INTERVAL 10  // SAMPLING_INTERVAL in minutes
uint16_t riverLevelRange = 600;  //  Currently Max range in cm
float rainGaugeCF = 0.200;  //  Rain gauge calibration factor

//  Deadline Constants in ms
const uint32_t ATTACH_DEADLINE_MS = 15000;
const uint32_t UDP_OPEN_DEADLINE_MS = 3000;
const uint32_t UDP_SEND_DEADLINE_MS = 4000;
const uint32_t UDP_RECV_TIMEOUT_MS  = 5000;
const uint32_t NTP_TOTAL_DEADLINE_MS = 10000;

// ---- ThingsBoard EU MQTT ----
const char TB_HOST[]   = "mqtt.eu.thingsboard.cloud"; // EU cluster
const int  TB_PORT    = 1883;
const char*   TB_TOPIC   = "v1/devices/me/telemetry";
const char*   TB_TOKEN   = "uo423fza9leqz4pry7cb"; // username for MQTT - Token on TB

// =======================================================
IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Initialise Library instances */
RTCZero rtc;
GPRS gprs;
NB nbAccess;        // controls modem and network registration
NBUDP udp;          // UDP socket
NBScanner scanner;  // for signal strength
NBClient nbClient;

MqttClient mqttClient(nbClient);

// Flags
bool NTP_updatedFlag = false;
volatile bool rtcWakeFlag = false;
volatile bool rainWakeFlag = false;
bool sendMsgFlag = false;
bool goToSleepFlag = false;

// Pins
const int PIN_RAIN = 7;  // contact-closure input (to GND)
const int PIN_RL_EN = 6;  // River Level Enable

// Globals
volatile uint32_t rainTipsCounter = 0;
volatile uint32_t rain24hrTipsCounter = 0;
volatile uint32_t lastPulseUs = 0;

const char* NTP_HOSTS[] = {
  "time.google.com",
  "pool.ntp.org",
  "time.cloudflare.com"
};
const IPAddress NTP_IP_FALLBACK(129, 6, 15, 28); // time.nist.gov


// minute samples
float batteryVolts = 0;
uint8_t sampleNo = 0;
uint16_t riverLevel = 0;
float currentRiverLevel = 0;
float riverLevelTotal = 0;
float riverLevelMax = 0;
float riverLevelMin = riverLevelRange;
uint8_t rainInterval = 0;
uint16_t rain24hr = 0; 
uint16_t currentSampleNo = 0;
uint32_t tsSendValue = 0;
float riverLevelAveSendValue = 0;
float riverLevelMaxSendValue = 0;    //  This equals the minimum river level in mm.
float riverLevelMinSendValue = riverLevelRange; //  This equals the maximum river level in mm.
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
    rtc.setAlarmSeconds(RTC_ALARM_SECOND);
    rtc.enableAlarm(rtc.MATCH_SS);
}

//////////////////////
//ERROR CHECKING - Back off helper
//////////////////////
uint32_t backoffMs(uint8_t attempt, uint32_t baseMs=1000, uint32_t capMs=30000) {
  uint32_t ms = baseMs << min<uint8_t>(attempt, 10); // cap growth
  if (ms > capMs) ms = capMs;
  // jitter ±20%
  uint32_t jitter = ms / 5;
  return ms - jitter + (rand() % (2*jitter + 1));
}

// ---- LED error blinker ----
void showError(NetErr e) {
  uint8_t blinks = 9; // default “unknown”
  switch (e) {
    case NetErr::SIM_NOT_READY:   blinks = 2; break;
    case NetErr::BAD_APN:         blinks = 3; break;
    case NetErr::NOT_REGISTERED:  blinks = 4; break;
    case NetErr::DNS_FAIL:        blinks = 5; break;
    case NetErr::UDP_OPEN_FAIL:   blinks = 6; break;
    case NetErr::UDP_RECV_TIMEOUT:
    case NetErr::NTP_BAD_REPLY:   blinks = 7; break;
    case NetErr::MODEM_TIMEOUT:   blinks = 8; break;
    default:                      blinks = 9; break;
  }

  for (uint8_t i = 0; i < blinks; ++i) {
    digitalWrite(LED_BUILTIN, HIGH); delay(200);
    digitalWrite(LED_BUILTIN, LOW);  delay(200);
  }
}


bool openUDPWithRetry(uint16_t port, uint32_t totalMs = 4000) {
  unsigned long t0 = millis();
  int attempt = 0;
  while (millis() - t0 < totalMs) {
    if (udp.begin(port)) return true;
    udp.stop();
    delay(200 + attempt * 100);
    attempt++;
  }
  // Ephemeral fallback
  if (udp.begin(0)) return true;
  udp.stop();
  return false;
}

bool waitForPDPReady(uint32_t deadlineMs = 8000) {
  unsigned long t0 = millis();
  while (millis() - t0 < deadlineMs) {
    IPAddress ip = gprs.getIPAddress();           // MKRNB API
    if (ip[0] != 0 || ip[1] != 0 || ip[2] != 0 || ip[3] != 0) {
      Serial.print("PDP IP: "); Serial.println(ip);
      delay(750);                                  // small settle
      return true;
    }
    delay(250);
  }
  return false;
}

bool isPDPUp() {
  IPAddress ip = gprs.getIPAddress();
  return (ip[0] | ip[1] | ip[2] | ip[3]) != 0;
}

// ---- Open UDP with patient retries & backoff (ephemeral port preferred) ----
bool openUDP_Patient(uint32_t totalMs = 12000) {
  // try ephemeral first (fewer conflicts on MKRNB)
  unsigned long start = millis();
  uint32_t attempt = 0;
  while (millis() - start < totalMs) {
    if (udp.begin(0)) return true;     // ephemeral local port
    udp.stop();
    uint32_t wait = 150 + (attempt * 150);
    if (wait > 800) wait = 800;
    delay(wait);
    attempt++;
  }
  return false;
}


///////////////////////////////////////////////////////////
//  SETUP
///////////////////////////////////////////////////////////
void setup()
{
  #if ENABLE_DEBUG
    Serial.begin(115200);
    delay(3000);
    Serial.println("EML NB Logger");
  #endif
  // Rain input
    pinMode(PIN_RAIN, INPUT_PULLUP);
    LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), rainWakeISR, FALLING);
    // connection state
  boolean connected = false;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(PIN_RL_EN, OUTPUT);
  digitalWrite(PIN_RL_EN, LOW);
 
  rtc.begin(); // initialize RTC 24H format
  rtc.setAlarmSeconds(RTC_ALARM_SECOND);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(rtcWakeISR);

  analogReadResolution(12);

// Modem/NB-IoT attach with observability
AttachResult ar = nbAttach(APN, APN_USER, APN_PASS, ATTACH_DEADLINE_MS);
if (ar.err != NetErr::OK) {
  Serial.print("Attach failed: "); Serial.println(netErrStr(ar.err));
  showError(ar.err);
  while (1) { delay(1000); } // stop; or implement a timed retry loop
}

Serial.print("Attached. CSQ="); Serial.print(ar.csq);
Serial.print(" CEREG="); Serial.println(ar.cereg);

Serial.println("\nStarting connection to server...");

IPAddress ip = gprs.getIPAddress();
Serial.print("PDP IP pre-NTP: "); Serial.println(ip);

NetErr ntpErr = syncTimeViaNTP(15000);
if (ntpErr != NetErr::OK) {
  Serial.print("NTP failed: "); Serial.println(netErrStr(ntpErr));
  showError(ntpErr);
  while (1) { delay(1000); } // or implement your backoff/retry loop
}
Serial.println("NTP sync OK");

// then do your mqttClient.connect(TB_HOST, TB_PORT) block…
mqttClient.setId(DEVICE_ID);
mqttClient.setUsernamePassword(TB_TOKEN, "");

Serial.print("Connecting MQTT ("); Serial.print(TB_HOST); Serial.print(":"); Serial.print(TB_PORT); Serial.println(")...");
if (!mqttClient.connect(TB_HOST, TB_PORT)) {
  int e = mqttClient.connectError();
  Serial.print(" failed, err="); Serial.println(e);
  switch (e) {
    case -1: Serial.println("MQTT: Timeout waiting for CONNACK"); break;
    case -2: Serial.println("MQTT: TCP connect failed (PDP/DNS/fw)"); break;
    case -3: Serial.println("MQTT: Protocol/broker rejected"); break;
    case -4: Serial.println("MQTT: Auth/token rejected"); break;
    default: Serial.println("MQTT: Unknown error"); break;
  }
  showError(NetErr::DNS_FAIL);
  while (1) { delay(1000); }
}
Serial.println("OK");



  delay(3000);
  rtcWakeFlag = false;  //  To avoid first false sample
  goToSleepFlag = true;
}

///////////////////////////////////////////////////////////
//  START OF LOOP
///////////////////////////////////////////////////////////
void loop()
{
  mqttClient.poll(); // keepalive
//takeRiverLevelSamples(currentSampleNo);  //RL TEST
//delay(1000);
  if(rtcWakeFlag){  //  RTC Alarm (1min)
  sampleTimeandDateFromRTC();
  //sampleTimeandDateFromRTC();
    rtcWakeFlag = false;
    #if ENABLE_DEBUG
      Serial.println("rtcWakeISR!");
    #endif
    //Read the LiPo battery volts level
    readBatteryVoltage();
    // Sample Sensors
    if(SENSORS_MODE == 0 || SENSORS_MODE == 2){  //River Level
      takeRiverLevelSamples(currentSampleNo);
    }
    if(SENSORS_MODE == 0 || SENSORS_MODE == 1){  //Rain
      takeRainSamples(currentSampleNo);
    }
    // Store/calculate sensor values (SD CARD?)
//    storeSamples(currentSampleNo);
    //See if 10minute period has arrived?
    int modTest = (rtc.getMinutes()+1)%SAMPLING_INTERVAL;  //  Need to add 1 as now sampling just before the minute change on 59secs
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

  if(SENSORS_MODE == 0 || SENSORS_MODE == 1){  //Rain
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
    tsSendValue = rtc.getEpoch();  //  Minus 1sec for 1sec consistant delay
    sendMsgFlag = false;  //clear flag
    #if ENABLE_DEBUG
      Serial.println("sendMessage!");
    #endif  

    //  This is where we do the sample averaging and message creation!
    calcSamples(currentSampleNo);
    createAndSendJsonMsg(); 

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
      //rtc.standbyMode();
      //LowPower.deepSleep();
    }


}
///////////////////////////////////////////////////////////
//  END OF LOOP
///////////////////////////////////////////////////////////

/****************************************************************/
//                        FUNCTIONS                             //
/****************************************************************/
//  Get sampling time from RTC
void sampleTimeandDateFromRTC(){
  sampleSecond = rtc.getSeconds();  
  sampleMinute = rtc.getMinutes();
  sampleHour = rtc.getHours();
  sampleYear = rtc.getYear();
  sampleMonth = rtc.getMonth();
  sampleDay = rtc.getDay();
}

// Sample Function

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
    Serial.print(" ");
    Serial.print(sampleHour); Serial.print(":"); Serial.print(sampleMinute); Serial.print(":"); Serial.print(sampleSecond); Serial.print(", ");
    Serial.print("Sample No = "); Serial.print(no_of_samples); Serial.print(", ");
    Serial.print("RiverLevel-Current = "); Serial.print(currentRiverLevel); Serial.print(", ");
    Serial.print("RiverLevel-Total = "); Serial.print(riverLevelTotal); 
    #if ENABLE_MAX_MIN_RL
      Serial.print(", ");
      Serial.print("RiverLevel-Max = "); Serial.print(riverLevelMax); Serial.print(", ");
      Serial.print("RiverLevel-Min = "); Serial.println(riverLevelMin);
    #endif
  #endif
}

void takeRainSamples(uint16_t no_of_samples) {

  #if ENABLE_DEBUG
    Serial.print(sampleYear+2000); Serial.print("-"); Serial.print(sampleMonth); Serial.print("-"); Serial.print(sampleDay);
    Serial.print(" ");
    Serial.print(sampleHour); Serial.print(":"); Serial.print(sampleMinute); Serial.print(":"); Serial.print(sampleSecond); Serial.print(", ");
    Serial.print("Sample No = "); Serial.print(no_of_samples); Serial.print(", ");
    Serial.print("rainInterval = "); Serial.print(rainTipsCounter); Serial.print(", ");
    Serial.print("rain24hr = "); Serial.println(rain24hrTipsCounter);
  #endif
}

// Process Sensors Function (inc Rain Reset if day change?)
void calcSamples(uint16_t no_of_samples){
Serial.println("calcSamples");
  if(SENSORS_MODE == 0 || SENSORS_MODE == 2){  //River Level
    riverLevelAveSendValue = riverLevelTotal/no_of_samples;  //  Calculate the average river level during the sampling period
    #if ENABLE_MAX_MIN_RL
      riverLevelMaxSendValue = riverLevelMax;
      riverLevelMinSendValue = riverLevelMin;
      riverLevelMax = 0;
      riverLevelMin = riverLevelRange;
    #endif
    riverLevelTotal = 0;
  }

  if(SENSORS_MODE == 0 || SENSORS_MODE == 1){  //Rain
    rainIntervalSendValue = rainTipsCounter * rainGaugeCF;
    rain24hrSendValue = rain24hrTipsCounter * rainGaugeCF;
    rainTipsCounter = 0;
  }
  currentSampleNo = 0;
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
void createAndSendJsonMsg(){
  String payload;
  payload.reserve(2048);
Serial.println("createAndSendJsonMsg");
  if(SENSORS_MODE == 0){  //Rain and River Level
    payload += "[";
    payload += "{\"ts\":";
    payload += tsSendValue;   // uint32_t seconds
    payload += "000";       // ms
    payload += ",\"values\":";
    payload += "{";
    payload += "\"device\":\"";
    payload += DEVICE_ID;
    payload += "\",";
    payload += "\"batV\":";
    payload += batteryVolts;
    payload += ",\"rlAve\":";
    payload += riverLevelAveSendValue;
    #if ENABLE_MAX_MIN_RL
    payload += ",\"rlMax\":";
      payload += riverLevelMaxSendValue;
      payload += ",\"rlMin\":";
      payload += riverLevelMinSendValue;
    #endif
    payload += ",\"rainInt\":";
    payload += rainIntervalSendValue;
    payload += ",\"rain24hr\":";
    payload += rain24hrSendValue;
    payload += "}}";
    payload += "]";
  }
  if(SENSORS_MODE == 1){  //Rain only
    payload += "[";
    payload += "{\"ts\":";
    payload += tsSendValue;   // uint32_t seconds
    payload += "000";       // ms
    payload += ",\"values\":";
    payload += "{";
    payload += "\"device\":\"";
    payload += DEVICE_ID;
    payload += "\",";
    payload += "\"rainInt\":";
    payload += rainIntervalSendValue;
    payload += ",\"rain24hr\":";
    payload += rain24hrSendValue;
    payload += "}}";
    payload += "]";
  }
  if(SENSORS_MODE == 2){  //River Level only
    payload += "[";
    payload += "{\"ts\":";
    payload += tsSendValue;   // uint32_t seconds
    payload += "000";       // ms
    payload += ",\"values\":";
    payload += "{";
    payload += "\"device\":\"";
    payload += DEVICE_ID;
    payload += "\",";
    payload += "\"riverLevelAve\":";
    payload += riverLevelAveSendValue;
    #if ENABLE_MAX_MIN_RL
      payload += ",\"riverLevelMax\":";
      payload += riverLevelMaxSendValue;
      payload += ",\"riverLevelMin\":";
      payload += riverLevelMinSendValue;
    #endif
    payload += "}}";
    payload += "]";
  }

Serial.println("mqttClient.beginMessage");
  mqttClient.beginMessage(TB_TOPIC);
  mqttClient.print(payload);
  mqttClient.endMessage();

  #if ENABLE_DEBUG
    Serial.print("Sent: "); Serial.println(payload);
  #endif
}


///////////////////////////////////////////////////////////
//  TIME Functions
///////////////////////////////////////////////////////////
void getNTP()
{
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  if ( udp.parsePacket() ) {
    #if ENABLE_DEBUG
      Serial.println("NTP packet received");
    #endif
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    // now convert NTP time into everyday time:
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;

    setTime(epoch);   // load epoch into time library
    rtc.setTime(hour(), minute(), second());
    rtc.setDate(day(), month(), (year()-2000));

    #if ENABLE_DEBUG
      Serial.print("UTC: ");
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
    NTP_updatedFlag = true;  // Time has been set
  }

}
bool openUDPForNTP(uint16_t preferredPort = 2390, uint32_t totalMs = 6000) {
  // try the preferred port for ~3s
  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    if (udp.begin(preferredPort)) return true;
    udp.stop();
    delay(250);
  }
  // fallback to ephemeral port for ~3s
  t0 = millis();
  while (millis() - t0 < 3000) {
    if (udp.begin(0)) return true;
    udp.stop();
    delay(250);
  }
  return false;
}


NetErr syncTimeViaNTP(uint32_t deadlineMs) {
  Serial.println("[NTP] Starting time sync");

  NBUDP ntpUdp;  // ← local, fresh UDP socket instance every time

  auto openUDP_PatientLocal = [&](uint32_t totalMs) -> bool {
    unsigned long start = millis();
    uint32_t attempt = 0;
    while (millis() - start < totalMs) {
      if (ntpUdp.begin(0)) return true;      // ephemeral local port
      ntpUdp.stop();
      uint32_t wait = 150 + attempt * 150;
      if (wait > 800) wait = 800;
      delay(wait);
      attempt++;
    }
    return false;
  };

  // Try to open UDP patiently
  if (!openUDP_PatientLocal(12000)) {
    Serial.println("[NTP] UDP open failed, retrying after PDP re-attach...");
    gprs.detachGPRS(); delay(600);
    (void)gprs.attachGPRS();
    // wait for IP again
    unsigned long t0 = millis();
    while (millis() - t0 < 6000) {
      IPAddress ip = gprs.getIPAddress();
      if ((ip[0] | ip[1] | ip[2] | ip[3]) != 0) break;
      delay(250);
    }
    if (!openUDP_PatientLocal(8000)) {
      Serial.println("[NTP] UDP still failed to open after re-attach.");
      ntpUdp.stop();
      return NetErr::UDP_OPEN_FAIL;
    }
  }

  // Build the NTP request
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12] = 49; packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49; packetBuffer[15] = 52;

  // Use stable NTP IPs (no DNS)
  const IPAddress servers[] = {
    IPAddress(129, 6, 15, 28),   // time.nist.gov
    IPAddress(162, 159, 200, 1), // time.cloudflare.com anycast
    IPAddress(216, 239, 35, 0)   // time.google.com anycast
  };

  auto tryOne = [&](const IPAddress& ip) -> NetErr {
    if (!ntpUdp.beginPacket(ip, 123)) return NetErr::UDP_SEND_FAIL;
    ntpUdp.write(packetBuffer, NTP_PACKET_SIZE);
    if (!ntpUdp.endPacket()) return NetErr::UDP_SEND_FAIL;

    unsigned long t0 = millis();
    while (millis() - t0 < 2500) {
      int sz = ntpUdp.parsePacket();
      if (sz >= NTP_PACKET_SIZE) {
        ntpUdp.read(packetBuffer, NTP_PACKET_SIZE);
        uint32_t secs1900 = ((uint32_t)packetBuffer[40] << 24) |
                            ((uint32_t)packetBuffer[41] << 16) |
                            ((uint32_t)packetBuffer[42] << 8)  |
                             (uint32_t)packetBuffer[43];
        const uint32_t seventyYears = 2208988800UL;
        if (secs1900 > seventyYears) {
          uint32_t epoch = secs1900 - seventyYears;
          setTime(epoch);
          rtc.setTime(hour(), minute(), second());
          rtc.setDate(day(), month(), (year() - 2000));
          NTP_updatedFlag = true;
          return NetErr::OK;
        }
        return NetErr::NTP_BAD_REPLY;
      }
      delay(50);
    }
    return NetErr::UDP_RECV_TIMEOUT;
  };

  // Try each server within overall deadline
  unsigned long startOverall = millis();
  for (const auto& ip : servers) {
    if (millis() - startOverall > deadlineMs) break;
    NetErr e = tryOne(ip);
    if (e == NetErr::OK) { ntpUdp.stop(); return NetErr::OK; }
  }

  ntpUdp.stop();
  return NetErr::MODEM_TIMEOUT;
}




void sendNTPpacket(IPAddress& address)
{
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  //Serial.println("4");
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  //Serial.println("5");
  udp.endPacket();
  //Serial.println("6");
}


// HTTP Date fallback over plain TCP, DNS-free (connect by IP)
NetErr syncTimeViaHTTPDate_IP() {
  NBClient http;  // plain TCP (no TLS)
  IPAddress hostIP(93, 184, 216, 34);  // example.com (static, DNS-free)
  const char* hostName = "example.com"; // for Host: header

  Serial.println("[HTTP-Date] Connecting to example.com (IP)");
  if (!http.connect(hostIP, 80)) {
    Serial.println("[HTTP-Date] TCP connect failed");
    return NetErr::MODEM_TIMEOUT;   // or NetErr::UDP_OPEN_FAIL if you prefer
  }

  // Minimal HTTP GET just to fetch the Date header
  http.print(
    "GET / HTTP/1.1\r\n"
    "Host: example.com\r\n"
    "Connection: close\r\n\r\n"
  );

  unsigned long t0 = millis();
  while (!http.available() && (millis() - t0 < 5000)) delay(50);

  String line, dateLine;
  while (http.connected()) {
    int c = http.read();
    if (c < 0) { delay(5); continue; }
    if (c == '\r') continue;
    if (c == '\n') {
      if (line.length() == 0) break;  // end of headers
      if (line.startsWith("Date:")) dateLine = line;
      line = "";
    } else {
      line += (char)c;
    }
  }
  http.stop();

  if (dateLine.length() == 0) return NetErr::NTP_BAD_REPLY;

  // Example: "Date: Wed, 06 Nov 2025 14:22:12 GMT"
  int comma = dateLine.indexOf(',');
  int firstSpace = dateLine.indexOf(' ', comma + 1);
  if (comma < 0 || firstSpace < 0) return NetErr::NTP_BAD_REPLY;

  String rest = dateLine.substring(firstSpace + 1); // "06 Nov 2025 14:22:12 GMT"
  rest.trim();
  int sp1 = rest.indexOf(' ');
  int sp2 = rest.indexOf(' ', sp1 + 1);
  int sp3 = rest.indexOf(' ', sp2 + 1);
  if (sp1 < 0 || sp2 < 0 || sp3 < 0) return NetErr::NTP_BAD_REPLY;

  String dd   = rest.substring(0, sp1);
  String mon  = rest.substring(sp1 + 1, sp2);
  String yyyy = rest.substring(sp2 + 1, sp3);
  String hms  = rest.substring(sp3 + 1); // "14:22:12 GMT"

  int spGMT = hms.indexOf(' ');
  if (spGMT > 0) hms = hms.substring(0, spGMT);

  int colon1 = hms.indexOf(':');
  int colon2 = hms.indexOf(':', colon1 + 1);
  if (colon1 < 0 || colon2 < 0) return NetErr::NTP_BAD_REPLY;

  int d  = dd.toInt();
  int yy = yyyy.toInt();
  int hh = hms.substring(0, colon1).toInt();
  int mm = hms.substring(colon1 + 1, colon2).toInt();
  int ss = hms.substring(colon2 + 1).toInt();

  // Month lookup
  int monthNum = 0;
  if      (mon == "Jan") monthNum = 1;
  else if (mon == "Feb") monthNum = 2;
  else if (mon == "Mar") monthNum = 3;
  else if (mon == "Apr") monthNum = 4;
  else if (mon == "May") monthNum = 5;
  else if (mon == "Jun") monthNum = 6;
  else if (mon == "Jul") monthNum = 7;
  else if (mon == "Aug") monthNum = 8;
  else if (mon == "Sep") monthNum = 9;
  else if (mon == "Oct") monthNum = 10;
  else if (mon == "Nov") monthNum = 11;
  else if (mon == "Dec") monthNum = 12;
  else return NetErr::NTP_BAD_REPLY;

  tmElements_t tme;
  tme.Second = ss;
  tme.Minute = mm;
  tme.Hour   = hh;
  tme.Day    = d;
  tme.Month  = monthNum;
  tme.Year   = yy - 1970;   // TimeLib expects years since 1970

  time_t epoch = makeTime(tme);
  if (epoch < 1609459200UL) return NetErr::NTP_BAD_REPLY; // sanity check (>= 2021)

  setTime(epoch);
  rtc.setTime(hour(), minute(), second());
  rtc.setDate(day(), month(), (year() - 2000));
  NTP_updatedFlag = true;

  Serial.print("[HTTP-Date] Time set from header: ");
  Serial.println(dateLine);

  return NetErr::OK;
}



// Modem/NB-IoT attach with observability
AttachResult nbAttach(const char* apn, const char* user, const char* pass,
                      uint32_t deadlineMs) {
  AttachResult r{NetErr::UNKNOWN, 99, 0};
  Serial.println("[nbAttach] entry");

  // 1) Bring up the modem & register (PIN ONLY on this core)
  Serial.println("[nbAttach] NB.begin(PINONLY) now");
  int st = nbAccess.begin(PINNUMBER);   // <<< KEY CHANGE: no APN args
  Serial.print("[nbAttach] NB.begin status="); Serial.println(st);
  if (st != NB_READY) {
    Serial.println("[nbAttach] NB.begin not ready -> NOT_REGISTERED/BAD_APN");
    r.err = NetErr::NOT_REGISTERED;   // or BAD_APN; library doesn’t distinguish
    return r;
  }
  r.cereg = 1;

  // 2) Nudge PDP, then poll for a real IP (no args on this core)
  Serial.println("[nbAttach] attachGPRS() nudge (no args)");
  (void)gprs.attachGPRS();   // harmless if already up

  Serial.println("[nbAttach] waiting for PDP IP...");
  unsigned long t0 = millis();
  IPAddress ip(0,0,0,0);
  while (millis() - t0 < deadlineMs) {
    ip = gprs.getIPAddress();
    if ((ip[0] | ip[1] | ip[2] | ip[3]) != 0) break;
    delay(250);
  }
  if ((ip[0] | ip[1] | ip[2] | ip[3]) == 0) {
    Serial.println("[nbAttach] PDP failed (no IP)");
    r.err = NetErr::PDP_FAIL;
    return r;
  }
  Serial.print("[nbAttach] PDP IP: "); Serial.println(ip);

  delay(750);               // small settle for sockets/DNS
  r.csq = 99;               // best-effort
  r.err = NetErr::OK;
  Serial.println("[nbAttach] done");
  return r;
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

// Interrupt Functions
void rtcWakeISR()
{
  rtcWakeFlag = true;
}

// Rain Interrupt function
// ISR for rain tips (debounced)
void rainWakeISR() {
  if(SENSORS_MODE == 0 || SENSORS_MODE == 1){  //Rain
    uint32_t t = micros();
    rainWakeFlag = true;
    if (t - lastPulseUs > 10000) { // ~10 ms debounce
      rainTipsCounter++;
      rain24hrTipsCounter++;
      lastPulseUs = t;
    }
  }
}