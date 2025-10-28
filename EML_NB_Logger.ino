/*
  MKR NB 1500 river/rain logger → ThingsBoard (EU)
  - ADS1115 (AIN0) for river level
  - Rain gauge on D7 (contact to GND), debounced in ISR
  - Sample every 60 s, publish every 10 min
*/

#include <MKRNB.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>
#include <TimeLib.h>
#include <Adafruit_ADS1X15.h>

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

// =======================================================

IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Initialise Library instances */
RTCZero rtc;
GPRS gprs;
NB nbAccess;
// A UDP instance to let us send and receive packets over UDP
NBUDP Udp;
Adafruit_ADS1115 ads;

// Flags
volatile bool rtcWakeFlag = false;
volatile bool rainWakeFlag = false;
bool sendMsgFlag = false;

// Pins
const int PIN_RAIN = 7;  // contact-closure input (to GND)

// Globals
volatile uint32_t rainTipsCounter = 0;
volatile uint32_t rain24hrTipsCounter = 0;
volatile uint32_t lastPulseUs = 0;

// minute samples

uint8_t sampleNo = 0;
uint16_t riverLevel = 0;
uint16_t currentRiverLevel = 0;
uint16_t riverLevelTotal = 0;
uint8_t rainInterval = 0;
uint16_t rain24hr = 0; 

uint16_t currentSampleNo = 0;

uint32_t tsSendValue = 0;
uint16_t riverLevelAveSendValue = 0;
uint16_t riverLevelMaxSendValue = 0;    //  This equals the minimum river level in mm.
uint16_t riverLevelMinSendValue = 5000; //  This equals the maximum river level in mm.
uint16_t rainIntervalSendValue = 0;
uint16_t rain24hrSendValue = 0;

///////////////////////////////////////////////////////////
//  Helper Functions
///////////////////////////////////////////////////////////
void resetAlarms(){
    rtc.setAlarmSeconds(0);
    rtc.enableAlarm(rtc.MATCH_SS);
}
///////////////////////////////////////////////////////////
//  SETUP
///////////////////////////////////////////////////////////
void setup()
{
  #if ENABLE_DEBUG
    Serial.begin(115200);
  #endif

  // Start peripherals
  Wire.begin();
  ads.begin(); 
  rtc.begin(); // initialize RTC 24H format

//  NEED AN RTC SAFETY NET HERE OR SOMEWHERE FOR FAILED NTP

  rtc.setAlarmSeconds(0);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(rtcWakeISR);

// Rain input
  pinMode(PIN_RAIN, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), rainWakeISR, FALLING);

    // connection state
  boolean connected = false;

  // After starting the modem with NB.begin()
  // attach the shield to the GPRS network with the APN, login and password
  while (!connected) {
    if ((nbAccess.begin(PINNUMBER) == NB_READY) &&
        (gprs.attachGPRS() == GPRS_READY)) {
      connected = true;
    } else {
      #if ENABLE_DEBUG
        Serial.println("Not connected");
      #endif
      delay(1000);
    }
  }
  #if ENABLE_DEBUG
    Serial.println("\nStarting connection to server...");
  #endif
  Udp.begin(localPort);

  getNTP();

  delay(3000);
}

///////////////////////////////////////////////////////////
//  START OF LOOP
///////////////////////////////////////////////////////////
void loop()
{

  if(rtcWakeFlag){  //  RTC Alarm (1min)
    rtcWakeFlag = false;
    #if ENABLE_DEBUG
      Serial.println("rtcWakeISR!");
    #endif
    // Sample Sensors
    takeSamples(currentSampleNo);
    // Store/calculate sensor values (SD CARD?)
//    storeSamples(currentSampleNo);
    //See if 10minute period has arrived?
    int modTest = rtc.getMinutes()%10;
    if(modTest == 0){
      sendMsgFlag = true;
      #if ENABLE_DEBUG
        Serial.println("Modulus!");
    #endif
    }
    //Increment currentSampleNo
    currentSampleNo++;  // Increment sample counter
    resetAlarms();
  }

  if(rainWakeFlag){  // Rain Sensor Alarm
    rainWakeFlag = false;
    #if ENABLE_DEBUG
      Serial.println("rainWakeISR!");
      Serial.print("rainCount:     "); Serial.println(rainTipsCounter);
      Serial.print("rainCount24hr: "); Serial.println(rain24hrTipsCounter);
    #endif
    resetAlarms();
  }

  if(sendMsgFlag){  //  Send Message
    sendMsgFlag = false;  //clear flag
    #if ENABLE_DEBUG
      Serial.println("sendMessage!");
    #endif  
    rainTipsCounter = 0;
  }

    // Sleep!
    delay(1000);
    LowPower.idle();
    //LowPower.deepSleep();
}
///////////////////////////////////////////////////////////
//  END OF LOOP
///////////////////////////////////////////////////////////

/****************************************************************/
//                        FUNCTIONS                             //
/****************************************************************/
// Modem Connect Function

// Sample Function
void takeSamples(uint16_t no_of_samples) {
  // Read River Level ADC(0)
  uint16_t adc = ads.readADC_SingleEnded(0);
  currentRiverLevel = adc;  //  Need multipliers and offsets?

  riverLevelTotal = riverLevelTotal + currentRiverLevel;  //  Totalise the river level samples

  #if ENABLE_DEBUG
    Serial.print(rtc.getYear()+2000); Serial.print("-"); Serial.print(rtc.getMonth()); Serial.print("-"); Serial.print(rtc.getDay());
    Serial.print(" ");
    Serial.print(rtc.getHours()); Serial.print(":"); Serial.print(rtc.getMinutes()); Serial.print(":"); Serial.print(rtc.getSeconds()); Serial.print(", ");
    Serial.print("Sample No = "); Serial.print(no_of_samples); Serial.print(", ");
    Serial.print("currentRiverLevel = "); Serial.print(currentRiverLevel); Serial.print(", ");
    Serial.print("rainInterval = "); Serial.print(rainTipsCounter); Serial.print(", ");
    Serial.print("rain24hr = "); Serial.println(rain24hrTipsCounter);
  #endif
}

// Process Sensors Function (inc Rain Reset if day change?)
void calcLogSamples(uint16_t no_of_samples){

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


///////////////////////////////////////////////////////////
//  TIME Functions
///////////////////////////////////////////////////////////
void getNTP()
{
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  if ( Udp.parsePacket() ) {
    #if ENABLE_DEBUG
      Serial.println("packet received");
    #endif
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
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
  }
  // wait ten seconds before asking for the time again
//  delay(10000);
}

unsigned long sendNTPpacket(IPAddress& address)
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
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket();
  //Serial.println("6");
}

// Interrupt Functions
void rtcWakeISR()
{
  rtcWakeFlag = true;
}

// Rain Interrupt function
// ISR for rain tips (debounced)
void rainWakeISR() {
  uint32_t t = micros();
  rainWakeFlag = true;
  if (t - lastPulseUs > 10000) { // ~10 ms debounce
    rainTipsCounter++;
    rain24hrTipsCounter++;
    lastPulseUs = t;
  }
}