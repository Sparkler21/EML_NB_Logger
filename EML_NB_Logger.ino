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
/************************************************************
 * Enables debug support. To disable this feature set to 0.
 ***********************************************************/
#define ENABLE_DEBUG                       1
// ===================== User config =====================
#define DEVICE_ID   "EML_HPT_001"
// Your APN for LTE-M
#define APN         "gigsky-02"
#define APN_USER    ""
#define APN_PASS    ""
#define PINNUMBER   ""          // SIM PIN if any

#define SENSORS_MODE 0  // 0 = Rain and River Level, 1 = Rain only, 2 = River Level only
#define RTC_ALARM_SECOND 58  //  The second the alarm is triggered
#define SAMPLING_INTERVAL 10  // SAMPLING_INTERVAL in minutes
uint16_t riverLevelRange = 600;  //  Currently Max range in cm
float rainGaugeCF = 0.200;  //  Rain gauge calibration factor

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
NB nbAccess;
NBClient nbClient;
MqttClient mqttClient(nbClient);
// A UDP instance to let us send and receive packets over UDP
NBUDP Udp;
//Adafruit_ADS1115 ads;

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

// minute samples
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
///////////////////////////////////////////////////////////
//  SETUP
///////////////////////////////////////////////////////////
void setup()
{
  #if ENABLE_DEBUG
    Serial.begin(115200);
    delay(1000);
  #endif

  // Start peripherals
  //Wire.begin();
  //ads.begin(); 
  rtc.begin(); // initialize RTC 24H format

//  NEED AN RTC SAFETY NET HERE OR SOMEWHERE FOR FAILED NTP

  rtc.setAlarmSeconds(RTC_ALARM_SECOND);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(rtcWakeISR);

// Rain input
    pinMode(PIN_RAIN, INPUT_PULLUP);
    LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), rainWakeISR, FALLING);
    // connection state
  boolean connected = false;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(PIN_RL_EN, OUTPUT);
  digitalWrite(PIN_RL_EN, HIGH);

  // After starting the modem with NB.begin()
  // attach the shield to the GPRS network with the APN, login and password
  while (!connected) {
    if ((nbAccess.begin(PINNUMBER, APN) == NB_READY) &&
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
  if(NTP_updatedFlag == false){
    delay(10000);  //Wait 10seconds then try again...
    getNTP();
  }

  mqttClient.setId(DEVICE_ID);
  // Auth: token as username, empty password
  mqttClient.setUsernamePassword(TB_TOKEN, "");

  #if ENABLE_DEBUG
    Serial.print("Connecting MQTT...");
  #endif
  if (!mqttClient.connect(TB_HOST, TB_PORT)) {
  #if ENABLE_DEBUG
    Serial.print(" failed, err=");
    Serial.println(mqttClient.connectError());
  #endif
    while (1);
  }
  #if ENABLE_DEBUG
    Serial.println("OK");
  #endif
  flashLED(5, 100);
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
    //LowPower.deepSleep();
    }


}
///////////////////////////////////////////////////////////
//  END OF LOOP
///////////////////////////////////////////////////////////

/****************************************************************/
//                        FUNCTIONS                             //
/****************************************************************/
// Modem Connect Function

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
void takeRiverLevelSamples(uint16_t no_of_samples) {
  // Read River Level ADC(0)
  uint32_t adc = 0;
  const int N = 10;
  
  //POWER UP SENSOR
  //SMALL DELAY of MORE THAN 20uS
  for (int i = 0; i < N; i++) {
    //adc = ads.readADC_SingleEnded(0);
    adc = adc + analogRead(2);
    delay(50); // small delay between samples
  }
  //SHUT DOWN SENSOR

  currentRiverLevel = adc/N;  //  Need multipliers and offsets?
  currentRiverLevel = (currentRiverLevel/1024) * riverLevelRange;
  adc = 0;  // reset
  riverLevelTotal = riverLevelTotal + currentRiverLevel;  //  Totalise the river level samples
  if(riverLevelMax < currentRiverLevel){
    riverLevelMax = currentRiverLevel;  //  New Max in the samples
  }
  if(riverLevelMin > currentRiverLevel){
    riverLevelMin = currentRiverLevel;  //  New Min in the samples
  }

  #if ENABLE_DEBUG
    Serial.print(sampleYear+2000); Serial.print("-"); Serial.print(sampleMonth); Serial.print("-"); Serial.print(sampleDay);
    Serial.print(" ");
    Serial.print(sampleHour); Serial.print(":"); Serial.print(sampleMinute); Serial.print(":"); Serial.print(sampleSecond); Serial.print(", ");
    Serial.print("Sample No = "); Serial.print(no_of_samples); Serial.print(", ");
    Serial.print("RiverLevel-Current = "); Serial.print(currentRiverLevel); Serial.print(", ");
    Serial.print("RiverLevel-Total = "); Serial.print(riverLevelTotal); Serial.print(", ");
    Serial.print("RiverLevel-Max = "); Serial.print(riverLevelMax); Serial.print(", ");
    Serial.print("RiverLevel-Min = "); Serial.println(riverLevelMin);
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

  if(SENSORS_MODE == 0 || SENSORS_MODE == 2){  //River Level
    riverLevelAveSendValue = riverLevelTotal/no_of_samples;  //  Calculate the average river level during the sampling period
    riverLevelMaxSendValue = riverLevelMax;
    riverLevelMinSendValue = riverLevelMin;
    riverLevelTotal = 0;
    riverLevelMax = 0;
    riverLevelMin = riverLevelRange;
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
    payload += "\"riverLevelAve\":";
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
    payload += ",\"riverLevelMax\":";
    payload += riverLevelMaxSendValue;
    payload += ",\"riverLevelMin\":";
    payload += riverLevelMinSendValue;
    payload += "}}";
    payload += "]";
  }
  mqttClient.beginMessage(TB_TOPIC);
  mqttClient.print(payload);
  mqttClient.endMessage();

  #if ENABLE_DEBUG
    Serial.print("Sent: "); Serial.println(payload);
    flashLED(1, 2000);
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
      Serial.println("NTP packet received");
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
    NTP_updatedFlag = true;  // Time has been set
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