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
//#include "NetKick.h"
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

void connectNB(){
  int bs = 0;
  Serial.println("[NET] nbAccess.begin()...");
  while((bs = nbAccess.begin() != NB_READY) || 
      (gprs.attachGPRS() != GPRS_READY)){
    delay(1000);
  }
  Serial.print("[NET] nbAccess.begin() -> "); Serial.println(bs);
}

void connectMQTT(){
  while(mqttClient.connect(TB_HOST, TB_PORT)){
    mqttClient.connectError();
    delay(5000);
  }
}

void publishMessage(){
  mqttClient.beginMessage("Mark/Dutton");
  mqttClient.print("Hello world");
  mqttClient.endMessage();
  delay(1000);
}

// -------------------- Setup --------------------
void setup() {
  //Pins
  pinMode(LED_Ext, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
//  pinMode(GPSpowerPin, OUTPUT);
//  pinMode(PIN_RAIN, INPUT_PULLUP);  // Rain input
//  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_RAIN), rainWakeISR, FALLING);
//  digitalWrite(GPSpowerPin, LOW);  
  digitalWrite(LED_BUILTIN, LOW);

  //unused pins:
  pinMode(2, INPUT_PULLUP);  // 
  pinMode(6, INPUT_PULLUP);  // 

  analogReadResolution(12);
  analogReference(AR_DEFAULT);  // 3.3 V ref (default), explicit for clarity

//  ledOn();//  Put LED On to signal booting up.

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  Serial.println("\n=== MIN STABLE BOOT ===");

  rtc.begin();
/*
  delay(3000);

  int bs = 0;
  t0 = millis();

  do {
    Serial.println("[NET] nbAccess.begin()...");
    bs = nbAccess.begin();
    Serial.print("[NET] nbAccess.begin() -> "); Serial.println(bs);
    if (bs >= 2) break;
    delay(1500);
  } while (millis() - t0 < 30000);

  if (bs < 2) {
    Serial.println("[BOOT] nbAccess.begin failed after 30s; reboot");
    NVIC_SystemReset();   // cleaner than watchdog here
  }
*/
  mqttClient.setUsernamePassword(TB_TOKEN, "");
  mqttClient.setId(DEVICE_ID);


}

void loop() {

  if(nbAccess.status() != NB_READY || gprs.status() != GPRS_READY){
    connectNB();
  }

  if (mqttClient.connected()){
    connectMQTT();
  }

  mqttClient.poll();
  publishMessage();
  delay(100);
  nbAccess.shutdown();
  delay(100);

}

