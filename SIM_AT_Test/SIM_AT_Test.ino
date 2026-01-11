/*
  MKR NB 1500 – AT command diagnostics via SerialSARA
  Open Serial Monitor at 115200 baud.
*/

#include <Arduino.h>
#include <MKRNB.h>

NB nb;  // used to power/init the modem

static const unsigned long DEFAULT_TIMEOUT_MS = 3000;

void flushSaraInput() {
  while (SerialSARA.available()) {
    SerialSARA.read();
  }
}

void sendAT(const char *cmd, unsigned long timeoutMs = DEFAULT_TIMEOUT_MS) {
  Serial.print(">> ");
  Serial.println(cmd);

  flushSaraInput();

  // Send command + CR
  SerialSARA.print(cmd);
  SerialSARA.print("\r");

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (SerialSARA.available()) {
      char c = (char)SerialSARA.read();
      Serial.write(c);
      start = millis(); // extend while data still arriving
    }
  }

  Serial.println();
  Serial.println("--------------------------------------------------");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println();
  Serial.println("=== MKR NB 1500 AT Diagnostic (SerialSARA) ===");

  // Power on / init modem (library-dependent return type -> treat as int)
  int r = nb.begin();
  Serial.print("nb.begin() -> ");
  Serial.println(r);

  // Give the modem a moment to settle
  delay(1000);

  // Force full functionality / RF on
  sendAT("AT+CMEE=2");
  sendAT("AT+CFUN=1", 8000);
  sendAT("AT+CFUN?");
  delay(2000);

  // Basic / identification
  sendAT("AT");
  sendAT("ATE0");          // echo off
  sendAT("ATI");
  sendAT("AT+GMR");        // firmware rev (often supported)

  // SIM status
  sendAT("AT+CPIN?");
  sendAT("AT+CCID", 5000);
  sendAT("AT+CIMI", 5000);

  // Radio / network status
  sendAT("AT+CSQ");
  sendAT("AT+CEREG?");
  sendAT("AT+CGATT?");
  sendAT("AT+COPS?");

  // PDP contexts / APN stored in modem
  sendAT("AT+CGDCONT?");

  // Optional extras (support varies)
  sendAT("AT+CFUN?");
  sendAT("AT+CGSN");       // IMEI on many modems

  Serial.println("=== Done ===");
}

void loop() {
  // Interactive passthrough: type AT commands into Serial Monitor
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      sendAT(line.c_str(), 5000);
    }
  }
}
