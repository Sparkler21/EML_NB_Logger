#include <MKRNB.h>

const char PINNUMBER[] = "";
const char APN[]       = "gigsky-02";
const char USERNAME[]  = "";
const char PASSWORD[]  = "";

NB nbAccess;
NBClient c;

void test(const char* host, uint16_t port) {
  Serial.print("[TCP] "); Serial.print(host); Serial.print(":"); Serial.print(port); Serial.print(" -> ");
  if (c.connect(host, port)) { Serial.println("OK"); c.stop(); }
  else { Serial.println("FAIL"); }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("[NET] NB.begin...");
  int status = nbAccess.begin(PINNUMBER, APN, USERNAME, PASSWORD);
  Serial.print("[NET] status="); Serial.println(status);
  if (status != NB_READY) while (true);

  // IP test avoids DNS entirely
  test("1.1.1.1", 80);
  test("1.1.1.1", 443);

  // DNS + common ports
  test("example.com", 80);
  test("example.com", 443);

  // ThingsBoard
  test("mqtt.eu.thingsboard.cloud", 1883);
  test("mqtt.eu.thingsboard.cloud", 8883);
  test("mqtt.eu.thingsboard.cloud", 443);
}

void loop() {}
