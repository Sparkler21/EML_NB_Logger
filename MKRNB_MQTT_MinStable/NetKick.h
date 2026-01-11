#ifndef NETKICK_H
#define NETKICK_H

#include <Arduino.h>

// ---------- SerialSARA AT helpers ----------
static inline void saraFlushInput() {
  while (SerialSARA.available()) SerialSARA.read();
}

static inline String saraReadUntilIdle(uint32_t timeoutMs) {
  String out;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (SerialSARA.available()) {
      out += (char)SerialSARA.read();
      start = millis();
    }
    delay(5);
  }
  return out;
}

static inline String saraAT(const char* cmd, uint32_t timeoutMs = 3000) {
  saraFlushInput();
  SerialSARA.print(cmd);
  SerialSARA.print("\r");
  return saraReadUntilIdle(timeoutMs);
}

static bool saraPing(uint32_t tries = 5) {
  for (uint32_t i = 0; i < tries; i++) {
    String r = saraAT("AT", 800);
    if (r.indexOf("OK") >= 0) return true;
    delay(200);
  }
  return false;
}

// ---------- Parse CEREG stat ----------
static inline int parseCeregStat(const String& resp) {
  int p = resp.indexOf("+CEREG:");
  if (p < 0) return -1;
  int comma1 = resp.indexOf(',', p);
  if (comma1 < 0) return -1;
  int i = comma1 + 1;
  while (i < (int)resp.length() && resp[i] == ' ') i++;
  int j = i;
  while (j < (int)resp.length() && isDigit(resp[j])) j++;
  if (j == i) return -1;
  return resp.substring(i, j).toInt();
}

static inline bool isRegisteredStat(int stat) { return stat == 1 || stat == 5; }

static inline bool alreadyRegisteredNow() {
  String r = saraAT("AT+CEREG?", 2000);
  int stat = parseCeregStat(r);
  Serial.print("[NET] ");
  Serial.println(r);
  return isRegisteredStat(stat);
}

static inline bool waitForRegistration(uint32_t totalWaitMs, uint32_t pollEveryMs = 3000) {
  uint32_t start = millis();
  while (millis() - start < totalWaitMs) {
    if (alreadyRegisteredNow()) return true;
    delay(pollEveryMs);
  }
  return false;
}

// ---------- Parse all candidates from COPS=? ----------
struct PlmnAct { String plmn; int act; };

static inline int parseAllCandidates(const String& scan, PlmnAct* out, int maxOut) {
  int count = 0;
  int i = 0;

  while (count < maxOut) {
    int q1 = scan.indexOf("\"", i); if (q1 < 0) break;
    int q2 = scan.indexOf("\"", q1 + 1); if (q2 < 0) break;
    int q3 = scan.indexOf("\"", q2 + 1); if (q3 < 0) break;
    int q4 = scan.indexOf("\"", q3 + 1); if (q4 < 0) break;
    int q5 = scan.indexOf("\"", q4 + 1); if (q5 < 0) break;
    int q6 = scan.indexOf("\"", q5 + 1); if (q6 < 0) break;

    String plmn = scan.substring(q5 + 1, q6);

    int commaAct = scan.indexOf(",", q6);
    if (commaAct < 0) { i = q6 + 1; continue; }

    int a = commaAct + 1;
    while (a < (int)scan.length() && scan[a] == ' ') a++;
    int b = a;
    while (b < (int)scan.length() && isDigit(scan[b])) b++;
    int act = scan.substring(a, b).toInt();

    if (plmn.length() >= 5 && (act == 7 || act == 9)) {
      out[count++] = { plmn, act };
    }

    i = b;
  }

  return count;
}

static inline bool tryCandidatesUntilRegistered(const String& scan) {
  PlmnAct cands[24];
  int n = parseAllCandidates(scan, cands, 24);
  if (n <= 0) return false;

  auto tryOne = [&](const String& plmn, int act) -> bool {
    String cmd = "AT+COPS=1,2,\"" + plmn + "\"," + String(act);
    Serial.print("[NET] Try force ");
    Serial.print(plmn);
    Serial.print(" act=");
    Serial.println(act);
    Serial.println(saraAT(cmd.c_str(), 8000));
    return waitForRegistration(60000, 4000);
  };

  for (int i = 0; i < n; i++) if (cands[i].act == 7) if (tryOne(cands[i].plmn, 7)) return true;
  for (int i = 0; i < n; i++) if (cands[i].act == 9) if (tryOne(cands[i].plmn, 9)) return true;

  return false;
}

// ---------- Main bootstrap ----------
static inline bool ensureNetworkReadyAnyLocation(uint32_t firstAutoWaitMs = 60000,
                                                 uint32_t secondAutoWaitMs = 60000) {
  Serial.println("[NET] RF on + basic config...");
  Serial.println(saraAT("AT+CMEE=2", 2000));
  Serial.println(saraAT("AT+CFUN=1", 8000));
  Serial.println(saraAT("AT+CEREG=2", 2000));

  Serial.println("[NET] AUTO select (COPS=0)...");
  Serial.println(saraAT("AT+COPS=0", 5000));

  Serial.println("[NET] Wait for registration (AUTO #1)...");
  if (waitForRegistration(firstAutoWaitMs)) return true;

  Serial.println("[NET] AUTO stalled. Generic kick: CFUN=0 -> CFUN=1 ...");
  Serial.println(saraAT("AT+CFUN=0", 8000));
  delay(1500);
  Serial.println(saraAT("AT+CFUN=1", 8000));
  delay(2000);
  Serial.println(saraAT("AT+COPS=0", 5000));

  Serial.println("[NET] Wait for registration (AUTO #2)...");
  if (waitForRegistration(secondAutoWaitMs)) return true;

  Serial.println("[NET] Still not registered. Scanning operators (COPS=?)...");
  String scan = saraAT("AT+COPS=?", 180000);
  Serial.println(scan);

  // If we registered during the scan, stop.
  if (alreadyRegisteredNow()) return true;

  Serial.println("[NET] Trying candidates from scan...");
  if (!tryCandidatesUntilRegistered(scan)) {
    Serial.println("[NET] Candidate force did not register.");
    Serial.println(saraAT("AT+COPS=0", 5000));
    return false;
  }

  Serial.println("[NET] Registered after candidate force. Returning to AUTO.");
  Serial.println(saraAT("AT+COPS=0", 5000));
  return true;
}

#endif // NETKICK_H
