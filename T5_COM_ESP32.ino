// ============================================================
// T5 CAN Interpreter (production) — Trionic 5 SRAM poller over
// MCP2515 CAN2 (615,384 bps) -> JSON snapshot over ESP-NOW.
//
// Hardened bring-up (fixes the cold-boot "garbage until power
// cycle" bug; validated on bench 2026-10-30 against a live T5):
//  1. initCan(): CNF writes are retried and READ-BACK verified
//     (CNF1/2/3 + CANSTAT normal mode). Retries every 2 s.
//  2. Nothing streams until Batt_volt reads plausibly
//     (10.0-18.9 V) on 3 consecutive polls. Implausible battery
//     re-closes the gate.
//  3. Every reply is range-checked per symbol; the ECU's
//     uninitialized-SRAM filler (zeros, 0xFF55) is rejected and
//     never enters the stream. No value is ever latched.
//  4. Stuck-RX detector: same nonzero reply to different symbols
//     3x in a row -> full MCP2515 reset. Identical all-zero
//     replies are legitimate ("no data yet") and ignored.
//  5. Watchdog: 20 consecutive rejects -> CAN re-init.
//  6. ESP-NOW / CAN bring-up failures are retried, never bypassed.
//  7. Bus diagnostics (EFLG/CANINTF/CANSTAT/CNF) printed at gate
//     transitions, rejects, and periodically while gated.
// ============================================================
#include <SPI.h>
#include <mcp_can.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>

#define CAN_CS_PIN 10
#define BOARD_LED 2  // ASL ESP32-CAN-X2 blue LED (GPIO2, active-low)

MCP_CAN CAN(CAN_CS_PIN);

// MCP2515 SPI commands and registers
#define MCP_WRITE   0x02
#define MCP_RESET   0xC0
#define MCP_READ    0x03
#define MCP_EFLG    0x2D
#define MCP_CNF1    0x2A
#define MCP_CNF2    0x29
#define MCP_CNF3    0x28
#define MCP_CANCTRL 0x0F
#define MCP_CANSTAT 0x0E
#define MCP_CANINTF 0x2C

// 615,384 bps @ 16 MHz crystal (ASL ESP32-CAN-X2 CAN2, confirmed on
// ASL wiki): BRP=1 -> TQ=125 ns, 13 TQ/bit (1+4+6+2), BTLMODE=1,
// sample point 84.6 %. NOTE: the "8 MHz" comment in the original
// sketch is wrong — these values are for the 16 MHz part.
#define CNF1_EXPECT 0x00
#define CNF2_EXPECT 0xAB
#define CNF3_EXPECT 0x01

//ASL Mac address 74:4D:BD:89:AB:98
//WS Display Mac address CC:BA:97:11:CB:28
uint8_t receiverMAC[]  = {0xCC, 0xBA, 0x97, 0x11, 0xCB, 0x28};  // 2.1" gauge (Waveshare)
uint8_t receiverMAC2[] = {0xCC, 0xBA, 0x97, 0x11, 0x0E, 0x28};  // 1.85" gauge (Waveshare)

// symbolMap: {label, symbolId, sram_addr, byte_length, scaler, adder,
//             pollInterval, minVal, maxVal}
struct SymbolMap {
  String label;
  String symbolId;
  int sramAddr;
  char byteLength;
  float scaler = 1.0;
  float adder = 0.0;
  unsigned long pollInterval = 1000;
  float minVal = -1e9f;   // plausibility range (post-scale units)
  float maxVal = 1e9f;
  unsigned long lastPolled = 0;
};

SymbolMap symbolsToPoll[] = {
  { "Batt",   "Batt_volt", 0x101B, 1, 0.1f,    0.0f,   250,  10.0f,  18.9f },
  { "RPM",    "Rpm",       0x1066, 2, 10.0f,   0.0f,    50,   0.0f, 7500.0f },
  // Final injector pw for FuelEdu (bus domain: ms — T5Suite CSV convention).
  // 0 is legitimate (decel cut); 130 ms is the ECU's own per-cycle pw clamp.
  { "InjPw",  "Insptid_ms10", 0x1062, 2, 0.1f,  0.0f,    50,   0.0f,  130.0f },
  { "Water",  "Kyl_temp",  0x102E, 1, 1.0f,    0.0f,  1000,   5.0f, 135.0f },
  { "Air",    "Lufttemp",  0x1033, 1, 1.0f,    0.0f,   250, -40.0f,  70.0f },
  { "NBo2",   "AD_sond",   0x1012, 1, 10.0f,   0.0f,   100,   0.0f, 200.0f },
  { "TPS",    "Medeltrot", 0x1038, 1, 0.3922f, 0.0f,    80,   0.0f, 101.0f },
  { "Boost",  "P_manifold",0x1013, 1, 1.0f,  -100.0f,   100, -100.0f, 300.0f },
  { "Timing", "Ign_angle", 0x10B4, 2, 0.1f,    0.0f,   100, -10.0f,  70.0f },
  { "Gear",   "Gear",      0x3355, 1, 1.0f,    0.0f,   250,   0.0f,   6.0f },
  { "Speed",  "Bil_hast",  0x101C, 1, 1.0f,    0.0f,   150,   0.0f, 255.0f },
  { "APC",    "PWM_ut10",  0x3359, 1, 10.0f,   0.0f,   100,   0.0f, 1100.0f },
  // TQ = torque-demand scalar (NOT Nm): 13/20/26/30 fixed floors, else
  // Last x 278/127 - 21, whole-word range 1..106 (wiki TQ page, 2026-11 RE).
  // 2-byte big-endian word @0x1074; plausibility window per measured range.
  { "Torque", "TQ",        0x1074, 2, 1.0f,    0.0f,   200,   1.0f,  110.0f },
  { "Load",   "Medellast", 0x1037, 1, 1.0f,    0.0f,   200,   0.0f, 300.0f },
};

typedef struct struct_message {
  char jsonData[250];  // ESP-NOW limit is ~250 bytes
} struct_message;

struct_message outgoingMsg;

// ---- diagnostic state ----
bool canReady = false;
bool espNowReady = false;
bool streamGated = true;      // true = refuse to stream
int battGoodStreak = 0;       // consecutive plausible Batt reads
int rejectStreak = 0;         // consecutive implausible frames
unsigned long lastDiagPrint = 0;
unsigned long lastCanRetry = 0;
unsigned long lastEspNowRetry = 0;
unsigned long gatedDropCount = 0;

/* ---------- SPI helpers ---------- */
byte readRegister(byte address) {
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(MCP_READ);
  SPI.transfer(address);
  byte value = SPI.transfer(0x00);
  digitalWrite(CAN_CS_PIN, HIGH);
  return value;
}

void writeRegister(byte address, byte value) {
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(MCP_WRITE);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CAN_CS_PIN, HIGH);
}

void resetMCP2515() {
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(MCP_RESET);
  digitalWrite(CAN_CS_PIN, HIGH);
  delay(10);
}

void flushMcp2515Rx();

/* ---------- bus diagnostics ---------- */
void printBusDiag(const char* why) {
  byte eflg = readRegister(MCP_EFLG);
  byte intf = readRegister(MCP_CANINTF);
  byte stat = readRegister(MCP_CANSTAT);
  Serial.printf("[DIAG %s] EFLG=%02X INTF=%02X STAT=%02X CNF=%02X/%02X/%02X : %s%s%s%s%s\n",
                why, eflg, intf, stat,
                readRegister(MCP_CNF1), readRegister(MCP_CNF2), readRegister(MCP_CNF3),
                (eflg & 0x08) ? "BUS-OFF " : "",
                (eflg & 0x10) ? "TX-PASSIVE " : "",
                (eflg & 0x20) ? "RX-PASSIVE " : "",
                (eflg & 0x04) ? "REC>0 " : "",
                ((stat & 0xE0) != 0x00) ? "NOT-IN-NORMAL-MODE" : "");
}

/* ---------- CAN bring-up with verification ---------- */
void flushMcp2515Rx() {
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId = 0;
    byte len = 0;
    byte rxBuf[8];
    CAN.readMsgBuf(&rxId, &len, rxBuf);
  }
}

bool initCan() {
  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CAN_CS_PIN, HIGH);
  SPI.begin();

  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.printf("[CAN] init attempt %d\n", attempt);
    resetMCP2515();
    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
      Serial.println("[CAN] begin failed");
      delay(200);
      continue;
    }
    writeRegister(MCP_CANCTRL, 0x80);  // configuration mode
    delay(10);
    writeRegister(MCP_CNF1, CNF1_EXPECT);
    writeRegister(MCP_CNF2, CNF2_EXPECT);
    writeRegister(MCP_CNF3, CNF3_EXPECT);
    writeRegister(MCP_CANCTRL, 0x00);  // normal mode
    delay(10);

    // READ-BACK VERIFY: a cold-boot SPI glitch that partially applies
    // these registers currently latches forever undetected.
    byte c1 = readRegister(MCP_CNF1);
    byte c2 = readRegister(MCP_CNF2);
    byte c3 = readRegister(MCP_CNF3);
    byte stat = readRegister(MCP_CANSTAT);
    if (c1 == CNF1_EXPECT && c2 == CNF2_EXPECT && c3 == CNF3_EXPECT &&
        (stat & 0xE0) == 0x00) {
      Serial.printf("[CAN] OK attempt %d: CNF %02X/%02X/%02X normal-mode, 13tq@16MHz = 615384 bps\n",
                    attempt, c1, c2, c3);
      flushMcp2515Rx();
      return true;
    }
    Serial.printf("[CAN] VERIFY FAILED attempt %d: readback CNF %02X/%02X/%02X STAT %02X\n",
                  attempt, c1, c2, c3, stat);
    delay(200);
  }
  printBusDiag("init-can-failed");
  return false;
}

// Re-align after a request/response lag condition: stop sending, let the
// ECU drain anything it has queued, then empty our RX path. A one-behind
// queue makes every reply land as the previous symbol's answer — real
// values under wrong keys, which range checks only partly catch.
void resyncCan() {
  Serial.println("[RESYNC] pausing polls 500 ms to drain ECU queue, then flushing RX");
  delay(500);
  flushMcp2515Rx();
}

/* ---------- ESP-NOW bring-up ---------- */
bool initEspNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed");
    return false;
  }
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESPNOW] add peer 1 failed");
    return false;
  }
  memcpy(peerInfo.peer_addr, receiverMAC2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESPNOW] add peer 2 failed");
    return false;
  }
  Serial.printf("[ESPNOW] ready. own MAC %s, channel %d\n",
                WiFi.macAddress().c_str(), WiFi.channel(WIFI_STA));
  return true;
}

/* ---------- CAN protocol (unchanged) ---------- */
byte* txMsg(uint32_t addr) {
  static byte txdata[8];
  addr += 5;
  txdata[0] = 0xC7;
  txdata[1] = (addr >> 24) & 0xFF;
  txdata[2] = (addr >> 16) & 0xFF;
  txdata[3] = (addr >> 8) & 0xFF;
  txdata[4] = addr & 0xFF;
  return txdata;
}

uint32_t getValueFromCAN(const byte* data, int len) {
  uint32_t val = 0;
  int offset = 8;
  while (len-- > 0 && offset > 0) {
    offset--;
    val = (val * 256) + data[offset];
  }
  return val;
}

void printCANmsg(const byte* data) {
  Serial.print(" Data: ");
  for (int v = 0; v < 8; v++) {
    Serial.printf("%02X ", data[v]);
  }
  Serial.println();
}

/* ---------- main ---------- */
void setup() {
  Serial.begin(115200);
  delay(3000);

  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, HIGH);

  canReady = initCan();
  espNowReady = initEspNow();
  Serial.println("[BOOT] hardened interpreter up; gated until 3 plausible Batt reads");
}

void loop() {
  unsigned long now = millis();

  // Retry bring-ups instead of running broken
  if (!canReady && (now - lastCanRetry >= 2000)) {
    lastCanRetry = now;
    canReady = initCan();
  }
  if (!espNowReady && (now - lastEspNowRetry >= 5000)) {
    lastEspNowRetry = now;
    espNowReady = initEspNow();
  }
  if (!canReady) return;

  digitalWrite(BOARD_LED, HIGH);

  bool anyData = false;
  StaticJsonDocument<250> stream;

  int symbolCount = sizeof(symbolsToPoll) / sizeof(SymbolMap);
  for (int i = 0; i < symbolCount; i++) {
    SymbolMap& sym = symbolsToPoll[i];

    if (now - sym.lastPolled >= sym.pollInterval) {
      flushMcp2515Rx();
      if (CAN.sendMsgBuf(0x05, 0, 8, txMsg(sym.sramAddr)) == CAN_OK) {
        delay(10);
        if (CAN.checkReceive() == CAN_MSGAVAIL) {
          byte len = 0;
          byte rxBuf[8];
          unsigned long rxId;
          CAN.readMsgBuf(&rxId, &len, rxBuf);

          if (rxId == 0x0C) {
            // Stuck-FIFO signature: identical reply to DIFFERENT requests.
            // A latched RX buffer/CANINTF flag makes every poll read the
            // same frame back — recovery requires a full chip reset, and
            // the flush path cannot release it.
            static byte lastRx[8];
            static byte lastRxLen = 0;
            static bool lastRxValid = false;
            static uint16_t lastReqAddr = 0xFFFF;
            bool allZeroReply = true;
            for (int b = 1; b < (int)len; b++) if (rxBuf[b]) { allZeroReply = false; break; }  // skip C7 cmd byte
            // Stuck = DIFFERENT symbol answered with the SAME nonzero frame.
            // Identical all-zero replies across symbols are legitimate (engine
            // off / symbol not yet computed) — never treat them as latched.
            bool stuckSig = lastRxValid && len == lastRxLen &&
                            memcmp(rxBuf, lastRx, 8) == 0 &&
                            lastReqAddr != sym.sramAddr &&
                            !allZeroReply;
            memcpy(lastRx, rxBuf, 8);
            lastRxLen = len;
            lastRxValid = true;
            lastReqAddr = sym.sramAddr;
            static int stuckStrikes = 0;
            if (stuckSig && streamGated) stuckStrikes++; else stuckStrikes = 0;
            if (stuckStrikes >= 3 && streamGated) {
              stuckStrikes = 0;
              Serial.print("[STUCK] 3x identical 0x0C replies — RX state latched, full MCP2515 reset. Frame was:");
              printCANmsg(rxBuf);
              printBusDiag("stuck-fifo");
              battGoodStreak = 0;
              rejectStreak = 0;
              streamGated = true;
              canReady = initCan();
              return;
            }

            uint32_t rawVal = getValueFromCAN(rxBuf, sym.byteLength);
            float val = (rawVal + sym.adder) * sym.scaler;

            if (val >= sym.minVal && val <= sym.maxVal) {
              stream[sym.symbolId] = val;
              anyData = true;
              rejectStreak = 0;

              if (sym.sramAddr == 0x101B) {  // Batt_volt gate
                if (++battGoodStreak >= 3 && streamGated) {
                  streamGated = false;
                  Serial.println("[GATE] OPEN — streaming enabled");
                  printBusDiag("gate-open");
                }
              }
            } else {
              rejectStreak++;
              Serial.printf("[REJECT] %s val=%.2f outside [%.1f, %.1f] raw=%lu len=%u rxId=%02X",
                            sym.label.c_str(), val, sym.minVal, sym.maxVal,
                            (unsigned long)rawVal, len, rxId);
              printCANmsg(rxBuf);
              printBusDiag("reject");
              if (rejectStreak == 3) {
                resyncCan();  // lagged-queue suspect before escalating
                return;
              }

              if (sym.sramAddr == 0x101B) {  // battery implausible: re-gate
                battGoodStreak = 0;
                if (!streamGated) {
                  streamGated = true;
                  Serial.println("[GATE] CLOSED — battery implausible");
                }
              }
              // Watchdog: 20 consecutive rejects = automatic power cycle
              if (rejectStreak >= 20) {
                Serial.println("[WDT] 20 consecutive rejects — re-initializing CAN");
                rejectStreak = 0;
                battGoodStreak = 0;
                streamGated = true;
                canReady = initCan();
                return;
              }
            }
          } else {
            Serial.printf("[RX] unexpected id=%02X len=%u (want 0C)", rxId, len);
            printCANmsg(rxBuf);
          }
        }
      } else {
        printBusDiag("tx-fail");
      }
      sym.lastPolled = now;
    }
  }

  // Periodic bus snapshot while gated
  if (streamGated && (now - lastDiagPrint >= 5000)) {
    lastDiagPrint = now;
    Serial.printf("[GATE] waiting: battStreak=%d gatedDrops=%lu\n",
                  battGoodStreak, gatedDropCount);
    printBusDiag("periodic");
  }

  if (anyData) {
    if (streamGated) {
      gatedDropCount++;
    } else if (espNowReady) {
      serializeJson(stream, outgoingMsg.jsonData);
      esp_now_send(receiverMAC, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
      esp_now_send(receiverMAC2, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
      digitalWrite(BOARD_LED, LOW);
      Serial.println(outgoingMsg.jsonData);
    }
  }

  delay(5);
}
