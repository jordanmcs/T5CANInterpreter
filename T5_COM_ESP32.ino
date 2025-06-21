#include <SPI.h>
#include <mcp_can.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>


//#include "driver/twai.h"


#define CAN_CS_PIN 10

MCP_CAN CAN(CAN_CS_PIN);
 
// MCP2515 SPI commands and registers
#define MCP_WRITE 0x02
#define MCP_RESET 0xC0
#define MCP_READ 0x03
#define MCP_CNF1 0x2A
#define MCP_CNF2 0x29
#define MCP_CNF3 0x28
#define MCP_CANCTRL 0x0F

//Setup for UART
//#define TX_PIN 40
//#define RX_PIN 41

//ASL Mac address 74:4D:BD:89:AB:98
//WS Display Mac address CC:BA:97:11:CB:28

uint8_t receiverMAC[] = {0xCC, 0xBA, 0x97, 0x11, 0xCB, 0x28}; 

// symbolMap: {label, symbolId, sram_addr, byte_length, scaler}
struct SymbolMap {
  String label;
  String symbolId;
  int sramAddr;
  char byteLength;
  float scaler = 1.0;                 //to adjust symbol data to correct value
  float adder = 0.0;
  unsigned long pollInterval = 1000;  // milliseconds
  unsigned long lastPolled = 0;  // timestamp of last poll
};

SymbolMap symbolsToPoll[] = {
  { "Batt", "Batt_volt", 0x101B, 1, 0.1,0, 250 },
  { "RPM", "Rpm", 0x1066, 2, 10,0, 50 },
  { "Water", "Kyl_temp", 0x102E, 1, 1,0, 1000 },
  { "Air", "Lufttemp", 0x1033, 1, 1,0, 250 },
  { "NBo2", "AD_sond", 0x1012, 1, 10,0, 100 },
  { "TPS", "Medeltrot", 0x1038, 1, 0.3922,0, 80 }, //0-255 converted to 0-100%
  { "Boost", "P_manifold", 0x1013, 1, 1,-100.0, 100  },
  { "Timing", "Ign_angle", 0x10B4, 2, 0.1,0, 100 },
  { "Gear", "Gear", 0x3355, 1, 1, 0, 250 },
  { "Speed", "Bil_hast", 0x101C, 1, 1,0, 150 },
  { "APC", "PWM_ut10", 0x3359, 1, 10,0, 100 },
  { "Torque", "TQ", 0x1074, 1, 1,0, 200 },
  { "Load", "Medellast", 0x1037, 1, 1,0, 200 },



};

typedef struct struct_message {
  char jsonData[250];  // ESP-NOW limit is ~250 bytes
} struct_message;

struct_message outgoingMsg;


/* CANBUS Helper Functions */
byte readRegister(byte address) {
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(MCP_READ);  // 0x03
  SPI.transfer(address);
  byte value = SPI.transfer(0x00);  // Dummy write to read
  digitalWrite(CAN_CS_PIN, HIGH);
  return value;
}
// Utility SPI write to MCP2515 registerF
void writeRegister(byte address, byte value) {
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(MCP_WRITE);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CAN_CS_PIN, HIGH);
}
void printBaudRateRegisters() {
  Serial.print("CNF1: 0x");
  Serial.println(readRegister(MCP_CNF1), HEX);
  Serial.print("CNF2: 0x");
  Serial.println(readRegister(MCP_CNF2), HEX);
  Serial.print("CNF3: 0x");
  Serial.println(readRegister(MCP_CNF3), HEX);
}

// Reset MCP2515 controller
void resetMCP2515() {
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(MCP_RESET);
  digitalWrite(CAN_CS_PIN, HIGH);
  delay(10);
}

void configureT5Baud() {
  //while (!Serial);
  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CAN_CS_PIN, HIGH);
  SPI.begin();
  Serial.println("Resetting MCP2515...");
  resetMCP2515();
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
   // delay(250);

    // Put MCP2515 into configuration mode (0x80)
    writeRegister(MCP_CANCTRL, 0x80);
    delay(10);

    // Set CNF registers for ~615384 bps @ 8MHz oscillator
    writeRegister(MCP_CNF1, 0x00);
    writeRegister(MCP_CNF2, 0xAB);
    writeRegister(MCP_CNF3, 0x01);
    delay(10);
    Serial.println("MCP2515 configured for ~615384 bps.");

    // Switch back to normal mode (0x00)
    writeRegister(MCP_CANCTRL, 0x00);
    delay(250);
    Serial.println("MCP_CAN mode set to Normal.");

    // Send symbol table request and receive response
    //sendSymbolTableRequest();

  } else {
    Serial.println("MCP_CAN Initialization Failed.");
  }
}
/* End CANBUS Helper Functions*/


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

// pull value from RX CAN Frame
uint32_t getValueFromCAN(const byte* data, int len) {
  uint32_t val = 0;
  //offset += 8;
  int offset = 8;
  while (len-- > 0 && offset > 0) {
    offset--;
    val = (val * 256) + data[offset];
  }
  return val;
}


void printCANmsg(byte data[8]) {
  Serial.print(" Data: ");
  for (int v = 0; v < 8; v++) {
    Serial.print("0x");
    if (data[v] < 10) {
      Serial.print("0");
    }
    Serial.print(data[v], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

//UART comm checksum
uint8_t calculateChecksum(const char* data) {
  uint8_t cs = 0;
  while (*data) {
    cs ^= *data++;
  }
  return cs;
}

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}


// main program
void setup() {
  Serial.begin(115200);
  delay(3000);  // Wait 5 seconds for ECU to come online

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //set LED off

  configureT5Baud();  //set CANBUS Speed on MCP2515 to match Trionic 5

  //UART
  //Serial1.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN); for hardwire

  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  //esp_now_register_send_cb(onSent); call back function
  
  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("Peer added.");

  } else {
    Serial.println("Failed to add peer");
  }
  delay(10);

  //Serial.println("RX/TX Initialized");

}


void loop() {

  unsigned long now = millis();
  bool anyData = false;
  StaticJsonDocument<250> stream;

  digitalWrite(LED_BUILTIN, HIGH); //turn LED OFF when started


  // Once done, do nothing or add other code here
  int symbolCount = sizeof(symbolsToPoll) / sizeof(SymbolMap);
  for (int i = 0; i < symbolCount; i++) {
    SymbolMap& sym = symbolsToPoll[i];

    if (now - sym.lastPolled >= sym.pollInterval) {
      // Send request
      if (CAN.sendMsgBuf(0x05, 0, 8, txMsg(sym.sramAddr)) == CAN_OK) {
        delay(10);  // wait briefly for reply

        if (CAN.checkReceive() == CAN_MSGAVAIL) {
          byte len = 0;
          byte rxBuf[8];
          long unsigned int rxId;

          CAN.readMsgBuf(&rxId, &len, rxBuf);

          if (rxId == 0x0C) {
            uint32_t rawVal = getValueFromCAN(rxBuf, sym.byteLength);
            float val = (rawVal +  sym.adder) * sym.scaler ;
            stream[sym.symbolId] = val;
            anyData = true;
          }
        }
      }
      sym.lastPolled = now;  // update last polled time
    }
  }

  // Only send JSON if we added any fields
  if (anyData) {
    serializeJson(stream, outgoingMsg.jsonData);
    esp_now_send(receiverMAC, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
    digitalWrite(LED_BUILTIN, LOW); //turn LED on when sending data

   // Serial.print(sizeof(outgoingMsg));
   //Serial.print(" : ");
    Serial.println(outgoingMsg.jsonData);
   // Serial1.print(jsonBuffer); //TX pin 40
  }

  delay(5);  // small delay to avoid busy loop
}


