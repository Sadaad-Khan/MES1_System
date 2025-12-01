#include <Arduino.h>
#include <Adafruit_MCP2515.h>

// Set Up Chip Select Pin
#ifdef ARDUINO_ADAFRUIT_FEATHER_RP2040_CAN
  #define CS_PIN PIN_CAN_CS
#else
  #define CS_PIN 19
#endif

#define CAN_BAUDRATE 500000
#define DRIVER_CAN_ID 0x15
#define DEBUG_CAN_ID 0x666
#define FEEDBACK_CAN_ID 0x667
#define RESULT_CAN_ID 0x668
#define WATCHDOG_TIMEOUT 5000 // 5 seconds
#define TIMEOUT_DURATION 10000

const uint8_t RETRACT_MESSAGE[] = {0x08, 0x10, 0x18, 0x01};
const uint8_t EXTEND_MESSAGE[]  = {0x08, 0x10, 0x18, 0x02};
const uint8_t STOP_MESSAGE[]    = {0x08, 0x10, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00};

const int fwdPin = 9;
const int revPin = 10;
const int retLim = 5;
const int extLim = 6;

const int maxSpd = 255;

enum ErrorCode {
  NONE               = 0,
  UNKNOWN            = 200,
  INVALID_DURATION   = 201,
  CAN_SEND_FAILED    = 202,
  TIMEOUT            = 203,
  NO_RESULT_RECEIVED = 204,
  CAN_INIT_FAILED    = 205,
  INVALID_COMMAND    = 206
};

enum DebugMessageType {
  MSG_INIT     = 0x01,
  MSG_CAN_DATA = 0x02,
  MSG_RETRACT  = 0x03,
  MSG_EXTEND   = 0x04,
  MSG_STOP     = 0x05,
  MSG_UNKNOWN  = 0x06,
  MSG_ERROR    = 0x07,
  MSG_FEEDBACK = 0x08,
  MSG_RESULT   = 0x09
};

Adafruit_MCP2515 mcp(CS_PIN);
unsigned long last_command_time = 0;
int watchdogCount = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 2000) delay(10);

  pinMode(fwdPin, OUTPUT);
  pinMode(revPin, OUTPUT);
  pinMode(retLim, INPUT_PULLUP);
  pinMode(extLim, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print("Initializing CAN... ");
  int attempts = 0;
  while (!mcp.begin(CAN_BAUDRATE)) {
    sendDebugMessage(MSG_ERROR, "CANFail");
    Serial.print(".");
    delay(1000);
    attempts++;
    if (attempts >= 5) {
      Serial.println("MCP2515 failed after 5 attempts. Halting.");
      while (1) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(500);
      }
    }
  }
  Serial.println("MCP2515 Initialized!");
  sendDebugMessage(MSG_INIT, "CANInit");

  delay(500);
  WinchStop();
  delay(1000);
}

void loop() {
  if (millis() - last_command_time > WATCHDOG_TIMEOUT) {
    WinchStop();
    watchdogCount++;
    Serial.print("Watchdog triggered: ");
    Serial.println(watchdogCount);
    sendDebugMessage(MSG_STOP, "WdogSTP");
    last_command_time = millis();
  }

  int packetSize = mcp.parsePacket();
  if (packetSize == 0) return;

  if (packetSize != 8) {
    Serial.print("Invalid CAN packet size: ");
    Serial.println(packetSize);
    sendDebugMessage(MSG_ERROR, "BadLen");
    sendResult(false, UNKNOWN);
    return;
  }

  uint32_t receivedId = mcp.packetId();
  if (receivedId != DRIVER_CAN_ID) {
    Serial.println("Received ID Incorrect!");
    sendDebugMessage(MSG_ERROR, "InvID");
    return;
  }

  uint8_t receivedData[8] = {0};
  int index = 0;
  while (mcp.available() && index < 8) {
    receivedData[index++] = mcp.read();
  }
  Serial.print("Received CAN Data: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(receivedData[i], HEX); Serial.print(" ");
  }
  Serial.println();
  sendDebugMessage(MSG_CAN_DATA, receivedData, 8);

  if (memcmp(receivedData, EXTEND_MESSAGE, 4) == 0 || memcmp(receivedData, RETRACT_MESSAGE, 4) == 0) {
    bool pull = memcmp(receivedData, RETRACT_MESSAGE, 4) == 0;
    bool use_timer = receivedData[4];
    uint32_t duration_int = (receivedData[5] << 16) | (receivedData[6] << 8) | receivedData[7];
    float duration = duration_int / 1000.0;

    if (use_timer && (duration < 0 || duration_int > 0xFFFFFF)) {
      Serial.println("Invalid duration!");
      sendDebugMessage(MSG_ERROR, "InvDur");
      sendResult(false, INVALID_DURATION);
      return;
    }

    // Limit check before motion
    if (!pull && digitalRead(extLim) == HIGH) {
      Serial.println("Already fully extended.");
      sendDebugMessage(MSG_ERROR, "AlExtd");
      sendResult(false, INVALID_COMMAND);
      return;
    }
    if (pull && digitalRead(retLim) == HIGH) {
      Serial.println("Already fully retracted.");
      sendDebugMessage(MSG_ERROR, "AlRetr");
      sendResult(false, INVALID_COMMAND);
      return;
    }

    Serial.print(pull ? "RETRACT" : "EXTEND");
    Serial.print(": use_timer="); Serial.print(use_timer);
    Serial.print(", duration="); Serial.println(duration);
    sendDebugMessage(pull ? MSG_RETRACT : MSG_EXTEND, "Start");
    executeCommand(pull, use_timer, duration);
    last_command_time = millis();
  }
  else if (memcmp(receivedData, STOP_MESSAGE, 8) == 0) {
    Serial.println("Received STOP command!");
    sendDebugMessage(MSG_STOP, "Stop");
    WinchStop();
    sendResult(true, NONE);
    last_command_time = millis();
  }
  else {
    Serial.println("Unknown CAN command.");
    sendDebugMessage(MSG_UNKNOWN, "Unknown");
    sendResult(false, INVALID_COMMAND);
  }

  while (mcp.parsePacket()) {
    while (mcp.available()) mcp.read();
  }
}

void executeCommand(bool pull, bool use_timer, float duration) {
  unsigned long start_time = millis();
  unsigned long duration_ms = (unsigned long)(duration * 1000);
  bool limit_reached = false;
  bool timeout = false;
  bool success = false;
  uint16_t error_code = NONE;

  WinchStop();
  delay(10);

  // Limit switch safety check before motion
  if (!pull && digitalRead(extLim) == LOW) {
    Serial.println("ERROR: Extend limit switch stuck or shorted!");
    sendDebugMessage(MSG_ERROR, "ExtLimErr");
    WinchStop();
    sendResult(false, UNKNOWN);
    return;
  }
  if (pull && digitalRead(retLim) == LOW) {
    Serial.println("ERROR: Retract limit switch stuck or shorted!");
    sendDebugMessage(MSG_ERROR, "RetLimErr");
    WinchStop();
    sendResult(false, UNKNOWN);
    return;
  }

  if (pull) {
    WinchRetract();
    Serial.println("Retracting...");
    sendDebugMessage(MSG_RETRACT, "Retract");
  } else {
    WinchExtend();
    Serial.println("Extending...");
    sendDebugMessage(MSG_EXTEND, "Extend");
  }

  if (!use_timer) {
    while (!limit_reached && !timeout) {
      if (!pull && digitalRead(extLim) == HIGH) {
        limit_reached = true;
        success = true;
        Serial.println("Extend Limit Reached.");
        sendDebugMessage(MSG_EXTEND, "Limit");
      }
      else if (pull && digitalRead(retLim) == HIGH) {
        limit_reached = true;
        success = true;
        Serial.println("Retract Limit Reached.");
        sendDebugMessage(MSG_RETRACT, "Limit");
      }
      if (millis() - start_time >= TIMEOUT_DURATION) {
        timeout = true;
        success = false;
        Serial.println("Timeout Occurred.");
        sendDebugMessage(MSG_STOP, "Timeout");
        error_code = TIMEOUT;
      }
      sendFeedback(128);
      delay(50);
    }
  } else {
    while (millis() - start_time < duration_ms) {
      float elapsed = (float)(millis() - start_time) / 1000;
      uint8_t status = (uint8_t)(min(elapsed / duration, 1) * 255);
      sendFeedback(status);
      delay(50);
    }
    Serial.println("Timer ended.");
    sendDebugMessage(MSG_STOP, "TimeEnd");
    success = true;
  }

  WinchStop();
  sendDebugMessage(MSG_STOP, "Stop");
  sendResult(success, error_code);
}

void WinchRetract() {
  digitalWrite(revPin, HIGH);
  digitalWrite(fwdPin, LOW);
}

void WinchExtend() {
  digitalWrite(revPin, LOW);
  digitalWrite(fwdPin, HIGH);
}

void WinchStop() {
  digitalWrite(revPin, LOW);
  digitalWrite(fwdPin, LOW);
}

void sendDebugMessage(uint8_t msgType, const char* msg) {
  uint8_t data[8] = {0};
  data[0] = msgType;
  int len = strlen(msg);
  if (len > 7) len = 7;
  for (int i = 0; i < len; i++) data[i + 1] = msg[i];
  sendCanMessage(DEBUG_CAN_ID, data);
}

void sendDebugMessage(uint8_t msgType, const uint8_t* data, uint8_t len) {
  uint8_t msg[8] = {0};
  msg[0] = msgType;
  if (len > 7) len = 7;
  memcpy(&msg[1], data, len);
  sendCanMessage(DEBUG_CAN_ID, msg);
}

void sendFeedback(uint8_t status) {
  uint8_t data[8] = {0};
  data[0] = MSG_FEEDBACK;
  data[4] = status;
  sendCanMessage(FEEDBACK_CAN_ID, data);
}

void sendResult(bool success, uint16_t error_code) {
  uint8_t data[8] = {0};
  data[0] = MSG_RESULT;
  data[1] = success ? 1 : 0;
  data[2] = (error_code >> 8) & 0xFF;
  data[3] = error_code & 0xFF;
  sendCanMessage(RESULT_CAN_ID, data);
}

void sendCanMessage(int canId, uint8_t* data) {
  mcp.beginPacket(canId);
  mcp.write(data, 8);
  if (!mcp.endPacket()) {
    Serial.print("CAN send failed to ID: 0x");
    Serial.println(canId, HEX);
    sendDebugMessage(MSG_ERROR, "SendFail");
  }
}
