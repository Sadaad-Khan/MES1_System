#include <Arduino.h>
#include <Adafruit_MCP2515.h>

// Set Up Chip Select Pin
#ifdef ARDUINO_ADAFRUIT_FEATHER_RP2040_CAN
  #define CS_PIN PIN_CAN_CS
#else
  #define CS_PIN 19
#endif

// Set Up CAN_IDs : Node_ID = 0x15
#define NODE_ID 0x15
#define TSDO_COB_ID (0x580 + NODE_ID) // 0x595
#define RSDO_COB_ID (0x600 + NODE_ID) // 0x615
#define HEARTBEAT_COB_ID (0x700 + NODE_ID)  // 0x715
#define DEBUG_CAN_ID 0x666

// Constants
#define CAN_BAUDRATE 1000000
#define HEARTBEAT_INTERVAL 5000 // 5 seconds
#define TIMEOUT_DURATION 20000  // 20 seconds
#define MOTOR_MAX_SPEED 255 // 1 to 255 (Arduino Analog Pin)

// SDO Command Codes
#define SDO_WRITE 0x23 // Client command to write data (server to winch)
#define SDO_RESPONSE 0x60 // Server acknowledgment (winch to server)
#define SDO_ERROR 0x80 // Error response

// SDO Object Dictionary Indices
#define OD_CONTROL_COMMAND 0x2000 // Sub-index 0: command, 1: use_timer, 2: duration
#define OD_FEEDBACK 0x2001 // Sub-index 0: status
#define OD_RESULT 0x2002 // Sub-index 0: success, 1: error_code
#define CMD_RETRACT 0x01
#define CMD_EXTEND 0x02
#define CMD_STOP 0x03

// Pin definitions
const int spdPin = A0;
const int brkPin = 9;
const int enaPin = 10;
const int dirPin = 11;
const int retLim = 5;
const int extLim = 6;

// Heartbeat node states
enum NodeState {
  NODE_BOOTUP           = 0x00, // Boot Up
  NODE_STOPPED          = 0x04, // Stopped
  NODE_OPERATIONAL      = 0x05, // Operational
  NODE_PRE_OPERATIONAL  = 0x7F  // Pre-operational
};

// Error codes
enum ErrorCode {
  NONE = 0,
  UNKNOWN = 200,
  INVALID_DURATION = 201,
  CAN_SEND_FAILED = 202,
  TIMEOUT = 203,
  NO_RESULT_RECEIVED = 204,
  CAN_INIT_FAILED = 205,
  INVALID_COMMAND = 206,
  SDO_OBJECT_NOT_FOUND = 207
};

// Debug message types
enum DebugMessageType {
  MSG_INIT = 0x01,
  MSG_CAN_DATA = 0x02,
  MSG_RETRACT = 0x03,
  MSG_EXTEND = 0x04,
  MSG_STOP = 0x05,
  MSG_UNKNOWN = 0x06,
  MSG_ERROR = 0x07,
  MSG_SDO_FEEDBACK = 0x08,
  MSG_SDO_RESULT = 0x09,
  MSG_SDO_ACK = 0x0A
};

Adafruit_MCP2515 mcp(CS_PIN);

uint8_t node_state = NODE_BOOTUP; // Initial state
unsigned long last_heartbeat_time = 0;
unsigned long last_ret_lim_time = 0;
unsigned long last_ext_lim_time = 0;
unsigned long start_time = 0;
unsigned long duration_ms = 0;
bool executing = false;
bool pull_cmd = false;
bool use_timer_cmd = false;
bool result_success = false;
uint16_t result_error_code = NONE;
bool ret_lim_triggered = false;
bool ext_lim_triggered = false;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  while (!Serial && millis() < 2000) delay(10);

  // Initialize Analog & Digital Pins
  pinMode(spdPin, OUTPUT);        // speed switch
  pinMode(brkPin, OUTPUT);        // brake switch
  pinMode(enaPin, OUTPUT);        // enable switch
  pinMode(dirPin, OUTPUT);        // direction switch
  pinMode(retLim, INPUT_PULLUP);  // retract limit switch
  pinMode(extLim, INPUT_PULLUP);  // extend limit switch

  // Initialize CAN bus
  Serial.print("Initializing CAN... ");
  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    node_state = NODE_STOPPED;
    sendHeartbeat(); // Heartbeat 0x04
    while (1);
  }
  Serial.println("MCP2515 Initialized!");
  sendHeartbeat(); // Heartbeat 0x00
  delay(500);
  
  node_state = NODE_PRE_OPERATIONAL;
  sendHeartbeat(); // Heartbeat 0x7F

  // Initialize speed of winch motor
  Serial.print("Initializing Motor Speed... ");
  analogWrite(spdPin, MOTOR_MAX_SPEED);
  Serial.print("Speed Set to ");
  Serial.println(MOTOR_MAX_SPEED);
  delay(500);

  WinchStop();
  node_state = NODE_OPERATIONAL;
  sendHeartbeat(); // Heartbeat 0x05
  delay(500);
}

void loop() {
  // Send periodic heartbeat (every 5 seconds)
  if (millis() - last_heartbeat_time >= HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    last_heartbeat_time = millis();
  }

  checkExecutionState();

  // Receive CAN messages (skip if received)
  int packetSize = mcp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  // Check correct COB_ID (skip if correct)
  uint32_t receivedId = mcp.packetId();
  if (receivedId != RSDO_COB_ID) {
    return;
  }

  // Read CAN messages
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
  // sendDebugMessage(MSG_CAN_DATA, receivedData, packetSize);

  /* 
  =======================
  = Process SDO command =
  =======================
  */

  // Check correct RSDO command specifier (skip if correct)
  if (receivedData[0] != SDO_WRITE || packetSize != 8) {
    Serial.println("Invalid Command Specifier!");
    sendSDOAck(0, 0, false, INVALID_COMMAND);
    node_state = NODE_STOPPED;
    sendHeartbeat();  // Heartbeat 0x04
    return;
  }

  uint16_t indexOD = (receivedData[2] << 8) | receivedData[1];
  uint8_t subIndex = receivedData[3];

  // Check valid control command (skip if valid)
  if (indexOD != OD_CONTROL_COMMAND) {
    Serial.println("Invalid Control Command!");
    sendSDOAck(indexOD, subIndex, false, SDO_OBJECT_NOT_FOUND);
    node_state = NODE_STOPPED;
    sendHeartbeat();  // Heartbeat 0x04
    return;
  }

  uint8_t command = receivedData[4];
  bool use_timer = receivedData[5];
  uint32_t duration_int = (receivedData[6]) | (receivedData[7] << 8) | (0 << 16);
  float duration = duration_int / 1000;

  // Check duration (0 to 60 seconds) (skip if valid)
  if (use_timer && (duration < 0 || duration_int > 0xFFFF)) {
    Serial.println("Invalid duration!");
    sendSDOAck(indexOD, subIndex, false, INVALID_DURATION);
    node_state = NODE_STOPPED;
    sendHeartbeat();  // Heartbeat 0x04
    return;
  }

  // Send acknowledgment via TSDO
  sendSDOAck(indexOD, subIndex, true, NONE);

  switch (command) {
    case CMD_EXTEND:
      Serial.print("EXTEND: use_timer=");
      Serial.print(use_timer);
      Serial.print(", duration=");
      Serial.println(duration);
      // sendDebugMessage(MSG_EXTEND, "Start");
      executeCommand(false, use_timer, duration);
      break;
    case CMD_RETRACT:
      Serial.print("RETRACT: use_timer=");
      Serial.print(use_timer);
      Serial.print(", duration=");
      Serial.println(duration);
      // sendDebugMessage(MSG_RETRACT, "Start");
      executeCommand(true, use_timer, duration);
      break;
    case CMD_STOP:
      Serial.println("Received STOP command!");
      // sendDebugMessage(MSG_STOP, "Stop");
      WinchStop();
      executing = false;
      sendSDOResult(true, NONE);
      node_state = NODE_STOPPED;
      sendHeartbeat();  // Heartbeat 0x04
      break;
    default:
      Serial.println("Unknown SDO command.");
      // sendDebugMessage(MSG_UNKNOWN, "Unknown");
      sendSDOAck(indexOD, subIndex, false, INVALID_COMMAND);
      node_state = NODE_STOPPED;
      sendHeartbeat();  // Heartbeat 0x04
      break;
  }
}

void executeCommand(bool pull, bool use_timer, float duration) {

  pull_cmd = pull;
  use_timer_cmd = use_timer;
  duration_ms = (unsigned long)(duration * 1000);
  start_time = millis();
  executing = true;
  result_success = false;
  result_error_code = NONE;
  ret_lim_triggered = false;
  ext_lim_triggered = false;

  WinchStop();
  delay(10);

  if (pull_cmd) {
    WinchRetract();
    Serial.println("Retracting...");
    // sendDebugMessage(MSG_RETRACT, "Retract");
  } else {
    WinchExtend();
    Serial.println("Extending...");
    // sendDebugMessage(MSG_EXTEND, "Extend");
  }
}

void checkExecutionState() {
  if (!executing) {
    return;
  }

  unsigned long now = millis();
  bool done = false;
  uint8_t status = 128; // Default feedback

  // Debug: Log pin states
  Serial.print("retLim: ");
  Serial.print(digitalRead(retLim));
  Serial.print(", extLim: ");
  Serial.print(digitalRead(extLim));
  Serial.print(", pull_cmd: ");
  Serial.println(pull_cmd);

  if (!use_timer_cmd) {

    bool ret_lim_state = digitalRead(retLim) == HIGH;
    
    if (ret_lim_state && !ret_lim_triggered) {
      last_ret_lim_time = now;
      ret_lim_triggered = true;
    } else if (ret_lim_triggered && now - last_ret_lim_time >= 50) {  // Debounce delay 50ms
      if (pull_cmd) {
        result_success = true;
        done = true;
        Serial.println("Retract Limit Reached.");
      }
    } else if (!ret_lim_state) {
      ret_lim_triggered = false;
    }

    bool ext_lim_state = digitalRead(extLim) == HIGH;
    
    if (ext_lim_state && !ext_lim_triggered) {
      last_ext_lim_time = now;
      ext_lim_triggered = true;
    } else if (ext_lim_triggered && now - last_ext_lim_time >= 50) {  // Debounce delay 50ms
      if (!pull_cmd) {
        result_success = true;
        done = true;
        Serial.println("Extend Limit Reached.");
      }
    } else if (!ext_lim_state) {
      ext_lim_triggered = false;
    }

    if (now - start_time >= TIMEOUT_DURATION) {
      result_success = false;
      result_error_code = TIMEOUT;
      done = true;
      Serial.println("Timeout Occured.");
      // sendDebugMessage(MSG_STOP, "Timeout");
    }

  } else {
    if (now - start_time >= duration_ms) {
      result_success = true;
      done = true;
      Serial.println("Timer ended.");
    } else {
      float elapsed_ratio = (float)(now - start_time) / duration_ms;
      status = (uint8_t)(min(elapsed_ratio, 1.0f) * 255);
    }
  }

  sendSDOFeedback(status);

  if (done) {
    WinchStop();
    executing = false;
    Serial.println("Command execution completed.");
    // sendDebugMessage(MSG_STOP, "Stop");
    sendSDOResult(result_success, result_error_code);
  }
}

/*
  CONTROL LOGIC
  -------------
  To ON Brake     : brkPin LOW
  To OFF Brake    : brkPin HIGH
  To Enable Winch : enaPin LOW
  To Disable      : enaPin HIGH
  Direction:
    - Extend      : dirPin LOW
    - Retract     : dirPin HIGH
*/

void WinchRetract() {
  digitalWrite(brkPin, HIGH);
  digitalWrite(enaPin, LOW);
  digitalWrite(dirPin, HIGH);
}

void WinchExtend() {
  digitalWrite(brkPin, HIGH);
  digitalWrite(enaPin, LOW);
  digitalWrite(dirPin, LOW);
}

void WinchStop() {
  digitalWrite(brkPin, LOW);
  digitalWrite(enaPin, HIGH);
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

void sendSDOFeedback(uint8_t status) {
  uint8_t data[8] = {0};
  data[0] = SDO_WRITE; // Client write to server's RSDO
  data[1] = OD_FEEDBACK & 0xFF;        // Low byte of index
  data[2] = (OD_FEEDBACK >> 8) & 0xFF; // High byte of index
  data[3] = 0x00; // Sub-index 0
  data[4] = status; // Status value (0-255)
  sendCanMessage(TSDO_COB_ID, data);
  // sendDebugMessage(MSG_SDO_FEEDBACK, data, 8);
}

void sendSDOResult(bool success, uint16_t error_code) {
  uint8_t data[8] = {0};
  data[0] = SDO_WRITE; // Client write to server's RSDO
  data[1] = OD_RESULT & 0xFF;        // Low byte of index
  data[2] = (OD_RESULT >> 8) & 0xFF; // High byte of index
  data[3] = 0x00; // Sub-index 0
  data[4] = success ? 1 : 0; // Success flag
  data[5] = (error_code >> 8) & 0xFF; // High byte of error code
  data[6] = error_code & 0xFF;        // Low byte of error code
  sendCanMessage(TSDO_COB_ID, data);
  // sendDebugMessage(MSG_SDO_RESULT, data, 8);
}

void sendSDOAck(uint16_t index, uint8_t subIndex, bool success, uint16_t error_code) {
  uint8_t data[8] = {0};
  if (success) {
    data[0] = SDO_RESPONSE; // Acknowledge successful write
  } else {
    data[0] = SDO_ERROR; // Error response
    data[4] = (error_code >> 8) & 0xFF; // High byte
    data[5] = error_code & 0xFF;        // Low byte
  }
  data[1] = index & 0xFF;        // Low byte of index
  data[2] = (index >> 8) & 0xFF; // High byte of index
  data[3] = subIndex;
  sendCanMessage(TSDO_COB_ID, data);
  // sendDebugMessage(MSG_SDO_ACK, data, 8);
}

void sendHeartbeat() {
  uint8_t data[8] = {0};
  data[0] = node_state;
  sendCanMessage(HEARTBEAT_COB_ID, data);
  Serial.print("Sent Heartbeat: State=");
  Serial.println(node_state, HEX);
}

void sendCanMessage(int canId, uint8_t* data) {
  mcp.beginPacket(canId);
  mcp.write(data, 8);
  if (!mcp.endPacket()) {
    Serial.println("Failed to send CAN message.");
    // sendDebugMessage(MSG_ERROR, "SendFail");
  }
}
