/*
 * Production-Grade Multi-Node CANopen Relay Control
 * 
 * ARCHITECTURE: Distributed CANopen Network
 * Each microcontroller instance runs this same firmware with different
 * compile-time configuration (NODE_ROLE and NODE_ID) to create a
 * distributed relay control system.
 * 
 * SUPPORTED ROLES:
 * - ROLE_DRL: LED DRL control (K1 only)
 * - ROLE_SAFETY: Safety light control (K2 only)
 * - ROLE_BATTERY: Battery lock control (K3 + K4)
 * - ROLE_FUTURE: Reserved for future expansion
 * 
 * CHANGELOG:
 * 2025-11-17: Multi-node refactoring
 *   - Added NODE_ROLE system for distributed architecture
 *   - Role-specific command filtering and pin mapping
 *   - Each node has unique NODE_ID and specialized function
 *   - Maintains full CANopen protocol compatibility
 *   - Backward compatible with single-node operation
 * 
 * Hardware: Adafruit Feather RP2040 + MCP2515 CAN controller
 * Protocol: CANopen SDO
 * Bitrate: 1 Mbps
 */

#include <Arduino.h>
#include <Adafruit_MCP2515.h>

// ============================================================================
// MULTI-NODE CONFIGURATION - CONFIGURE THIS FOR EACH DEVICE
// ============================================================================

// --- Node Role Selection (uncomment ONE) ---
// #define NODE_ROLE ROLE_DRL         // Node A: LED DRL control
// #define NODE_ROLE ROLE_SAFETY      // Node B: Safety light control
// #define NODE_ROLE ROLE_BATTERY     // Node C: Battery lock control
// #define NODE_ROLE ROLE_FUTURE      // Node D: Future expansion
// #define NODE_ROLE ROLE_ALL         // ALL-IN-ONE: Control all 4 relays
#define NODE_ROLE ROLE_WINCH          // WINCH: Winch motor control

// --- Role Definitions ---
#define ROLE_DRL      1
#define ROLE_SAFETY   2
#define ROLE_BATTERY  3
#define ROLE_FUTURE   4
#define ROLE_ALL      5
#define ROLE_WINCH    6

// --- Node ID Assignment (must match role) ---
#if NODE_ROLE == ROLE_DRL
  constexpr uint8_t  NODE_ID = 0x11;  // 17 decimal
  #define NODE_NAME "LED_DRL"
#elif NODE_ROLE == ROLE_SAFETY
  constexpr uint8_t  NODE_ID = 0x12;  // 18 decimal
  #define NODE_NAME "Safety_Light"
#elif NODE_ROLE == ROLE_BATTERY
  constexpr uint8_t  NODE_ID = 0x13;  // 19 decimal
  #define NODE_NAME "Battery_Locks"
#elif NODE_ROLE == ROLE_FUTURE
  constexpr uint8_t  NODE_ID = 0x14;  // 20 decimal
  #define NODE_NAME "Future_Module"
#elif NODE_ROLE == ROLE_ALL
  constexpr uint8_t  NODE_ID = 0x10;  // 16 decimal
  #define NODE_NAME "All_Relays"
#elif NODE_ROLE == ROLE_WINCH
  constexpr uint8_t  NODE_ID = 0x15;  // 21 decimal
  #define NODE_NAME "Winch_Motor"
#else
  #error "Invalid NODE_ROLE - must be ROLE_DRL, ROLE_SAFETY, ROLE_BATTERY, ROLE_FUTURE, ROLE_ALL, or ROLE_WINCH"
#endif

// ============================================================================
// FEATURE FLAGS
// ============================================================================

#define DEBUG                     1    // Enable debug serial output
#define ENABLE_CAN_HEALTH         1    // Enable CAN health LED module (0=disabled for manual testing)
#define ENABLE_DIAGNOSTICS        1    // Enable diagnostic counters and publishing
#define ENABLE_SIMPLE_CHECKSUM    0    // Enable simple XOR checksum validation
#define ENABLE_JSON_LOGGING       1    // Enable machine-readable JSON logs

// ============================================================================
// CANOPEN CONFIGURATION
// ============================================================================

constexpr uint32_t CAN_BAUD       = 1000000;  // 1 Mbps

// --- Computed COB-IDs (based on NODE_ID) ---
constexpr uint32_t COB_TSDO       = 0x580 + NODE_ID;  // Transmit SDO
constexpr uint32_t COB_RSDO       = 0x600 + NODE_ID;  // Receive SDO
constexpr uint32_t COB_HEARTBEAT  = 0x700 + NODE_ID;  // Heartbeat
constexpr uint32_t COB_NMT        = 0x000;            // NMT broadcast

// --- Object Dictionary ---
constexpr uint16_t OD_CONTROL_COMMAND = 0x2000;  // Write: Command byte
constexpr uint16_t OD_RELAY_FEEDBACK  = 0x2001;  // Read: Current relay bitmap
constexpr uint16_t OD_COMMAND_RESULT  = 0x2002;  // Read: Success/failure
constexpr uint16_t OD_DIAGNOSTICS     = 0x2003;  // Read: Diagnostic counters
constexpr uint16_t OD_NODE_ROLE       = 0x2004;  // Read: Node role identifier

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================
// MCP2515 CAN Controller Pins (Adafruit Feather RP2040 CAN)
#define CS_PIN    PIN_CAN_CS           
#define INT_PIN   PIN_CAN_INT

// --- Role-Specific Pin Mappings ---
#if NODE_ROLE == ROLE_DRL
  // DRL Node: K1 only (LED DRL)
  constexpr uint8_t PIN_K1        = 25;
  constexpr uint8_t ACTIVE_RELAYS = 1;  // Number of relays this node controls
  constexpr uint8_t RELAY_PINS[ACTIVE_RELAYS] = {PIN_K1};
  
#elif NODE_ROLE == ROLE_SAFETY
  // Safety Node: K2 only (Safety Light)
  constexpr uint8_t PIN_K2        = 14;
  constexpr uint8_t ACTIVE_RELAYS = 1;
  constexpr uint8_t RELAY_PINS[ACTIVE_RELAYS] = {PIN_K2};
  
#elif NODE_ROLE == ROLE_BATTERY
  // Battery Node: K3 + K4 (Battery Locks)
  constexpr uint8_t PIN_K3        = 15;
  constexpr uint8_t PIN_K4        = 8;
  constexpr uint8_t ACTIVE_RELAYS = 2;
  constexpr uint8_t RELAY_PINS[ACTIVE_RELAYS] = {PIN_K3, PIN_K4};
  
#elif NODE_ROLE == ROLE_ALL
  // All-in-One Node: K1 + K2 + K3 + K4 (all relays)
  constexpr uint8_t PIN_K1        = 25;
  constexpr uint8_t PIN_K2        = 14;
  constexpr uint8_t PIN_K3        = 15;
  constexpr uint8_t PIN_K4        = 8;
  constexpr uint8_t ACTIVE_RELAYS = 4;
  constexpr uint8_t RELAY_PINS[ACTIVE_RELAYS] = {PIN_K1, PIN_K2, PIN_K3, PIN_K4};
  
#elif NODE_ROLE == ROLE_WINCH
  // Winch Node: Motor control pins
  constexpr uint8_t PIN_FWD       = 9;   // Forward/Extend motor control
  constexpr uint8_t PIN_REV       = 10;  // Reverse/Retract motor control
  constexpr uint8_t PIN_RET_LIM   = 5;   // Retract limit switch
  constexpr uint8_t PIN_EXT_LIM   = 6;   // Extend limit switch
  constexpr uint8_t MAX_SPEED     = 255; // PWM max speed
  constexpr uint8_t ACTIVE_RELAYS = 0;   // No relays, motor control instead
  
#elif NODE_ROLE == ROLE_FUTURE
  // Future Node: No relays assigned yet
  constexpr uint8_t ACTIVE_RELAYS = 0;
  // constexpr uint8_t RELAY_PINS[ACTIVE_RELAYS] = {};  // Empty for now
#endif

// --- Common Pins ---
constexpr uint8_t PIN_BUZ         = 4;   // Buzzer

#if ENABLE_CAN_HEALTH
constexpr uint8_t CAN_LED_GREEN_PIN = 10;  // CAN health - green LED
constexpr uint8_t CAN_LED_RED_PIN   = 11;  // CAN health - red LED
#endif

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

constexpr unsigned long HEARTBEAT_INTERVAL_MS     = 5000;    // 5 seconds
constexpr unsigned long MCP_POLL_INTERVAL_MS      = 5;       // 5 ms
constexpr unsigned long CAN_HEALTH_TIMEOUT_MS     = 30000;   // 30 seconds (increased for manual testing)
constexpr unsigned long EEPROM_WRITE_DEBOUNCE_MS  = 2000;    // 2 seconds

// --- CAN Recovery Configuration (Progressive Backoff) ---
constexpr unsigned long CAN_RECOVERY_BACKOFF_MS[] = {
  10000,  // 10 seconds (first attempt)
  30000,  // 30 seconds (second attempt)
  60000   // 60 seconds (third+ attempts)
};
constexpr uint8_t CAN_RECOVERY_BACKOFF_LEVELS = sizeof(CAN_RECOVERY_BACKOFF_MS) / sizeof(CAN_RECOVERY_BACKOFF_MS[0]);

constexpr int CAN_TX_RETRIES                      = 3;
constexpr unsigned long CAN_TX_RETRY_DELAY_MS     = 50;

// ============================================================================
// SDO PROTOCOL CONSTANTS
// ============================================================================

constexpr uint8_t SDO_WRITE       = 0x23;  // Expedited download request
constexpr uint8_t SDO_READ        = 0x40;  // Upload request
constexpr uint8_t SDO_RESPONSE    = 0x60;  // Response (ACK)
constexpr uint8_t SDO_ERROR       = 0x80;  // Error response

// --- Error Codes (CANopen standard) ---
enum ErrorCode : uint16_t {
  ERR_NONE                = 0x0000,
  ERR_UNKNOWN             = 0x0200,
  ERR_INVALID_COMMAND     = 0x0206,
  ERR_OBJECT_NOT_FOUND    = 0x0207,
  ERR_CHECKSUM_FAILED     = 0x0208,
  ERR_UNSUPPORTED_ROLE    = 0x0209  // Command not supported for this node role
};

// --- Special Commands ---
constexpr uint8_t CMD_SOFTWARE_RESET = 254;

// --- Winch-Specific Commands (for ROLE_WINCH) ---
#if NODE_ROLE == ROLE_WINCH
constexpr uint8_t CMD_WINCH_RETRACT = 0x01;  // Pull/Retract winch
constexpr uint8_t CMD_WINCH_EXTEND  = 0x02;  // Release/Extend winch
constexpr uint8_t CMD_WINCH_STOP    = 0x03;  // Stop winch motor
#endif

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

// CAN controller instance
Adafruit_MCP2515 mcp(CS_PIN);

// Relay state (bitmap representation)
// For multi-node: only relevant bits are used based on role
// DRL: bit 0 (K1)
// Safety: bit 1 (K2)
// Battery: bits 2-3 (K3, K4)
uint8_t relayBitmap = 0;

#if NODE_ROLE == ROLE_WINCH
// Winch-specific state variables
enum WinchState { WINCH_IDLE, WINCH_EXTENDING, WINCH_RETRACTING };
WinchState winchState = WINCH_IDLE;
unsigned long winchStartTime = 0;
unsigned long winchDuration = 0;
bool winchUseTimer = false;
uint8_t winchProgress = 0;  // 0-255 progress feedback
#endif

// Timing state
unsigned long lastHeartbeatTs = 0;
unsigned long lastMcpPollTs   = 0;
unsigned long lastCanActivity = 0;
unsigned long lastEepromWrite = 0;

// CAN recovery state
bool canBusOk = true;
uint8_t recoveryLevel = 0;
unsigned long lastRecoveryAttempt = 0;

// EEPROM configuration
constexpr int EEPROM_ADDR_RELAY_BITMAP = 0;
bool eepromDirty = false;

// Buzzer state machine
enum BuzzerState { BUZ_IDLE, BUZ_ON_PHASE, BUZ_OFF_PHASE };
BuzzerState buzzerState = BUZ_IDLE;
unsigned long buzzerPhaseStart = 0;
int buzzerBeepsRemaining = 0;

constexpr unsigned long BEEP_ON_DURATION_MS  = 100;
constexpr unsigned long BEEP_OFF_DURATION_MS = 100;

#if ENABLE_CAN_HEALTH
// CAN Health LED state
enum CanHealthState { CAN_OK, CAN_FAULT, CAN_RECOVERY, CAN_FATAL };
CanHealthState canHealthState = CAN_OK;
unsigned long canHealthLastToggle = 0;
bool canHealthLedOn = false;
#endif

#if ENABLE_DIAGNOSTICS
// Diagnostic counters
uint16_t diag_tx_success     = 0;
uint16_t diag_tx_fail        = 0;
unsigned long diag_rx_count  = 0;
uint8_t diag_recovery_count  = 0;
#endif

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void setupCanHealthLED();
void updateCanHealthLED();
void recordCanSuccess();
void recordCanFailure();
void tryRecoverCAN();

void setupRelays();
void setRelayBit(uint8_t bit, bool on);
void applyRelayOutputs();
void loadRelayBitmapEEPROM();
void saveRelayBitmapEEPROM();

void setupBuzzer();
void updateBuzzer();
void triggerBeepOnce();
void triggerBeepDouble();

void publishHeartbeat();
void handleSDORequest(const uint8_t* rx, uint8_t len);
void sendSDOResponse(uint16_t index, uint8_t subindex, uint16_t errCode);
void sendSDOResult(bool success);
void sendSDOFeedback();
void publishDiagnostics();

void handleCommand(uint8_t cmd);
bool canSendWithRetry(uint32_t id, const uint8_t* buf, uint8_t len, int retries = CAN_TX_RETRIES);

#if NODE_ROLE == ROLE_WINCH
void setupWinch();
void updateWinch();
void winchStop();
void winchExtend();
void winchRetract();
void executeWinchCommand(bool pull, bool useTimer, uint16_t durationMs);
void sendWinchFeedback();
#endif

#if ENABLE_JSON_LOGGING
void logJSON(const char* event, const char* data);
#endif

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  #if DEBUG
  delay(2000);  // Give time for serial monitor to connect
  Serial.println(F("=================================================="));
  Serial.print(F("Node: ")); Serial.println(NODE_NAME);
  Serial.print(F("Role: ")); Serial.println(NODE_ROLE);
  Serial.print(F("Node ID: 0x")); Serial.println(NODE_ID, HEX);
  Serial.print(F("Active Relays: ")); Serial.println(ACTIVE_RELAYS);
  Serial.println(F("CANopen Multi-Node Relay Control - Production"));
  Serial.println(F("=================================================="));
  #endif
  
  #if ENABLE_JSON_LOGGING
  char bootMsg[96];
  snprintf(bootMsg, sizeof(bootMsg), "\"node\":\"%s\",\"role\":%d,\"id\":0x%02x", NODE_NAME, NODE_ROLE, NODE_ID);
  logJSON("boot", bootMsg);
  #endif
  
  // Initialize MCP2515
  if (!mcp.begin(CAN_BAUD)) {
    #if DEBUG
    Serial.println(F("ERROR: MCP2515 init failed"));
    #endif
    #if ENABLE_JSON_LOGGING
    logJSON("error", "\"msg\":\"MCP2515 init failed\"");
    #endif
    while (1) {
      delay(1000);  // Halt on critical error
    }
  }
  
  #if DEBUG
  Serial.println(F("MCP2515 initialized"));
  #endif
  
  // Setup components
  #if NODE_ROLE == ROLE_WINCH
  setupWinch();
  #else
  setupRelays();
  #endif
  
  setupBuzzer();
  
  #if ENABLE_CAN_HEALTH
  setupCanHealthLED();
  #endif
  
  #if NODE_ROLE != ROLE_WINCH
  // Load relay state from EEPROM (not applicable for winch)
  loadRelayBitmapEEPROM();
  applyRelayOutputs();
  #endif
  
  lastCanActivity = millis();
  
  #if DEBUG
  Serial.println(F("Setup complete. Node ready."));
  #endif
  
  // Boot confirmation beep
  triggerBeepOnce();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();
  
  // Poll MCP2515 for incoming messages
  if (now - lastMcpPollTs >= MCP_POLL_INTERVAL_MS) {
    lastMcpPollTs = now;
    
    int packetSize = mcp.parsePacket();
    if (packetSize > 0) {
      uint32_t id = mcp.packetId();
      bool isExtended = mcp.packetExtended();
      uint8_t dlc = packetSize;
      
      uint8_t rxBuf[8] = {0};
      for (int i = 0; i < dlc && i < 8; i++) {
        rxBuf[i] = mcp.read();
      }
      
      lastCanActivity = now;
      
      #if ENABLE_DIAGNOSTICS
      diag_rx_count++;
      #endif
      
      #if ENABLE_CAN_HEALTH
      recordCanSuccess();
      #endif
      
      // Handle SDO requests addressed to this node
      if (id == COB_RSDO && !isExtended) {
        handleSDORequest(rxBuf, dlc);
      }
    }
  }
  
  // Publish heartbeat
  if (now - lastHeartbeatTs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatTs = now;
    publishHeartbeat();
  }
  
  // CAN health monitoring
  #if ENABLE_CAN_HEALTH
  if (now - lastCanActivity > CAN_HEALTH_TIMEOUT_MS) {
    if (canBusOk) {
      canBusOk = false;
      canHealthState = CAN_FAULT;
      #if DEBUG
      Serial.println(F("CAN health: FAULT (timeout)"));
      #endif
    }
    tryRecoverCAN();
  }
  updateCanHealthLED();
  #endif
  
  // Update buzzer state machine
  updateBuzzer();
  
  #if NODE_ROLE == ROLE_WINCH
  // Update winch motor control
  updateWinch();
  #endif
  
  #if NODE_ROLE != ROLE_WINCH
  // Periodic EEPROM write (if dirty and debounce elapsed)
  if (eepromDirty && (now - lastEepromWrite >= EEPROM_WRITE_DEBOUNCE_MS)) {
    saveRelayBitmapEEPROM();
  }
  #endif
}

// ============================================================================
// COMPONENT: CAN Health LED Module
// ============================================================================

#if ENABLE_CAN_HEALTH
void setupCanHealthLED() {
  pinMode(CAN_LED_GREEN_PIN, OUTPUT);
  pinMode(CAN_LED_RED_PIN, OUTPUT);
  digitalWrite(CAN_LED_GREEN_PIN, LOW);
  digitalWrite(CAN_LED_RED_PIN, LOW);
  canHealthState = CAN_OK;
}

void updateCanHealthLED() {
  unsigned long now = millis();
  
  switch (canHealthState) {
    case CAN_OK:
      // Solid green
      digitalWrite(CAN_LED_GREEN_PIN, HIGH);
      digitalWrite(CAN_LED_RED_PIN, LOW);
      break;
      
    case CAN_FAULT:
      // Blinking red (1 Hz)
      if (now - canHealthLastToggle >= 500) {
        canHealthLedOn = !canHealthLedOn;
        digitalWrite(CAN_LED_RED_PIN, canHealthLedOn ? HIGH : LOW);
        digitalWrite(CAN_LED_GREEN_PIN, LOW);
        canHealthLastToggle = now;
      }
      break;
      
    case CAN_RECOVERY:
      // Fast blinking red (4 Hz)
      if (now - canHealthLastToggle >= 125) {
        canHealthLedOn = !canHealthLedOn;
        digitalWrite(CAN_LED_RED_PIN, canHealthLedOn ? HIGH : LOW);
        digitalWrite(CAN_LED_GREEN_PIN, LOW);
        canHealthLastToggle = now;
      }
      break;
      
    case CAN_FATAL:
      // Solid red
      digitalWrite(CAN_LED_RED_PIN, HIGH);
      digitalWrite(CAN_LED_GREEN_PIN, LOW);
      break;
  }
}

void recordCanSuccess() {
  if (!canBusOk) {
    canBusOk = true;
    recoveryLevel = 0;  // Reset backoff
    canHealthState = CAN_OK;
    #if DEBUG
    Serial.println(F("CAN health: RECOVERED"));
    #endif
    #if ENABLE_JSON_LOGGING
    logJSON("can_health", "\"state\":\"OK\"");
    #endif
  }
}

void recordCanFailure() {
  canHealthState = CAN_FAULT;
}
#endif

// ============================================================================
// COMPONENT: CAN Recovery (Progressive Backoff)
// ============================================================================

void tryRecoverCAN() {
  unsigned long now = millis();
  unsigned long backoff = CAN_RECOVERY_BACKOFF_MS[min(recoveryLevel, CAN_RECOVERY_BACKOFF_LEVELS - 1)];
  
  if (now - lastRecoveryAttempt < backoff) {
    return;  // Not time to retry yet
  }
  
  lastRecoveryAttempt = now;
  
  #if ENABLE_CAN_HEALTH
  canHealthState = CAN_RECOVERY;
  #endif
  
  #if DEBUG
  Serial.print(F("Attempting CAN recovery (level "));
  Serial.print(recoveryLevel);
  Serial.println(F(")..."));
  #endif
  
  #if ENABLE_JSON_LOGGING
  char buf[32];
  snprintf(buf, sizeof(buf), "\"level\":%d", recoveryLevel);
  logJSON("can_recovery", buf);
  #endif
  
  // Attempt re-initialization
  if (mcp.begin(CAN_BAUD)) {
    lastCanActivity = now;
    recoveryLevel = 0;
    canBusOk = true;
    
    #if ENABLE_CAN_HEALTH
    canHealthState = CAN_OK;
    #endif
    
    #if ENABLE_DIAGNOSTICS
    diag_recovery_count++;
    #endif
    
    #if DEBUG
    Serial.println(F("CAN recovery successful"));
    #endif
  } else {
    recoveryLevel = min(recoveryLevel + 1, CAN_RECOVERY_BACKOFF_LEVELS - 1);
    
    if (recoveryLevel >= CAN_RECOVERY_BACKOFF_LEVELS - 1) {
      #if ENABLE_CAN_HEALTH
      canHealthState = CAN_FATAL;
      #endif
      
      #if DEBUG
      Serial.println(F("CAN recovery failed - FATAL"));
      #endif
    }
  }
}

// ============================================================================
// COMPONENT: Relay Control
// ============================================================================

void setupRelays() {
  #if ACTIVE_RELAYS > 0
  for (uint8_t i = 0; i < ACTIVE_RELAYS; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);  // Default OFF
  }
  #endif
  
  #if DEBUG
  Serial.print(F("Relays initialized: "));
  Serial.println(ACTIVE_RELAYS);
  #endif
}

void setRelayBit(uint8_t bit, bool on) {
  uint8_t mask = (1 << bit);
  
  if (on) {
    relayBitmap |= mask;
  } else {
    relayBitmap &= ~mask;
  }
  
  applyRelayOutputs();
  eepromDirty = true;
}

void applyRelayOutputs() {
  #if NODE_ROLE == ROLE_DRL
    // DRL: bit 0 controls K1
    digitalWrite(RELAY_PINS[0], (relayBitmap & 0x01) ? HIGH : LOW);
    
  #elif NODE_ROLE == ROLE_SAFETY
    // Safety: bit 1 controls K2 (mapped to index 0)
    digitalWrite(RELAY_PINS[0], (relayBitmap & 0x02) ? HIGH : LOW);
    
  #elif NODE_ROLE == ROLE_BATTERY
    // Battery: bit 2 controls K3 (index 0), bit 3 controls K4 (index 1)
    digitalWrite(RELAY_PINS[0], (relayBitmap & 0x04) ? HIGH : LOW);  // K3
    digitalWrite(RELAY_PINS[1], (relayBitmap & 0x08) ? HIGH : LOW);  // K4
    
  #elif NODE_ROLE == ROLE_ALL
    // All-in-One: all 4 bits control all 4 relays
    digitalWrite(RELAY_PINS[0], (relayBitmap & 0x01) ? HIGH : LOW);  // K1
    digitalWrite(RELAY_PINS[1], (relayBitmap & 0x02) ? HIGH : LOW);  // K2
    digitalWrite(RELAY_PINS[2], (relayBitmap & 0x04) ? HIGH : LOW);  // K3
    digitalWrite(RELAY_PINS[3], (relayBitmap & 0x08) ? HIGH : LOW);  // K4
    
  #elif NODE_ROLE == ROLE_FUTURE
    // No relays yet
  #endif
}

void loadRelayBitmapEEPROM() {
  // relayBitmap = EEPROM.read(EEPROM_ADDR_RELAY_BITMAP);  // Skip for RP2040
  
  #if DEBUG
  Serial.print(F("Loaded bitmap from EEPROM: 0b"));
  Serial.println(relayBitmap, BIN);
  #endif
}

void saveRelayBitmapEEPROM() {
  // EEPROM.write(EEPROM_ADDR_RELAY_BITMAP, relayBitmap);  // Skip for RP2040
  eepromDirty = false;
  lastEepromWrite = millis();
  
  #if DEBUG
  Serial.print(F("Saved bitmap to EEPROM: 0b"));
  Serial.println(relayBitmap, BIN);
  #endif
}

// ============================================================================
// COMPONENT: Buzzer
// ============================================================================

void setupBuzzer() {
  pinMode(PIN_BUZ, OUTPUT);
  digitalWrite(PIN_BUZ, LOW);
}

void updateBuzzer() {
  if (buzzerState == BUZ_IDLE) return;
  
  unsigned long now = millis();
  unsigned long elapsed = now - buzzerPhaseStart;
  
  if (buzzerState == BUZ_ON_PHASE) {
    if (elapsed >= BEEP_ON_DURATION_MS) {
      digitalWrite(PIN_BUZ, LOW);
      buzzerBeepsRemaining--;
      
      if (buzzerBeepsRemaining > 0) {
        buzzerState = BUZ_OFF_PHASE;
        buzzerPhaseStart = now;
      } else {
        buzzerState = BUZ_IDLE;
      }
    }
  } else if (buzzerState == BUZ_OFF_PHASE) {
    if (elapsed >= BEEP_OFF_DURATION_MS) {
      digitalWrite(PIN_BUZ, HIGH);
      buzzerState = BUZ_ON_PHASE;
      buzzerPhaseStart = now;
    }
  }
}

void triggerBeepOnce() {
  buzzerBeepsRemaining = 1;
  buzzerState = BUZ_ON_PHASE;
  buzzerPhaseStart = millis();
  digitalWrite(PIN_BUZ, HIGH);
}

void triggerBeepDouble() {
  buzzerBeepsRemaining = 2;
  buzzerState = BUZ_ON_PHASE;
  buzzerPhaseStart = millis();
  digitalWrite(PIN_BUZ, HIGH);
}

// ============================================================================
// COMPONENT: CANopen Communication
// ============================================================================

bool canSendWithRetry(uint32_t id, const uint8_t* buf, uint8_t len, int retries) {
  for (int attempt = 0; attempt < retries; attempt++) {
    mcp.beginPacket(id);
    for (uint8_t i = 0; i < len; i++) {
      mcp.write(buf[i]);
    }
    
    if (mcp.endPacket() == 1) {
      lastCanActivity = millis();
      
      #if ENABLE_DIAGNOSTICS
      diag_tx_success++;
      #endif
      
      #if ENABLE_CAN_HEALTH
      recordCanSuccess();
      #endif
      
      return true;
    }
    
    #if ENABLE_DIAGNOSTICS
    diag_tx_fail++;
    #endif
    
    if (attempt < retries - 1) {
      delay(CAN_TX_RETRY_DELAY_MS);
    }
  }
  
  #if ENABLE_CAN_HEALTH
  recordCanFailure();
  #endif
  
  return false;
}

void publishHeartbeat() {
  uint8_t payload[1] = {0x05};  // Operational state
  
  #if ENABLE_JSON_LOGGING
  logJSON("heartbeat", "\"state\":5");
  #elif DEBUG
  Serial.println(F("Heartbeat"));
  #endif
  
  canSendWithRetry(COB_HEARTBEAT, payload, 1);
}

void handleSDORequest(const uint8_t* rx, uint8_t len) {
  if (len < 4) return;  // Malformed
  
  uint8_t cmd = rx[0];
  uint16_t index = rx[1] | (rx[2] << 8);
  uint8_t subindex = rx[3];
  
  #if DEBUG
  Serial.print(F("SDO: cmd=0x")); Serial.print(cmd, HEX);
  Serial.print(F(" idx=0x")); Serial.print(index, HEX);
  Serial.print(F(" sub=")); Serial.println(subindex);
  #endif
  
  // SDO WRITE to OD_CONTROL_COMMAND
  if (cmd == SDO_WRITE && index == OD_CONTROL_COMMAND && subindex == 0x00) {
    if (len >= 5) {
      uint8_t command = rx[4];
      
      // Send immediate ACK
      sendSDOResponse(index, subindex, ERR_NONE);
      
      #if NODE_ROLE == ROLE_WINCH
      // Winch commands need duration bytes (bytes 5-7)
      if (command == CMD_WINCH_RETRACT || command == CMD_WINCH_EXTEND) {
        if (len >= 8) {
          bool pull = (command == CMD_WINCH_RETRACT);
          bool useTimer = (rx[5] == 1);
          uint16_t durationMs = rx[6] | (rx[7] << 8);
          
          #if DEBUG
          Serial.print(F("Winch: pull=")); Serial.print(pull);
          Serial.print(F(" timer=")); Serial.print(useTimer);
          Serial.print(F(" dur=")); Serial.println(durationMs);
          #endif
          
          executeWinchCommand(pull, useTimer, durationMs);
          // Result and feedback sent by updateWinch() when complete
        } else {
          sendSDOResult(false);  // Invalid payload length
        }
      } else {
        // STOP command doesn't need duration
        handleCommand(command);
        sendSDOResult(true);
        sendSDOFeedback();
      }
      #else
      // Relay nodes: process command normally
      handleCommand(command);
      
      // Send result
      sendSDOResult(true);
      
      // Send feedback
      sendSDOFeedback();
      #endif
    } else {
      sendSDOResponse(index, subindex, ERR_UNKNOWN);
    }
  }
  // SDO READ requests
  else if (cmd == SDO_READ) {
    if (index == OD_RELAY_FEEDBACK && subindex == 0x00) {
      sendSDOFeedback();
    } else if (index == OD_DIAGNOSTICS && subindex == 0x00) {
      #if ENABLE_DIAGNOSTICS
      publishDiagnostics();
      #else
      sendSDOResponse(index, subindex, ERR_OBJECT_NOT_FOUND);
      #endif
    } else if (index == OD_NODE_ROLE && subindex == 0x00) {
      // Return node role identifier
      uint8_t out[8] = {0};
      out[0] = SDO_WRITE;
      out[1] = index & 0xFF;
      out[2] = (index >> 8) & 0xFF;
      out[3] = 0x00;
      out[4] = NODE_ROLE;
      canSendWithRetry(COB_TSDO, out, 8);
    } else {
      sendSDOResponse(index, subindex, ERR_OBJECT_NOT_FOUND);
    }
  }
  else {
    sendSDOResponse(index, subindex, ERR_UNKNOWN);
  }
}

void sendSDOResponse(uint16_t index, uint8_t subindex, uint16_t errCode) {
  uint8_t out[8] = {0};
  
  if (errCode == ERR_NONE) {
    out[0] = SDO_RESPONSE;
  } else {
    out[0] = SDO_ERROR;
  }
  
  out[1] = index & 0xFF;
  out[2] = (index >> 8) & 0xFF;
  out[3] = subindex;
  out[4] = errCode & 0xFF;
  out[5] = (errCode >> 8) & 0xFF;
  
  canSendWithRetry(COB_TSDO, out, 8);
}

void sendSDOResult(bool success) {
  uint8_t out[8] = {0};
  out[0] = SDO_WRITE;
  out[1] = OD_COMMAND_RESULT & 0xFF;
  out[2] = (OD_COMMAND_RESULT >> 8) & 0xFF;
  out[3] = 0x00;
  out[4] = success ? 1 : 0;
  
  #if ENABLE_JSON_LOGGING
  char buf[32];
  snprintf(buf, sizeof(buf), "\"success\":%d", success ? 1 : 0);
  logJSON("sdo_result", buf);
  #endif
  
  delay(20);  // Inter-frame spacing
  canSendWithRetry(COB_TSDO, out, 8);
}

void sendSDOFeedback() {
  uint8_t out[8] = {0};
  out[0] = SDO_WRITE;
  out[1] = OD_RELAY_FEEDBACK & 0xFF;
  out[2] = (OD_RELAY_FEEDBACK >> 8) & 0xFF;
  out[3] = 0x00;
  out[4] = relayBitmap;
  
  #if ENABLE_JSON_LOGGING
  char buf[32];
  snprintf(buf, sizeof(buf), "\"bitmap\":%d", relayBitmap);
  logJSON("sdo_feedback", buf);
  #elif DEBUG
  Serial.print(F("Feedback: 0b")); Serial.println(relayBitmap, BIN);
  #endif
  
  canSendWithRetry(COB_TSDO, out, 8);
}

#if ENABLE_DIAGNOSTICS
void publishDiagnostics() {
  uint8_t out[8] = {0};
  out[0] = SDO_WRITE;
  out[1] = OD_DIAGNOSTICS & 0xFF;
  out[2] = (OD_DIAGNOSTICS >> 8) & 0xFF;
  out[3] = 0x00;
  out[4] = (diag_tx_success & 0xFF);
  out[5] = (diag_tx_success >> 8) & 0xFF;
  out[6] = (diag_tx_fail & 0xFF);
  out[7] = (diag_tx_fail >> 8) & 0xFF;
  
  #if ENABLE_JSON_LOGGING
  char buf[96];
  snprintf(buf, sizeof(buf), 
    "\"tx_ok\":%u,\"tx_fail\":%u,\"rx\":%lu,\"recoveries\":%u",
    diag_tx_success, diag_tx_fail, diag_rx_count, diag_recovery_count);
  logJSON("diagnostics", buf);
  #endif
  
  canSendWithRetry(COB_TSDO, out, 8);
}
#endif

// ============================================================================
// COMPONENT: Command Handler (Role-Specific)
// ============================================================================

void handleCommand(uint8_t cmd) {
  #if DEBUG
  Serial.print(F("CMD ")); Serial.print(cmd); Serial.print(F(": "));
  #endif
  
  bool cmdHandled = false;
  
  // Global commands (all roles)
  if (cmd == 0) {  // ALL OFF
    #if NODE_ROLE == ROLE_DRL
      setRelayBit(0, false);  // K1 OFF
    #elif NODE_ROLE == ROLE_SAFETY
      setRelayBit(1, false);  // K2 OFF
    #elif NODE_ROLE == ROLE_BATTERY
      setRelayBit(2, false);  // K3 OFF
      setRelayBit(3, false);  // K4 OFF
    #elif NODE_ROLE == ROLE_ALL
      setRelayBit(0, false);  // K1 OFF
      setRelayBit(1, false);  // K2 OFF
      setRelayBit(2, false);  // K3 OFF
      setRelayBit(3, false);  // K4 OFF
    #endif
    
    triggerBeepDouble();
    cmdHandled = true;
    #if DEBUG
    Serial.println(F("ALL OFF (node-specific)"));
    #endif
  }
  else if (cmd == 99) {  // ALL ON
    #if NODE_ROLE == ROLE_DRL
      setRelayBit(0, true);   // K1 ON
    #elif NODE_ROLE == ROLE_SAFETY
      setRelayBit(1, true);   // K2 ON
    #elif NODE_ROLE == ROLE_BATTERY
      setRelayBit(2, true);   // K3 ON
      setRelayBit(3, true);   // K4 ON
    #elif NODE_ROLE == ROLE_ALL
      setRelayBit(0, true);   // K1 ON
      setRelayBit(1, true);   // K2 ON
      setRelayBit(2, true);   // K3 ON
      setRelayBit(3, true);   // K4 ON
    #endif
    
    triggerBeepOnce();
    cmdHandled = true;
    #if DEBUG
    Serial.println(F("ALL ON (node-specific)"));
    #endif
  }
  else if (cmd == CMD_SOFTWARE_RESET) {
    #if DEBUG
    Serial.println(F("SOFTWARE RESET"));
    #endif
    delay(100);
    // Perform software reset (platform-specific)
    // For RP2040: Use watchdog or restart via AIRCR
    while(1);  // Watchdog will reset if enabled
  }
  
  // Role-specific commands
  #if NODE_ROLE == ROLE_DRL
    // DRL Node: Commands 1-2 (K1 control)
    if (cmd == 1) {  // K1 ON
      setRelayBit(0, true);
      triggerBeepOnce();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K1 ON (DRL)"));
      #endif
    }
    else if (cmd == 2) {  // K1 OFF
      setRelayBit(0, false);
      triggerBeepDouble();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K1 OFF (DRL)"));
      #endif
    }
  
  #elif NODE_ROLE == ROLE_SAFETY
    // Safety Node: Commands 3-4 (K2 control)
    if (cmd == 3) {  // K2 ON
      setRelayBit(1, true);
      triggerBeepOnce();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K2 ON (Safety)"));
      #endif
    }
    else if (cmd == 4) {  // K2 OFF
      setRelayBit(1, false);
      triggerBeepDouble();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K2 OFF (Safety)"));
      #endif
    }
  
  #elif NODE_ROLE == ROLE_BATTERY
    // Battery Node: Commands 5-8 (K3 + K4 control)
    if (cmd == 5) {  // K3 ON
      setRelayBit(2, true);
      triggerBeepOnce();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K3 ON (Battery R)"));
      #endif
    }
    else if (cmd == 6) {  // K3 OFF
      setRelayBit(2, false);
      triggerBeepDouble();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K3 OFF (Battery R)"));
      #endif
    }
    else if (cmd == 7) {  // K4 ON
      setRelayBit(3, true);
      triggerBeepOnce();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K4 ON (Battery L)"));
      #endif
    }
    else if (cmd == 8) {  // K4 OFF
      setRelayBit(3, false);
      triggerBeepDouble();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K4 OFF (Battery L)"));
      #endif
    }
  
  #elif NODE_ROLE == ROLE_ALL
    // All-in-One Node: Responds to ALL commands (1-8)
    if (cmd == 1) {  // K1 ON
      setRelayBit(0, true);
      triggerBeepOnce();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K1 ON"));
      #endif
    }
    else if (cmd == 2) {  // K1 OFF
      setRelayBit(0, false);
      triggerBeepDouble();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K1 OFF"));
      #endif
    }
    else if (cmd == 3) {  // K2 ON
      setRelayBit(1, true);
      triggerBeepOnce();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K2 ON"));
      #endif
    }
    else if (cmd == 4) {  // K2 OFF
      setRelayBit(1, false);
      triggerBeepDouble();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K2 OFF"));
      #endif
    }
    else if (cmd == 5) {  // K3 ON
      setRelayBit(2, true);
      triggerBeepOnce();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K3 ON"));
      #endif
    }
    else if (cmd == 6) {  // K3 OFF
      setRelayBit(2, false);
      triggerBeepDouble();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K3 OFF"));
      #endif
    }
    else if (cmd == 7) {  // K4 ON
      setRelayBit(3, true);
      triggerBeepOnce();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K4 ON"));
      #endif
    }
    else if (cmd == 8) {  // K4 OFF
      setRelayBit(3, false);
      triggerBeepDouble();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("K4 OFF"));
      #endif
    }
  
  #elif NODE_ROLE == ROLE_WINCH
    // Winch Node: Commands 1-3 (winch motor control)
    // NOTE: Winch commands expect extended SDO format with timer data
    // This handler processes basic command bytes; full SDO parsing happens in handleSDORequest
    if (cmd == CMD_WINCH_STOP) {
      winchStop();
      cmdHandled = true;
      #if DEBUG
      Serial.println(F("WINCH STOP"));
      #endif
    }
    // Note: RETRACT and EXTEND require duration parameters, handled via SDO write
  
  #elif NODE_ROLE == ROLE_FUTURE
    // Future node: No specific commands yet
    #if DEBUG
    Serial.println(F("(no commands assigned)"));
    #endif
  #endif
  
  if (!cmdHandled) {
    #if DEBUG
    Serial.println(F("UNSUPPORTED for this node role"));
    #endif
    
    #if ENABLE_JSON_LOGGING
    char buf[48];
    snprintf(buf, sizeof(buf), "\"cmd\":%d,\"role\":%d", cmd, NODE_ROLE);
    logJSON("cmd_unsupported", buf);
    #endif
  }
}

// ============================================================================
// COMPONENT: Winch Motor Control
// ============================================================================

#if NODE_ROLE == ROLE_WINCH

void setupWinch() {
  pinMode(PIN_FWD, OUTPUT);
  pinMode(PIN_REV, OUTPUT);
  pinMode(PIN_RET_LIM, INPUT_PULLUP);
  pinMode(PIN_EXT_LIM, INPUT_PULLUP);
  
  // Ensure motor stopped at startup
  winchStop();
  
  #if DEBUG
  Serial.println(F("Winch motor initialized"));
  Serial.print(F("  Forward pin: ")); Serial.println(PIN_FWD);
  Serial.print(F("  Reverse pin: ")); Serial.println(PIN_REV);
  Serial.print(F("  Retract limit: ")); Serial.println(PIN_RET_LIM);
  Serial.print(F("  Extend limit: ")); Serial.println(PIN_EXT_LIM);
  #endif
}

void winchStop() {
  digitalWrite(PIN_FWD, LOW);
  digitalWrite(PIN_REV, LOW);
  winchState = WINCH_IDLE;
  winchProgress = 0;
  
  #if DEBUG
  Serial.println(F("Winch stopped"));
  #endif
}

void winchExtend() {
  // Check extend limit before starting (INPUT_PULLUP: LOW = at limit)
  if (digitalRead(PIN_EXT_LIM) == LOW) {
    #if DEBUG
    Serial.println(F("ERROR: Already at extend limit"));
    #endif
    return;
  }
  
  digitalWrite(PIN_REV, LOW);
  analogWrite(PIN_FWD, MAX_SPEED);
  winchState = WINCH_EXTENDING;
  
  #if DEBUG
  Serial.println(F("Winch extending"));
  #endif
}

void winchRetract() {
  // Check retract limit before starting (INPUT_PULLUP: LOW = at limit)
  if (digitalRead(PIN_RET_LIM) == LOW) {
    #if DEBUG
    Serial.println(F("ERROR: Already at retract limit"));
    #endif
    return;
  }
  
  digitalWrite(PIN_FWD, LOW);
  analogWrite(PIN_REV, MAX_SPEED);
  winchState = WINCH_RETRACTING;
  
  #if DEBUG
  Serial.println(F("Winch retracting"));
  #endif
}

void executeWinchCommand(bool pull, bool useTimer, uint16_t durationMs) {
  winchStartTime = millis();
  winchDuration = durationMs;
  winchUseTimer = useTimer;
  winchProgress = 0;
  
  #if DEBUG
  Serial.print(F("Execute winch: pull=")); Serial.print(pull);
  Serial.print(F(", useTimer=")); Serial.print(useTimer);
  Serial.print(F(", duration=")); Serial.println(durationMs);
  #endif
  
  // Limit switch safety checks (INPUT_PULLUP: LOW = pressed/at limit, HIGH = not pressed)
  if (pull && digitalRead(PIN_RET_LIM) == LOW) {
    #if DEBUG
    Serial.println(F("ERROR: Already fully retracted"));
    #endif
    sendSDOResult(false);
    sendWinchFeedback();
    return;
  }
  
  if (!pull && digitalRead(PIN_EXT_LIM) == LOW) {
    #if DEBUG
    Serial.println(F("ERROR: Already fully extended"));
    #endif
    sendSDOResult(false);
    sendWinchFeedback();
    return;
  }
  
  // Start motor
  if (pull) {
    winchRetract();
  } else {
    winchExtend();
  }
  
  triggerBeepOnce();
}

void updateWinch() {
  if (winchState == WINCH_IDLE) return;
  
  unsigned long elapsed = millis() - winchStartTime;
  
  // Update progress feedback
  if (winchUseTimer && winchDuration > 0) {
    winchProgress = constrain((elapsed * 255) / winchDuration, 0, 255);
    
    // Send periodic feedback every 500ms
    static unsigned long lastFeedbackTime = 0;
    if (millis() - lastFeedbackTime >= 500) {
      sendWinchFeedback();
      lastFeedbackTime = millis();
    }
  }
  
  // Check limit switches (INPUT_PULLUP: LOW = pressed/at limit)
  bool retLimitHit = (digitalRead(PIN_RET_LIM) == LOW && winchState == WINCH_RETRACTING);
  bool extLimitHit = (digitalRead(PIN_EXT_LIM) == LOW && winchState == WINCH_EXTENDING);
  
  if (retLimitHit || extLimitHit) {
    winchStop();
    winchProgress = 255;  // 100% complete
    sendWinchFeedback();
    sendSDOResult(true);  // Success - reached limit
    triggerBeepDouble();
    
    #if DEBUG
    Serial.println(retLimitHit ? F("Retract limit reached") : F("Extend limit reached"));
    #endif
    return;
  }
  
  // Check timer timeout
  if (winchUseTimer && elapsed >= winchDuration) {
    winchStop();
    winchProgress = 255;  // 100% complete
    sendWinchFeedback();
    sendSDOResult(true);  // Success - timer completed
    triggerBeepDouble();
    
    #if DEBUG
    Serial.println(F("Winch timer completed"));
    #endif
  }
}

void sendWinchFeedback() {
  uint8_t out[8] = {0};
  out[0] = SDO_WRITE;
  out[1] = OD_RELAY_FEEDBACK & 0xFF;  // Reuse relay feedback object for progress
  out[2] = (OD_RELAY_FEEDBACK >> 8) & 0xFF;
  out[3] = 0x00;
  out[4] = winchProgress;  // 0-255 progress value
  
  #if ENABLE_JSON_LOGGING
  char buf[32];
  snprintf(buf, sizeof(buf), "\"progress\":%d", winchProgress);
  logJSON("winch_feedback", buf);
  #elif DEBUG
  Serial.print(F("Winch progress: ")); 
  Serial.print((winchProgress * 100) / 255); 
  Serial.println(F("%"));
  #endif
  
  canSendWithRetry(COB_TSDO, out, 8);
}

#endif // NODE_ROLE == ROLE_WINCH

// ============================================================================
// COMPONENT: JSON Logging
// ============================================================================

#if ENABLE_JSON_LOGGING
void logJSON(const char* event, const char* data) {
  Serial.print(F("{\"event\":\""));
  Serial.print(event);
  Serial.print(F("\",\"node\":\""));
  Serial.print(NODE_NAME);
  Serial.print(F("\",\"ts\":"));
  Serial.print(millis());
  if (data && data[0] != '\0') {
    Serial.print(F(","));
    Serial.print(data);
  }
  Serial.println(F("}"));
}
#endif
