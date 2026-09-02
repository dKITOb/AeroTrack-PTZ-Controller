// ============================================
// BLOCK 1: ANTENNA CONTROLLER
// ESP32-S3-N16R8 + W5500 ETHERNET
// Modular Architecture with Module Management
// ============================================

#define BLOCK_1_CONTROLLER
#include "../config.h"

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ============================================
// ===== GLOBAL OBJECTS =====
// ============================================

Servo servoPan;
Servo servoTilt;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(ETH_LOCAL_IP_1, ETH_LOCAL_IP_2, ETH_LOCAL_IP_3, ETH_LOCAL_IP_4);
IPAddress remote_ip(ETH_REMOTE_IP_1, ETH_REMOTE_IP_2, ETH_REMOTE_IP_3, ETH_REMOTE_IP_4);
EthernetUDP udp;
char packetBuffer[256];

// ============================================
// ===== MODULE STATES =====
// ============================================

struct ModuleStates {
  bool servo_control = MODULE_SERVO_CONTROL;
  bool compass = MODULE_COMPASS;
  bool frequency = MODULE_FREQUENCY;
  bool telemetry = MODULE_TELEMETRY;
};

ModuleStates modules;

// ============================================
// ===== GLOBAL VARIABLES =====
// ============================================

int panAngle = 0;
int tiltAngle = 0;
int compassAngle = 0;
bool compassOk = false;
bool calibrated = false;
int offsetX = 0, offsetY = 0, offsetZ = 0;

unsigned long impulseEndTime = 0;
bool impulseActive = false;

unsigned long lastCommandTime = 0;
unsigned long lastHeartbeatCheck = 0;
bool watchdogTriggered = false;

// ============================================
// ===== MODULE: SERVO CONTROL =====
// ============================================

class ServoControlModule {
public:
  bool enabled = true;
  
  void init() {
    servoPan.attach(PIN_PAN_SERVO);
    servoTilt.attach(PIN_TILT_SERVO);
    servoPan.write(90);
    servoTilt.write(90);
    Serial.println("[✓] Servo Control Module initialized");
  }
  
  void setPan(int angle) {
    if (!enabled) return;
    panAngle = constrain(angle, SERVO_PAN_MIN, SERVO_PAN_MAX);
    servoPan.write(90 + panAngle);
  }
  
  void setTilt(int angle) {
    if (!enabled) return;
    tiltAngle = constrain(angle, SERVO_TILT_MIN, SERVO_TILT_MAX);
    servoTilt.write(90 + tiltAngle);
  }
  
  void setPanTilt(int pan, int tilt) {
    setPan(pan);
    setTilt(tilt);
  }
};

ServoControlModule servoModule;

// ============================================
// ===== MODULE: COMPASS =====
// ============================================

class CompassModule {
public:
  bool enabled = true;
  
  void init() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    EEPROM.begin(EEPROM_SIZE);
    
    if (!mag.begin()) {
      compassOk = false;
      Serial.println("[✗] Compass not found!");
    } else {
      compassOk = true;
      Serial.println("[✓] Compass found (HMC5883L)");
      
      byte flag;
      EEPROM.get(12, flag);
      if (flag == EEPROM_MAGIC) {
        EEPROM.get(0, offsetX);
        EEPROM.get(4, offsetY);
        EEPROM.get(8, offsetZ);
        calibrated = true;
        Serial.println("[✓] Calibration loaded");
      }
    }
  }
  
  void calibrate() {
    if (!compassOk || !enabled) return;
    
    sensors_event_t event;
    int minX = 32767, maxX = -32768;
    int minY = 32767, maxY = -32768;
    int minZ = 32767, maxZ = -32768;
    
    Serial.println("🔄 Compass calibration... Rotate sensor for 10 seconds");
    unsigned long start = millis();
    
    while (millis() - start < 10000) {
      mag.getEvent(&event);
      minX = min(minX, (int)event.magnetic.x);
      maxX = max(maxX, (int)event.magnetic.x);
      minY = min(minY, (int)event.magnetic.y);
      maxY = max(maxY, (int)event.magnetic.y);
      minZ = min(minZ, (int)event.magnetic.z);
      maxZ = max(maxZ, (int)event.magnetic.z);
      delay(10);
    }
    
    offsetX = (minX + maxX) / 2;
    offsetY = (minY + maxY) / 2;
    offsetZ = (minZ + maxZ) / 2;
    
    EEPROM.put(0, offsetX);
    EEPROM.put(4, offsetY);
    EEPROM.put(8, offsetZ);
    EEPROM.put(12, (byte)EEPROM_MAGIC);
    EEPROM.commit();
    
    calibrated = true;
    Serial.println("✅ Calibration complete!");
  }
  
  void update() {
    if (!compassOk || !calibrated || !enabled) return;
    
    sensors_event_t event;
    mag.getEvent(&event);
    
    float fx = event.magnetic.x - offsetX;
    float fy = event.magnetic.y - offsetY;
    float heading = atan2(fy, fx) * 180.0 / PI;
    if (heading < 0) heading += 360;
    compassAngle = (int)heading;
  }
};

CompassModule compassModule;

// ============================================
// ===== MODULE: IMPULSE CONTROL =====
// ============================================

void sendChannelImpulse(int direction) {
  if (direction > 0) {
    digitalWrite(PIN_CHANNEL_FWD, HIGH);
    digitalWrite(PIN_CHANNEL_BACK, LOW);
    Serial.println("[IMPULSE] Forward!");
  } else {
    digitalWrite(PIN_CHANNEL_FWD, LOW);
    digitalWrite(PIN_CHANNEL_BACK, HIGH);
    Serial.println("[IMPULSE] Backward!");
  }
  
  impulseActive = true;
  impulseEndTime = millis() + IMPULSE_DURATION;
}

void updateImpulse() {
  if (impulseActive && millis() >= impulseEndTime) {
    digitalWrite(PIN_CHANNEL_FWD, LOW);
    digitalWrite(PIN_CHANNEL_BACK, LOW);
    impulseActive = false;
  }
}

// ============================================
// ===== WATCHDOG =====
// ============================================

void checkWatchdog() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastHeartbeatCheck < HEARTBEAT_INTERVAL) return;
  lastHeartbeatCheck = currentTime;
  
  if (currentTime - lastCommandTime > WATCHDOG_TIMEOUT) {
    if (!watchdogTriggered) {
      watchdogTriggered = true;
      Serial.println("⚠️ CONNECTION LOST!");
    }
  } else {
    watchdogTriggered = false;
  }
}

// ============================================
// ===== PROTOCOL: COMMAND PARSING =====
// ============================================

void parseCommand(const char* cmd) {
  Serial.printf("[RX] %s\n", cmd);
  
  // === SERVO COMMAND: M<pan>,<tilt>,<calflag> ===
  if (cmd[0] == 'M' && modules.servo_control) {
    int pan, tilt, calFlag;
    if (sscanf(cmd, "M%d,%d,%d", &pan, &tilt, &calFlag) == 3) {
      servoModule.setPanTilt(pan, tilt);
      Serial.printf("[SERVO] Pan=%d°, Tilt=%d°\n", panAngle, tiltAngle);
      
      if (calFlag && modules.compass) {
        compassModule.calibrate();
      }
    }
  }
  
  // === CHANNEL COMMAND: C<direction> ===
  if (cmd[0] == 'C') {
    int direction;
    if (sscanf(cmd, "C%d", &direction) == 1) {
      if (direction == 1 || direction == -1) {
        sendChannelImpulse(direction);
      }
    }
  }
  
  // === MODULE MANAGEMENT: MOD<module>,<state> ===
  if (strncmp(cmd, "MOD", 3) == 0) {
    char module_name[32];
    int state;
    if (sscanf(cmd, "MOD%31[^,],%d", module_name, &state) == 2) {
      if (strcmp(module_name, "SERVO") == 0) modules.servo_control = (state != 0);
      else if (strcmp(module_name, "COMPASS") == 0) modules.compass = (state != 0);
      else if (strcmp(module_name, "FREQ") == 0) modules.frequency = (state != 0);
      else if (strcmp(module_name, "TELEM") == 0) modules.telemetry = (state != 0);
      
      Serial.printf("[MOD] %s = %d\n", module_name, state);
    }
  }
}

// ============================================
// ===== PROTOCOL: STATUS TRANSMISSION =====
// ============================================

void sendStatusPacket() {
  char packet[128];
  snprintf(packet, sizeof(packet), "A=%d,%d,C=%d,OK=%d,Cal=%d",
    panAngle, tiltAngle, compassAngle, 
    compassOk ? 1 : 0, calibrated ? 1 : 0);
  
  udp.beginPacket(remote_ip, ETH_REMOTE_PORT);
  udp.write((const uint8_t*)packet, strlen(packet));
  udp.endPacket();
}

// ============================================
// ===== SETUP =====
// ============================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║  BLOCK 1: ANTENNA CONTROLLER WITH W5500  ║");
  Serial.println("║       Modular Architecture v1.0          ║");
  Serial.println("╚═══════════════════════════════════════════╝");
  Serial.println();
  
  // === IMPULSE PINS ===
  pinMode(PIN_CHANNEL_FWD, OUTPUT);
  pinMode(PIN_CHANNEL_BACK, OUTPUT);
  digitalWrite(PIN_CHANNEL_FWD, LOW);
  digitalWrite(PIN_CHANNEL_BACK, LOW);
  
  // === ETHERNET ===
  Ethernet.init(PIN_W5500_CS);
  if (Ethernet.begin(mac) == 0) {
    Serial.println("[!] Ethernet DHCP failed, using static IP");
    Ethernet.begin(mac, ip);
  }
  
  Serial.printf("[✓] Ethernet IP: %s\n", Ethernet.localIP().toString().c_str());
  udp.begin(ETH_LOCAL_PORT);
  Serial.printf("[✓] UDP listening on port %d\n", ETH_LOCAL_PORT);
  
  // === INITIALIZE MODULES ===
  servoModule.init();
  compassModule.init();
  
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║         SYSTEM READY                      ║");
  Serial.println("╚═══════════════════════════════════════════╝");
  Serial.println();
  Serial.printf("📍 Local IP: %s\n", Ethernet.localIP().toString().c_str());
  Serial.printf("📍 Remote IP: %s\n", remote_ip.toString().c_str());
  
  lastCommandTime = millis();
  lastHeartbeatCheck = millis();
}

// ============================================
// ===== MAIN LOOP =====
// ============================================

void loop() {
  // === WATCHDOG ===
  checkWatchdog();
  
  // === UPDATE MODULES ===
  updateImpulse();
  compassModule.update();
  
  // === RECEIVE COMMANDS ===
  int packetSize = udp.parsePacket();
  if (packetSize) {
    lastCommandTime = millis();
    watchdogTriggered = false;
    
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      parseCommand(packetBuffer);
    }
  }
  
  // === SEND STATUS ===
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 100) {
    sendStatusPacket();
    lastSend = millis();
  }
  
  delay(10);
}
