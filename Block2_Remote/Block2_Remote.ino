// ============================================
// BLOCK 2: REMOTE CONTROL WITH MENU
// ESP32-S3-N16R8 + W5500 ETHERNET + TFT ST7735
// Modular Architecture with On-Screen Menu Button
// ============================================

#define BLOCK_2_REMOTE
#include "../config.h"

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ============================================
// ===== GLOBAL OBJECTS =====
// ============================================

Adafruit_ST7735 tft = Adafruit_ST7735(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_MOSI, PIN_TFT_SCLK, PIN_TFT_RST);

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};
IPAddress ip(ETH_LOCAL_IP_1, ETH_LOCAL_IP_2, ETH_LOCAL_IP_3, ETH_LOCAL_IP_4);
IPAddress controller_ip(ETH_CONTROLLER_IP_1, ETH_CONTROLLER_IP_2, ETH_CONTROLLER_IP_3, ETH_CONTROLLER_IP_4);
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

// Servo
int servoX = 0, servoY = 0;
int targetX = 0, targetY = 0;
float smoothX = 0, smoothY = 0;
const float SMOOTHING_FACTOR = 0.3f;
unsigned long lastStepTimeX = 0, lastStepTimeY = 0;

// Compass
int compassAngle = 0;
bool compassOk = false;
bool calibrated = false;

// Joystick
int joystickCenterX = 2048, joystickCenterY = 2048;
bool joystickCalibrated = false;

// Channels
int currentChannel = 0;

// Mode
int currentMode = MODE_BOTH;

// UI State
bool mainMenuActive = false;
bool subMenuActive = false;
bool moduleMenuActive = false;
int menuSelection = 0;
int moduleSelection = 0;
unsigned long lastMenuActivity = 0;

// Link status
unsigned long lastResponseTime = 0;
bool linkLost = false;

// Diagnostics
bool diagnosticMode = false;
bool calibrationActive = false;
unsigned long calibrationStart = 0;
bool calRequest = false;

// Screen update
bool needUpdate = true;
int lastServoX = -999, lastServoY = -999, lastCompass = -999;
bool lastCompassOk = false;
int lastChannel = -1, lastMode = -1;
bool lastLinkStatus = false;

// ============================================
// ===== UI: BUTTONS ON SCREEN =====
// ============================================

struct ScreenButton {
  int x, y, w, h;
  const char* label;
  uint16_t color;
  bool pressed;
  
  void draw(Adafruit_ST7735& display, bool highlight = false) {
    uint16_t borderColor = highlight ? ST7735_WHITE : ST7735_YELLOW;
    uint16_t bgColor = highlight ? ST7735_YELLOW : ST7735_BLACK;
    uint16_t textColor = highlight ? ST7735_BLACK : ST7735_WHITE;
    
    display.fillRect(x, y, w, h, bgColor);
    display.drawRect(x, y, w, h, borderColor);
    display.setTextColor(textColor);
    display.setTextSize(1);
    
    int textX = x + 3;
    int textY = y + 3;
    display.setCursor(textX, textY);
    display.print(label);
  }
  
  bool contains(int px, int py) {
    return px >= x && px <= x + w && py >= y && py <= y + h;
  }
};

// Menu button on screen (top right)
ScreenButton btnMenu = {140, 0, 28, 14, "MNU", ST7735_YELLOW, false};
// Module manager button
ScreenButton btnModules = {140, 104, 28, 14, "MOD", ST7735_CYAN, false};

// ============================================
// ===== MODULE: JOYSTICK CALIBRATION =====
// ============================================

void calibrateJoystick() {
  Serial.println("[CALIB] Joystick calibration...");
  
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_YELLOW);
  tft.setCursor(10, 30);
  tft.print("CALIBRATE");
  tft.setTextSize(1);
  tft.setCursor(10, 60);
  tft.print("Don't touch!");
  
  long sumX = 0, sumY = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    sumX += analogRead(PIN_VRX);
    sumY += analogRead(PIN_VRY);
    delay(5);
  }
  
  joystickCenterX = sumX / samples;
  joystickCenterY = sumY / samples;
  joystickCalibrated = true;
  
  Serial.printf("[CALIB] Center: X=%d, Y=%d\n", joystickCenterX, joystickCenterY);
  
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_GREEN);
  tft.setCursor(10, 30);
  tft.print("CALIB OK!");
  delay(1000);
}

// ============================================
// ===== MODULE: JOYSTICK READING =====
// ============================================

void readJoystickServo() {
  if (!joystickCalibrated) return;
  
  int rawX = analogRead(PIN_VRX);
  int rawY = analogRead(PIN_VRY);
  
  int diffX = rawX - joystickCenterX;
  int diffY = rawY - joystickCenterY;
  
  if (abs(diffX) < JOYSTICK_DEADZONE) diffX = 0;
  if (abs(diffY) < JOYSTICK_DEADZONE) diffY = 0;
  
  if (currentMode == MODE_BOTH) {
    if (diffX == 0 && diffY == 0) {
      targetX = round(smoothX);
      targetY = round(smoothY);
      return;
    }
    if (diffX != 0) targetX += (diffX > 0 ? STEP_X : -STEP_X);
    if (diffY != 0) targetY += (diffY > 0 ? STEP_Y : -STEP_Y);
    
  } else if (currentMode == MODE_TILT) {
    if (diffY == 0) {
      targetY = round(smoothY);
      return;
    }
    if (diffY != 0) targetY += (diffY > 0 ? STEP_Y : -STEP_Y);
    targetX = 0;
    
  } else if (currentMode == MODE_PAN) {
    if (diffX == 0) {
      targetX = round(smoothX);
      return;
    }
    if (diffX != 0) targetX += (diffX > 0 ? STEP_X : -STEP_X);
    targetY = 0;
  }
  
  targetX = constrain(targetX, -180, 180);
  targetY = constrain(targetY, -30, 60);
}

void readJoystickChannel() {
  if (!joystickCalibrated) return;
  
  static unsigned long lastSwitch = 0;
  const unsigned long switchDelay = 300;
  
  int rawX = analogRead(PIN_VRX);
  int diffX = rawX - joystickCenterX;
  
  if (abs(diffX) < CHANNEL_SWITCH_THRESHOLD) return;
  
  if (millis() - lastSwitch > switchDelay) {
    int direction = 0;
    
    if (diffX > 0) {
      currentChannel = (currentChannel + 1) % NUM_CHANNELS;
      direction = 1;
    } else {
      currentChannel = (currentChannel - 1 + NUM_CHANNELS) % NUM_CHANNELS;
      direction = -1;
    }
    
    sendChannelCommand(direction);
    lastSwitch = millis();
    needUpdate = true;
  }
}

void updateServoPosition() {
  unsigned long currentTime = millis();
  
  if (targetX != smoothX && currentTime - lastStepTimeX >= SPEED_DELAY) {
    if (smoothX < targetX) smoothX += STEP_X;
    else if (smoothX > targetX) smoothX -= STEP_X;
    lastStepTimeX = currentTime;
  }
  
  if (targetY != smoothY && currentTime - lastStepTimeY >= SPEED_DELAY) {
    if (smoothY < targetY) smoothY += STEP_Y;
    else if (smoothY > targetY) smoothY -= STEP_Y;
    lastStepTimeY = currentTime;
  }
  
  static float currentSmoothX = 0, currentSmoothY = 0;
  currentSmoothX += (smoothX - currentSmoothX) * SMOOTHING_FACTOR;
  currentSmoothY += (smoothY - currentSmoothY) * SMOOTHING_FACTOR;
  
  int newX = constrain(round(currentSmoothX), -180, 180);
  int newY = constrain(round(currentSmoothY), -30, 60);
  
  if (newX != servoX || newY != servoY) {
    servoX = newX;
    servoY = newY;
    needUpdate = true;
  }
}

// ============================================
// ===== MODULE: MODE CYCLING =====
// ============================================

void cycleMode() {
  currentMode = (currentMode + 1) % 4;
  
  targetX = 0;
  targetY = 0;
  smoothX = 0;
  smoothY = 0;
  servoX = 0;
  servoY = 0;
  
  Serial.printf("[MODE] %s\n", currentMode == 0 ? "TILT+PAN" : 
                               currentMode == 1 ? "TILT" :
                               currentMode == 2 ? "PAN" : "CHANNELS");
  needUpdate = true;
}

// ============================================
// ===== PROTOCOL: COMMANDS =====
// ============================================

void sendServoCommand() {
  char packet[64];
  snprintf(packet, sizeof(packet), "M%d,%d,%d", servoX, servoY, calRequest ? 1 : 0);
  
  udp.beginPacket(controller_ip, ETH_CONTROLLER_PORT);
  udp.write((const uint8_t*)packet, strlen(packet));
  udp.endPacket();
  
  if (calRequest) calRequest = false;
}

void sendChannelCommand(int direction) {
  char packet[16];
  snprintf(packet, sizeof(packet), "C%d", direction);
  
  udp.beginPacket(controller_ip, ETH_CONTROLLER_PORT);
  udp.write((const uint8_t*)packet, strlen(packet));
  udp.endPacket();
}

void sendModuleCommand(const char* moduleName, int state) {
  char packet[64];
  snprintf(packet, sizeof(packet), "MOD%s,%d", moduleName, state);
  
  udp.beginPacket(controller_ip, ETH_CONTROLLER_PORT);
  udp.write((const uint8_t*)packet, strlen(packet));
  udp.endPacket();
  
  Serial.printf("[TX] %s\n", packet);
}

void parseStatusPacket(const char* packet) {
  int pan, tilt, compass, ok, cal;
  if (sscanf(packet, "A=%d,%d,C=%d,OK=%d,Cal=%d", &pan, &tilt, &compass, &ok, &cal) == 5) {
    servoX = pan;
    servoY = tilt;
    compassAngle = compass;
    compassOk = (ok != 0);
    calibrated = (cal != 0);
    needUpdate = true;
  }
}

// ============================================
// ===== UI: MAIN SCREEN =====
// ============================================

void drawStaticLabels() {
  tft.setTextSize(1);
  tft.setTextColor(ST7735_YELLOW);
  
  tft.setCursor(0, 0); tft.print("Status:");
  tft.setCursor(0, 16); tft.print("X:");
  tft.setCursor(0, 32); tft.print("Y:");
  tft.setCursor(0, 48); tft.print("C:");
  tft.setCursor(0, 64); tft.print("CH:");
  tft.setCursor(0, 80); tft.print("Mode:");
  tft.setCursor(0, 96); tft.print("Cal:");
  
  tft.drawRect(55, 18, 80, 10, ST7735_WHITE);
  tft.drawRect(55, 34, 80, 10, ST7735_WHITE);
  
  // Draw menu buttons
  btnMenu.draw(tft);
  btnModules.draw(tft);
}

void updateLinkStatus() {
  bool currentLink = !linkLost;
  if (currentLink != lastLinkStatus) {
    tft.fillRect(40, 0, 80, 14, ST7735_BLACK);
    tft.setCursor(40, 0);
    tft.setTextColor(currentLink ? ST7735_GREEN : ST7735_RED);
    tft.print(currentLink ? "LINK OK" : "NO LINK");
    lastLinkStatus = currentLink;
  }
}

void updateServoX() {
  if (servoX != lastServoX) {
    tft.fillRect(15, 16, 40, 14, ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(15, 16);
    tft.print(servoX);
    tft.print(" ");
    tft.fillRect(56, 19, 78, 8, ST7735_BLACK);
    int barWidth = constrain(map(servoX, -180, 180, 0, 78), 0, 78);
    if (barWidth > 0) tft.fillRect(56, 19, barWidth, 8, ST7735_RED);
    lastServoX = servoX;
  }
}

void updateServoY() {
  if (servoY != lastServoY) {
    tft.fillRect(15, 32, 40, 14, ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(15, 32);
    tft.print(servoY);
    tft.print(" ");
    tft.fillRect(56, 35, 78, 8, ST7735_BLACK);
    int barWidth = constrain(map(servoY, -30, 60, 0, 78), 0, 78);
    if (barWidth > 0) tft.fillRect(56, 35, barWidth, 8, ST7735_GREEN);
    lastServoY = servoY;
  }
}

void updateCompass() {
  if (compassAngle != lastCompass || compassOk != lastCompassOk) {
    tft.fillRect(15, 48, 110, 14, ST7735_BLACK);
    tft.setCursor(15, 48);
    
    if (compassOk) {
      tft.setTextColor(ST7735_WHITE);
      tft.print(compassAngle);
      tft.print("°");
      
      tft.setCursor(70, 48);
      tft.setTextColor(ST7735_CYAN);
      
      if (compassAngle >= 337 || compassAngle < 22) tft.print("↑N");
      else if (compassAngle >= 22 && compassAngle < 67) tft.print("↗NE");
      else if (compassAngle >= 67 && compassAngle < 112) tft.print("→E");
      else if (compassAngle >= 112 && compassAngle < 157) tft.print("↘SE");
      else if (compassAngle >= 157 && compassAngle < 202) tft.print("↓S");
      else if (compassAngle >= 202 && compassAngle < 247) tft.print("↙SW");
      else if (compassAngle >= 247 && compassAngle < 292) tft.print("←W");
      else if (compassAngle >= 292 && compassAngle < 337) tft.print("↖NW");
    } else {
      tft.setTextColor(ST7735_RED);
      tft.print("---");
    }
    
    lastCompass = compassAngle;
    lastCompassOk = compassOk;
  }
}

void updateChannel() {
  if (currentChannel != lastChannel) {
    tft.fillRect(25, 64, 100, 14, ST7735_BLACK);
    tft.setCursor(25, 64);
    tft.setTextColor(ST7735_WHITE);
    tft.print(CHANNELS_TABLE[currentChannel].name);
    tft.print(" ");
    tft.print(CHANNELS_TABLE[currentChannel].freq);
    lastChannel = currentChannel;
  }
}

void updateMode() {
  if (currentMode != lastMode) {
    tft.fillRect(30, 80, 80, 14, ST7735_BLACK);
    tft.setCursor(30, 80);
    tft.setTextColor(ST7735_CYAN);
    tft.print(currentMode == 0 ? "TILT+PAN" : 
              currentMode == 1 ? "TILT" :
              currentMode == 2 ? "PAN" : "CHANNELS");
    lastMode = currentMode;
  }
}

// ============================================
// ===== UI: MODULE MANAGER MENU =====
// ============================================

void drawModuleMenu() {
  static int lastSelection = -1;
  
  if (lastSelection == -1) {
    tft.fillScreen(ST7735_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(0, 0);
    tft.print("Module Manager:");
    tft.setCursor(0, 20);
    tft.print("1. Servo");
    tft.setCursor(0, 32);
    tft.print("2. Compass");
    tft.setCursor(0, 44);
    tft.print("3. Frequency");
    tft.setCursor(0, 56);
    tft.print("4. Telemetry");
    tft.setCursor(0, 76);
    tft.print("Press SW to toggle");
    tft.setCursor(0, 88);
    tft.print("Joystick Y to select");
    lastSelection = moduleSelection;
  }
  
  // Show current selection
  if (moduleSelection != lastSelection) {
    tft.fillRect(0, 20 + lastSelection * 12, 7, 12, ST7735_BLACK);
    tft.fillRect(0, 20 + moduleSelection * 12, 7, 12, ST7735_YELLOW);
    lastSelection = moduleSelection;
  }
  
  // Show states
  tft.fillRect(90, 20, 58, 12, ST7735_BLACK);
  tft.setTextColor(modules.servo_control ? ST7735_GREEN : ST7735_RED);
  tft.setCursor(90, 20);
  tft.print(modules.servo_control ? "[ON]" : "[OFF]");
  
  tft.fillRect(90, 32, 58, 12, ST7735_BLACK);
  tft.setTextColor(modules.compass ? ST7735_GREEN : ST7735_RED);
  tft.setCursor(90, 32);
  tft.print(modules.compass ? "[ON]" : "[OFF]");
  
  tft.fillRect(90, 44, 58, 12, ST7735_BLACK);
  tft.setTextColor(modules.frequency ? ST7735_GREEN : ST7735_RED);
  tft.setCursor(90, 44);
  tft.print(modules.frequency ? "[ON]" : "[OFF]");
  
  tft.fillRect(90, 56, 58, 12, ST7735_BLACK);
  tft.setTextColor(modules.telemetry ? ST7735_GREEN : ST7735_RED);
  tft.setCursor(90, 56);
  tft.print(modules.telemetry ? "[ON]" : "[OFF]");
}

void toggleModule() {
  const char* modNames[] = {"SERVO", "COMPASS", "FREQ", "TELEM"};
  
  if (moduleSelection == 0) {
    modules.servo_control = !modules.servo_control;
    sendModuleCommand("SERVO", modules.servo_control);
  } else if (moduleSelection == 1) {
    modules.compass = !modules.compass;
    sendModuleCommand("COMPASS", modules.compass);
  } else if (moduleSelection == 2) {
    modules.frequency = !modules.frequency;
    sendModuleCommand("FREQ", modules.frequency);
  } else if (moduleSelection == 3) {
    modules.telemetry = !modules.telemetry;
    sendModuleCommand("TELEM", modules.telemetry);
  }
}

// ============================================
// ===== SETUP =====
// ============================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║  BLOCK 2: REMOTE WITH MODULE MANAGER    ║");
  Serial.println("║       On-Screen Menu Button v1.0         ║");
  Serial.println("╚═══════════════════════════════════════════╝");
  Serial.println();
  
  pinMode(PIN_SW, INPUT_PULLUP);
  
  // === TFT ===
  pinMode(PIN_TFT_LEDA, OUTPUT);
  digitalWrite(PIN_TFT_LEDA, HIGH);
  
  pinMode(PIN_TFT_RST, OUTPUT);
  digitalWrite(PIN_TFT_RST, HIGH);
  delay(100);
  digitalWrite(PIN_TFT_RST, LOW);
  delay(100);
  digitalWrite(PIN_TFT_RST, HIGH);
  delay(100);
  
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0);
  
  // === ETHERNET ===
  Ethernet.init(PIN_W5500_CS);
  if (Ethernet.begin(mac) == 0) {
    Ethernet.begin(mac, ip);
  }
  
  udp.begin(ETH_LOCAL_PORT);
  Serial.printf("[✓] Ethernet: %s\n", Ethernet.localIP().toString().c_str());
  
  // === EEPROM ===
  EEPROM.begin(EEPROM_SIZE);
  
  // === JOYSTICK ===
  calibrateJoystick();
  
  tft.fillScreen(ST7735_BLACK);
  drawStaticLabels();
  
  lastResponseTime = millis();
  
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║         SYSTEM READY                      ║");
  Serial.println("╚═══════════════════════════════════════════╝");
}

// ============================================
// ===== MAIN LOOP =====
// ============================================

void loop() {
  // === JOYSTICK READING ===
  if (!mainMenuActive && !subMenuActive && !moduleMenuActive && !calibrationActive) {
    if (currentMode != MODE_CHANNELS) {
      readJoystickServo();
      updateServoPosition();
    } else {
      readJoystickChannel();
    }
  }
  
  // === BUTTON HANDLING ===
  bool btn = !digitalRead(PIN_SW);
  static bool btnPrev = false;
  static unsigned long btnDownTime = 0;
  static bool btnHandled = false;
  
  int yVal = analogRead(PIN_VRY);
  static unsigned long lastMove = 0;
  
  // === MODULE MENU ===
  if (moduleMenuActive) {
    drawModuleMenu();
    
    if (yVal < 1500 && millis() - lastMove > 200) {
      moduleSelection = (moduleSelection - 1 + 4) % 4;
      lastMove = millis();
    }
    if (yVal > 2500 && millis() - lastMove > 200) {
      moduleSelection = (moduleSelection + 1) % 4;
      lastMove = millis();
    }
    
    if (btn && !btnPrev) {
      toggleModule();
      btnPrev = btn;
      delay(100);
      btnPrev = !digitalRead(PIN_SW);
      return;
    }
    
    if (btn && !btnPrev) btnDownTime = millis();
    if (!btn && btnPrev && (millis() - btnDownTime) > 50) {
      moduleMenuActive = false;
      tft.fillScreen(ST7735_BLACK);
      drawStaticLabels();
      needUpdate = true;
    }
  }
  
  // === SHORT PRESS: CYCLE MODE ===
  if (btn && !btnPrev) {
    btnDownTime = millis();
    btnHandled = false;
  }
  
  if (!btn && btnPrev) {
    if (!btnHandled && (millis() - btnDownTime) > 50 && (millis() - btnDownTime) < 1000) {
      cycleMode();
      btnHandled = true;
    }
  }
  
  btnPrev = btn;
  
  // === SEND SERVO COMMAND ===
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 100) {
    sendServoCommand();
    lastSend = millis();
  }
  
  // === RECEIVE STATUS ===
  int packetSize = udp.parsePacket();
  if (packetSize) {
    lastResponseTime = millis();
    linkLost = false;
    
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      parseStatusPacket(packetBuffer);
    }
  }
  
  // === WATCHDOG ===
  if (millis() - lastResponseTime > LINK_TIMEOUT) {
    if (!linkLost) {
      linkLost = true;
      needUpdate = true;
    }
  }
  
  // === UPDATE SCREEN ===
  if (needUpdate && !mainMenuActive && !moduleMenuActive) {
    updateLinkStatus();
    updateServoX();
    updateServoY();
    updateCompass();
    updateChannel();
    updateMode();
    
    // Redraw buttons
    btnMenu.draw(tft);
    btnModules.draw(tft);
    
    needUpdate = false;
  }
  
  // === CHECK MENU BUTTON CLICK ===
  static bool prevBtn = false;
  bool btnState = !digitalRead(PIN_SW);
  
  if (btnState && !prevBtn) {
    btnDownTime = millis();
  }
  if (!btnState && prevBtn && (millis() - btnDownTime) > 1000) {
    // Long press opens module menu
    moduleMenuActive = true;
    moduleSelection = 0;
  }
  
  prevBtn = btnState;
  
  delay(10);
}
