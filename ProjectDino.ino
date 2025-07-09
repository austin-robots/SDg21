#include <Ethernet.h>
#include <ArduinoModbus.h>
#include <Dynamixel2Arduino.h>

// Network Configuration
byte mac[] = { 0x00, 0xB1, 0x1A, 0x44, 0x90, 0x8C };
byte ip[] = { 192, 168, 0, 100 };
IPAddress plcIp(192, 168, 0, 200);

EthernetClient ethClient;
ModbusTCPClient modbusClient(ethClient);

// Dynamixel Configuration
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2;
#else
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2;
#endif

//  ID definitions
#define NeckYaw     10
#define NeckPitch   15
#define Body        20
#define Tail        30
#define Jaw         40
#define Skull       50

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// Global Variables
const float DXL_PROTOCOL_VERSION = 2.0;
volatile int pselect = 0;
volatile int state = 0;
volatile int run = 0;
const uint8_t DXL_IDS[] = {NeckYaw, NeckPitch, Body, Jaw, Skull, Tail};
const uint16_t MODBUS_POSITION_ADDRS[] = {0, 1, 2, 3, 4, 5};
const uint16_t MODBUS_CURRENT_ADDRS[] = {6, 7, 8, 9, 10, 11};
volatile uint32_t timerStartValue = 0;
volatile uint32_t lastTimerCount = 0;
volatile uint32_t ProfileT = 0;
volatile bool ProfileComplete = false;
volatile bool timerRunning = false;
volatile bool hasHeldPose = false;
volatile bool Auto = false;
volatile bool Off = false;
volatile bool Maint = false;
volatile bool Heart = false;
volatile bool NeckYawEnable = true;
volatile bool NeckPitchEnable = true;
volatile bool BodyEnable = true;
volatile bool JawEnable = true;
volatile bool SkullEnable = true;
volatile bool TailEnable = true;
volatile bool MBEnable = true;
volatile bool OneTimeA = false;
volatile bool OneTimeM = false;
volatile bool OneTimeO = false;

// Group definitions
const uint8_t groupNeckBase[] = {NeckYaw, NeckPitch};
const uint8_t groupJawSkull[] = {Jaw, Skull};
const uint8_t groupAll[] = {NeckYaw, NeckPitch, Body, Tail, Jaw, Skull};

// Single Dynamixel movement
void setGoalPosition(uint8_t id, float degrees) {
  switch (id) {
    case NeckYaw:
      if (NeckYawEnable) dxl.setGoalPosition(id, degrees, UNIT_DEGREE);
      break;
    case NeckPitch:
      if (NeckPitchEnable) dxl.setGoalPosition(id, degrees, UNIT_DEGREE);
      break;
    case Body:
      if (BodyEnable) dxl.setGoalPosition(id, degrees, UNIT_DEGREE);
      break;
    case Jaw:
      if (JawEnable) dxl.setGoalPosition(id, degrees, UNIT_DEGREE);
      break;
    case Skull:
      if (SkullEnable) dxl.setGoalPosition(id, degrees, UNIT_DEGREE);
      break;
    case Tail:
      if (TailEnable) dxl.setGoalPosition(id, degrees, UNIT_DEGREE);
      break;
    default:
    // Edge Case: Exit if unknown ID
      break;
  }
}

// Group Dynamixel movement
void setGoalPositionToGroup(const uint8_t* ids, uint8_t count, float degrees) {
  for (uint8_t i = 0; i < count; i++) {
    setGoalPosition(ids[i], degrees);
  }
}

// Timer Setup
void setupTimer() {
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TC4_TC5) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);
  TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV1024;
  TC4->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
  SerialUSB.println("32-bit Timer Configured.");
}

// Main Setup 
void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB);
  Ethernet.begin(mac, ip);
  delay(1000);
  if (!modbusClient.begin(plcIp)) {
    SerialUSB.println("Initial connection to Modbus server failed.");
  } else {
    SerialUSB.println("Connected to Modbus server.");
  }
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  for (uint8_t id : DXL_IDS) {
    dxl.ping(id);
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
  }
  setupTimer();
}

void updateModbus() {
  // Update Dynamixel Status
  for (int i = 0; i < 6; i++) {
    int32_t currentPosition = dxl.getPresentPosition(DXL_IDS[i], UNIT_DEGREE);
    float currentReading = dxl.getPresentCurrent(DXL_IDS[i], UNIT_MILLI_AMPERE);
    modbusClient.holdingRegisterWrite(MODBUS_POSITION_ADDRS[i], currentPosition);
    modbusClient.holdingRegisterWrite(MODBUS_CURRENT_ADDRS[i], static_cast<int32_t>(currentReading));
  }

  // Read Coils
  run = modbusClient.coilRead(0);
  ProfileComplete = modbusClient.coilRead(1);
  Auto = modbusClient.coilRead(2);
  Maint = modbusClient.coilRead(3);
  Off = modbusClient.coilRead(4);
  NeckYawEnable = modbusClient.coilRead(6);
  NeckPitchEnable = modbusClient.coilRead(7);
  BodyEnable = modbusClient.coilRead(8);
  JawEnable = modbusClient.coilRead(9);
  SkullEnable = modbusClient.coilRead(10);
  TailEnable = modbusClient.coilRead(11);
  MBEnable = modbusClient.coilRead(12);

  // Read Holding Registers
  pselect = modbusClient.holdingRegisterRead(13);

  // Write Coils
  modbusClient.coilWrite(5, Heart);

  // Write Holding Registers
  modbusClient.holdingRegisterWrite(12, state);

}

void AutoMode(){
  if (!OneTimeA){
    OneTimeM = 0;
    OneTimeO = 0;

    // Anything one time on switch goes here

    for (uint8_t id : DXL_IDS) {
      dxl.torqueOn(id);
      dxl.writeControlTableItem(PROFILE_VELOCITY, id, 7);
    }

    // ADD Enable pin for stepper ON

    OneTimeA = 1;
  }

  if (run) {
    if (!timerRunning) {
      timerStartValue = TC4->COUNT32.COUNT.reg - lastTimerCount;
      timerRunning = true;
      SerialUSB.println("Timer Started.");
    }
    hasHeldPose = false;
  } else {
    if (timerRunning) {
      lastTimerCount = TC4->COUNT32.COUNT.reg - timerStartValue;
      timerRunning = false;
      SerialUSB.println("Timer Paused.");
      if (state > 1) state--;
    }
    if (!hasHeldPose) {
      for (uint8_t id : DXL_IDS) {
        int32_t currentPosition = dxl.getPresentPosition(id, UNIT_DEGREE);
        dxl.setGoalPosition(id, currentPosition, UNIT_DEGREE);
      }
      hasHeldPose = true;
    }
  }

  if (timerRunning) {
    uint32_t currentTimerValue = TC4->COUNT32.COUNT.reg;
    uint32_t elapsedTime = currentTimerValue - timerStartValue;
    float elapsedSeconds = elapsedTime / 46875.0;
    ProfileT = elapsedSeconds;

    SerialUSB.print("Elapsed Time: ");
    SerialUSB.print(elapsedSeconds, 3);
    SerialUSB.println(" sec");

    if (elapsedSeconds >= 20.0 && !ProfileComplete) {
      modbusClient.coilWrite(1, 1);
      state = 0;
      SerialUSB.println("Profile Complete! Timer Reset.");
      timerStartValue = currentTimerValue;
      ProfileT = 0;
    }
  }

  if(pselect == 0){
    MainProfile();
  }


  
}

void MaintMode(){
  if (!OneTimeM){
    OneTimeA = 0;
    OneTimeO = 0;

    // Anything one time on switch goes here
    for (uint8_t id : DXL_IDS) {
      dxl.torqueOn(id);
      int32_t currentPosition = dxl.getPresentPosition(id, UNIT_DEGREE);
      dxl.setGoalPosition(id, currentPosition, UNIT_DEGREE);
    }

    // ADD something for stepper

    OneTimeM = 1;
  }

}

void OffMode(){
  if (!OneTimeO){
    OneTimeM = 0;
    OneTimeA = 0;

    // Anything one time on switch goes here
    for (uint8_t id : DXL_IDS) {
      dxl.torqueOff(id);
    }

    // ADD Enable pin for stepper turn off

    OneTimeO = 1;
  }

}

void MainProfile() {
  // Motion profile steps
  if (ProfileT > 0 && state == 0) {
    setGoalPositionToGroup(groupAll, sizeof(groupAll), 180);
    state++;
  }

  if (ProfileT >= 1 && state == 1) {
    state++;
  }

  if (ProfileT >= 2 && state == 2) {
    setGoalPosition(Jaw, 190);
    state++;
  }

  if (ProfileT >= 3 && state == 3) {
    state++;
  }

  if (ProfileT >= 4 && state == 4) {
    setGoalPositionToGroup(groupAll, sizeof(groupAll), 170);
    state++;
  }

  if (ProfileT >= 5 && state == 5) {
    state++;
  }

  if (ProfileT >= 6 && state == 6) {
    setGoalPositionToGroup(groupAll, sizeof(groupAll), 180);
    state++;
  }

  if (ProfileT >= 7 && state == 7) {
    state++;
  }

  if (ProfileT >= 8 && state == 8) {
    setGoalPositionToGroup(groupAll, sizeof(groupAll), 160);
    state++;
  }

  if (ProfileT >= 9 && state == 9) {
    state++;
  }

  if (ProfileT >= 10 && state == 10) {
    setGoalPositionToGroup(groupAll, sizeof(groupAll), 180);
    state++;
  }

  if (ProfileT >= 11 && state == 11) {
    state++;
  }

  if (ProfileT >= 12 && state == 12) {
    setGoalPositionToGroup(groupAll, sizeof(groupAll), 170);
    state++;
  }

  if (ProfileT >= 13 && state == 13) {
    state++;
  }

  if (ProfileT >= 14 && state == 14) {
    setGoalPositionToGroup(groupAll, sizeof(groupAll), 190);
    state++;
  }

  if (ProfileT >= 15 && state == 15) {
    state++;
  }

  if (ProfileT >= 16 && state == 16) {
    setGoalPositionToGroup(groupAll, sizeof(groupAll), 170);
    state++;
  }

  if (ProfileT >= 17 && state == 17) {
    state++;
  }

  if (ProfileT >= 18 && state == 18) {
    setGoalPositionToGroup(groupAll, sizeof(groupAll), 180);
    state++;
  }
}

void loop() {
  if (!modbusClient.connected()) {
    SerialUSB.println("Modbus connection lost. Attempting to reconnect...");
    if (!modbusClient.begin(plcIp)) {
      SerialUSB.println("Reconnection failed. Will retry...");
      // ADD FLASHING LED PB08
      delay(1000);
      return;
    }
    SerialUSB.println("Reconnected to Modbus server.");
  }

  if(Auto){
    AutoMode();
  }

  if(Maint){
    MaintMode();
  }

  if(Off){
    OffMode();
  }
  
  // Heartbeat and Modbus Update
  Heart = !Heart;
  updateModbus();
  delay(10);
}