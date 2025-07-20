#include <Arduino.h>
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

// Stepper definitions
#define EN_PIN 20 // PA06
#define STEP_PIN 19 // PA05
#define DIR_PIN  18 // PA04

// LED definition
#define LED_PIN 32 // PB08

//  Stepper Configuration 
volatile uint32_t stepIntervalMicros = 1000;
const uint16_t stepsPerMove = 7500;
volatile uint16_t stepsRemaining = stepsPerMove;
volatile uint16_t stepsDone = 0;           // Track how far into profile
volatile bool stepPinState = false;
volatile bool stepperEnabled = false;
volatile bool updateSpeed = false;
volatile bool goingBack = false;
volatile bool stepperHasRun = false;

//  Acceleration Config 
const uint16_t minInterval = 400;      // Fastest µs per step
const uint16_t maxInterval = 1500;     // Slowest µs per step
const uint16_t accelRampLength = 2000; // Number of steps to fully accelerate

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// Global Variables
const float DXL_PROTOCOL_VERSION = 2.0;
volatile int proselect = 0;
volatile int state = 0;
volatile int run = 0;
const uint8_t DXL_IDS[] = {NeckYaw, NeckPitch, Body, Jaw, Skull, Tail};
const uint16_t MODBUS_POSITION_ADDRS[] = {0, 1, 2, 3, 4, 5};
const uint16_t MODBUS_CURRENT_ADDRS[] = {6, 7, 8, 9, 10, 11};
const uint16_t DYNAMIXEL_FAULT_ADDRS[] = {15, 16, 17, 18, 19, 20};
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
volatile bool DynMtrReset = false;
volatile bool HomeCommand = false;
volatile bool HomeSet = false;

// Set Dynamixel Position if Allowed
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

// Reset Dynamixel Motors if needed
void resetIfFault() {
  for(uint8_t id : DXL_IDS){
    uint8_t err = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id);
    if (err != 0) {
      DEBUG_SERIAL.print(err, HEX);
      dxl.reboot(id);  // Sends Protocol 2.0 REBOOT instruction
      delay(200);      // Wait for reboot
      dxl.torqueOn(id);
    }
  }
}

void HomeFig() {
  if (!HomeSet){
    setGoalPosition(Jaw, 190);
    // Set Home Positions for Dynamixels
  }

  // Run Stepper Backwards until home sensor flags.


}

// Timer Setup Profile TC5
void setupTimer() {
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TC4_TC5) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);
  TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV1024;
  TC4->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
  SerialUSB.println("32-bit Timer Configured.");
}

// Timer Setup Stepper TC3
void setupStepperTimer(uint32_t intervalMicros) {
  // Use TC3 clock ID
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC2_TC3 | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);

  PM->APBCMASK.reg |= PM_APBCMASK_TC3;

  TC3->COUNT16.CTRLA.reg =
    TC_CTRLA_MODE_COUNT16 |
    TC_CTRLA_PRESCALER_DIV16 |
    TC_CTRLA_WAVEGEN_MFRQ;

  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

  TC3->COUNT16.CC[0].reg = (3000000 / (1000000 / intervalMicros));
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

  TC3->COUNT16.INTENSET.bit.MC0 = 1;
  NVIC_EnableIRQ(TC3_IRQn);

  TC3->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

// Stepper Acceleration
void adjustAcceleration() {
  if (stepsDone < accelRampLength) {
    stepIntervalMicros = maxInterval - ((maxInterval - minInterval) * stepsDone) / accelRampLength;
  } else if (stepsRemaining < accelRampLength) {
    stepIntervalMicros = maxInterval - ((maxInterval - minInterval) * stepsRemaining) / accelRampLength;
  } else {
    stepIntervalMicros = minInterval;
  }

  TC3->COUNT16.CC[0].reg = (3000000 / (1000000 / stepIntervalMicros));
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

// Stepper Timer ISR TC3
void TC3_Handler() {
  TC3->COUNT16.INTFLAG.bit.MC0 = 1;

  if (!stepperEnabled || stepsRemaining == 0) return;

  stepPinState = !stepPinState;
  digitalWrite(STEP_PIN, stepPinState);

  if (!stepPinState) {
    stepsRemaining--;
    stepsDone++;
    updateSpeed = true;

    if (stepsRemaining == 0) {
      if (!goingBack) {
        // Begin reverse
        goingBack = true;
        stepsRemaining = stepsPerMove;
        stepsDone = 0;
        digitalWrite(DIR_PIN, LOW);
        stepperEnabled = true;
        SerialUSB.println("AutoMode: Returning...");
      } 
      else {
        // Done
        stepperEnabled = false;
        SerialUSB.println("AutoMode: Stepper round-trip complete.");
      }
    }
  }
}

// Main setup 
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
  for (uint8_t id : DXL_IDS){
    dxl.ping(id);
    dxl.torqueOff(id);
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, 15);
    dxl.setOperatingMode(id, OP_POSITION);
  }
  dxl.writeControlTableItem(PROFILE_VELOCITY, NeckYaw, 30);

  // READ current Shutdown settings
  uint8_t shutdown_mask = dxl.readControlTableItem(SHUTDOWN, NeckYaw);
  DEBUG_SERIAL.print("Shutdown original mask: 0x");
  DEBUG_SERIAL.println(shutdown_mask, HEX);

  // CLEAR overload bit (bit 5 / 0x20)
  uint8_t new_mask = shutdown_mask & ~0x20;
  dxl.writeControlTableItem(SHUTDOWN, NeckYaw, new_mask);
  DEBUG_SERIAL.print("Shutdown updated mask: 0x");
  DEBUG_SERIAL.println(new_mask, HEX);
  
  // Profile Timer
  setupTimer();

  // Stepper Setup
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);

  setupStepperTimer(stepIntervalMicros);

  // LED initialize
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

// Update all modbus variables
void updateModbus() {
  // Update Dynamixel Status
  for (int i = 0; i < 6; i++) {
    int32_t currentPosition = dxl.getPresentPosition(DXL_IDS[i], UNIT_DEGREE);
    float currentReading = dxl.getPresentCurrent(DXL_IDS[i], UNIT_MILLI_AMPERE);
    uint8_t fault = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_IDS[i]);
    modbusClient.holdingRegisterWrite(MODBUS_POSITION_ADDRS[i], currentPosition);
    modbusClient.holdingRegisterWrite(MODBUS_CURRENT_ADDRS[i], static_cast<int32_t>(currentReading));
    modbusClient.holdingRegisterWrite(DYNAMIXEL_FAULT_ADDRS[i], fault);
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
  DynMtrReset = modbusClient.coilRead(13);
  HomeCommand = modbusClient.coilRead(14);

  // Read Holding Registers
  proselect = modbusClient.holdingRegisterRead(13);

  // Write Coils
  modbusClient.coilWrite(5, Heart);

  // Write Holding Registers
  modbusClient.holdingRegisterWrite(12, state);
  modbusClient.holdingRegisterWrite(14, ProfileT);

}

// Auto Mode Switch
void AutoMode() {
  if (!OneTimeA){
    OneTimeM = 0;
    OneTimeO = 0;

    for (uint8_t id : DXL_IDS) {
      dxl.torqueOn(id);
      dxl.writeControlTableItem(PROFILE_VELOCITY, id, 15);
    }

    // normal speed stepper & sets the initial direction to be forward
    minInterval = 400;
    digitalWrite(DIR_PIN, HIGH);

    OneTimeA = 1;
  }

  if (run) {
    if (!timerRunning) {
      timerStartValue = TC4->COUNT32.COUNT.reg - lastTimerCount;
      timerRunning = true;
      SerialUSB.println("Timer Started.");
    }

    hasHeldPose = false;

    if (!stepperEnabled && !stepperHasRun && MBEnable) {
      // Start stepper forward
      stepsRemaining = stepsPerMove;
      stepsDone = 0;
      goingBack = false;
      digitalWrite(DIR_PIN, HIGH);
      stepperEnabled = true;
      stepperHasRun = true;
      SerialUSB.println("AutoMode: Stepper started forward.");
    }
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

    // Stop stepper if still running
    if (stepperEnabled) {
      stepperEnabled = false;
      SerialUSB.println("AutoMode: Stepper forcibly stopped.");
    }
    stepperHasRun = false;
  }

  if (timerRunning) {
    uint32_t currentTimerValue = TC4->COUNT32.COUNT.reg;
    uint32_t elapsedTime = currentTimerValue - timerStartValue;
    float elapsedSeconds = elapsedTime / 46875.0;
    ProfileT = elapsedSeconds;

    SerialUSB.print("Elapsed Time: ");
    SerialUSB.print(elapsedSeconds, 3);
    SerialUSB.println(" sec");

    if (elapsedSeconds >= 25.0 && !ProfileComplete) {
      modbusClient.coilWrite(1, 1);
      state = 0;
      SerialUSB.println("Profile Complete! Timer Reset.");
      timerStartValue = currentTimerValue;
      ProfileT = 0;
    }
  }

  if (proselect == 0) {
    MainProfile();
  }

  MainProfile();
}

// Maintenance Mode Switch
void MaintMode(){
  if (!OneTimeM){
    OneTimeA = 0;
    OneTimeO = 0;

    // Anything one time on switch goes here
    for (uint8_t id : DXL_IDS) {
      dxl.torqueOn(id);
      int32_t currentPosition = dxl.getPresentPosition(id, UNIT_DEGREE);
      dxl.setGoalPosition(id, currentPosition, UNIT_DEGREE);
      dxl.writeControlTableItem(PROFILE_VELOCITY, id, 7);
    }

    // slow stepper
     minInterval = 1000;

    OneTimeM = 1;
  }

  // Profile Select Occurs Purely via Modbus

  // Stop Reset and Fault Reset purely in PLC

  // Ignore Fault / Fault Bypass is purely in PLC

  // Home Figure
  if (HomeCommand){
    HomeFig();
  }
  else{
    HomeSet = 0;
  }
  // Motor Reset
  if (DynMtrReset){
    resetIfFault();
  }

  // Manual Axis Movement



  // Maint Profile



}

// Off Mode Switch
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

  // Do Nothing, Off.

}

// Main Profile (P1)
void MainProfile() {

  if (ProfileT > 0 && state == 0) {
    setGoalPosition(Jaw, 190);
    setGoalPosition(Skull, 180);
    setGoalPosition(NeckPitch, 180);
    setGoalPosition(NeckYaw, 100);
    setGoalPosition(Body, 180);
    setGoalPosition(Tail, 180);
    state++;
  }


  if (ProfileT >= 1 && state == 1) {
    setGoalPosition(Jaw, 210);
    setGoalPosition(Skull, 170);
    setGoalPosition(NeckPitch, 190);
    setGoalPosition(NeckYaw, 95);
    setGoalPosition(Body, 185);
    setGoalPosition(Tail, 190);
    state++;
  }

  if (ProfileT >= 2 && state == 2) {
    setGoalPosition(Jaw, 180);
    setGoalPosition(Skull, 190);
    setGoalPosition(NeckPitch, 170);
    setGoalPosition(NeckYaw, 105);
    setGoalPosition(Body, 175);
    setGoalPosition(Tail, 170);
    state++;
  }

  if (ProfileT >= 3 && state == 3) {
    setGoalPosition(Jaw, 20);
    setGoalPosition(Skull, 170);
    setGoalPosition(NeckPitch, 190);
    setGoalPosition(NeckYaw, 95);
    setGoalPosition(Body, 185);
    setGoalPosition(Tail, 190);
    state++;
  }

  if (ProfileT >= 4 && state == 4) {
    setGoalPosition(Jaw, 180);
    setGoalPosition(Skull, 190);
    setGoalPosition(NeckPitch, 170);
    setGoalPosition(NeckYaw, 105);
    setGoalPosition(Body, 175);
    setGoalPosition(Tail, 170);
    state++;
  }

  if (ProfileT >= 5 && state == 5) {
    setGoalPosition(Jaw, 210);
    setGoalPosition(Skull, 170);
    setGoalPosition(NeckPitch, 190);
    setGoalPosition(NeckYaw, 95);
    setGoalPosition(Body, 185);
    setGoalPosition(Tail, 190);
    state++;
  }

  if (ProfileT >= 6 && state == 6) {
    setGoalPosition(Jaw, 180);
    setGoalPosition(Skull, 190);
    setGoalPosition(NeckPitch, 170);
    setGoalPosition(NeckYaw, 105);
    setGoalPosition(Body, 175);
    setGoalPosition(Tail, 170);
    state++;
  }
  

  if (ProfileT >= 7 && state == 7) {
    setGoalPosition(Jaw, 210);
    setGoalPosition(Skull, 170);
    setGoalPosition(NeckPitch, 190);
    setGoalPosition(NeckYaw, 95);
    setGoalPosition(Body, 185);
    setGoalPosition(Tail, 190);
    state++;
  }

  if (ProfileT >= 8 && state == 8) {
    setGoalPosition(Jaw, 180);
    setGoalPosition(Skull, 190);
    setGoalPosition(NeckPitch, 170);
    setGoalPosition(NeckYaw, 105);
    setGoalPosition(Body, 175);
    setGoalPosition(Tail, 170);
    state++;
  }

  if (ProfileT >= 9 && state == 9) {
    setGoalPosition(Jaw, 210);
    setGoalPosition(Skull, 170);
    setGoalPosition(NeckPitch, 190);
    setGoalPosition(NeckYaw, 95);
    setGoalPosition(Body, 185);
    setGoalPosition(Tail, 190);
    state++;
  }

  if (ProfileT >= 10 && state == 10) {
    setGoalPosition(Jaw, 180);
    setGoalPosition(Skull, 190);
    setGoalPosition(NeckPitch, 170);
    setGoalPosition(NeckYaw, 105);
    setGoalPosition(Body, 175);
    setGoalPosition(Tail, 170);
    state++;
  }

  if (ProfileT >= 11 && state == 11) {
    setGoalPosition(Jaw, 210);
    setGoalPosition(Skull, 170);
    setGoalPosition(NeckPitch, 190);
    setGoalPosition(NeckYaw, 95);
    setGoalPosition(Body, 185);
    setGoalPosition(Tail, 190);
    state++;
  }

  if (ProfileT >= 12 && state == 12) {
    setGoalPosition(Jaw, 180);
    setGoalPosition(Skull, 190);
    setGoalPosition(NeckPitch, 170);
    setGoalPosition(NeckYaw, 105);
    setGoalPosition(Body, 175);
    setGoalPosition(Tail, 170);
    state++;
  }

  if (ProfileT >= 13 && state == 13) {
    setGoalPosition(Jaw, 210);
    setGoalPosition(Skull, 170);
    setGoalPosition(NeckPitch, 190);
    setGoalPosition(NeckYaw, 95);
    setGoalPosition(Body, 185);
    setGoalPosition(Tail, 190);
    state++;
  }

  if (ProfileT >= 14 && state == 14) {
    setGoalPosition(Jaw, 180);
    setGoalPosition(Skull, 190);
    setGoalPosition(NeckPitch, 170);
    setGoalPosition(NeckYaw, 105);
    setGoalPosition(Body, 175);
    setGoalPosition(Tail, 170);
    state++;
  }

  if (ProfileT >= 15 && state == 15) {
    setGoalPosition(Jaw, 210);
    setGoalPosition(Skull, 170);
    setGoalPosition(NeckPitch, 190);
    setGoalPosition(NeckYaw, 95);
    setGoalPosition(Body, 185);
    setGoalPosition(Tail, 190);
    state++;
  }

  if (ProfileT >= 16 && state == 16) {
    setGoalPosition(Jaw, 180);
    setGoalPosition(Skull, 190);
    setGoalPosition(NeckPitch, 170);
    setGoalPosition(NeckYaw, 105);
    setGoalPosition(Body, 175);
    setGoalPosition(Tail, 170);
    state++;
  }

  if (ProfileT >= 17 && state == 17) {
    setGoalPosition(Jaw, 210);
    setGoalPosition(Skull, 170);
    setGoalPosition(NeckPitch, 190);
    setGoalPosition(NeckYaw, 95);
    setGoalPosition(Body, 185);
    setGoalPosition(Tail, 190);
    state++;
  }

  if (ProfileT >= 18 && state == 18) {
    setGoalPosition(Jaw, 190);
    setGoalPosition(Skull, 180);
    setGoalPosition(NeckYaw, 100);
    setGoalPosition(Body, 180);
    setGoalPosition(Tail, 180);
    setGoalPosition(NeckPitch, 180);
    state++;
  }
}

// Main loop
void loop() {
  if (!modbusClient.connected()) {
    SerialUSB.println("Modbus connection lost. Attempting to reconnect...");
    if (!modbusClient.begin(plcIp)) {
      SerialUSB.println("Reconnection failed. Will retry...");
      digitalWrite(LED_PIN, HIGH);
      OffMode();
      delay(500);
      return;
    }
    SerialUSB.println("Reconnected to Modbus server.");
  }

  if (Auto) AutoMode();
  if (Maint) MaintMode();
  if (Off) OffMode();

  if (updateSpeed) {
    noInterrupts();
    updateSpeed = false;
    adjustAcceleration();
    interrupts();
  }
  
  // Heartbeat and Modbus Update
  Heart = !Heart;
  updateModbus();
  digitalWrite(LED_PIN, LOW);
  delay(10);
}
