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
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2;
#endif

const uint8_t DXL_IDS[] = {10, 15, 40, 50};
const float DXL_PROTOCOL_VERSION = 2.0;
const uint16_t MODBUS_POSITION_ADDRS[] = {0, 1, 2, 3, 4, 5, 6};
const uint16_t MODBUS_CURRENT_ADDRS[] = {7, 8, 9, 10, 11, 12};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// Global variable to track profile completion
volatile bool ProfileComplete = false;

volatile int state = 0;
volatile int coilState = 0;

// Timer states
volatile bool timerRunning = false; // To track if the timer is running or paused
volatile uint32_t timerStartValue = 0; // To store the start value when the timer is paused
volatile uint32_t lastTimerCount = 0;  // To store the last timer count when paused
volatile uint32_t ProfileT = 0;

// Timer Setup
void setupTimer() {
  // Enable the clock for TC4 and TC5 as a 32-bit timer
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TC4_TC5) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY); // Wait for sync

   // Configure TC4 as a 32-bit counter
  TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV1024;

  // Enable the timer
  TC4->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
   while (TC4->COUNT32.STATUS.bit.SYNCBUSY); // Wait for sync

   DEBUG_SERIAL.println("32-bit Timer Configured.");
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL);

  Ethernet.begin(mac, ip);
  delay(1000);

  if (!modbusClient.begin(plcIp)) {
    DEBUG_SERIAL.println("Initial connection to Modbus server failed.");
  } else {
    DEBUG_SERIAL.println("Connected to Modbus server.");
  }

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    
  for (uint8_t id : DXL_IDS) {
    dxl.ping(id);
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.torqueOn(id);
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, 20);
   }

  // Initialize the timer
  setupTimer();
}

// Function to update modbus with data and also debug print
void updateModbus() {
   // ~-~-~ WRITE SECTION ~-~-~
  for (int i = 0; i < 4; i++) {
    int32_t currentPosition = dxl.getPresentPosition(DXL_IDS[i], UNIT_DEGREE);
    float currentReading = dxl.getPresentCurrent(DXL_IDS[i], UNIT_MILLI_AMPERE);

    modbusClient.holdingRegisterWrite(MODBUS_POSITION_ADDRS[i], currentPosition);
    modbusClient.holdingRegisterWrite(MODBUS_CURRENT_ADDRS[i], static_cast<int32_t>(currentReading));
  }
  modbusClient.holdingRegisterWrite(13, state);

  // ~-~-~ READ SECTION ~-~-~
  coilState = modbusClient.coilRead(0);
  ProfileComplete = modbusClient.coilRead(1);
}

void loop() {
  // Check Modbus Status
  if (!modbusClient.connected()) {
   DEBUG_SERIAL.println("Modbus connection lost. Attempting to reconnect...");
  if (!modbusClient.begin(plcIp)) {
    DEBUG_SERIAL.println("Reconnection failed. Will retry...");
    delay(1000);
    return;
  }
    DEBUG_SERIAL.println("Reconnected to Modbus server.");
  }

  if (coilState == 1) {
    // Start the timer if it is not already running
    if (!timerRunning) {
      // If we are starting the timer from a pause, resume from lastTimerCount
      timerStartValue = TC4->COUNT32.COUNT.reg - lastTimerCount; // Adjust the start value to resume where it left off
      timerRunning = true;
      DEBUG_SERIAL.println("Timer Started.");
    }
  } else {
    // Pause the timer if coil state is 0
    if (timerRunning) {
      lastTimerCount = TC4->COUNT32.COUNT.reg - timerStartValue; // Store the elapsed time when paused
      timerRunning = false;
      DEBUG_SERIAL.println("Timer Paused.");
      if (state > 1) state--;
      }
      for (uint8_t id : DXL_IDS) {
        int32_t currentPosition = dxl.getPresentPosition(id, UNIT_DEGREE);
        dxl.setGoalPosition(id, currentPosition, UNIT_DEGREE);  // Hold the current position
      }
  }

  // If the timer is running, check the elapsed time
  if (timerRunning) {
    uint32_t currentTimerValue = TC4->COUNT32.COUNT.reg;
    uint32_t elapsedTime = currentTimerValue - timerStartValue; // Elapsed time in timer counts

    // Convert timer counts to seconds
    float elapsedSeconds = elapsedTime / 46875.0;
    ProfileT = elapsedSeconds;

    // Debug output
    DEBUG_SERIAL.print("Elapsed Time: ");
    DEBUG_SERIAL.print(elapsedSeconds, 3);
    DEBUG_SERIAL.println(" sec");

    // If the timer reaches 15 seconds, set ProfileComplete = 1 and reset the timer
    if (elapsedSeconds >= 20.0 && !ProfileComplete) {
      modbusClient.coilWrite(1, 1);
      state = 0;
      DEBUG_SERIAL.println("Profile Complete! Timer Reset.");
      timerStartValue = currentTimerValue; // Reset timer start value
      ProfileT = 0;
      }
    }



    // Profile Begins Here
    if (ProfileT > 0 && state == 0){
      dxl.setGoalPosition(15, 210, UNIT_DEGREE);
      dxl.setGoalPosition(10, 210, UNIT_DEGREE);
      dxl.setGoalPosition(40, 210, UNIT_DEGREE);
      dxl.setGoalPosition(50, 210, UNIT_DEGREE);
      state++;
    }

    if (ProfileT >= 2 && state == 1){
      dxl.setGoalPosition(15, 150, UNIT_DEGREE);
      dxl.setGoalPosition(10, 150, UNIT_DEGREE);
      dxl.setGoalPosition(40, 150, UNIT_DEGREE);
      dxl.setGoalPosition(50, 150, UNIT_DEGREE);
      state++;
    }

    if (ProfileT >= 4 && state == 2){
      dxl.setGoalPosition(15, 210, UNIT_DEGREE);
      dxl.setGoalPosition(10, 210, UNIT_DEGREE);
      dxl.setGoalPosition(40, 210, UNIT_DEGREE);
      dxl.setGoalPosition(50, 210, UNIT_DEGREE);
      state++;
    }

    if (ProfileT >= 6 && state == 3){
      dxl.setGoalPosition(15, 150, UNIT_DEGREE);
      dxl.setGoalPosition(10, 150, UNIT_DEGREE);
      dxl.setGoalPosition(40, 150, UNIT_DEGREE);
      dxl.setGoalPosition(50, 150, UNIT_DEGREE);
      state++;
    }

    if (ProfileT >= 8 && state == 4){
      dxl.setGoalPosition(15, 210, UNIT_DEGREE);
      dxl.setGoalPosition(10, 210, UNIT_DEGREE);
      dxl.setGoalPosition(40, 210, UNIT_DEGREE);
      dxl.setGoalPosition(50, 210, UNIT_DEGREE);
      state++;
    }

    if (ProfileT >= 10 && state == 5){
      dxl.setGoalPosition(15, 230, UNIT_DEGREE);
      dxl.setGoalPosition(10, 230, UNIT_DEGREE);
      dxl.setGoalPosition(40, 230, UNIT_DEGREE);
      dxl.setGoalPosition(50, 230, UNIT_DEGREE);
      state++;
    }

    if (ProfileT >= 12 && state == 6){
      dxl.setGoalPosition(15, 150, UNIT_DEGREE);
      dxl.setGoalPosition(10, 150, UNIT_DEGREE);
      dxl.setGoalPosition(40, 150, UNIT_DEGREE);
      dxl.setGoalPosition(50, 150, UNIT_DEGREE);
      state++;
    }

    if (ProfileT >= 14 && state == 7){
      dxl.setGoalPosition(15, 210, UNIT_DEGREE);
      dxl.setGoalPosition(10, 210, UNIT_DEGREE);
      dxl.setGoalPosition(40, 210, UNIT_DEGREE);
      dxl.setGoalPosition(50, 210, UNIT_DEGREE);
      state++;
    }


    if (ProfileT >= 16 && state == 8){
      dxl.setGoalPosition(15, 150, UNIT_DEGREE);
      dxl.setGoalPosition(10, 150, UNIT_DEGREE);
      dxl.setGoalPosition(40, 150, UNIT_DEGREE);
      dxl.setGoalPosition(50, 150, UNIT_DEGREE);
      state++;
    }

    if (ProfileT >= 18 && state == 9){
      dxl.setGoalPosition(15, 180, UNIT_DEGREE);
      dxl.setGoalPosition(10, 180, UNIT_DEGREE);
      dxl.setGoalPosition(40, 180, UNIT_DEGREE);
      dxl.setGoalPosition(50, 180, UNIT_DEGREE);
      state++;
    }

    updateModbus();
    delay(10);
}