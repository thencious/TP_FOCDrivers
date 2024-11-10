//// FOC driver Test-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- //
#include "Arduino.h"

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"

#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"

#include "HardwareSerial.h"

// Define custom UART pins for RS485 communication
HardwareSerial Serial485(PB7, PB6); // TX, RX

// Define DE/RE pin for RS485 transceiver
#define RS485_CONTROL_PIN PA12

// Gantry Y Instance ----------//
#define ENC1_nCS PB9
#define ENC2_nCS PA15

// ENC SPI 3 Instance
#define ENC_MOSI PB_5_ALT1
#define ENC_MISO PB_4_ALT1
#define ENC_SCK PC10

// ENC1 HW ABZ Instance
#define ENC1_PPR 16384
#define ENC1_A PA_0
#define ENC1_B PA_1
#define ENC1_I PA_5

// // Gantry X Instances ----------//
// #define ENC1_nCS PC10
// #define ENC2_nCS PA4

// // Gantry Z Instance ----------//
// #define ENC1_nCS PA4

// // Shared X & Z Instances ----------//
// // ENC SPI 1 Instance
// #define ENC_MOSI PA_7
// #define ENC_MISO PA_6
// #define ENC_SCK PA_5

// // ENC Hardware ABZ Instance
// #define ENC1_PPR 16384
// #define ENC1_A PB_4
// #define ENC1_B PB_5
// #define ENC1_I PB_3

// HW ABZ Instance

// Shared Instances -----------------------//
// TIM1 PWM Instance
#define PWM_A0 PA_8
#define PWM_A1 PA_9
#define PWM_B0 PA_10
#define PWM_B1 PA_11

// Driver settings pin definition
#define DRV_TOFF PC6
#define DRV_SLEEP PB15
#define DRV_DECAY PB14

// Encoder settings pin definition
#define ENC1_CAL PC14

// Calibration state variable
enum CalibrationState
{
  CALIBRATION_STARTUP,
  CALIBRATION_START,
  CALIBRATION_IN_PROGRESS,
  CALIBRATION_FINISH,
  CALIBRATION_DONE
};

CalibrationState calibrationState = CALIBRATION_STARTUP;

SPIClass SPI_3(ENC_MOSI, ENC_MISO, ENC_SCK); // (MOSI, MISO, SCK)

SPISettings myMT6835SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 Enc1_SPI = MagneticSensorMT6835(ENC1_nCS, myMT6835SPISettings);

// // Stepper motor instance --------------//
// StepperMotor motor = StepperMotor(50, 2.0, 14); // StepperMotor(int pp, (optional R, KV))
StepperMotor motor = StepperMotor(50, 0.9, 40);
// // Stepper driver instance
StepperDriver4PWM driver = StepperDriver4PWM(PWM_A0, PWM_A1, PWM_B0, PWM_B1);

void startCalibration();
void finishCalibration();
void writeMT6835(uint16_t reg, uint8_t data);
void checkCalibrationStatus();
uint8_t readMT6835(uint16_t reg);
bool writeRomMT6835();
uint32_t burstReadMT6835();

// // commander interface
// Commander command = Commander(Serial485);
// void onMotor(char *cmd) { command.motor(&motor, cmd); }

void setup()
{
  //* Check before running code ! ------------------------------------------*//
  bool NEW_SENSOR_PAIR = false;

  // Initialize the RS485 control pin
  pinMode(RS485_CONTROL_PIN, OUTPUT);
  // Set to receive mode initially
  digitalWrite(RS485_CONTROL_PIN, HIGH);

  // Initialize UART communication at 115200 baud
  Serial485.begin(115200);

  delay(5000); // Wait for serial monitor to open
  Serial485.println("RS485 communication initialized.");

  // Enable debugging to Serial485
  SimpleFOCDebug::enable(&Serial485);
  Serial485.println("SimpleFOCDebug enabled.");

  // Enc1 SPI setup --------------------------------//

  pinMode(ENC1_nCS, OUTPUT);
  digitalWrite(ENC1_nCS, HIGH); // Keep CS high initially

  pinMode(ENC1_CAL, OUTPUT);
  digitalWrite(ENC1_CAL, LOW); // Start with calibration disabled

  SPI_3.begin();
  // Set SPI mode to Mode 3
  SPI_3.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // 1 MHz SPI clock, Mode 3
  Enc1_SPI.init(&SPI_3);
  Serial485.println("SPI3 Initialized");

  // // Set enc1 ABZ resolution (after enco1 init through SPI3)
  // Enc1_SPI.setABZResolution(ENC1_PPR - 1);

  pinMode(DRV_SLEEP, OUTPUT);
  pinMode(DRV_TOFF, OUTPUT);
  pinMode(DRV_DECAY, OUTPUT);

  digitalWrite(DRV_SLEEP, HIGH);
  digitalWrite(DRV_TOFF, HIGH);
  digitalWrite(DRV_DECAY, HIGH);

  // digitalWrite(DRV_SLEEP, LOW);
  // digitalWrite(DRV_TOFF, LOW);
  // digitalWrite(DRV_DECAY, LOW);

  // driver setup
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 47500;
  // power supply voltage [V]
  driver.voltage_power_supply = 36.0f;
  driver.init();

  // choose FOC modulation (optional) - SinePWM or SpaceVectorPWM
  motor.foc_modulation = FOCModulationType::SinePWM;

  // link the motor to the sensor
  motor.linkDriver(&driver);

  // // // limiting voltage
  // // motor.voltage_limit = 3; // Volts
  // or current  - if phase resistance provided
  motor.current_limit = 2.8f; // Amps

  // Set the motor to open-loop velocity mode
  motor.controller = MotionControlType::velocity_openloop;

  // initialise motor
  motor.init();

  // set the initial target value
  motor.target = -20;

  if (NEW_SENSOR_PAIR == true)
  {
    Serial485.println("Sensor setup begining...");
    delay(1000);

    writeMT6835(0x0011, 0x07); // set Bandwidth to 0x7 (no filter)
    Serial485.println("Sensor Bandwidth set to 0x7 (no filter)");
    delay(100);
    writeMT6835(0x000D, 0b00001100); // set hysteresis to 0x4 (0)
    Serial485.println("Sensor hysteresis removed");
    delay(100);
    // writeMT6835(0x000E, 0b01000000); // set autocal_frequency to 0x4 (200-400 RPM)
    // Serial485.println("MT6835 autocal freq set to 200-400 RPM");
    writeMT6835(0x000E, 0b01010000); // set autocal_frequency to 0x5 (100-200 RPM)
    Serial485.println("Sensor autocal freq set to 100-200 RPM");
    delay(100);

    if (writeRomMT6835())
    {
      Serial485.println("Sensor EEPROM saved, please wait 6s before poweroff");
      delay(6000);
    }
    else
    {
      Serial485.println("Sensor EEPROM save failed");
    }
  }

  // align encoder and start FOC
  motor.initFOC();

  Serial485.println("Starting MT6835 Auto calibration, please wait 35s...");
}

void loop()
{
  motor.loopFOC();
  motor.move();

  static unsigned long calibrationStartMillis = millis();
  unsigned long currentMillis = millis();

  switch (calibrationState)
  {
  case CALIBRATION_STARTUP:
    // Wait 2 seconds for calibration to start
    if (currentMillis - calibrationStartMillis >= 2000)
    {
      calibrationState = CALIBRATION_START;
    }
    break;

  case CALIBRATION_START:
    startCalibration();
    calibrationStartMillis = currentMillis;
    calibrationState = CALIBRATION_IN_PROGRESS;
    break;

  case CALIBRATION_IN_PROGRESS:
    // Wait 30 seconds for calibration to complete
    if (currentMillis - calibrationStartMillis >= 33000)
    {
      calibrationState = CALIBRATION_FINISH;
    }
    break;

  case CALIBRATION_FINISH:
    finishCalibration();
    checkCalibrationStatus();
    calibrationState = CALIBRATION_DONE;
    break;

  default:
    break;
  }
}

void startCalibration()
{
  // Step 2: Enable Calibration (pull CAL_EN high)
  digitalWrite(ENC1_CAL, HIGH);
}

void finishCalibration()
{
  // Disable Calibration (pull CAL_EN low)
  digitalWrite(ENC1_CAL, LOW);
}

void writeMT6835(uint16_t reg, uint8_t data)
{
  // Combine the 4-bit write command (0110) with the 12-bit register address into a 16-bit value
  uint16_t command_and_reg = (0b0110 << 12) | (reg & 0x0FFF); // 0x6 is the write command (0110)

  digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

  // Send the combined 16-bit value as two 8-bit parts
  SPI_3.transfer((command_and_reg >> 8) & 0xFF); // Send the upper 8 bits (command + upper part of address)
  SPI_3.transfer(command_and_reg & 0xFF);        // Send the lower 8 bits (lower part of address)

  // Write the 8-bit data (D7~D0)
  SPI_3.transfer(data);

  digitalWrite(ENC1_nCS, HIGH); // Pull CSN high to end communication
}

bool writeRomMT6835()
{
  uint8_t data;
  // Combine the 4-bit write command (0110) with the 12-bit register address into a 16-bit value
  uint16_t command_and_reg = 0b1100000000000000; // 0x6 is the write command (0110)

  digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

  // Send the combined 16-bit value as two 8-bit parts
  SPI_3.transfer((command_and_reg >> 8) & 0xFF); // Send the upper 8 bits (command + upper part of address)
  SPI_3.transfer(command_and_reg & 0xFF);        // Send the lower 8 bits (lower part of address)

  // Read the 8-bit data (D7~D0) from MISO
  data = SPI_3.transfer(0x00);

  digitalWrite(ENC1_nCS, HIGH); // Pull CSN high to end communication

  if (data==0x55)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

bool zeroMT6835()
{
  uint8_t data;
  // Combine the 4-bit write command (0110) with the 12-bit register address into a 16-bit value
  uint16_t command_and_reg = 0b0101000000000000; // 0x6 is the write command (0110)

  digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

  // Send the combined 16-bit value as two 8-bit parts
  SPI_3.transfer((command_and_reg >> 8) & 0xFF); // Send the upper 8 bits (command + upper part of address)
  SPI_3.transfer(command_and_reg & 0xFF);        // Send the lower 8 bits (lower part of address)

  // Read the 8-bit data (D7~D0) from MISO
  data = SPI_3.transfer(0x00);

  digitalWrite(ENC1_nCS, HIGH); // Pull CSN high to end communication

  if (data == 0x55)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

uint8_t readMT6835(uint16_t reg)
{
  uint8_t data;

  // Combine the 4-bit read command (0011) with the 12-bit register address into a 16-bit value
  uint16_t command_and_reg = (0b0011 << 12) | (reg & 0x0FFF); // 0x3 is the read command (0011)

  digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

  // Send the combined 16-bit value as two 8-bit parts
  SPI_3.transfer((command_and_reg >> 8) & 0xFF); // Send the upper 8 bits (command + upper part of address)
  SPI_3.transfer(command_and_reg & 0xFF);        // Send the lower 8 bits (lower part of address)

  // Read the 8-bit data (D7~D0) from MISO
  data = SPI_3.transfer(0x00);

  digitalWrite(ENC1_nCS, HIGH); // Pull CSN high to end communication

  return data;
}

void checkCalibrationStatus()
{
  uint8_t status = readMT6835(0x113); // Read the entire 0x113 register

  // Mask the top two bits (7:6) to get the calibration status
  uint8_t cal_status = (status >> 6) & 0x03; // Shift right by 6 bits and mask to get only two bits

  switch (cal_status)
  {
  case 0x00:
    Serial485.println("No Cal");
    break;
  case 0x01:
    Serial485.println("Calibrating");
    break;
  case 0x02:
    Serial485.println("Cal Fail");
    break;
  case 0x03:
    Serial485.println("Cal Success");
    break;
  }

  // Optionally print the full status byte if needed for debugging
  Serial485.println(cal_status);
}

uint32_t burstReadMT6835()
{
  uint32_t angle_data = 0;

  digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

  // Send the 4-bit burst read command (1010) + 12-bit address (0x003)
  SPI_3.transfer(0b10100000); // Command (0b1010) + upper bits of address (0x00)
  SPI_3.transfer(0x30);       // Lower 8 bits of address (0x003)

  // Read the 24-bit angle data (3 bytes from registers 0x003 to 0x006)
  uint8_t byte1 = SPI_3.transfer(0x00); // Read first byte (from register 0x003)
  uint8_t byte2 = SPI_3.transfer(0x00); // Read second byte (from register 0x004)
  uint8_t byte3 = SPI_3.transfer(0x00); // Read third byte (from register 0x005)

  digitalWrite(ENC1_nCS, HIGH); // Pull CSN high to end communication

  // Combine the 3 bytes into a 24-bit result
  angle_data = ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | (uint32_t)byte3;

  return angle_data; // Return the 24-bit angle data
}