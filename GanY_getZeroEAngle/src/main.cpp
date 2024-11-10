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

// // Gantry X Instance ----------//
// #define ENC1_nCS PC10
// #define ENC2_nCS PA4

// // Gantry Z Instance ----------//
// #define ENC1_nCS PA4

// // Shared X & Z Instance ----------//
// SPI 1 Instance

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

SPIClass SPI_3(ENC_MOSI, ENC_MISO, ENC_SCK); // (MOSI, MISO, SCK)

SPISettings myMT6835SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 Enc1_SPI = MagneticSensorMT6835(ENC1_nCS, myMT6835SPISettings);

STM32HWEncoder Enc1_ABZ = STM32HWEncoder(ENC1_PPR, ENC1_A, ENC1_B, ENC1_I);

// // Stepper motor instance --------------//
// StepperMotor motor = StepperMotor(50, 2.0, 14); // StepperMotor(int pp, (optional R, KV))
StepperMotor motor = StepperMotor(50, 0.9, 40);
// // Stepper driver instance
StepperDriver4PWM driver = StepperDriver4PWM(PWM_A0, PWM_A1, PWM_B0, PWM_B1);

//  InlineCurrentSense(shunt_resistance, gain, adc_a, adc_b)
// InlineCurrentSense current_sense = InlineCurrentSense(0.005, 100.0f, PB11, PB12);

void startCalibration();
void finishCalibration();
void writeMT6835(uint16_t reg, uint8_t data);
void checkCalibrationStatus();
uint8_t readMT6835(uint16_t reg);
bool writeRomMT6835();
uint32_t burstReadMT6835();

// Commander interface
Commander command = Commander(Serial485);
void onPrintEAngle(char *cmd)
{
  float eAngle = motor.electrical_angle;
  float Angle = Enc1_SPI.getAngle();
  digitalWrite(RS485_CONTROL_PIN, HIGH);
  Serial485.print("E: ");
  Serial485.println(eAngle,7);
  Serial485.print("A: ");
  Serial485.println(Angle,7);
  Serial485.flush();
  digitalWrite(RS485_CONTROL_PIN, LOW);
}

void setup()
{
  // Initialize the RS485 control pin
  pinMode(RS485_CONTROL_PIN, OUTPUT);
  // Set to send mode initially
  digitalWrite(RS485_CONTROL_PIN, HIGH);

  // Initialize UART communication at 115200 baud
  Serial485.begin(115200);

  delay(5000); // Wait for serial monitor to open up
  Serial485.println("RS485 communication initialized.");

  command.add('E', onPrintEAngle);

  // Enable debugging to Serial485
  // SimpleFOCDebug::enable(&Serial485);
  // Serial485.println("SimpleFOCDebug enabled.");

  // Enc1 SPI setup --------------------------------//

  SPI_3.begin();
  // Set SPI mode to Mode 3
  SPI_3.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // 1 MHz SPI clock, Mode 3
  Enc1_SPI.init(&SPI_3);
  Serial485.println("SPI3 Initialized");

  // // Enc1 ABZ setup --------------------------------//
  Enc1_ABZ.init();

  motor.linkSensor(&Enc1_SPI);

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
  // motor.controller = MotionControlType::velocity_openloop;
  // motor.controller = MotionControlType::torque;
  motor.controller = MotionControlType::angle;

  // controller configuration based on the control type
  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller -  default P=20
  motor.P_angle.P = 20;

  // initialise motor
  motor.init();

  Serial485.println("Sensor setup begining...");
  delay(100);

  // align encoder and start FOC
  motor.initFOC();
  motor.loopFOC();
  motor.move();

  Serial485.print("MT6835 Zero electric angle: ");
  Serial485.println(motor.zero_electric_angle, 7);

  Serial485.print("MT6835 sensor direction: ");
  Serial485.println(motor.sensor_direction);

  Serial485.print("MT6835 electrical angle: ");
  Serial485.println(motor.electrical_angle, 7);

  Serial485.print("MT6835 absolute angle: ");
  Serial485.println(Enc1_SPI.getAngle(), 7);

  Serial485.print("MT6835 ABZ angle: ");
  Serial485.println(Enc1_ABZ.getAngle(), 7);

  delay(500);

  // set the initial target value
  static float zeroEAngle = motor.zero_electric_angle / (-motor.sensor_direction * motor.pole_pairs);
  
  motor.target = zeroEAngle;

  digitalWrite(RS485_CONTROL_PIN, LOW);
}

void loop()
{
  motor.loopFOC();
  motor.move();
  command.run();
}

void startCalibration()
{
  // Step 2: Enable Calibration (pull CAL_EN high)
  digitalWrite(RS485_CONTROL_PIN, HIGH);
  Serial485.println("start Cal");
  Serial485.flush();
  digitalWrite(RS485_CONTROL_PIN, LOW);
  delay(5);
  digitalWrite(ENC1_CAL, HIGH);
}

void finishCalibration()
{
  // Disable Calibration (pull CAL_EN low)
  digitalWrite(ENC1_CAL, LOW);
  digitalWrite(RS485_CONTROL_PIN, HIGH);
  Serial485.println("end Cal");
  Serial485.flush();
  digitalWrite(RS485_CONTROL_PIN, LOW);
  delay(5);
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
    digitalWrite(RS485_CONTROL_PIN, HIGH);
    Serial485.println("No Cal");
    Serial485.flush();
    digitalWrite(RS485_CONTROL_PIN, LOW);
    break;
  case 0x01:
    digitalWrite(RS485_CONTROL_PIN, HIGH);
    Serial485.println("Calibrating");
    Serial485.flush();
    digitalWrite(RS485_CONTROL_PIN, LOW);
    break;
  case 0x02:
    digitalWrite(RS485_CONTROL_PIN, HIGH);
    Serial485.println("Cal Fail");
    Serial485.flush();
    digitalWrite(RS485_CONTROL_PIN, LOW);
    break;
  case 0x03:
    digitalWrite(RS485_CONTROL_PIN, HIGH);
    Serial485.println("Cal Success");
    Serial485.println("Please wait 6s or more before powering off");
    Serial485.flush();
    digitalWrite(RS485_CONTROL_PIN, LOW);
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