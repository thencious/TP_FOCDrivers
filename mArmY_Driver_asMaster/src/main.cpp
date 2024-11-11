#include "Arduino.h"
#include "HardwareTimer.h"

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"

#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_it.h"
#include "stm32g4xx_hal_uart.h"

#define motorID 0xAA
#define nextMotorID 0xBA
float priorMotorAngle = 0.0f;

// Create a HardwareTimer instance for Timer 3
HardwareTimer timer(TIM3);
#define TIMEOUT_PERIOD 1 // Communication timeout period in milliseconds

// Define DE/RE pin for RS485 transceiver
#define RS485_nCS PA12

// // Gantry Y Instances ----------//
// #define ENC1_nCS PB9
// #define ENC2_nCS PA15

// // ENC SPI 3 Instance
// #define ENC_MOSI PB_5_ALT1
// #define ENC_MISO PB_4_ALT1
// #define ENC_SCK PC10

// // ENC1 Hardware ABZ Instance
// #define ENC1_PPR 16384
// #define ENC1_A PA_0
// #define ENC1_B PA_1
// #define ENC1_I PA_5

// Gantry X Instances ----------//
#define ENC1_nCS PC10
#define ENC2_nCS PA4

// // Gantry Z Instance ----------//
// #define ENC1_nCS PA4

// Shared X & Z Instances ----------//
// ENC SPI 1 Instance
#define ENC_MOSI PA_7
#define ENC_MISO PA_6
#define ENC_SCK PA_5

// ENC1 Hardware ABZ Instance
#define ENC1_PPR 16384
#define ENC1_A PB_4
#define ENC1_B PB_5
#define ENC1_I PB_3

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

SPIClass SPI_1(ENC_MOSI, ENC_MISO, ENC_SCK); // (MOSI, MISO, SCK)

SPISettings myMT6835SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 Enc1_SPI = MagneticSensorMT6835(ENC1_nCS, myMT6835SPISettings);

STM32HWEncoder Enc1_ABZ = STM32HWEncoder(ENC1_PPR, ENC1_A, ENC1_B, ENC1_I);

// // // Stepper motor instance --------------//
StepperMotor motor = StepperMotor(50, 0.9, 74.5, 0.002); // StepperMotor(int pp, (optional R, KV))
// // // Stepper driver instance
StepperDriver4PWM driver = StepperDriver4PWM(PWM_A0, PWM_A1, PWM_B0, PWM_B1, DRV_SLEEP, DRV_SLEEP);

float currentTarget = 0;

// // InlineCurrentSense(shunt_resistance, gain, adc_a, adc_b)
// InlineCurrentSense current_sense = InlineCurrentSense(0.005, 100.0f, PB11, PB12);

void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
void USART1_IRQHandler(void);

UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim3;

void writeMT6835(uint16_t reg, uint8_t data);
void DMA_sendFloat(float dataf);

void commComplete(void);
void handleTimeout(void);

#define RX_BUFFER_SIZE 6
uint8_t dma_rx_buffer[RX_BUFFER_SIZE];
DMA_HandleTypeDef hdma_usart1_rx;

#define TX_BUFFER_SIZE 6
uint8_t dma_tx_buffer[TX_BUFFER_SIZE];
DMA_HandleTypeDef hdma_usart1_tx;

void setup()
{
  delay(1000);

  /* Configure the system clock */
  SystemClock_Config();

  HAL_Init();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();

  // Disable UART before configuring character match settings
  __HAL_UART_DISABLE(&huart1);

  huart1.Instance->CR2 &= ~USART_CR2_ADD_Msk;             // Clear any existing character match value
  huart1.Instance->CR2 |= (motorID << USART_CR2_ADD_Pos); // Set the character match value
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_CM);              // Enable the character match interrupt

  // Enable UART after configuring
  __HAL_UART_ENABLE(&huart1);

  // Start DMA reception on UART1
  HAL_UART_Receive_DMA(&huart1, dma_rx_buffer, RX_BUFFER_SIZE);

  // Enc1 SPI setup --------------------------------//

  pinMode(ENC1_nCS, OUTPUT);
  digitalWrite(ENC1_nCS, HIGH); // Keep CS high initially

  SPI_1.begin();
  SPI_1.beginTransaction(myMT6835SPISettings); // Mode 3
  Enc1_SPI.init(&SPI_1);

  // // Enc1 Setup (won't be needed if ran Auto Calibration)
  // Enc1_SPI.setABZResolution(ENC1_PPR-1); // Set enc1 ABZ resolution (after enco1 init through SPI3)
  // delay(100);
  // writeMT6835(0x0011, 0x07); // set Bandwidth to 0x7 (no filter)
  // delay(100);
  // writeMT6835(0x000D, 0b00001100); // set hysteresis to 0x4 (0)
  // delay(100);

  // // Enc1 ABZ setup --------------------------------//
  Enc1_ABZ.init();

  // initialize encoder sensor hardware
  motor.linkSensor(&Enc1_ABZ); //use ABZ which is much faster and handled via hardware but with lower definition and isn't absolute. Therefore we will want to append this information to ABZ on startup.
  // motor.linkSensor(&Enc1_SPI);

  // pinMode(DRV_SLEEP, OUTPUT);
  pinMode(DRV_TOFF, OUTPUT);
  pinMode(DRV_DECAY, OUTPUT);

  // digitalWrite(DRV_SLEEP, HIGH); // Driver chip awake
  // digitalWrite(DRV_SLEEP, LOW);  // Driver chip sleep
  digitalWrite(DRV_TOFF, HIGH); // 14ms current chopping
  digitalWrite(DRV_DECAY, HIGH); // Dynamic Control (see more on DRV8434P datasheet)

  // driver setup --------------------------------//
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 47500;
  // power supply voltage [V]
  driver.voltage_power_supply = 36.0f;
  driver.init();

  // // link current sense and driver
  // current_sense.linkDriver(&driver);

  // // link the motor to the sensor
  motor.linkDriver(&driver);

  // // choose FOC modulation - SinePWM or SpaceVectorPWM
  motor.foc_modulation = FOCModulationType::SinePWM;
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // control loop type to be used --------------------------------------//
  motor.controller = MotionControlType::torque;
  // motor.controller = MotionControlType::velocity;
  // motor.torque_controller = TorqueControlType::foc_current;
  // motor.controller = MotionControlType::velocity_openloop;

  // // controller configuration based on the control type
  // // velocity PID controller parameters
  // // default P=0.5 I = 10 D =0
  // motor.PID_velocity.P = 0.2;
  // motor.PID_velocity.I = 20;
  // motor.PID_velocity.D = 0.001;
  // // jerk control using voltage voltage ramp
  // // default value is 300 volts per sec  ~ 0.3V per millisecond
  // motor.PID_velocity.output_ramp = 1000;

  // // velocity low pass filtering
  // // default 5ms - try different values to see what is the best.
  // // the lower the less filtered
  // motor.LPF_velocity.Tf = 0.01;

  // // limiting voltage
  // motor.voltage_limit = 3; // Volts
  // or current  - if phase resistance provided
  motor.current_limit = 2.0f; // Max 3 Amps, but don't go there until a bigger heatsink is added

  // // angle loop controller
  // motor.P_angle.P = 20;
  // angle loop velocity limit
  // motor.velocity_limit = 10;

  // initialise motor
  motor.init();

  // // init current sense
  // current_sense.init();

  // // link motor and current sense
  // motor.linkCurrentSense(&current_sense);

  // sensor offset [rad]
  motor.sensor_offset = Enc1_SPI.getAngle();
  // motor.zero_electric_angle = 0;      // to be calculated and set after SPI has been zeroed with zeroing rig as to skip sensor alignment step which turns the motor 1/50 of a rotation to search for an ok electric zero angle.
  motor.sensor_direction = Direction::CW; // CW or CCW along the motor axis (not facing it)
  // align encoder and start FOC (skipped if the previous two statements are entered)
  motor.initFOC(); // this will run sensor alignment (try to find zero electric angle) if zero_electric_angle and sensor_direction are not both set.

  // set the initial target value
  motor.target = currentTarget;

  // downsampling value
  motor.motion_downsample = 0; // - runs torque loop x times more than control loop.

  // Configure Timer 3 for timeout monitoring
  timer.setOverflow(TIMEOUT_PERIOD, MICROSEC_FORMAT);  // Set timer overflow frequency
  timer.attachInterrupt(handleTimeout);                // Attach the timeout handler
  timer.resume();                                  // Start the timer
  handleTimeout();

  // delay(1000); // the master would want to have a long enough delay as to wait for all other boards to power up.
}

void loop()
{
  motor.loopFOC();
  motor.move(10 * (priorMotorAngle - motor.shaft_angle));
}

//interrupt callbacks ---------------------------------//

void USART1_IRQHandler(void)
{
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_CMF)) // If character match interrupt flag is set
  {
    // Clear character match flag
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_CMF);

    if (dma_rx_buffer[0] == 0xF0) // Check start byte
    {
      DMA_sendFloat(Enc1_ABZ.getAngle());
      commComplete();
      // Extract the float bytes (assuming little-endian)
      uint8_t floatBytes[4];
      floatBytes[0] = dma_rx_buffer[4];
      floatBytes[1] = dma_rx_buffer[3];
      floatBytes[2] = dma_rx_buffer[2];
      floatBytes[3] = dma_rx_buffer[1];

      // Convert bytes back to float
      memcpy(&priorMotorAngle, floatBytes, sizeof(priorMotorAngle)); // Copy bytes to a float variable
    }
  }

  // Handle other UART interrupts
  HAL_UART_IRQHandler(&huart1);
}

// functions ------------------------------------------//

void DMA_sendFloat(float dataf)
{
  // // Convert the float value to bytes and add it to the buffer
  uint8_t *angleBytes = reinterpret_cast<uint8_t *>(&dataf);

  // Add the Start byte
  dma_tx_buffer[0] = 0xF0; // Start

  // Fill the buffer with angle bytes (4 bytes for float)
  dma_tx_buffer[4] = angleBytes[0];
  dma_tx_buffer[3] = angleBytes[1];
  dma_tx_buffer[2] = angleBytes[2];
  dma_tx_buffer[1] = angleBytes[3];

  // Add the next motor Character Match byte
  dma_tx_buffer[5] = nextMotorID;

  HAL_UART_Transmit_DMA(&huart1, dma_tx_buffer, TX_BUFFER_SIZE);
}

void commComplete()
{
  timer.setCount(0);
}

void handleTimeout()
{
  // restarts rs485 loop
  DMA_sendFloat(Enc1_ABZ.getAngle());
  timer.setCount(0);
}

void writeMT6835(uint16_t reg, uint8_t data)
{
  // Combine the 4-bit write command (0110) with the 12-bit register address into a 16-bit value
  uint16_t command_and_reg = (0b0110 << 12) | (reg & 0x0FFF); // 0x6 is the write command (0110)

  digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

  // Send the combined 16-bit value as two 8-bit parts
  SPI_1.transfer((command_and_reg >> 8) & 0xFF); // Send the upper 8 bits (command + upper part of address)
  SPI_1.transfer(command_and_reg & 0xFF);        // Send the lower 8 bits (lower part of address)

  // Write the 8-bit data (D7~D0)
  SPI_1.transfer(data);

  digitalWrite(ENC1_nCS, HIGH); // Pull CSN high to end communication
}

bool zeroMT6835()
{
  uint8_t data;
  // Combine the 4-bit write command (0110) with the 12-bit register address into a 16-bit value
  uint16_t command = 0b0101000000000000; // 0x6 is the write command (0110)

  digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

  // Send the combined 16-bit value as two 8-bit parts
  SPI_1.transfer((command >> 8) & 0xFF); // Send the upper 8 bits (command + upper part of address)
  SPI_1.transfer(command & 0xFF);        // Send the lower 8 bits (lower part of address)

  // Read the 8-bit data (D7~D0) from MISO
  data = SPI_1.transfer(0x00);

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

bool writeRomMT6835()
{
  uint8_t data;
  // Combine the 4-bit write command (0110) with the 12-bit register address into a 16-bit value
  uint16_t command_and_reg = 0b1100000000000000; // 0x6 is the write command (0110)

  digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

  // Send the combined 16-bit value as two 8-bit parts
  SPI_1.transfer((command_and_reg >> 8) & 0xFF); // Send the upper 8 bits (command + upper part of address)
  SPI_1.transfer(command_and_reg & 0xFF);        // Send the lower 8 bits (lower part of address)

  // Read the 8-bit data (D7~D0) from MISO
  data = SPI_1.transfer(0x00);

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

//HAL Init ----------------------------------------------------------------------------//

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
// #ifdef USBCON
//   RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
// #endif

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
  // RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  // RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  // RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  // Enable the USART1 IRQ in the NVIC
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 1); // Set priority
  HAL_NVIC_EnableIRQ(USART1_IRQn);         // Enable the IRQ in the NVIC
  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

// Timer 3 Initialization
void MX_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE(); // Enable the Timer 3 clock

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84 - 1; // Prescaler to reduce the timer clock (assuming 84 MHz system clock)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000 - 1; // Set period for 5 ms interrupt (or desired timeout duration)
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  // Enable the interrupt in NVIC
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

// /**
//  * @brief UART MSP Initialization
//  * This function configures the hardware resources used in this example
//  * @param huart: UART handle pointer
//  * @retval None
//  */
// void HAL_UART_MspInit(UART_HandleTypeDef *huart)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
//   RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
//   if (huart->Instance == USART1)
//   {
//     /* USER CODE BEGIN USART1_MspInit 0 */

//     /* USER CODE END USART1_MspInit 0 */

//     /** Initializes the peripherals clocks
//      */
//     PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
//     PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
//     if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//     {
//       Error_Handler();
//     }

//     /* Peripheral clock enable */
//     __HAL_RCC_USART1_CLK_ENABLE();

//     __HAL_RCC_GPIOA_CLK_ENABLE();
//     __HAL_RCC_GPIOB_CLK_ENABLE();
//     /**USART1 GPIO Configuration
//     PA12     ------> USART1_DE
//     PB6     ------> USART1_TX
//     PB7     ------> USART1_RX
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_12;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//     /* USART1 interrupt Init */
//     HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
//     HAL_NVIC_EnableIRQ(USART1_IRQn);
//     /* USER CODE BEGIN USART1_MspInit 1 */

//     /* USER CODE END USART1_MspInit 1 */
//   }
// }

// /**
//  * @brief UART MSP De-Initialization
//  * This function freeze the hardware resources used in this example
//  * @param huart: UART handle pointer
//  * @retval None
//  */
// void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
// {
//   if (huart->Instance == USART1)
//   {
//     /* USER CODE BEGIN USART1_MspDeInit 0 */

//     /* USER CODE END USART1_MspDeInit 0 */
//     /* Peripheral clock disable */
//     __HAL_RCC_USART1_CLK_DISABLE();

//     /**USART1 GPIO Configuration
//     PA12     ------> USART1_DE
//     PB6     ------> USART1_TX
//     PB7     ------> USART1_RX
//     */
//     HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);

//     HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);

//     /* USART1 interrupt DeInit */
//     HAL_NVIC_DisableIRQ(USART1_IRQn);
//     /* USER CODE BEGIN USART1_MspDeInit 1 */

//     /* USER CODE END USART1_MspDeInit 1 */
//   }
// }

// /**
//  * Initializes the Global MSP.
//  */
// void HAL_MspInit(void)
// {

//   /* USER CODE BEGIN MspInit 0 */

//   /* USER CODE END MspInit 0 */

//   __HAL_RCC_SYSCFG_CLK_ENABLE();
//   __HAL_RCC_PWR_CLK_ENABLE();

//   /* System interrupt init*/

//   /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
//    */
//   HAL_PWREx_DisableUCPDDeadBattery();

//   /* USER CODE BEGIN MspInit 1 */

//   /* USER CODE END MspInit 1 */
// }
