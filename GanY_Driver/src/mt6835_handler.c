// #include "stm32g4xx_hal.h"
// #include "mt6835_handler.h"
// #include "main.h"

// void startCalibration()
// {
//     // Step 2: Enable Calibration (pull CAL_EN high)
//     digitalWrite(ENC1_CAL, HIGH);
//     // Record the start time for calibration
//     startTime = millis();
// }

// void finishCalibration()
// {
//     // Disable Calibration (pull CAL_EN low)
//     digitalWrite(ENC1_CAL, LOW);

//     // Check calibration status (for feedback)
//     checkCalibrationStatus();
// }

// void writeMT6835(uint16_t reg, uint8_t data)
// {
//     // Combine the 4-bit write command (0110) with the 12-bit register address into a 16-bit value
//     uint16_t command_and_reg = (0b0110 << 12) | (reg & 0x0FFF); // 0x6 is the write command (0110)

//     digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

//     // Send the combined 16-bit value as two 8-bit parts
//     SPI_3.transfer((command_and_reg >> 8) & 0xFF); // Send the upper 8 bits (command + upper part of address)
//     SPI_3.transfer(command_and_reg & 0xFF);        // Send the lower 8 bits (lower part of address)

//     // Write the 8-bit data (D7~D0)
//     SPI_3.transfer(data);

//     digitalWrite(ENC1_nCS, HIGH); // Pull CSN high to end communication
// }

// bool writeRomMT6835()
// {
//     uint8_t data;
//     // Combine the 4-bit write command (0110) with the 12-bit register address into a 16-bit value
//     uint16_t command_and_reg = 0b1100000000000000; // 0x6 is the write command (0110)

//     digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

//     // Send the combined 16-bit value as two 8-bit parts
//     SPI_3.transfer((command_and_reg >> 8) & 0xFF); // Send the upper 8 bits (command + upper part of address)
//     SPI_3.transfer(command_and_reg & 0xFF);        // Send the lower 8 bits (lower part of address)

//     // Read the 8-bit data (D7~D0) from MISO
//     data = SPI_3.transfer(0x00);

//     digitalWrite(ENC1_nCS, HIGH); // Pull CSN high to end communication

//     if (data == 0x55)
//     {
//         return 1;
//     }
//     else
//     {
//         return 0;
//     }
// }

// uint8_t readMT6835(uint16_t reg)
// {
//     uint8_t data;

//     // Combine the 4-bit read command (0011) with the 12-bit register address into a 16-bit value
//     uint16_t command_and_reg = (0b0011 << 12) | (reg & 0x0FFF); // 0x3 is the read command (0011)

//     digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

//     // Send the combined 16-bit value as two 8-bit parts
//     SPI_3.transfer((command_and_reg >> 8) & 0xFF); // Send the upper 8 bits (command + upper part of address)
//     SPI_3.transfer(command_and_reg & 0xFF);        // Send the lower 8 bits (lower part of address)

//     // Read the 8-bit data (D7~D0) from MISO
//     data = SPI_3.transfer(0x00);

//     digitalWrite(ENC1_nCS, HIGH); // Pull CSN high to end communication

//     return data;
// }

// void checkCalibrationStatus()
// {
//     uint8_t status = readMT6835(0x113); // Read the entire 0x113 register

//     // Mask the top two bits (7:6) to get the calibration status
//     uint8_t cal_status = (status >> 6);

//     switch (status)
//     {
//     case 0x00:
//         Serial485.println("No Cal");
//         break;
//     case 0x01:
//         Serial485.println("Calibrating");
//         break;
//     case 0x02:
//         Serial485.println("Cal Fail");
//         break;
//     case 0x03:
//         Serial485.println("Cal Success");
//         break;
//     }
//     Serial485.println(status);
// }

// uint32_t burstReadMT6835()
// {
//     uint32_t angle_data = 0;

//     digitalWrite(ENC1_nCS, LOW); // Pull CSN low to start communication

//     // Send the 4-bit burst read command (1010) + 12-bit address (0x003)
//     SPI_3.transfer(0b10100000); // Command (0b1010) + upper bits of address (0x00)
//     SPI_3.transfer(0x30);       // Lower 8 bits of address (0x003)

//     // Read the 24-bit angle data (3 bytes from registers 0x003 to 0x006)
//     uint8_t byte1 = SPI_3.transfer(0x00); // Read first byte (from register 0x003)
//     uint8_t byte2 = SPI_3.transfer(0x00); // Read second byte (from register 0x004)
//     uint8_t byte3 = SPI_3.transfer(0x00); // Read third byte (from register 0x005)

//     digitalWrite(ENC1_nCS, HIGH); // Pull CSN high to end communication

//     // Combine the 3 bytes into a 24-bit result
//     angle_data = ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | (uint32_t)byte3;

//     return angle_data; // Return the 24-bit angle data
// }