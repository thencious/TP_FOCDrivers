#include "stm32g4xx_hal.h"

void startCalibration();
void finishCalibration();
void writeMT6835(uint16_t reg, uint8_t data);
void checkCalibrationStatus();
uint8_t readMT6835(uint16_t reg);
bool readRomMT6835();
uint32_t burstReadMT6835();