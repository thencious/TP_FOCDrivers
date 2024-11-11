# Steps to setup motors
#### 1. Read these before you read the rest: 
1. Basics: https://docs.simplefoc.com/steppermotor
2. Torque Control (Voltage mode -estimated current): docs.simplefoc.com/torque_control
3. Communication & board IO: https://miro.com/welcomeonboard/WWRvTDV0TkJGNzBaMU1rTTNhOUptRS9xOXlBalByc1NNZEh5WHU0b0ZoVWQzRDhvTHZxTC83NE4vM1Vqa3gxeFJncytxTzVZL0ZISU5lRzlyNzk2Qjkra21YbEFibUZXT2prcmZUMmtaOWlNQ1kyUVhiY3VYa3FrUHZUTjFnb04hZQ==?share_link_id=483281023795
#### 2. Tools:
1. RS485 PC End point: https://drive.google.com/file/d/1kWFYFm4FZqrVX-wjg3YuICJs8cdf8yDp/view?usp=drive_link
2. MCUViewer (plot without taking much/if any CPU overhead, important for foc type control, especially through SWO): https://github.com/klonyyy/MCUViewer
#### 3. Install PlatformIO on VSCode, replace this file with that of the same name in installation directory (not PIO): https://drive.google.com/file/d/1FmgtKNir01CYgvvhrD2Y63TgZtJBBQMX/view?usp=sharing
#### 4. For each motor type, set phase resistance, KV value, impedance and pwm frequency experimentally to reach highest speed with minimal noise and vibration. [Sample code: GanY_Driver](github.com/thencious/TP_FOCDrivers/tree/main/GanY_Driver) If possible, run anti-cogging on all motors.
#### 5. on the final rotor shaft magnet - encoder couple, spin and calibrate all motors for linearity. Get and set optimal SPI absolute zero electrical offsets and encoder directions for X1, X2 and Z driver boards. [Sample code: GanY_AutoCal](github.com/thencious/TP_FOCDrivers/tree/main/GanY_AutoCal)
#### 6. In the case of the Y axis motor, unplug motor 1, set motor 2 (furthest motor from encoder) electrical angle to 0, hold motor 2 shaft in place and loosen couple on motor 1. Unplug motor 2 and set motor 1 electrical angle to 0. Get and set SPI absolute zero electrical offset and encoder direction. Tighten coupling between motor 1 and 2 with motor 2 still held in the electrical angle = 0 position. [Sample code: GanY_getZeroEAngle](github.com/thencious/TP_FOCDrivers/tree/main/GanY_getZeroEAngle)
#### 7. Adjust:
1. gain on torque target (potentially adding position control at low target torque to fight cogging and static friction) [Sample code: GanY_Driver](github.com/thencious/TP_FOCDrivers/tree/main/GanY_Driver)
2. down sampling multiplier [Sample code: GanY_Driver](github.com/thencious/TP_FOCDrivers/tree/main/GanY_Driver)
#### 8. With the zeroing rig in place, get all joint SPI positions, set all SPI position to 0 and using the previous SPI positions, calculate the new optimal electrical angle offset. (remember to write EEPROM) Set full rot counter to zero and future zeroing should only ever be zeroing full rot counter, not absolute position, since this can disrupt the optimal electrical angle offsets and anti-cogging and is unnecessary. [Sample code: GanY_AutoCal](github.com/thencious/TP_FOCDrivers/tree/main/GanY_AutoCal)
#### 9. Further readings:
1. STM32G4xx Reference Manual: st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
2. Driver Datasheet: ti.com/lit/ds/symlink/drv8434e.pdf?ts=1729057575578
3. Encoder Datasheet: https://www.magntek.com.cn/upload/pdf/202407/MT6835_Rev.1.3.pdf
4. Current Sensor Datasheet: ti.com/lit/ds/symlink/ina240.pdf?ts=1727274007428&ref_url=https%253A%252F%252Fwww.google.com%252F
