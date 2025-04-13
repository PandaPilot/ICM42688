#include <stdio.h>
#include "ICM42688.h"
// Register addresses in Bank 0
#define REG_BANK_SEL       0x7F
#define APEX_CONFIG0       0x56
#define SMD_CONFIG         0x57
#define SIGNAL_PATH_RESET  0x4B
#define PWR_MGMT0          0x4E
#define GYRO_CONFIG0       0x4F
#define ACCEL_CONFIG0      0x50


ICM42688::ICM42688(i2c_inst_t* i2c, uint8_t address, uint8_t sda_pin, uint8_t scl_pin)
    : _i2c(i2c), _address(address), _sda_pin(sda_pin), _scl_pin(scl_pin) {
    // Configure I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
}

int ICM42688::begin() {
    // Initialize I2C
    i2c_init(_i2c, I2C_CLK);

    // Reset the device
    reset();

    // Check the WHO_AM_I register
    if (whoAmI() != WHO_AM_I) {
        return -1;  // Device not found
    }

    // // Turn on accelerometer and gyroscope in Low Noise (LN) mode
    // if (writeRegister(0x1E, 0x0F) < 0) {  // UB0_REG_PWR_MGMT0
    //     return -2;  // Failed to configure power management
    // }

    return 0;  // Success
}

int ICM42688::setAccel(AccelFS fssel, AccelODR odr) {
    uint8_t settings = (fssel << 4) | static_cast<uint8_t>(odr);
    return writeRegister(ACCEL_CONFIG0, settings);  // UB0_REG_ACCEL_CONFIG0
}

int ICM42688::setGyro(GyroFS fssel, GyroODR odr) {
    uint8_t settings = (fssel << 4) | static_cast<uint8_t>(odr);
    return writeRegister(GYRO_CONFIG0, settings);  // UB0_REG_GYRO_CONFIG0
}


int ICM42688::getAGT() {
    // 1) Ensure we're in User Bank 0
    writeRegister(REG_BANK_SEL, 0x00);
    
    uint8_t buffer[14];
    if (readRegisters(0x1D, 14, buffer) < 0) {  // UB0_REG_TEMP_DATA1
        return -1;
    }

    // Parse temperature, accelerometer, and gyroscope data
    _t = ((int16_t)((buffer[0] << 8) | buffer[1]) - 25) / 132.48f;  // Temperature in Celsius

    _acc[0] = (int16_t)((buffer[2] << 8) | buffer[3]) * 0.000488f;  // Scale factor for accel
    _acc[1] = (int16_t)((buffer[4] << 8) | buffer[5]) * 0.000488f;
    _acc[2] = (int16_t)((buffer[6] << 8) | buffer[7]) * 0.000488f;

    _gyr[0] = (int16_t)((buffer[8] << 8) | buffer[9]) * 0.015267f;  // Scale factor for gyro
    _gyr[1] = (int16_t)((buffer[10] << 8) | buffer[11]) * 0.015267f;
    _gyr[2] = (int16_t)((buffer[12] << 8) | buffer[13]) * 0.015267f;

    printf("Accel: X= B%d, S%d\n", buffer[2], buffer[3]);
    printf("Gyro: X=%.3f, Y=%.3f, Z=%.3f\n", _gyr[0], _gyr[1], _gyr[2]);
    printf("Temp: %.2f C\n", _t);//(TEMP_DATA / 132.48) + 25
    return 0;
}

void ICM42688::acc(volatile float *out) {
    out[0] = _acc[0];
    out[1] = _acc[1];
    out[2] = _acc[2];
}

void ICM42688::gyr(volatile float *out) {
    out[0] = _gyr[0];
    out[1] = _gyr[1];
    out[2] = _gyr[2];
}


int ICM42688::writeRegister(uint8_t subAddress, uint8_t data) {
    uint8_t buffer[2] = {subAddress, data};
    int result = i2c_write_blocking(_i2c, _address, buffer, 2, false);
    return (result == 2) ? 0 : -1;
}

int ICM42688::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {
    int result = i2c_write_blocking(_i2c, _address, &subAddress, 1, true);
    if (result != 1) {
        return -1;
    }
    result = i2c_read_blocking(_i2c, _address, dest, count, false);
    return (result == count) ? 0 : -1;
}

void ICM42688::reset() {
    
    // 1) Ensure we're in User Bank 0
    writeRegister(REG_BANK_SEL, 0x00);

    // 2) Disable all APEX / DMP features
    //    APEX_CONFIG0 = 0x00 => no pedometer/tilt/tap, etc.
    //    SMD_CONFIG   = 0x00 => no Wake on Motion or Significant Motion
    writeRegister(APEX_CONFIG0, 0x00);
    writeRegister(SMD_CONFIG,   0x00);

    // 3) Disable DMP init/reset bits in SIGNAL_PATH_RESET
    //    Write 0x00 so no leftover DMP operations
    writeRegister(SIGNAL_PATH_RESET, 0x00);

    // 4) Configure power modes: enable accel & gyro in low-noise mode
    //    PWR_MGMT0 = 0x0F => ACCEL_MODE = 3 (LN), GYRO_MODE = 3 (LN)
    writeRegister(PWR_MGMT0, 0x0F);

    // 5) Optionally configure the output data rates and full-scale range
    //    For example, ±2000 dps at 1 kHz for gyro => GYRO_CONFIG0 = 0x06
    //    and ±16 g at 1 kHz for accel => ACCEL_CONFIG0 = 0x06
    writeRegister(GYRO_CONFIG0,  0x06);
    writeRegister(ACCEL_CONFIG0, 0x06);

    printf("ICM-42688: All APEX features disabled; IMU only.\n");
    sleep_ms(100);              // Wait for reset to complete
}

uint8_t ICM42688::whoAmI() {
    uint8_t who_am_i = 0;
    readRegisters(0x75, 1, &who_am_i);  // UB0_REG_WHO_AM_I
    return who_am_i;
}