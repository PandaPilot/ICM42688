#include "ICM42688.h"

ICM42688::ICM42688(i2c_inst_t* i2c, uint8_t address, uint8_t sda_pin, uint8_t scl_pin)
    : _i2c(i2c), _address(address), _sda_pin(sda_pin), _scl_pin(scl_pin) {
    // Configure I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
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

    // Turn on accelerometer and gyroscope in Low Noise (LN) mode
    if (writeRegister(0x1E, 0x0F) < 0) {  // UB0_REG_PWR_MGMT0
        return -2;  // Failed to configure power management
    }

    return 0;  // Success
}

int ICM42688::setAccelFS(AccelFS fssel) {
    return writeRegister(0x20, fssel);  // UB0_REG_ACCEL_CONFIG0
}

int ICM42688::setGyroFS(GyroFS fssel) {
    return writeRegister(0x21, fssel);  // UB0_REG_GYRO_CONFIG0
}

int ICM42688::setAccelODR(ODR odr) {
    return writeRegister(0x20, odr);  // UB0_REG_ACCEL_CONFIG0
}

int ICM42688::setGyroODR(ODR odr) {
    return writeRegister(0x21, odr);  // UB0_REG_GYRO_CONFIG0
}

int ICM42688::getAGT() {
    uint8_t buffer[15];
    if (readRegisters(0x25, 15, buffer) < 0) {  // UB0_REG_TEMP_DATA1
        return -1;
    }

    // Parse accelerometer, gyroscope, and temperature data
    _acc[0] = (int16_t)((buffer[0] << 8) | buffer[1]) * 0.000488f;  // Scale factor for accel
    _acc[1] = (int16_t)((buffer[2] << 8) | buffer[3]) * 0.000488f;
    _acc[2] = (int16_t)((buffer[4] << 8) | buffer[5]) * 0.000488f;

    _gyr[0] = (int16_t)((buffer[6] << 8) | buffer[7]) * 0.015267f;  // Scale factor for gyro
    _gyr[1] = (int16_t)((buffer[8] << 8) | buffer[9]) * 0.015267f;
    _gyr[2] = (int16_t)((buffer[10] << 8) | buffer[11]) * 0.015267f;

    _t = ((int16_t)((buffer[12] << 8) | buffer[13]) - 25) / 132.48f;  // Temperature in Celsius

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
    writeRegister(0x1F, 0x01);  // UB0_REG_SIGNAL_PATH_RESET
    sleep_ms(100);              // Wait for reset to complete
}

uint8_t ICM42688::whoAmI() {
    uint8_t who_am_i = 0;
    readRegisters(0x75, 1, &who_am_i);  // UB0_REG_WHO_AM_I
    return who_am_i;
}