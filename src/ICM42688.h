#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"

class ICM42688 {
 public:
    // Enum for Accelerometer Full-Scale Range (FS)
    enum AccelFS : uint8_t {
        gpm16,  // ±16g
        gpm8,   // ±8g
        gpm4,   // ±4g
        gpm2    // ±2g
    };

    // Static constexpr arrays for data
    static constexpr uint8_t AccelFSVal[] = {
        0b0000,  // gpm16
        0b0010,  // gpm8
        0b0100,  // gpm4
        0b0110   // gpm2
    };

    static constexpr float AccelSens[] = {
        1.0f / 2048.0f,   // 0.00048828125
        1.0f / 4096.0f,   // 0.00024414063
        1.0f / 8192.0f,   // 0.00012207031
        1.0f / 16384.0f   // 0.00006103516
    };

    enum class AccelODR : uint8_t {
        odr32k    = 0b0001,   // 32kHz (LN mode)
        odr16k    = 0b0010,   // 16kHz (LN mode)
        odr8k     = 0b0011,   // 8kHz (LN mode)
        odr4k     = 0b0100,   // 4kHz (LN mode)
        odr2k     = 0b0101,   // 2kHz (LN mode)
        odr1k     = 0b0110,   // 1kHz (LN mode) (default)
        odr200    = 0b0111,   // 200Hz (LP or LN mode)
        odr100    = 0b1000,   // 100Hz (LP or LN mode)
        odr50     = 0b1001,   // 50Hz (LP or LN mode)
        odr25     = 0b1010,   // 25Hz (LP or LN mode)
        odr12_5   = 0b1011,   // 12.5Hz (LP or LN mode)
        odr6_25   = 0b1100,   // 6.25Hz (LP mode)
        odr3_125  = 0b1101,   // 3.125Hz (LP mode)
        odr1_5625 = 0b1110,   // 1.5625Hz (LP mode)
        odr500    = 0b1111    // 500Hz (LP or LN mode)
    };

    // Enum for Gyro Full-Scale Range (FS)
    enum GyroFS : uint8_t {
        dps2000,
        dps1000,
        dps500,
        dps250,
        dps125,
        dps62_5,
        dps31_25,
        dps15_625
    };

    // Static constexpr arrays for data
    static constexpr uint8_t GyroFSVal[] = {
        0b0000,  // dps2000
        0b0010,  // dps1000
        0b0100,  // dps500
        0b0110,  // dps250
        0b1000,  // dps125
        0b1010,  // dps62.5
        0b1100,  // dps31.25
        0b1110   // dps15.625
    };

    static constexpr float GyroSens[] = {
        1.0f / 16.4f,    // dps2000, reciprocal of the sensitivities
        1.0f / 32.8f,    // dps1000, hard coded for faster multiplication
        1.0f / 65.5f,    // dps500,  instead of dividing every read
        1.0f / 131.0f,   // dps250
        1.0f / 262.0f,   // dps125
        1.0f / 524.3f,   // dps62.5
        1.0f / 1048.6f,  // dps31.25
        1.0f / 2097.2f   // dps15.625
    };

    enum class GyroODR : uint8_t {
        odr32k   = 0b0001,  // 0x01
        odr16k   = 0b0010,  // 0x02
        odr8k    = 0b0011,  // 0x03
        odr4k    = 0b0100,  // 0x04
        odr2k    = 0b0101,  // 0x05
        odr1k    = 0b0110,  // 0x06
        odr200   = 0b0111,  // 0x07
        odr100   = 0b1000,  // 0x08
        odr50    = 0b1001,  // 0x09
        odr25    = 0b1010,  // 0x0A
        odr12_5  = 0b1011,   // 0x0B
        odr500   = 0b1111   // 0x07
    };

    ICM42688(i2c_inst_t* i2c, uint8_t address, uint8_t sda_pin, uint8_t scl_pin);

    int begin();
    int setAccel(AccelFS fssel, AccelODR odr);
    int setGyro(GyroFS fssel, GyroODR odr);
    int getAGT();

    float accX() const { return _acc[0]; }
    float accY() const { return _acc[1]; }
    float accZ() const { return _acc[2]; }
    float gyrX() const { return _gyr[0]; }
    float gyrY() const { return _gyr[1]; }
    float gyrZ() const { return _gyr[2]; }
    float temp() const { return _t; }

    void acc(volatile float *out);
    void gyr(volatile float *out);

 private:
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    void reset();
    uint8_t whoAmI();

    i2c_inst_t* _i2c;
    uint8_t _address;
    uint8_t _sda_pin;
    uint8_t _scl_pin;

    float _acc[3] = {};
    float _gyr[3] = {};
    float _t = 0.0f;
    float AccelSensitivity = 0.0f;
    float GyroSensitivity = 0.0f;

    static constexpr uint32_t I2C_CLK = 400000;  // 400 kHz
    static constexpr uint8_t WHO_AM_I = 0x47;    // Expected WHO_AM_I value
};
