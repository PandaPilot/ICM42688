#ifndef ICM42688_H
#define ICM42688_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

class ICM42688 {
 public:
    enum GyroFS : uint8_t {
        dps2000 = 0x00,
        dps1000 = 0x01,
        dps500 = 0x02,
        dps250 = 0x03,
        dps125 = 0x04,
        dps62_5 = 0x05,
        dps31_25 = 0x06,
        dps15_625 = 0x07
    };

    enum AccelFS : uint8_t {
        gpm16 = 0x00,
        gpm8 = 0x01,
        gpm4 = 0x02,
        gpm2 = 0x03
    };

    enum ODR : uint8_t {
        odr32k = 0x01,
        odr16k = 0x02,
        odr8k = 0x03,
        odr4k = 0x04,
        odr2k = 0x05,
        odr1k = 0x06,
        odr200 = 0x07,
        odr100 = 0x08,
        odr50 = 0x09,
        odr25 = 0x0A,
        odr12_5 = 0x0B
    };

    ICM42688(i2c_inst_t* i2c, uint8_t address, uint8_t sda_pin, uint8_t scl_pin);

    int begin();
    int setAccelFS(AccelFS fssel);
    int setGyroFS(GyroFS fssel);
    int setAccelODR(ODR odr);
    int setGyroODR(ODR odr);
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

    static constexpr uint32_t I2C_CLK = 400000;  // 400 kHz
    static constexpr uint8_t WHO_AM_I = 0x47;    // Expected WHO_AM_I value
};

#endif  // ICM42688_H