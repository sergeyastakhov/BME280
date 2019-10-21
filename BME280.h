#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

#ifndef BME280_H
#define BME280_H

#include <Arduino.h>
#include <Wire.h>

#define BME280_I2C_ADDRESS 0x76

#define CHIPID_BME280      0x60
#define CHIPID_BMP280      0x58

class BME280
{
  private:
    uint8_t address;

    uint8_t chipId;

    class CalibrationData
    {
      private:
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;

        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;

        uint8_t dig_H1;
        int16_t dig_H2;
        uint8_t dig_H3;
        int16_t dig_H4;
        int16_t dig_H5;
        int8_t dig_H6;

      public:
        bool init(uint8_t calib[42]);

        int32_t calcTFine(int32_t adc_T) const;

        uint64_t calcPressure(int32_t adc_P, int32_t t_fine) const;

        uint32_t calcHumidity(int32_t adc_H, int32_t t_fine) const;
    };

    CalibrationData calData;
    bool initialized;

    void writeRegister(uint8_t reg, uint8_t val);

    bool readRegisters(uint8_t startReg, uint8_t count, uint8_t *buf);

  public:
    struct RawMeasurement
    {
        int32_t rawPressure;
        int32_t rawTemperature;
        int32_t rawHumidity;
    };

    bool readMeasurement(RawMeasurement &rawMeasurement);

    enum Mode
    {
        Sleep = 0, Forced = 1, Normal = 3
    };

    enum OverSampling
    {
        Over_Skipped, Over_1, Over_2, Over_4, Over_8, Over_16
    };

    enum StandbyTime
    {
        Standby_0_5,
        Standby_125,
        Standby_250,
        Standby_500,
        Standby_1000,
        Standby_10,
        Standby_20
    };

    enum IRRFilterCoefficient
    {
        Filter_off,
        Filter_2,
        Filter_4,
        Filter_8,
        Filter_16
    };

    explicit BME280(uint8_t _address = BME280_I2C_ADDRESS):
        address(_address), calData(), chipId(), initialized(false) {}

    bool init();

    void softReset();

    uint8_t getChipId() const { return chipId; }

    bool isInitialized() const { return initialized; }

    void setMode(Mode mode, OverSampling osrs_t, OverSampling osrs_p, OverSampling osrs_h);

    void setConfig(bool enableSPI, IRRFilterCoefficient filterCoefficient, StandbyTime standbyTime);

    class Measurement
    {
      private:
        bool valid;
        RawMeasurement rawMeasurement;
        const CalibrationData &calData;

        int32_t t_fine;

      public:
        Measurement(bool valid, const RawMeasurement &rawMeasurement, const CalibrationData &calData);

        bool isValid() const { return valid; }

        const RawMeasurement &getRawMeasurement() const { return rawMeasurement; }

        float getTemperature() const;

        float getPressure() const;

        float getHumidity() const;
    };

    Measurement readMeasurement();
};

#endif //BME280_H

#pragma clang diagnostic pop