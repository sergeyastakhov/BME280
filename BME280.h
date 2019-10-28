#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

#ifndef BME280_H
#define BME280_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

namespace BME280
{
  const uint8_t BME280_I2C_ADDRESS = 0x76;

  enum ChipId
  {
      CHIPID_BME280 = 0x60,
      CHIPID_BMP280 = 0x58
  };

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
      void init(const uint8_t calib[42]);

      static bool isValid(uint8_t calib[42]);

      int32_t calcTFine(int32_t adc_T) const;

      uint32_t calcHumidity(int32_t adc_H, int32_t t_fine) const;

      uint64_t calcPressure64(int32_t adc_P, int32_t t_fine) const;

      uint32_t calcPressure32(int32_t adc_P, int32_t t_fine) const;
  };

  struct RawMeasurement
  {
      int32_t rawPressure;
      int32_t rawTemperature;
      int32_t rawHumidity;
  };

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

      float getPressure64() const;

      uint32_t getPressure32() const;

      float getHumidity() const;
  };

  enum RegisterNumber
  {
      /* 5.4.1 Register 0xD0 “id”
The “id” register contains the chip identification number chip_id[7:0], which is 0x60.
This number can be read as soon as the device finished the power-on-reset.*/
          REG_CHIPID = 0xD0,

      /* 5.4.2 Register 0xE0 “reset”
The “reset” register contains the soft reset word reset[7:0]. If the value 0xB6 is written to the register,
the device is reset using the complete power-on-reset procedure. Writing other values than 0xB6 has no effect.
The readout value is always 0x00.*/
          REG_SOFTRESET = 0xE0,

      /* 5.4.3 Register 0xF2 “ctrl_hum”
The “ctrl_hum” register sets the humidity data acquisition options of the device.
Changes to this register only become effective after a write operation to “ctrl_meas”.*/
          REG_CONTROL_HUMIDITY = 0xF2,

      /* 5.4.4 Register 0xF3 “status”
The “status” register contains two bits which indicate the status of the device.*/
          REG_STATUS = 0xF3,

      /* 5.4.5 Register 0xF4 “ctrl_meas”
The “ctrl_meas” register sets the pressure and temperature data acquisition options of the device.
The register needs to be written after changing “ctrl_hum” for the changes to become effective.*/
          REG_CONTROL_MEAS = 0xF4,

      /* 5.4.6 Register 0xF5 “config”
The “config” register sets the rate, filter and interface options of the device.
Writes to the “config” register in normal mode may be ignored.
In sleep mode writes are not ignored.*/
          REG_CONFIG = 0xF5,

      /* First block of calibration data 0x88..0xA1
calib00..calib25 */
          REG_CALIB_1 = 0x88,
      /* Second block of calibration data 0xE1..0xF0
       calib26..calib41 */
          REG_CALIB_2 = 0xE1,

      /* Start address of mesurement registers */
          REG_MEASUREMENT = 0xF7,

      REG_PRESSURE_MSB = 0xF7,
      REG_PRESSURE_LSB = 0xF8,
      REG_PRESSURE_XLSB = 0xF9,

      REG_TEMP_MSB = 0xFA,
      REG_TEMP_LSB = 0xFB,
      REG_TEMP_XLSB = 0xFC,

      REG_HUMIDITY_MSB = 0xFD,
      REG_HUMIDITY_LSB = 0xFE
  };

  class Protocol
  {
    public:
      virtual void init() = 0;

      virtual bool isSpi3Wire() const;

      virtual void writeRegister(RegisterNumber reg, uint8_t val) = 0;

      virtual bool readRegisters(RegisterNumber startReg, uint8_t count, uint8_t *buf) = 0;
  };

  class I2CProtocol : public Protocol
  {
    private:
      uint8_t address;

    public:
      explicit I2CProtocol(uint8_t address = BME280_I2C_ADDRESS): address(address) {}

      void init();

      void writeRegister(RegisterNumber reg, uint8_t val);

      bool readRegisters(RegisterNumber startReg, uint8_t count, uint8_t *buf);
  };

  class SPIProtocol : public Protocol
  {
    private:
      SPISettings spiSettings;
      int chipSelectPin;
      bool spi3wire;

    public:
      explicit SPIProtocol(const SPISettings &spiSettings, int chipSelectPin, bool spi3wire = false):
          spiSettings(spiSettings), chipSelectPin(chipSelectPin), spi3wire(spi3wire) {}

      void init();

      bool isSpi3Wire() const;

      void writeRegister(RegisterNumber reg, uint8_t val);

      bool readRegisters(RegisterNumber startReg, uint8_t count, uint8_t *buf);
  };

  class SensorStatus
  {
    private:
      bool valid;
      uint8_t value;

    public:
      SensorStatus(bool valid, uint8_t value): valid(valid), value(value) {}

      bool isValid() const { return valid; }

      bool isMeasuring() const { return bitRead(value, 3); }

      bool isImageUpdate() const { return bitRead(value, 0); }
  };

  class BME280Sensor
  {
    private:
      Protocol *protocol;

      uint8_t chipId;

      CalibrationData calData;
      bool initialized;

      const uint8_t SOFTRESET_ACTIVATE = 0xB6;

    public:
      explicit BME280Sensor(Protocol *protocol):
          protocol(protocol), calData(), chipId(), initialized(false) {}

      bool init(bool checkCalibrationCRC = true);

      /* 2 ms - startup time from specification */
      void softReset()
      {
        protocol->writeRegister(REG_SOFTRESET, SOFTRESET_ACTIVATE);
      }

      uint8_t getChipId() const { return chipId; }

      bool isInitialized() const { return initialized; }

      void setHumidityMode(OverSampling osrs_h)
      {
        protocol->writeRegister(REG_CONTROL_HUMIDITY, (osrs_h & 0x7));
      }

      void setMode(Mode mode, OverSampling osrs_t, OverSampling osrs_p)
      {
        protocol->writeRegister(REG_CONTROL_MEAS, mode | ((osrs_p & 0x7) << 2) | ((osrs_t & 0x7) << 5));
      }

      void setMode(Mode mode, OverSampling osrs_t, OverSampling osrs_p, OverSampling osrs_h)
      {
        setHumidityMode(osrs_h);
        setMode(mode, osrs_t, osrs_p);
      }

      void setConfig(IRRFilterCoefficient filterCoefficient, StandbyTime standbyTime)
      {
        protocol->writeRegister(REG_CONFIG,
                                static_cast<uint8_t>(protocol->isSpi3Wire()) |
                                ((filterCoefficient & 0x7) << 2) |
                                ((standbyTime & 0x7) << 5));
      }

      SensorStatus readStatus();

      bool readMeasurement(RawMeasurement &rawMeasurement);

      Measurement readMeasurement();
  };
}
#endif //BME280_H

#pragma clang diagnostic pop