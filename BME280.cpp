/**
 * BME280
 * Product page: https://www.bosch-sensortec.com/bst/products/all_products/bme280
 * Datasheet: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280-DS002.pdf
 */
#include "BME280.h"

/*5.4.1 Register 0xD0 “id”
The “id” register contains the chip identification number chip_id[7:0], which is 0x60. This number
can be read as soon as the device finished the power-on-reset.*/
#define REG_CHIPID         0xD0

/*5.4.2 Register 0xE0 “reset”
The “reset” register contains the soft reset word reset[7:0]. If the value 0xB6 is written to the
register, the device is reset using the complete power-on-reset procedure. Writing other values
than 0xB6 has no effect. The readout value is always 0x00.*/
#define REG_SOFTRESET      0xE0

#define SOFTRESET_ACTIVATE 0xB6

/*5.4.3 Register 0xF2 “ctrl_hum”
The “ctrl_hum” register sets the humidity data acquisition options of the device. Changes to
this register only become effective after a write operation to “ctrl_meas”.*/
#define REG_CONTROL_HUMIDITY  0xF2

/*5.4.4 Register 0xF3 “status”
The “status” register contains two bits which indicate the status of the device.*/
#define REG_STATUS    0xF3

/*5.4.5 Register 0xF4 “ctrl_meas”
The “ctrl_meas” register sets the pressure and temperature data acquisition options of the
device. The register needs to be written after changing “ctrl_hum” for the changes to become
effective.*/
#define REG_CONTROL_MEAS      0xF4

/*5.4.6 Register 0xF5 “config”
The “config” register sets the rate, filter and interface options of the device. Writes to the “config”
register in normal mode may be ignored. In sleep mode writes are not ignored.*/
#define REG_CONFIG  0xF5

#define REG_CALIB_1 0x88
#define REG_CALIB_2 0xE1

#define REG_MEASUREMENT  0xF7

bool BME280::init()
{
  Wire.begin();

  // Read and check chip id
  if( !readRegisters(REG_CHIPID, 1, &chipId) )
    return false;

  if( chipId != CHIPID_BME280 && chipId != CHIPID_BMP280 )
    return false;

  // Read calibration data
  uint8_t calib[42];

  // calib00…calib25 at memory addresses 0x88…0xA1
  if( !readRegisters(REG_CALIB_1, 26, calib) )
    return false;

  // calib26…calib41 at memory addresses 0xE1…0xF0
  if( !readRegisters(REG_CALIB_2, 16, calib + 26) )
    return false;

  if( !calData.init(calib) )
    return false;

  setMode(Normal, Over_16, Over_16, Over_16);   // Choose 16X oversampling

  initialized = true;

  return true;
}

void BME280::writeRegister(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool BME280::readRegisters(uint8_t startReg, uint8_t count, uint8_t *buf)
{
  Wire.beginTransmission(address);
  Wire.write(startReg);
  Wire.endTransmission();

  uint8_t read = Wire.requestFrom(address, count);

  if( read < count )
    return false;

  for( int i = 0; i < count; ++i )
  {
    int value = Wire.read();
    if( value < 0 )
      return false;

    buf[i] = static_cast<uint8_t>(value);
  }

  return true;
}

void BME280::setMode(Mode mode, OverSampling osrs_t, OverSampling osrs_p, OverSampling osrs_h)
{
  writeRegister(REG_CONTROL_HUMIDITY, (osrs_h & 0x7));
  writeRegister(REG_CONTROL_MEAS, mode | ((osrs_p & 0x7) << 2) | ((osrs_t & 0x7) << 5));
}

void BME280::setConfig(bool enableSPI, IRRFilterCoefficient filterCoefficient, StandbyTime standbyTime)
{
  writeRegister(REG_CONFIG,
                static_cast<uint8_t>(enableSPI) | ((filterCoefficient & 0x7) << 2) | ((standbyTime & 0x7) << 5));
}

bool BME280::readMeasurement(RawMeasurement &rawMeasurement)
{
  uint8_t buf[8];

  if( !readRegisters(REG_MEASUREMENT, 8, buf) )
    return false;

  uint32_t v = buf[0];
  v <<= 8;
  v |= buf[1];
  v <<= 8;
  v |= buf[2];
  rawMeasurement.rawPressure = v >> 4;

  v = buf[3];
  v <<= 8;
  v |= buf[4];
  v <<= 8;
  v |= buf[5];
  rawMeasurement.rawTemperature = v >> 4;

  v = buf[6];
  v <<= 8;
  v |= buf[7];
  rawMeasurement.rawHumidity = v;

  return true;
}

BME280::Measurement BME280::readMeasurement()
{
  RawMeasurement rawMeasurement = {};

  bool valid = readMeasurement(rawMeasurement);

  return Measurement(valid, rawMeasurement, calData);
}

void BME280::softReset()
{
  writeRegister(REG_SOFTRESET, SOFTRESET_ACTIVATE);
  delay(2); // 2 ms - startup time from specification
}

float BME280::Measurement::getTemperature() const
{
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.

  int32_t T = (t_fine * 5 + 128) >> 8;
  return T / 100.f;
}

BME280::Measurement::Measurement(bool valid, const RawMeasurement &rawMeasurement,
                                 const CalibrationData &calData)
    : valid(valid), rawMeasurement(rawMeasurement), calData(calData)
{
  t_fine = calData.calcTFine(rawMeasurement.rawTemperature);
}

float BME280::Measurement::getPressure() const
{
  int64_t p = calData.calcPressure(rawMeasurement.rawPressure, t_fine);

  return p / 256.f;
}

float BME280::Measurement::getHumidity() const
{
  int32_t h = calData.calcHumidity(rawMeasurement.rawHumidity, t_fine);

  return h / 1024.f;
}

/* CRC checking from
 * https://github.com/BoschSensortec/BME280_driver/blob/master/selftest/bme280_selftest.c
 */
static uint8_t crc_calculate(const uint8_t *mem_values, uint8_t mem_length)
{
  uint32_t crc_reg = 0xFF;
  uint8_t polynomial = 0x1D;
  uint8_t bitNo, index;
  uint8_t din = 0;

  for( index = 0; index < mem_length; index++ )
  {
    uint8_t value = mem_values[index];

    for( bitNo = 0; bitNo < 8; bitNo++ )
    {
      if( ((crc_reg & 0x80) > 0) != ((value & 0x80) > 0) )
      {
        din = 1;
      }
      else
      {
        din = 0;
      }

      /* Truncate 8th bit for crc_reg and mem_values */
      crc_reg = (uint32_t) ((crc_reg & 0x7F) << 1);
      value = (uint8_t) ((value & 0x7F) << 1);
      crc_reg = (uint32_t) (crc_reg ^ (polynomial * din));
    }
  }

  return (uint8_t) (crc_reg ^ 0xFF);
}

bool BME280::CalibrationData::init(uint8_t calib[42])
{
  uint8_t crc = crc_calculate(calib, 33);
  if( crc != calib[33] )
    return false;

#define LES16(array, offset) (array[offset] | (array[offset+1]<<8))
#define LEU16(array, offset) (uint16_t)(LES16(array,offset))

  dig_T1 = LEU16(calib, 0);
  dig_T2 = LES16(calib, 2);
  dig_T3 = LES16(calib, 4);

  dig_P1 = LEU16(calib, 6);
  dig_P2 = LES16(calib, 8);
  dig_P3 = LES16(calib, 10);
  dig_P4 = LES16(calib, 12);
  dig_P5 = LES16(calib, 14);
  dig_P6 = LES16(calib, 16);
  dig_P7 = LES16(calib, 18);
  dig_P8 = LES16(calib, 20);
  dig_P9 = LES16(calib, 22);

  dig_H1 = calib[25];
  dig_H2 = LEU16(calib, 26);
  dig_H3 = calib[28];
  dig_H4 = (calib[29] << 4) | (calib[30] & 0xf);
  dig_H5 = ((calib[30] & 0xf0) >> 4) | (calib[31] << 4);
  dig_H6 = calib[32];

  return true;
}

int32_t BME280::CalibrationData::calcTFine(int32_t adc_T) const
{
  int32_t var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;

  int32_t var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) *
                    ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) *
                  ((int32_t) dig_T3)) >> 14;

  return var1 + var2;
}

/*Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa*/
uint64_t BME280::CalibrationData::calcPressure(int32_t adc_P, int32_t t_fine) const
{
  int64_t var1 = ((int64_t) t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t) dig_P6;
  var2 = var2 + ((var1 * (int64_t) dig_P5) << 17);
  var2 = var2 + (((int64_t) dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t) dig_P3) >> 8) + ((var1 * (int64_t) dig_P2) << 12);
  var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) dig_P1) >> 33;
  if( var1 == 0 )
  {
    return 0; // avoid exception caused by division by zero
  }
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t) dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t) dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t) dig_P7) << 4);

  return (uint64_t) p;
}

/*Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
Output value of “47445” represents 47445/1024 = 46.333 %RH*/
uint32_t BME280::CalibrationData::calcHumidity(int32_t adc_H, int32_t t_fine) const
{
  int32_t v_x1_u32r = (t_fine - ((int32_t) 76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t) dig_H4) << 20) - (((int32_t) dig_H5) * v_x1_u32r)) +
                 ((int32_t) 16384)) >> 15) *
               (((
                     ((((v_x1_u32r * ((int32_t) dig_H6)) >> 10) *
                       (((v_x1_u32r * ((int32_t) dig_H3)) >> 11) + ((int32_t) 32768))) >> 10)
                     + ((int32_t) 2097152))
                 * ((int32_t) dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

  return (uint32_t) (v_x1_u32r >> 12);
}
