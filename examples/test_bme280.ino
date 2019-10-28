#include <Arduino.h>
#include <LiquidCrystal.h>
#include "BME280.h"

#define PA_2_MMHG(pressurePa)  (pressurePa*760.f/101325.f)

using namespace BME280;

LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

class CustomI2CProtocol : public I2CProtocol
{
  public:
    virtual void init()
    {
      I2CProtocol::init();
      Wire.setClock(400000);
    }
};

BME280Sensor bme280 = BME280Sensor(new CustomI2CProtocol());
//BME280Sensor bme280 = BME280Sensor(new I2CProtocol());

void setup()
{
  lcd.begin(16, 2);

  if( !bme280.init() )
  {
    lcd.print("Init error!");
    return;
  }

  bme280.setMode(Normal, Over_16, Over_16, Over_16);   // Choose 16X oversampling
}

void loop()
{
  if( !bme280.isInitialized() )
    return;

  Measurement measurement = bme280.readMeasurement();

  if( !measurement.isValid() )
  {
    lcd.clear();
    lcd.print("Read error!");
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(measurement.getTemperature());
    lcd.print("C");

//    Serial.print(measurement.getTemperature());
//    Serial.print(',');

    lcd.setCursor(9, 0);
    lcd.print("H:");
    lcd.print(measurement.getHumidity());
    lcd.print("%");

//    Serial.print(measurement.getHumidity());
//    Serial.print(',');

    float pressurePa = measurement.getPressure64();
    float pressureHg = PA_2_MMHG(pressurePa);

    lcd.setCursor(0, 1);
    lcd.print("P:");
    lcd.print(pressureHg);
    lcd.print("Hg");

//    Serial.print(pressureHg);
//    Serial.println();
  }

  delay(1000);
}
