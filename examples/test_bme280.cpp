#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include "BME280.h"

#define PA_2_MMHG(pressurePa)  (pressurePa*760.f/101325.f)

LiquidCrystal lcd(13, 12, 11, 10, 9, 8);
BME280 bme280;

void setup()
{
  lcd.begin(16, 2);

  if( !bme280.init() )
  {
    lcd.print("Init error!");
  }
}

void loop()
{
  if( !bme280.isInitialized() )
    return;

  BME280::Measurement measurement = bme280.readMeasurement();

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

    float pressurePa = measurement.getPressure();
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
