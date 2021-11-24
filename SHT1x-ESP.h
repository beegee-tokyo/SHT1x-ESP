/**
 * SHT1x Library
 *
 * Copyright 2019 Bernd Giesecke <beegee@giesecke.tk>
 * Copyright 2017 Vincent Pang <wingshun.pang@gmail.com>
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 * Based on previous work by:
 *    Vincent Pang <wingshun.pang@gmail.com>
 *    Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 *    Maurice Ribble: <www.glacialwanderer.com/hobbyrobotics/?p=5>
 *    Wayne ?: <ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html>
 *
 * Manages communication with SHT1x series (SHT10, SHT11, SHT15)
 * temperature / humidity sensors from Sensirion (www.sensirion.com).
 */
#ifndef SHT1x_h
#define SHT1x_h

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

class SHT1x
{
  public:
    enum class Voltage : uint8_t
    {
      DC_5_0v = 0
      , DC_4_0v
      , DC_3_5v
      , DC_3_3v
      , DC_3_0v
      , DC_2_5v
    };

    enum class TemperatureMeasurementResolution : uint8_t
    {
      Temperature_14bit = 0
      , Temperature_12bit
    };

    enum class HumidityMeasurementResolution : uint8_t
    {
      Humidity_12bit = 0
      , Humidity_8bit
    };

    enum class ShtCommand : uint8_t
    {
      MeasureTemperature        = 0b00000011
      , MeasureRelativeHumidity = 0b00000101
      , ReadStatusRegister      = 0b00000111
      , WriteStatusRegister     = 0b00000110
      , SoftReset               = 0b00011110
    };

    SHT1x(uint8_t dataPin, uint8_t clockPin, Voltage voltage = Voltage::DC_5_0v);

    float readHumidity() const;
    float readTemperatureC() const;
    float readTemperatureF() const;

  private:
    uint16_t readRawData(ShtCommand command, uint8_t dataPin, uint8_t clockPin) const;
    bool sendCommandSHT(ShtCommand command, uint8_t dataPin, uint8_t clockPin) const;
    bool waitForResultSHT(uint8_t dataPin) const;
    uint16_t getData16SHT(uint8_t dataPin, uint8_t clockPin) const;
    void skipCrcSHT(uint8_t dataPin, uint8_t clockPin) const;

    double getC1(HumidityMeasurementResolution resolution) const;
    double getC2(HumidityMeasurementResolution resolution) const;
    double getC3(HumidityMeasurementResolution resolution) const;

    double getT1(HumidityMeasurementResolution resolution) const;
    double getT2(HumidityMeasurementResolution resolution) const;

    double getD1ForC(Voltage voltage) const;
    double getD1ForF(Voltage voltage) const;

    double getD2ForC(TemperatureMeasurementResolution resolution) const;
    double getD2ForF(TemperatureMeasurementResolution resolution) const;

    void   controlDataPin(uint8_t dataPin, uint8_t val) const;
    
    const uint8_t _dataPin;
    const uint8_t _clockPin;

    const Voltage _voltage;
    const TemperatureMeasurementResolution _tempResolution;
    const HumidityMeasurementResolution _humidityResolution;
};

#endif
