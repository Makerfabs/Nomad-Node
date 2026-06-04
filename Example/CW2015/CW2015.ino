/*
Arduino IDE V2.3.6
Additional boards manager URLs:https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
Board: nrf52 V1.6.1

Tools:
Board: "Nordic nRF52840 DK"
*/

// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.

#include <Wire.h>

#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27

void setup() {
  Serial.begin(9600);  // start serial for output
  while(!Serial);
  Serial.println("serial start");

  Wire.setPins(I2C_SDA_PIN,I2C_SCL_PIN);
  Wire.begin();
}


void loop() {

  Serial.print("Battery_voltage: ");
  Serial.println(read_battery_voltage());
  Serial.print("Battery_percentage: ");
  Serial.println(read_battery_percentage());
  // Serial.print("Version: ");
  // Serial.println(read_version(), HEX);
  write_mode(0x0);
  delay(1000);

}

float read_battery_voltage()
{
  float battery_voltage;
  char batt_lsb, batt_msb;
  int result;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x02));
  result = Wire.endTransmission();
  Wire.requestFrom(0x62, 1);
  if (Wire.available()) { // slave may send less than requested
    batt_msb = Wire.read(); // receive a byte as character
    //Serial.println(batt_msb, HEX);        // print the character
  }

  battery_voltage = batt_msb << 8;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x03));
  result = Wire.endTransmission();
  Wire.requestFrom(0x62, 1);
  if (Wire.available()) { // slave may send less than requested
    batt_lsb = Wire.read(); // receive a byte as character
    //Serial.println(batt_lsb, HEX);        // print the character
  }

  battery_voltage += batt_lsb;
  battery_voltage = battery_voltage * 305 / 1000000;
  return battery_voltage;
}

float read_battery_percentage()
{
  float battery_percentage;
  char percent_lsb, percent_msb;
  int result;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x04));
  result = Wire.endTransmission();
  Wire.requestFrom(0x62, 1);
  if (Wire.available()) { // slave may send less than requested
    percent_msb = Wire.read(); // receive a byte as character
    //Serial.println(percent_msb, HEX);        // print the character
  }

  battery_percentage = percent_msb;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x05));
  result = Wire.endTransmission();
  Wire.requestFrom(0x62, 1);
  if (Wire.available()) { // slave may send less than requested
    percent_lsb = Wire.read(); // receive a byte as character
    //Serial.println(percent_lsb, HEX);        // print the character
  }

  battery_percentage += (float(percent_lsb) / 256);
  return battery_percentage;
}

char read_version()
{
  char ic_version;
  int result;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x00));
  result = Wire.endTransmission();
  Wire.requestFrom(0x62, 1);
  if (Wire.available()) { // slave may send less than requested
    ic_version = Wire.read(); // receive a byte as character
    //Serial.println(ic_version, HEX);        // print the character
  }

  return ic_version;
}

char read_mode()
{
  char ic_mode;
  int result;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x0a));
  result = Wire.endTransmission();
  Wire.requestFrom(0x62, 1);
  if (Wire.available()) { // slave may send less than requested
    ic_mode = Wire.read(); // receive a byte as character
    //Serial.println(ic_mode, HEX);        // print the character
  }
  return ic_mode;
}

void write_mode(char ic_mode)
{
  Wire.beginTransmission(0x62);
  Wire.write(byte(0x0a));
  Wire.write(ic_mode);
  int result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("write byte nok : ");
    Serial.println(result);
  }
}