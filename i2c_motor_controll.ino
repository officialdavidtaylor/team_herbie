// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.

#include <Wire.h>

byte data[3];
int command;
int prevCommand = 0;
int MotorR = 12;
int MotorL = 11;

//void parseValues(byte data[]){
//  byte motorValue[3];
//  motorValue[0] = data[0];
//  motorValue[1] = data[1];
//  motorValue[2] = data[2];
//
//  Serial.print(motorValue[1]);
//  Serial.print(",");
//  Serial.println(motorValue[2]);
//}
void parseValues(byte data[]){
  union float_tag{
    byte b[4];
    float fval;
  }ft;

  ft.b[0] =data[1];
  ft.b[1] = data[2];
  ft.b[2] = data[3];
  ft.b[3] = data[4];

  Serial.println(ft.fval);
}

void setup()
{
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(38400);           // start serial for output
//  analogWriteFrequency(MotorR, 250); //4ns period
//  analogWriteFrequency(MotorL, 250); //4ns period
}

void loop()
{
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
    command = Wire.read();
    if(command != prevCommand){
      if (command==1){
      int i=0;
      while(1 <= Wire.available()) // loop through all but the last
      {
        data[i] = Wire.read(); // receive byte as a character
        i = i+1;
      }
        parseValues(data);
      }
      prevCommand = command;
    }

//  while(1 < Wire.available()) // loop through all but the last
//  {
//    int valueR = Wire.read(); // receive byte as a character
//    int valueL = Wire.read();
//    Serial.print(valueR);         // print the character
//    Serial.print(valueL);
//    analogWrite(MotorR, valueR);
//    analogWrite(MotorL, valueL);
//  }
//  int x = Wire.read();    // receive byte as an integer
//  Serial.println(x);         // print the integer
}
