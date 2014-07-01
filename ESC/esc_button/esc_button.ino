/*This program uses a diode led and a button to
turn the led on and off and change brightness (using pwm).
The button is connected to the arduino via switchPin
and the led is connected to ledPin (and ground, obvi)
*/
// define Global variables

#include <Servo.h>

Servo myservo;
int switchPin = 8;
int escPin = 3;
int ledPin = 11;
//Booleans can have two values - ON/OFF, HIGH/LOW, 0/1, TRUE/FALSE
boolean lastButton = LOW; //boolean for previous loop
boolean currentButton = LOW; //boolean for current loop
int escLevel = 55; //integer from 0-255 to control brightness of LED via PWM
void setup()
{
  pinMode(switchPin, INPUT);
  pinMode(escPin, OUTPUT);
  Serial.begin(9600);
}
// To deal with the bouncing switch (signal bouncing from high/low)
// Creating a function called "debounce" that returns a boolean and has an
// input boolean called "last"
boolean debounce(boolean last)
{
  //setting a boolean named current to the value of the switch pin
  boolean current = digitalRead(switchPin);
  if (last != current)
  {
    delay(5);
    current = digitalRead(switchPin);
  }
  
  return current; //returns value of "current"
}

void loop()
{
  currentButton = debounce(lastButton);
  if (lastButton == LOW && currentButton == HIGH)
  {
    escLevel = escLevel + 10;
  }
  
  lastButton = currentButton;
    // To deal with case when ledLevel becomes > than 255
    // Because "then" is only one line, it can go on same line as if statement
    // and does not need brackets
  if (escLevel > 90) escLevel = 55;
    // Send new value to LED!
  myservo.write(escLevel);
  Serial.println(escLevel);         // print the value from the sensor
}
