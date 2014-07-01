// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo
Servo myservo1; 
Servo myservo2; 
Servo myservo3; 
 
int potpin = A0;  // analog pin used to connect the potentiometer
float val = 0;       // variable to store the value coming from the sensor
int oldVal = 0;   // used for updating the serial print
 
void setup() 
{ 
  //myservo.attach(3);  // attaches the servo on pin 3 to the servo object
  //myservo1.attach(5);
  //myservo2.attach(6);
  //myservo3.attach(9);  
  Serial.begin(9600);
} 
 
void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  //val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  
  myservo.write( 12.5f );                  // sets the servo position according to the scaled value
  //myservo1.write(val);
  //myservo2.write(val);
  //myservo3.write(val);
  
  if( val != oldVal )
  {
    Serial.println(val);         // print the value from the potentiometer
    oldVal = val;
  }
  
  delay(50);
}
