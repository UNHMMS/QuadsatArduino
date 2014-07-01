#include <MatrixMath.h>  //Used to transform the output of the PIDs into for values for each motor
#include <Servo.h>        //Used to write values to the brushless motors
#include <PID_v1.h>       //Used to do PID control for attitude and spin rates

#define SLOPEVAL (0.043)   //the flope of the (Force/Potentiometer) curve 
#define MINPOT (6)         //the minimum valid potentiometer value for the motors
#define MAXPOT (179)        //the maximum valid potentiometer value for the motors
#define ZMAX (32)
#define SIDE (4)            //used to define the size of output arrays

double desAtt[6] =        //position values we're trying to hold the quad at
  { 0.0f,  0.0f, 0.0f,  0.35f,  0.0f, 0.0f };  
//{ roll, pitch,  yaw, z position, x pos, y pos }

double curAtt[6] =        //values read in from the IMU, current positions
  { 0.0f,  0.0f, 0.0f,       0.0f,  0.0f, 0.0f };  
//{ roll, pitch,  yaw, z position, x pos, y pos }

double curGyro[3] =         //current spin rates based on gyro data
  { 0.0f, 0.0f, 0.0f };
//{    x,    y,   z  }  

float combOut[SIDE][SIDE] =    //Used to combine the force outputs from the rate PID
{                              //  into force values for each individual motor
  {    0,  0.5,  0.25, 0.25 },
  {  0.5,    0, -0.25, 0.25 },
  {    0, -0.5,  0.25, 0.25 },
  { -0.5,    0, -0.25, 0.25 }
};
        
double attOut[SIDE] = 
  { 0.0,   0.0, 0.0, 0.0 };           //The output values from the attitude PID control
//{roll, pitch, yaw,   z }

double rawAtt[SIDE] = 
  { 0.0, 0.0 };
 
double rateOut[SIDE];          //the output of the rate PID control 
 //{roll, pitch, yaw,   z }
 
float motorOutput[SIDE];            //the motor output value. starts in force and is converted to PWM

                                          
float motorTrims[SIDE] =            //allows for manual motor calibration
  { 0.0, 0.0, 0.0, 0.0 };           
// motor: { 1, 2, 3, 4 }  
                        
Servo motors[SIDE];           //servo objects that will be used to
                              //  interface with the brushless motors
                           
int motorVals[SIDE] =         //The motor numbers in the correct order  
  {1, 2, 3, 4};               //  used for outputs

int motorPins[] =          //pins that the motors are tied to
  {13, 8, 10, 12};      

double zero = 0.0;         // passed into PID for yaw rate


//PID position controllers
PID rollCtrl( &curAtt[0], &attOut[0], &desAtt[0], 2, 0, 0, DIRECT );
PID pitchCtrl(&curAtt[1], &attOut[1], &desAtt[1], 2, 0, 0, DIRECT );
//PID yawCtrl(  &curAtt[2], &attOut[2], &desAtt[2], 0, 0, 0, DIRECT );
PID zPosCtrl( &curAtt[3], &attOut[3], &desAtt[3], 60, 0, 0, DIRECT );
/*
PID xPosCtrl( &curState[4], &output[4], &desState[4], 0, 0, 0, DIRECT );
PID yPosCtrl( &curState[5], &output[5], &desState[5], 0, 0, 0, DIRECT );
*/

//PID rate controllers
PID roll_R_Ctrl( &curGyro[0], &rateOut[0], &attOut[0], 1, .005, .01, DIRECT );
PID pitch_R_Ctrl(&curGyro[1], &rateOut[1], &attOut[1], 1, .005, .01, DIRECT );
PID yaw_R_Ctrl(  &curGyro[2], &rateOut[2], &zero, 1, .005, .01, DIRECT );




unsigned long curTime = 0;     //the time the current reading is taken at
unsigned long loopStart = 0;   //time the loop started at
unsigned long lastTime = 0;    //the last recorded time of a reading

unsigned long timeBefore = 0;  //time before the PID loop
unsigned long timeAfter = 0;   //time after the PID loop
float diff = 0;                //used for frequency calculations
float loopFreq = 0.0;          //frequency of the loop

float timeChunk = 0.0f;        //amount of time from last reading to current for LQR rates
boolean doneOne = false;      //used to wait wait till after the first iteration to do some math

boolean bladesOn = false;    //used to determine if the blades are receiving power
boolean ctrlOn = false;     //used to determine if the control algorithm is running

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//prepares the arduino for controlled flight
void setup()
{
    Serial.begin( 9600 );  //Start XBEE communication
    
    Serial.println( "Ready..." );  //print for start script
    
    initIMU();  //sets up the IMU to read values
    
    //setup PID timings (milliseconds)
    rollCtrl.SetSampleTime(20);
    pitchCtrl.SetSampleTime(20);
    //yawCtrl.SetSampleTime(20);
    zPosCtrl.SetSampleTime(20);
    /*
    xPosCtrl.SetSampleTime(20);
    yPosCtrl.SetSampleTime(20);
    */
    
    roll_R_Ctrl.SetSampleTime(20);
    pitch_R_Ctrl.SetSampleTime(20);
    yaw_R_Ctrl.SetSampleTime(20);
    
    
    //set the PID output limits
    rollCtrl.SetOutputLimits( -14, 14 );
    pitchCtrl.SetOutputLimits( -14, 14 );
    //yawCtrl.SetOutputLimits( -14, 14 );
    zPosCtrl.SetOutputLimits( 0, ZMAX );
    /*
    xPosCtrl.SetOutputLimits(20);
    yPosCtrl.SetOutputLimits(20);
    */
    
    roll_R_Ctrl.SetOutputLimits( -2, 2 );
    pitch_R_Ctrl.SetOutputLimits( -2, 2 );
    yaw_R_Ctrl.SetOutputLimits( -0.5, 0.5 );
    
    //turn on PID
    rollCtrl.SetMode( AUTOMATIC );
    pitchCtrl.SetMode( AUTOMATIC );
    //yawCtrl.SetMode( AUTOMATIC );
    zPosCtrl.SetMode( AUTOMATIC );
    /*
    xPosCtrl.setMode( AUTOMATIC );
    yPosCtrl.setMode( AUTOMATIC );
    */
    
    roll_R_Ctrl.SetMode(AUTOMATIC);
    pitch_R_Ctrl.SetMode(AUTOMATIC);
    yaw_R_Ctrl.SetMode(AUTOMATIC);
    
    
    //map the motors to their respective pins
    setupMotorPins();
    
    //startUp();
    //spinUp();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//the full startup protocol, spins up the blades and starts control
void startUp()
{ 
  //slowly start motors
  spinUp();
  bladesOn = true;
  
  //start the control loop
  controlStart();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Slowly spins up the motors to their lowest value
//quad didn't function without this (could be retested)
void spinUp()
{
  Serial.println( "Steady..." );
  
  //set the initial attitude in the kalman filter
  setKalmanRollAngle();
  setKalmanPitchAngle();
  
  for( int i = 0; i < MINPOT; i++)
  {
     for( int j = 0; j < 4; j++ )
     {
        motors[j].write( i );
     }
     delay( 200 );
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//turns the control scheme on.
void controlStart()
{
  ctrlOn = true;
  Serial.println( "GO!" );
  //Serial1.begin(115200);
  
  //time of the control starting
  loopStart = micros();
}

//LQR loop for flying the quad
void loop()
{
    //timeBefore = micros();

    //get current position and rates from IMU
    readIMUVals();
     
     if( ctrlOn )
    {
      //do PID control to get potentiometer outputs
      doPID();
   
      //write values to the motors
      fly();
      
    
      /*
      timeAfter = micros();
      
      //Serial.print( "time Before: " );
      //Serial.println( timeBefore );
        
      //Serial.print( "time After: " );
      //Serial.println( timeAfter );
      
      diff = timeAfter - timeBefore;
        
      //Serial.print( "time difference (in microseconds): " );
      //Serial.println( diff );
        
      diff = diff / 1000000.0;
        
      //Serial.print( "time difference (in Seconds): " );
      //Serial.println( diff, 6 );
        
      loopFreq =  1.0 / diff;
      
      Serial.print( "Frequency: " );
      Serial.println( loopFreq );
      */
    
    
      /*
      //if we've been flying for 3 seconds stop doing anything
      if( ((micros() - loopStart) > 20000000) )
      {
        Serial.println( "Timeout" );
        
        ctrlOn = false;
        bladesOn = false;
         
        spinDown();
        
      }
      */     
      
      
      
      
    }
    
    //check to see if there are commands for the arduino
    getCommands();
}


//kills the motors, turns off control
void kill()
{
  //turnes the blades off
  bladesOn = false;
  
  //writes 0 to motors
  spinDown();
  
  //reset the filter
  resetFilter();
  
  //turn control off
  ctrlOn = false;
}

//force the quad to slowly descend
void land()
{
  zPosCtrl.SetOutputLimits( 0, 12 );
}

//writes motors to 0 and sits on the serial communication for commands
void spinDown()
{
  for( int i = 0; i < 4; i++ )
  {
    motors[i].write( 0 ); 
  }
}

//write output values to the motors
void fly()
{
  //int total = 0;
  
  for( int i = 0; i < 4; i++ )
  {
    /*
     //total += (int)motorOutput[i];
     Serial.print( motorVals[i] );
     Serial.print( ": " );
     Serial.print( (int)motorOutput[i] );
     Serial.print( " " );
     */
     
     motors[i].write( (int)( motorOutput[i] + motorTrims[i] ) ); 
     
     //motors[i].write( (int)( motorOutput[i]) ); 
  }
  
  //Serial.print( total );
  //Serial.println();
}

//read values from the IMU and store them in the current state
void readIMUVals()
{
    //get new values for gyro and accel
    updateIMU();
    
    //attitude rates
    curGyro[0] = getGyroRoll();
    curGyro[1] = getGyroPitch();
    curGyro[2] = getGyroYaw();
    
    //attitude position angles
    curAtt[0] = getRoll();
    curAtt[1] = getPitch();
    
    //raw attitude position angles for graphing purposes
    rawAtt[0] = getRawRoll();
    rawAtt[1] = getRawPitch();
    
    //get Position data
    curAtt[3] = getZ();
    //curState[4] = getX();
    //curState[5] = getY();
   /* Serial.print( curAtt[3] );
    Serial.print(" ");
    //get desired roll from the XBee, used for showing off in uni-axial tests
    //desAtt[3] = getDesZ();
    Serial.print( desAtt[3] );
    Serial.print(" ");
    Serial.print( attOut[3] );
    Serial.println();*/
    
    
    
   /* Serial.print("~ ");
    
    for(int i = 0; i < 2; i++ )
    {
      Serial.print(rawAtt[i]);
      Serial.print(" ");
    }
    
    for(int i = 0; i < 2; i++ )
    {
      Serial.print(curAtt[i]);
      Serial.print(" ");
    }
    
    Serial.println();*/
    
    /*
    Serial.print( curGyro[0] );
    Serial.print( " " );
    Serial.print( curGyro[1] );
    Serial.print( " " );
    Serial.print( curGyro[2] );
    Serial.print( " " );
    Serial.print( curAtt[0] );
    Serial.print( " " );
    Serial.print( curAtt[1] );
    Serial.print( " " );
    Serial.println();
    */
    
    /*
    Serial.print( "~ " );
    Serial.print( millis() );
    Serial.print( " " );
    
    for( int i = 0 ; i < 3; i++ )
    {
      Serial.print( curState[i] );
      Serial.print( " " );
    }
    
    Serial.println();
    */
    
    /*
    Serial.print( curState[0] );
    Serial.print( " " );
    Serial.print( curState[1] );
    Serial.print( " " );
    Serial.print( curState[2] );
    Serial.println();
    */
}

//calculates the error vector and the output potentiometer values using LQR
void doPID()
{ 
  //PID for position
  rollCtrl.Compute();
  pitchCtrl.Compute();
  //yawCtrl.Compute();
  zPosCtrl.Compute();
  
  /*
  for( int i = 0; i < 4; i++ )
  {
    Serial.print( attOut[i] );
    Serial.print( " " );
  }
  */
  

    
  //PID for rates
  roll_R_Ctrl.Compute();
  pitch_R_Ctrl.Compute();
  yaw_R_Ctrl.Compute();
  
  //move the 
  rateOut[3] = attOut[3];
  
  if( rateOut[3] > ZMAX )
  {
     rateOut[3] = ZMAX; 
  }
  
  if( rateOut[3] < 0 )
  {
     rateOut[3] = 0; 
  }
  
  /*
  for( int i = 0; i < 4; i++ )
  {
    Serial.print( rateOut[i] );
    Serial.print( " " );
  }
  */
  
  //multiple the PID outputs by the combinational matrix
  MatrixCalc.Multiply( (float*)combOut, (float*)rateOut, SIDE, SIDE, 1, (float*)motorOutput );
  
  
  for( int i = 0; i < 4; i++ )
  {
    /*
    Serial.print( motorOutput[i] );
    Serial.print( " " );
    */
    
    motorOutput[i] = (motorOutput[i] / SLOPEVAL) + 4;
  }
  
  //Serial.println();
}

//ties the motors to their pins
void setupMotorPins()
{
  for( int i = 0; i < 4; i++ )
  {
    motors[i].attach( motorPins[i] );
    motors[i].write( 0 );
  }
}
