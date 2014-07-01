 #include <Servo.h>
#include <MatrixMath.h>
#include <MPULib.h>
#include <Wire.h>
#include <PID_v1.h>

#define H (3)              //the height of the LQR gain matrix
#define W (3)              //the width of the LQR gain matrix
#define MINPOT (4)         //the minimum valid potentiometer value
#define MAXPOT (179)       //the maximum valid potentiometer value

#define MINI (12)
#define MAXI (32)
#define LANDZ (13)

#define SPINVAL (5)
double ZMAX = MAXI;          //the maximum value (in newtons) that the motors can push
double ZMIN = MINI;

float K0[3][3] =            //LQR Gain Matrix
{
   { 0.0316,         0,         0, },
   {      0,    0.0316,         0, },
   {      0,         0,         3.5, }
};

float combOut[4][4] = 
{
  {  0,  .5, -.25, .25},
  { .5,   0,  .25, .25},
  {  0, -.5, -.25, .25},
  {-.5,   0,  .25, .25} 
};

double curRates[3] = 
    { 0.0, 0.0, 0.0 };
  //pitch, roll, yaw
    
double curAngles[3] =
    { 0.0, 0.0, 0.0 };
   // pitch, roll, yaw
   
double desZ = 0.75; //desired altitude
double curZ = 0.0; //current altitude
double outZ = 0.0; //output of altitude PID

double attOut[3]; //output of the PID attitude controller
double ctrlOut[4]; //output of the LQR rate controller
double motorOuts[4]; //output from the combinational matrix to be written to the motors

double zero = 0.0;

//PID attitude and altitude controllers
PID pitchCtrl(&curAngles[0], &attOut[0], &zero, 4, 8, 1, DIRECT ); //22, 45, 3.2
PID rollCtrl (&curAngles[1], &attOut[1], &zero, 4, 8, 1, DIRECT ); //18, 45, 3.2
PID yawCtrl  (&curAngles[2], &attOut[2], &zero, 0, 0, 0, DIRECT );
PID zPosCtrl( &curZ, &outZ, &desZ, 4, 4, .5, DIRECT );

double error[3] =  //error values for the LQR rate controller
    { 0.0, 0.0, 0.0 };
                        
Servo motors[4];           //servo objects that will be used to
                           //    interface with the brushless motors
                           
double slopeVal[4] = 
    { 0.0395, 0.0436, 0.0395, 0.0433  };
    
//double slopeVal[4] = 
//    { 0.043, 0.043, 0.043, 0.043  };
                           
int motorVals[4] =         //The motor numbers in the correct order  
  {2, 3, 4, 1};   

int motorPins[] =          //pins that the motors are tied to
  {8, 10, 12, 13};      
  
unsigned long curTime = 0;     //the time the current reading is taken at
unsigned long loopStart = 0;   //time the loop started at
unsigned long lastTime = 0;    //the last recorded time of a reading

boolean ctrlOn = false;
boolean bladesOn = false;
boolean landing = false;

//setup the K matrix and the desired state vector
//start the serial communication
void setup()
{
    Serial.begin( 9600 );
    
    initIMU();
    
    //setup PID
    zPosCtrl.SetSampleTime(20);
    zPosCtrl.SetOutputLimits(ZMIN, ZMAX);
    zPosCtrl.SetMode( AUTOMATIC );
    
    rollCtrl.SetSampleTime(20);
    rollCtrl.SetOutputLimits( -14, 14 );
    rollCtrl.SetMode( AUTOMATIC );
    
    pitchCtrl.SetSampleTime(20);
    pitchCtrl.SetOutputLimits( -14, 14 );
    pitchCtrl.SetMode( AUTOMATIC );
    
    yawCtrl.SetSampleTime(20);
    yawCtrl.SetOutputLimits( -14, 14 );
    yawCtrl.SetMode( AUTOMATIC );
    
    //setup motor stuff
    setupMotorPins();
    
}

//LQR loop for flying the quad
void loop()
{
    
    //timeBefore = micros() - loopStart;
    
    
    //get current state from the sensor
    readIMUVals();
    
    if( ctrlOn )
    {
        
    
        //do matrix math to get potentiometer outputs
        doControl();
    
        //write values to the motors
        fly();
    }

    /*
    timeAfter = micros() - loopStart;
    
    Serial.print( "time Before: " );
    Serial.println( timeBefore );
      
    Serial.print( "time After: " );
    Serial.println( timeAfter );
    
    diff = timeAfter - timeBefore;
      
    Serial.print( "time difference (in microseconds): " );
    Serial.println( diff );
      
    diff = diff / 1000000.0;
      
    Serial.print( "time difference (in Seconds): " );
    Serial.println( diff, 6 );
      
    loopFreq =  1 / diff;
    
    Serial.print( "Frequency: " );
    Serial.println( loopFreq );
    */
    
    /*
    //if we've been flying for 3 seconds stop doing anything
    if( ((micros() - loopStart) > 20000000) )
    {
      Serial.println( "Timeout" );
       
      spinDown();
    }
    */
    
    getCommands();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//the full startup protocol, spins up the blades and starts control
void startUp()
{
  updateIMU();
  setYawDomain();
  
  setInitialKalmanAngles();
  setInitialComplementaryAngles();
  
  //slowly start motors
  spinUp();
  bladesOn = true;
  
  //start the control loop
  controlStart();
  
  landing = false;
  
  ZMAX = MAXI;
  ZMIN = MINI;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Slowly spins up the motors to their lowest value
//quad didn't function without this (could be retested)
void spinUp()
{
  Serial.println( "Steady..." );
  
  for( int i = 0; i < SPINVAL; i++)
  {
     for( int j = 0; j < 4; j++ )
     {
        motors[j].write( i );
     }
     delay( 200 );
  }
}

//---------------------------------------------------------------------------------------
//Starts the landing sequence for the quad
void land()
{
  ZMAX = LANDZ;
  ZMIN = 0;
  landing = true;
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

//kills the motors, turns off control
void kill()
{
  //turnes the blades off
  bladesOn = false;
  
  //writes 0 to motors
  spinDown();
  
  //reset the filter
  resetFilter();
  resetYawDomain();
  
  //clear the PID I terms
  pitchCtrl.ResetITerm();
  rollCtrl.ResetITerm();
  yawCtrl.ResetITerm();
  zPosCtrl.ResetITerm();
  
  //reset ZMAX
  
  
  //turn control off
  ctrlOn = false;
  
  //turn landing off
  
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
  for( int i = 0; i < 4; i++ )
  {
    
     motors[i].write( motorOuts[i] ); 
  }
   Serial.println();
  //Serial.println();
}

//read values from the IMU and store them in the current state
void readIMUVals()
{
  updateIMU();
  
  curRates[0] = getGyroPitch();
  curRates[1] = getGyroRoll();
  curRates[2] = getGyroYaw();
  
  curAngles[0] = getPitch();
  curAngles[1] = getRoll();
  curAngles[2] = getYaw();
  
  curZ = getZ();
  //desZ = getDesZ();
  
  if( landing && curZ < 0.07 )
  {
     kill(); 
  }
  
  
  Serial.print( "~ " );

  Serial.print( getRawGyroRoll() );
  Serial.print( " " );
  Serial.print( getRawGyroPitch() );
  Serial.print( " " );
  Serial.print( getRawYaw() );
  Serial.print( " " );  
  
  Serial.print( curAngles[1] );
  Serial.print( " " );
  Serial.print( curAngles[0] );
  Serial.print( " " );
  Serial.print( curAngles[2] );
  Serial.print( " " );  
  
  Serial.print( curRates[2] );
  Serial.print( " " );
  Serial.println();
}

//calculates the error vector and the output potentiometer values using LQR
void doControl()
{ 
   //PID attitude controllers
   pitchCtrl.Compute();
   rollCtrl.Compute();
   yawCtrl.Compute();
   zPosCtrl.Compute();
   
   if( outZ < ZMIN )
   {
     outZ = ZMIN; 
   }
   
   if( outZ > ZMAX )
   {
     outZ = ZMAX; 
   }
   
   //subtract gyro rates from the PID outputs
   for(int i = 0; i < 3; i++ )
   {
     error[i] = attOut[i] - curRates[i];
   }
    
    //calculate attitude 
    MatrixCalc.Multiply( (float *)K0, (float *)error, H, W, 1, (float *)ctrlOut );
    
    ctrlOut[3] = outZ;
  
    MatrixCalc.Multiply( (float *)combOut, (float *)ctrlOut, 4, 4, 1, (float *)motorOuts );
    
    //convert force output to potentiometer output
    float temp = 0.0f;
    
    for( int i = 0; i < 4; i++ )
    {
       temp = (motorOuts[i] / slopeVal[i] ) + 4;
      
        if( temp < MINPOT ) 
        {
           temp = MINPOT; 
        }
        
        if( temp > MAXPOT )
        {
           temp = MAXPOT; 
        }
        
        motorOuts[i] = temp;
    }
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


