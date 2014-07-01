#include <Servo.h>
#include <MatrixMath.h>
#include <MPULib.h>
#include <Wire.h>
#include <PID_v1.h>

#define H (4)              //the height of the LQR gain matrix
#define W (6)              //the width of the LQR gain matrix
#define SLOPEVAL (0.043)  //the flope of the (Force/Potentiometer) curve 
#define MINPOT (4)         //the minimum valid potentiometer value
#define MAXPOT (179)       //the maximum valid potentiometer value
#define ZMAX (10)          //the maximum value (in newtons) that the motors can push

float K0[H][W] =            //LQR Gain Matrix
{
   {       0,         0,    0.7071,    0.5225,    0.3536,    3.3956 },
   {  0.7071,    0.5221,         0,         0,   -0.3536,   -3.3956 },
   {       0,         0,   -0.7071,   -0.5225,    0.3536,    3.3956 },
   { -0.7071,   -0.5221,         0,         0,   -0.3536,   -3.3956
 }
};

float K1[H][W] =            //LQR Gain Matrix
{
   {  0.1033,    0.0005,    0.6760,    0.5227,    0.3757,    3.3784 },
   {  0.7627,    0.5235,   -0.2090,    0.0004,   -0.2495,   -3.3015 },
   { -0.3056,   -0.0004,   -0.6752,   -0.5226,    0.3168,    3.3425 },
   { -0.5605,   -0.5235,    0.2083,   -0.0004,   -0.4430,   -3.4193 }
};

float K2[H][W] =            //LQR Gain Matrix
{
   {  0.1640,    0.0019,    0.6189,    0.5236,    0.4124,    3.3512 },
   {  0.7466,    0.5254,   -0.3376,    0.0018,   -0.1983,   -3.2327 },
   { -0.4786,   -0.0018,   -0.6220,   -0.5235,    0.2588,    3.2660 },
   { -0.4320,   -0.5255,    0.3407,   -0.0019,   -0.4729,   -3.3845 }
};

float curK[H][W];

double desZ = 0.3;
double curZ = 0.0;
double outZ = 0.0;

PID zPosCtrl( &curZ, &outZ, &desZ, 10, 30, 1, DIRECT );

float desState[W] =        //Desired State Vector (state we want to achieve)
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  
//{ pitch, pitch rate, roll, roll rate, yaw, yaw rate, x position, x pos rate, y pos, y pos rate, z pos, z pos rate }

float curState[W]=        //Current State Vector (Current State accourding to 9DoF Sensor Stick)
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
float error[W];            //How far of the desired state is from the actual
float output[H];           //The output values for the motors
                           //    This will originially be in Newtons 
                           //    Then converts to potentiometer values
                        
Servo motors[4];           //servo objects that will be used to
                           //    interface with the brushless motors
                           
int motorVals[4] =         //The motor numbers in the correct order  
  {2, 3, 4, 1};   

int motorPins[] =          //pins that the motors are tied to
  {8, 10, 12, 13};      

float curYaw    = 0.0f;
float lastYaw   = 0.0f;
float rateYaw   = 0.0f;
float lastPitch = 0.0f;
float ratePitch = 0.0f;
float lastRoll  = 0.0f;
float rateRoll  = 0.0f;
unsigned long curTime = 0;     //the time the current reading is taken at
unsigned long loopStart = 0;   //time the loop started at
unsigned long lastTime = 0;    //the last recorded time of a reading
unsigned long inVal = 0;

unsigned long timeBefore = 0;  //time before the lqr loop
unsigned long timeAfter = 0;   //time after the lqr loop

float timeChunk = 0.0f;    // amount of time from last reading to current
boolean doneOne = false;

int spinVal = 70;  //motor resting value

boolean ctrlOn = false;
boolean bladesOn = false;

//setup the K matrix and the desired state vector
//start the serial communication
void setup()
{
    Serial.begin( 9600 );
    
    initIMU();
    
    //setup PID
    zPosCtrl.SetSampleTime(20);
    zPosCtrl.SetOutputLimits(0, ZMAX);
    zPosCtrl.SetMode( AUTOMATIC );
    
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
        doLQR();
    
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
  setYawDomain();
  
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
  
  updateIMU();
  setYawDomain();
  
  setKalmanRollAngle();
  setKalmanPitchAngle();
  
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
  
  //turn control off
  ctrlOn = false;
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
  for( int i = 0; i < H; i++ )
  {
     //Serial.print( motorVals[i] );
     //Serial.print( ": " );
     //Serial.println( (int)output[i] );
     motors[i].write( (int)output[i] ); 
  }
  
  //Serial.println();
}

//read values from the IMU and store them in the current state
void readIMUVals()
{
  updateIMU();
  curState[0] = getPitch(); //pitch
  curState[1] = getGyroPitch(); //pitch rate
  curState[2] = getRoll(); //roll
  curState[3] = getGyroRoll(); //roll rate
  //curState[4] = getYaw(); 
  //curState[5] = getGyroYaw(); //yaw rate
  
  curZ = getZ();
  desZ = getDesZ();
  
  Serial.print( "~ " );

  Serial.print( getRawRoll() );
  Serial.print( " " );
  Serial.print( getRawPitch() );
  Serial.print( " " );
  Serial.print( getRawYaw() );
  Serial.print( " " );  
  
  Serial.print( curState[2] );
  Serial.print( " " );
  Serial.print( curState[0] );
  Serial.print( " " );
  Serial.print( getYaw() );
  Serial.print( " " );  
  
  Serial.print( getGyroYaw() );
  Serial.print( " " );
  Serial.println();
  /*
  //get in 9DoF sensor stick information
    curTime = micros();
    
    inVal = Serial1.parseInt();
  
    if( inVal < 180000000 && inVal > 541000000 )
    {
      readIMUVals();
    }
    
    for( int i = 0; i < 3; i = i + 2 )
    {
      curState[i] = (float)((inVal % 1000) - 360.0);
      inVal = inVal / 1000;
    }
    
    curYaw = (float)((inVal % 1000) - 360.0);
    
    curState[0] = curState[0] * (3.14159/180.0);
    curState[2] = curState[2] * (3.14159/180.0);
    curYaw = curYaw * (3.14159/180.0);
    */
    
    //Serial.println( curState[0] );
    //Serial.println( curState[1] );
    //Serial.println( curYaw );
    //Serial.println();
}

//calculates the error vector and the output potentiometer values using LQR
void doLQR()
{ 
   //calculate error vector
    MatrixCalc.Subtract( (float *)desState, (float *)curState, 1, W, (float *)error );
        
    float absPitch = abs(curState[0]);
    float absRoll = abs(curState[2]);
 
    float checker = max( absPitch, absRoll );   
    
    //calculate output force vector
    
    if ( checker > 0.5 )
    {
      Serial.println( "3" );
      MatrixCalc.Multiply( (float *)K2, (float *)error, H, W, 1, (float *)output );
    }
    else if ( checker > 0.3 && checker < 0.5 )
    {
      Serial.println( "2" );
      MatrixCalc.Multiply( (float *)K1, (float *)error, H, W, 1, (float *)output );
    }
    else
    {
      Serial.println( "1" );
      MatrixCalc.Multiply( (float *)K0, (float *)error, H, W, 1, (float *)output );
    }
    
    //MatrixCalc.Multiply( (float *)K0, (float *)error, H, W, 1, (float *)output );
    
    //zPosCtrl.Compute();
    //outZ = outZ*.25;
    
    //convert force output to potentiometer output
    float temp = 0.0f;
    
    /*
    Serial.print(millis());
    Serial.print(" ");
    for(int i = 0; i < H; i++)
    {
      Serial.print(output[i]);
      Serial.print(" ");
    }
    Serial.print("\n");
    */
    
    for( int i = 0; i < H; i++ )
    {
       temp = ((output[i] + outZ) / SLOPEVAL) + 4;
      
        if( temp < MINPOT ) 
        {
           temp = MINPOT; 
        }
        
        if( temp > MAXPOT )
        {
           temp = MAXPOT; 
        }
        
        output[i] = temp;
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


