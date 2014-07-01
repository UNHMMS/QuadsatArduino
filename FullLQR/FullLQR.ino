#include <Servo.h>
#include <MatrixMath.h>

#define H (4)              //the height of the LQR gain matrix
#define W (12)              //the width of the LQR gain matrix
#define SLOPEVAL (0.0907)   //the flope of the (Force/Potentiometer) curve 
#define MINPOT (70)        //the minimum valid potentiometer value
#define MAXPOT (120)       //the maximum valid potentiometer value

float K[H][W] =            //LQR Gain Matrix
{
   { -0.9856, -0.0164, -11.3949, -0.9325,  3.5355,  11.2494,  4.2758,  2.8602,  0.7259,  0.4742, 6.1141, 1.8581},
   {  9.9237,  0.9048,  -0.4728, -0.0078, -3.5355, -11.2494, -0.5574, -0.4767,  5.5581,  3.8089, 3.5486, 1.1814},
   { -0.4715, -0.0077,   9.9328,  0.9081,  3.5355,  11.2494, -5.5572, -3.8099,  0.5580,  0.4771, 3.5513, 1.1819},
   {-11.3808, -0.9288,  -0.9893, -0.0166, -3.5355, -11.2494, -0.7241, -0.4730, -4.2742, -2.8575, 6.1167, 1.8585}
};

float desState[W] =        //Desired State Vector (state we want to achieve)
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f};  
//{ roll, roll rate, pitch, pitch rate, yaw, yaw rate, x position, x pos rate, y pos, y pos rate, z pos, z pos rate }

float curState[W]=        //Current State Vector (Current State accourding to 9DoF Sensor Stick)
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};         
float error[W];            //How far of the desired state is from the actual
float output[H];           //The output values for the motors
                           //    This will originially be in Newtons 
                           //    Then converts to potentiometer values
                        
Servo motors[4];           //servo objects that will be used to
                           //    interface with the brushless motors

int motorPins[] =          //pins that the motors are tied to
  {2, 4, 6, 8};      

float lastYaw   = 0.0f;
float rateYaw   = 0.0f;
float lastPitch = 0.0f;
float ratePitch = 0.0f;
float lastRoll  = 0.0f;
float rateRoll  = 0.0f;
unsigned long curTime = 0;     //the time the current reading is taken at
unsigned long loopStart = 0;   //time the loop started at
unsigned long lastTime = 0;    //the last recorded time of a reading

unsigned long whyDoesThisSuck = 0;
unsigned long thatSucks = 0;

unsigned long inVal = 0;
float timeChunk = 0.0f;    // amount of time from last reading to current
boolean doneOne = false;

//setup the K matrix and the desired state vector
//start the serial communication
void setup()
{
    Serial.begin( 57600 );
    Serial1.begin( 57600 );
    
    //setup motor stuff
    setupMotorPins();
    
    Serial.println( "Ready..." );
    delay( 1000 );
    Serial.println( "Steady..." );
    
    slowStart();
    
    
    Serial.println( "Go!" );
    
    loopStart = micros();
    
    Serial.print( "start: " );
    Serial.println( loopStart );
}

//slowly initialize motors
void slowStart()
{
    for( int i = 0; i < 70; i++)
    {
       for( int j = 0; j < 4; j++ )
       {
          motors[j].write( i );
       }
       //delay( 100 );
    } 
}

//LQR loop for flying the quad
void loop()
{
    //get current state from the sensor
    getCurState();
    
    //do matrix math to get potentiometer outputs
    doLQR();
    
    Serial.print( "start3: " );
    Serial.println( loopStart );
    
    //write values to the motors
    fly();
    
    
    whyDoesThisSuck = micros();
    
    Serial.print( "micros(): " );
    Serial.println( whyDoesThisSuck );
    
    Serial.print( "start4: " );
    Serial.println( loopStart );
    
    thatSucks = whyDoesThisSuck - loopStart;
    
    Serial.print( "running time: " );
    Serial.println( thatSucks );
    
    //if we've been flying for 10 seconds stop doing anything
    if( (micros() - loopStart) > 10000000 )
    {
      for( int i = 0; i < 4; i++ )
      {
       motors[i].write( 0 ); 
      }
    
       Serial.println( "Done" );
      
       while( true )
       {} 
    }
}

//write output values to the motors
void fly()
{
  for( int i = 0; i < H; i++ )
  {
     //Serial.print( i + 1 );
     //Serial.print( ": " );
     //Serial.println( (int)output[i] );
     motors[i].write( (int)output[i] ); 
  }
  
  //Serial.println();
}

//retrieves data from 9DoF sensor stick and sets current state
void getCurState()
{
    //get 9DOF readings   
    readIMUVals();
    
    //get in 9DoF sensor stick information
    curTime = micros();
    
    if( doneOne )
    { 
      //find the time difference between the last run and this run
      timeChunk = (curTime - lastTime) / 1000000.0f;
      
      //print the time difference
      Serial.println( timeChunk );
      
      //find the rates for the attitude readings
      rateRoll = (float)(lastRoll - curState[0]) / timeChunk;
      ratePitch = (float)(lastPitch - curState[2]) / timeChunk;
      rateYaw = (float)(lastYaw - curState[4]) / timeChunk;
      
      //Serial.print( "Roll Rate: " );
      //Serial.println( rateRoll );
      //Serial.print( "Pitch Rate: " );
      //Serial.println( ratePitch );
      //Serial.print( "Yaw Rate: ");
      //Serial.println( rateYaw );
      
      //Serial.println();
    }
    
    //assign rate values values into curState array
        //pitch rate
    curState[1] = ratePitch;
        //roll rate
    curState[3] = rateRoll;
        //yaw rate
    curState[5] = rateYaw;
    
    //set last values for next iteration
    lastRoll = curState[0];
    lastPitch = curState[2];
    lastYaw = curState[4];
    
    lastTime = curTime;
    
    doneOne = true;
}

//read values from the IMU and store them in the current state
void readIMUVals()
{  
    inVal = Serial1.parseInt();
  
    if( inVal < 180000000 && inVal > 541000000 )
    {
      readIMUVals();
    }
    
    for( int i = 0; i < 5; i = i + 2 )
    {
      curState[i] = (float)((inVal % 1000) - 360.0);
      inVal = inVal / 1000;
    }
    
    curState[0] = curState[0] * (3.14159/180.0);
    curState[2] = curState[2] * (3.14159/180.0);
    curState[4] = curState[4] * (3.14159/180.0);
    
    //Serial.println( curState[0] );
    //Serial.println( curState[2] );
    //Serial.println( curState[4] );
    //Serial.println();
}

//calculates the error vector and the output potentiometer values using LQR
void doLQR()
{ 
  
   Serial.print( "1: " );
   Serial.println( loopStart );
   
   //calculate error vector
    Matrix.Subtract( (float *)desState, (float *)curState, 1, 12, (float *)error );
        
    Serial.print( "2: " );
    Serial.println( loopStart ); 
        
    //calculate output force vector
    Matrix.Multiply( (float *)K, (float *)error, H, W, W, (float *)output );
    
    Serial.print( "3: " );
    Serial.println( loopStart );
        
    //convert force output to potentiometer output
    float temp = 0.0f;
    
    Serial.print( "4: " );
    Serial.println( loopStart );
    
    //for each output value make sure it's within our functional range
    for( int i = 0; i < H; i++ )
    {
       temp = (output[i] / SLOPEVAL) + 60;
      
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

/*
//test method with some bogus values to test logic
void testError()
{
    curState[0] = 1.0f;
    curState[1] = 0.0f;
    curState[2] = 1.0f;
    curState[3] = 0.0f;
    curState[4] = 1.0f;
    
    //print the current state
    Matrix.Print( (float *)curState, W, 1, "Current State:" );
    
    //subtract for the error vector
    Matrix.Subtract( (float *)desState, (float *)curState, W, 1, (float *)error );
    
    //print the error
    Matrix.Print( (float *)error, W, 1, "Error:" );
    
    //find the output vector
    Matrix.Multiply( (float *)K, (float *)error, H, W, 1, (float *)output );
    
    //print the output vector
    Matrix.Print( (float *)output, H, 1, "Output Forces:" );
    
    //convert forces to potentiometer values
    float temp = 0.0f;
    
    for( int i = 0; i < H; i++ )
    {
       temp = output[i] / SLOPEVAL;
      
        if( temp < 70.0f ) 
        {
           temp = 70.0f; 
        }
        
        if( temp > 140.0f )
        {
           temp = 140.0f; 
        }
        
        output[i] = temp;
    }
    
    //print the output vector
    Matrix.Print( (float *)output, H, 1, "Output Potentiometer:" );
}
*/


