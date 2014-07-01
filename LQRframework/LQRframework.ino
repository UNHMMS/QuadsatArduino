#include <Servo.h>
#include <MatrixMath.h>

#define H (4)              //the height of the LQR gain matrix
#define W (5)              //the width of the LQR gain matrix
#define SLOPEVAL (0.907)   //the flope of the (Force/Potentiometer) curve 
#define MINPOT (70)        //the minimum valid potentiometer value
#define MAXPOT (140)       //the maximum valid potentiometer value

float K[H][W] =            //LQR Gain Matrix
{
   {0.0f      , 0.0f     , -223.6068f, -33.0267f, 111.8034f }, 
   {223.6068f , 33.0267f , 0.0f      , 0.0f     , -111.8034f}, 
   {0.0f      , 0.0f     , 223.6068f , 33.0267f , 111.8034f }, 
   {-223.6068f, -33.0267f, 0.0f      , 0.0f     , -111.8034f}, 
};

float desState[W] =        //Desired State Vector (state we want to achieve)
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  
//{ pitch, pitch rate, roll, roll rate, yaw }

float curState[W];         //Current State Vector (Current State accourding to 9DoF Sensor Stick)
float error[W];            //How far of the desired state is from the actual
float output[H];           //The output values for the motors
                           //    This will originially be in Newtons 
                           //    Then converts to potentiometer values
                        
Servo motors[4];           //servo objects that will be used to
                           //    interface with the brushless motors

int motorPins[] =          //pins that the motors are tied to
  {2, 4, 7, 8};      

unsigned long elapsedTime; //time the loop took
unsigned long loopStart;   //time the loop started at

//setup the K matrix and the desired state vector
//start the serial communication
void setup()
{
    Serial.begin(9600);
    
    //setup motor stuff
    setupMotorPins();
}

//LQR loop for flying the quad
void loop()
{
    loopStart = micros();
  
    //get current state from the sensor
    getSensorValues();
    
    //do matrix math to get potentiometer outputs
    doLQR();
    
    elapsedTime = micros() - loopStart;
}

//retrieves data from 9DoF sensor stick and sets current state
void getSensorValues()
{
    //get in 9DoF sensor stick information
    
    //calculate any necessary values
        //pitch
        //pitch rate
        //roll
        //roll rate
        //yaw
        
    //put values into curState vector
}

//calculates the error vector and the output potentiometer values using LQR
void doLQR()
{ 
   //calculate error vector
    Matrix.Subtract( (float *)desState, (float *)curState, W, 1, (float *)error );
        
    //calculate output force vector
    Matrix.Multiply( (float *)K, (float *)error, H, W, 1, (float *)output );
        
    //convert force output to potentiometer output
    float temp = 0.0f;
    
    for( int i = 0; i < H; i++ )
    {
       temp = output[i] / SLOPEVAL;
      
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
    motors[0].attach( motorPin1 );
    motors[1].attach( motorPin2 );
    motors[2].attach( motorPin3 );
    motors[3].attach( motorPin4 );
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


