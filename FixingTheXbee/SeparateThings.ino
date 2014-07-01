boolean bladesOn;

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

//writes motors to 0 and sits on the serial communication for commands
void spinDown()
{
  for( int i = 0; i < 4; i++ )
  {
    motors[i].write( 0 ); 
  }
}
