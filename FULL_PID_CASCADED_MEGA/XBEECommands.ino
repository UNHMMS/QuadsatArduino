String command = "";
double posData[3];
double desRoll = 0.0;

//gets a command off the 
void getCommands()
{
    if( Serial.available() )
    {
      char c;
    
      while(Serial.available())
      {
        c = Serial.read();
        
        command += c;
      }
      
      checkCommands();
    }
}

//checks serial input for commands
void checkCommands()
{
  
  if(command.equals("kill"))
  {
    Serial.println( "KILLED" );
    kill();
    command = ""; 
  }
  else if( command.equals("start") )
  {
    Serial.println( "FULL START" );
    startUp();
    command = "";
  }
  else if( command.equals("spinup") )
  {
    Serial.println( "SPINNING UP" );
    spinUp();
    command = "";
  }
  else if( command.equals("ctrlstart") )
  {
    Serial.println(  "STARTING CONTROL" );
    controlStart();
    command = "";
  }
  else if( command.equals("land") )
  {
    Serial.println( "LANDING" );
    land();
    command = "";
  }
  else if( command.startsWith("~") && command.endsWith("$") )
  {
    Serial.println( "READING POS DATA" );
    setPos();
    command = "";
  }
  else if( command.startsWith( "SetRollAngle:" ) && command.endsWith("*") )
  {
    Serial.println( "SETTING ROLL ANGLE" );
    setDesRoll();
    command = "";
  }
}

void setPos()
{
  int index = 0;
  int len = command.length();
  String strVal = "";
  char cStr[10];
  
  for( int i = 0; i < 3; i++ )
  {
    index = command.indexOf(" ");
    strVal = command.substring( 0, index );
    strVal.toCharArray( cStr, 10 );
    command = command.substring( index+1, len );
    
    posData[i] = atof(cStr);
  }
}

void setDesRoll()
{
  int index = 0;
  int len = command.length();
  String strVal = "";
  char cStr[10];
  
  index = command.indexOf( " " );
  strVal = command.substring( index, len );
  strVal.toCharArray( cStr, 10 );
  
  desRoll = atof(cStr);
  

  if( desRoll < -0.95 )
  {
    desRoll = -0.95;
  }
  
  if( desRoll > 0.95 )
  {
    desRoll = 0.95;
  }
  
  
}

double getX()
{
  return posData[0];
}

double getY()
{
  return posData[1];
}

double getZ()
{
  return posData[2];
}

double getDesRoll()
{
  return desRoll; 
}
