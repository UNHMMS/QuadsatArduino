String command = "";
double posData[3];
double desZ = 0.0;
double velZ = 0.0;

//gets a command off the 

void getCommands()
{
    if( Serial.available() )
    {
      char c = ' ';

      
      while(Serial.available())
      {
        c = Serial.read();
        command += c;
        
        if( c == '!')
        {
          kill();
          Serial.println( "KILLED" );
          command = "";
        }
        else if ( c == '@' )
        {
          startUp();
          Serial.println("STARTED");
          command = "";
        }
        else if( c == '*' || c == '$' )
        {
          checkCommands();
          command = "";
        }
       
      }
      Serial.println( command );
      
      checkCommands();
    } 
}

//checks serial input for commands
void checkCommands()
{
  
//  if(command.contains("kill"))
//  {
//    Serial.println( "KILLED" );
//    kill();
//    command = ""; 
//  }
  if( command.equals("start") )
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
    //Serial.println( "READING POS DATA" );
    Serial.println( command );
    parseCurZ();
    command = "";
  }
  else if( command.startsWith( "SetZPos:" ) && command.endsWith("*") )
  {
    Serial.println( "SETTING Z POS" );
    setZ();
    command = "";
  }
}

void parseCurZ()
{
  int index = 0;
  int len = command.length();
  String strVal = "";
  char cStr[10];
  
  //for( int i = 0; i < 3; i++ )
  //{
    index = command.indexOf(" ");
    strVal = command.substring( 1, index );
    strVal.toCharArray( cStr, 10 );
    
    posData[2] = atof(cStr);
    
    strVal = command.substring( index+1, len-1 );
    strVal.toCharArray( cStr, 10 );
    //command = command.substring( index+1, len );
    
    velZ = atof(cStr);
    
    //Serial.print( posData[2] );
   // Serial.print( " " );
    //Serial.print( velZ );
    //Serial.println();
}

void setZ()
{
  int index = 0;
  int len = command.length();
  String strVal = "";
  char cStr[10];
  
  index = command.indexOf( " " );
  strVal = command.substring( index, len );
  strVal.toCharArray( cStr, 10 );
  
  desZ = atof(cStr);
  

  if( desZ < -0.95 )
  {
    desZ = -0.95;
  }
  
  if( desZ > 0.95 )
  {
    desZ = 0.95;
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
  //Serial.println( posData[2] );
  return posData[2];
}

double getZVel()
{
  return velZ; 
}

double getDesZ()
{
  return desZ; 
}
