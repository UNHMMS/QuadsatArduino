String command = "";
char c = ' ';
double posData[3];
double newDesZ = 0.1;
double velZ = 0.0;

//gets a command off the 

void getCommands()
{
    if( Serial.available() )
    {
      c = ' ';

      
      while(Serial.available())
      {
        c = Serial.read();
        
        command += c;
        if( c == '*' || c == '$' || c == '@' || c == '!' || c == '&')
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
  
  if(c == '!')
  {
    Serial.println( "KILLED" );
    kill();
    command = ""; 
    c=' ';
  }
  if( c== '&')
  {
    Serial.println("LANDING");
    land();
    command = "";
    c= ' ';
  }
  if( c== '@' )
  {
    Serial.println( "FULL START" );
    startUp();
    command = "";
    c=' ';
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
    //land();
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
    index = command.indexOf("-");
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
  
  newDesZ = atof(cStr);
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
  return newDesZ; 
}
