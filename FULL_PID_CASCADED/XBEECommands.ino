boolean isCommand = false;
String command = "";
double posData[3];

//gets a command off the 
void getCommands()
{
    if( Serial.available() )
    {
      char c;
    
      while(Serial.available())
      {
        c = Serial.read();
        
        /*
        if( c = '~' )
        {
           isCommand = false;
        }
        */
        
        
        command += c;
      }
      
      //Serial.println( command.length() );
    
    
    checkCommands();
    /*
      if( isCommand )
      {
        checkCommands();
      }
      else
      {
        setPos();
        isCommand = true; 
      }
     */
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
  else if( command.equals("spinto") )
  {
    Serial.println( "SPINNING TO SET AMOUNT" );
    command = "";
  }
  else if( command.equals("land") )
  {
    land();
    Serial.println( "SPINNING TO SET AMOUNT" );
    command = "";
  } 
}

void setPos()
{
  command = "";
  char c;
  char* input;
  c = Serial.read();
  
  while( c != '$' )
  {
    input += c;
  }
  
  Serial.println( input );
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


