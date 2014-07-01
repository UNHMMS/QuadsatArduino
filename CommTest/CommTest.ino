#import <Serial.h>
String command = "";
double posData[3];

void setup() 
{
  Serial.begin(9600);
  // put your setup code here, to run once:
  Serial.println( "Ready..." );

}
void loop()
{
   getCommands(); 
}

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
    command = ""; 
  }
  else if( command.equals("start") )
  {
    Serial.println( "FULL START" );
    command = "";
  }
  else if( command.equals("spinup") )
  {
    Serial.println( "SPINNING UP" );
    command = "";
  }
  else if( command.equals("ctrlstart") )
  {
    Serial.println(  "STARTING CONTROL" );
    command = "";
  }
  else if( command.equals("spinto") )
  {
    Serial.println( "SPINNING TO SET AMOUNT" );
    command = "";
  }
  else if( command.equals("land") )
  {
    Serial.println( "LANDING" );
    command = "";
  } 
  else if( command.startsWith("~") && command.endsWith("$") )
  {
    //Serial.println( command );
    setPos();
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
    Serial.print( posData[i] );
    Serial.print( " " );
  }
  Serial.println();
  
  
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
