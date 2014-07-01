String command = "";
double posData[3];
double desZ = 0.35;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin( 9600 );
}

void loop() 
{
  // put your main code here, to run repeatedly:
  getCommands();
}


