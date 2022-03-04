# 1 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
/*******************************************************************************

* Title                 :   Simple Smart Home 

* Filename              :   SmartHome.ino

* Author                :   Mohamed Hafez

* Origin Date           :   11/01/2022

* Version               :   1.0.0

* Compiler              :   TODO: COMPILER GOES HERE

* Target                :   arduino uno 

* Notes                 :   None 

*

*****************************************************************************/
# 12 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
/*************** SOURCE REVISION LOG *****************************************

*

*    Date    Version   Author          Description 

*  17/01/22   1.0.0   Mohamed Hafez   Initial Release.

*

*******************************************************************************/
# 18 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
/** @file SmartHome.ino

 *  @brief This is the source file to make a smart Home. 

 */
# 21 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
/******************************************************************************

* Includes

*******************************************************************************/
# 24 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
# 25 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino" 2
# 26 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino" 2
/******************************************************************************

* Module Preprocessor Constants

*******************************************************************************/
# 29 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
// Sensors Pins





// Actuators Pins




// Fan Motor Pins
// Speed Control Pin

// Direction Control Pin       


// Air suction Motor Pins  
// Speed Control Pin     

// Direction Control Pin 


// Sensors' data






// Speed Value    



// Initialization values    





/******************************************************************************

* Module Preprocessor Macros

*******************************************************************************/
# 73 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
/******************************************************************************

* Module Typedefs

*******************************************************************************/
# 77 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
/******************************************************************************

* Module Variable and Objects Definitions

*******************************************************************************/
# 80 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
/***************************Object of DHT Class************************/
DHT dht(13, DHT11);
/***************************Object of Servo Class****************************/
Servo DoorServo;
// Global variables to read ir sensors 
int Global_TemperatureValue;
int Global_FlameState;
int Global_MotionState;
int Global_GasLevel;
int Global_LightLevel;
char Global_SerialData;
/******************************************************************************

* Function Prototypes

*******************************************************************************/
# 94 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
void Home_InitializeDevices(void);
void Home_ReadSensors(void);
void Home_HandleAllEventsManual(void);
void Home_HandleAllEventsAutomatic(void);

void Home_ActFlameEvent(void);
void Home_ActGasEvent(void);
void Home_ActMotionEvent(void);
void Home_ActTemperatureEvent(void);
void Home_ActLightEvent(void);

void Home_StartAlarmSystem(void);
void Home_StopAlarmSystem(void);
void HOME_OpenDoorServo(void);
void HOME_CloseDoorServo(void);
void Home_RunFan(void);
void Home_RunAirSuction(void);
void Home_StopFan(void);
void Home_StopAirSuction(void);
/******************************************************************************

* Function Definitions

*******************************************************************************/
# 117 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
void setup()
{
  // Variable initialization
  Global_GasLevel = 0;
  Global_LightLevel = 0;
  Global_TemperatureValue = 0;
  Global_MotionState = 0;
  Global_FlameState = 0x1;
  // Initialize devices
  Home_InitializeDevices();
}
void loop()
{
  // Read sensor values
  Home_ReadSensors();
  // Handle and act on all events using bluetooth.
  Home_HandleAllEventsManual();
  // Handle and act on all events automatic.
  Home_HandleAllEventsAutomatic();
}

void Home_InitializeHome(void)
{
  for(int LocalPinsCounter = 3;LocalPinsCounter <= 9;LocalPinsCounter++)
  {
    pinMode(LocalPinsCounter, 0x1);
  }
  for(int LocalPinsCounter = 11;LocalPinsCounter <= 12;LocalPinsCounter++)
  {
    pinMode(LocalPinsCounter, 0x0);
  }
  pinMode(A0, 0x0);
  pinMode(A1, 0x0);
  pinMode(A2, 0x1);
  Serial.begin(9600);
  DoorServo.attach(10);
  DoorServo.write(0);
}

void Home_ReadSensors(void)
{
  Global_GasLevel = analogRead(A1);
  Global_LightLevel = analogRead(A0);
  Global_FlameState = digitalRead(11);
  Global_MotionState = digitalRead(12);
  Global_TemperatureValue = dht.readTemperature();
}

void HOME_OpenDoorServo(void)
{
  for(int LocalServo = 0;LocalServo <= 90;LocalServo++)
  {
    DoorServo.write(LocalServo);
    delay(10);
  }
}

void HOME_CloseDoorServo(void)
{
  for(int LocalServo = 90;LocalServo >= 0;LocalServo--)
  {
    DoorServo.write(LocalServo);
    delay(10);
  }
}

void Home_StartAlarmSystem(void)
{
  digitalWrite(A2, 0x1);
  digitalWrite(9, 0x1);
}

void Home_StopAlarmSystem(void)
{
  digitalWrite(A2, 0x0);
  digitalWrite(9, 0x0);
}

void Home_HandleAllEventsManual(void)
{
  if(Serial.available() > 0)
  {
    Global_SerialData = Serial.read();
    switch(Global_SerialData)
    {
      case 'F':
        Home_RunFan();
        Serial.println("Fan Started");
      break;
      case 'A':
        Home_RunAirSuction();
        Serial.println("Air Suction Started");
      break;
      case 'S':
        Home_StopFan();
        Serial.println("Fan Stopped");
      break;
      case 'R':
        digitalWrite(8, 0x1);
        Serial.println("Lamp opened");
      break;
      case 'O':
        HOME_OpenDoorServo();
        Serial.println("Door Opened");
      break;
      case 'C':
        HOME_CloseDoorServo();
        Serial.println("Door closed");
      break;
    }
  }
}

void Home_HandleAllEventsAutomatic(void)
{
  Home_ActFlameEvent();
  Home_ActGasEvent();
  Home_ActMotionEvent();
  Home_ActTemperatureEvent();
  Home_ActLightEvent();
}

void Home_ActFlameEvent(void)
{
  if(Global_FlameState == 0x0)
  {
    HOME_OpenDoorServo();
    Home_StartAlarmSystem();
    Serial.println("Door Opened");
    Serial.println("Flame! Flame! Alarm system started");
  }
  else
  {
    Home_StopAlarmSystem();
    Serial.println("Door Closed");
    Serial.println("Alarm system stoped");
  }
}

void Home_ActGasEvent(void)
{
  if(Global_GasLevel > 300)
  {
    Home_StopFan();
    HOME_OpenDoorServo();
    Home_RunAirSuction();
    Home_StartAlarmSystem();
    Serial.println("Door Opened");
    Serial.println("Gas! Gas! Alarm system started");
  }
  else
  {
    HOME_CloseDoorServo();
    Home_StopAirSuction();
    Home_StopAlarmSystem();
    Serial.println("Door Closed");
    Serial.println("Alarm system stoped");
  }
}

void Home_ActMotionEvent(void)
{
  if(Global_MotionState == 0x1)
  {
    Home_StartAlarmSystem();
    Serial.println("Motion! Alarm system started");
  }
  else
  {
    Home_StopAlarmSystem();
    Serial.println("Alarm system stoped");
  }
}

void Home_ActTemperatureEvent(void)
{
  if(Global_TemperatureValue > 30)
  {
    Home_RunFan();
    Serial.println("Fan opened");
  }
  else
  {
    Home_StopFan();
    Serial.println("Fan closed");
  }
}

void Home_ActLightEvent(void)
{
  if(Global_LightLevel > 40)
  {
    digitalWrite(8, 0x1);
  }
  else
  {
    digitalWrite(8, 0x0);
  }
}

void Home_RunFan(void)
{
  digitalWrite(2, 0x1);
  digitalWrite(4, 0x0);
  analogWrite(3, 100);
}

void Home_RunAirSuction(void)
{
  digitalWrite(2, 0x0);
  digitalWrite(4, 0x1);
  digitalWrite(6, 0x0);
  digitalWrite(7, 0x1);
  analogWrite(3, 100);
  analogWrite(5, 100);
}

void Home_StopFan(void)
{
  digitalWrite(2, 0x0);
  digitalWrite(4, 0x0);
  analogWrite(3, 0);
}

void Home_StopAirSuction(void)
{
  digitalWrite(6, 0x0);
  digitalWrite(7, 0x0);
  analogWrite(5, 0);
}
