#include <Arduino.h>
#line 1 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
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
/*************** SOURCE REVISION LOG *****************************************
*
*    Date    Version   Author          Description 
*  17/01/22   1.0.0   Mohamed Hafez   Initial Release.
*
*******************************************************************************/
/** @file SmartHome.ino
 *  @brief This is the source file to make a smart Home. 
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <DHT.h>
#include <Servo.h>
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
// Sensors Pins
#define  FLAME_SENSOR_PIN                11
#define  PIR_MOTION_SENSOR_PIN           12
#define  TEMPERATURE_SENSOR_PIN          13
#define  LDR_SENSOR_PIN                  A0
#define  GAS_SENSOR_PIN                  A1 
// Actuators Pins
#define RED_LED                          A2
#define RELAY_PIN                        8
#define BUZZER_PIN                       9
#define SERVO_PIN                        10
// Fan Motor Pins
// Speed Control Pin
#define FAN_MOTOR_SPEED_PIN              3
// Direction Control Pin       
#define FAN_MOTOR_PIN_1                  2
#define FAN_MOTOR_PIN_2                  4
// Air suction Motor Pins  
// Speed Control Pin     
#define AIR_SUCTION_MOTOR_SPEED_PIN      5
// Direction Control Pin 
#define AIR_SUCTION_MOTOR_PIN_1          6
#define AIR_SUCTION_MOTOR_PIN_2          7
// Sensors' data
#define UP_TEMPERATURE_THRESHOLD         30
#define UP_GAS_THRESHOLD                 300
#define LIGHT_LEVEL_THRESHOLD            40
#define MOTION_DETECTED                  HIGH
#define FLAME_DETECTED                   LOW
#define FLAME_NOT_DETECTED               HIGH
// Speed Value    
#define MOTOR_DELAY                      10
#define MOTOR_SPEED_VALUE_0              0
#define MOTOR_SPEED_VALUE_1              100
// Initialization values    
#define ZERO_INITIALIZATION_VALUE        0
#define BAUD_RATE                        9600
#define SERVO_INITIAL_VALUE              0
#define SERVO_VALUE                      90
#define SERVO_DELAY                      10
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable and Objects Definitions
*******************************************************************************/
/***************************Object of DHT Class************************/
DHT dht(TEMPERATURE_SENSOR_PIN, DHT11);
/***************************Object of Servo Class****************************/
Servo DoorServo;
// Global variables to read ir sensors 
int    Global_TemperatureValue;
int    Global_FlameState;
int    Global_MotionState;
int    Global_GasLevel;
int    Global_LightLevel;
char   Global_SerialData;
/******************************************************************************
* Function Prototypes
*******************************************************************************/
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

#line 117 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
void setup();
#line 128 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
void loop();
#line 138 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
void Home_InitializeHome(void);
#line 117 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\04- Simple Smart Home\\Smart Home code\\SmartHome.ino"
void setup() 
{
  // Variable initialization
  Global_GasLevel         = ZERO_INITIALIZATION_VALUE;
  Global_LightLevel       = ZERO_INITIALIZATION_VALUE;
  Global_TemperatureValue = ZERO_INITIALIZATION_VALUE; 
  Global_MotionState      = ZERO_INITIALIZATION_VALUE;
  Global_FlameState       = FLAME_NOT_DETECTED;
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
  for(int LocalPinsCounter = FAN_MOTOR_SPEED_PIN;LocalPinsCounter <= BUZZER_PIN;LocalPinsCounter++)
  {
    pinMode(LocalPinsCounter, OUTPUT);
  }
  for(int LocalPinsCounter = FLAME_SENSOR_PIN;LocalPinsCounter <= PIR_MOTION_SENSOR_PIN;LocalPinsCounter++)
  {
    pinMode(LocalPinsCounter, INPUT);
  }
  pinMode(LDR_SENSOR_PIN, INPUT);
  pinMode(GAS_SENSOR_PIN, INPUT);
  pinMode(RED_LED, OUTPUT);
  Serial.begin(BAUD_RATE);
  DoorServo.attach(SERVO_PIN);
  DoorServo.write(SERVO_INITIAL_VALUE);
}

void Home_ReadSensors(void)
{
  Global_GasLevel         = analogRead(GAS_SENSOR_PIN);
  Global_LightLevel       = analogRead(LDR_SENSOR_PIN);
  Global_FlameState       = digitalRead(FLAME_SENSOR_PIN);
  Global_MotionState      = digitalRead(PIR_MOTION_SENSOR_PIN);
  Global_TemperatureValue = dht.readTemperature();
}

void HOME_OpenDoorServo(void)
{
  for(int LocalServo = SERVO_INITIAL_VALUE;LocalServo <= SERVO_VALUE;LocalServo++)
  {
    DoorServo.write(LocalServo);
    delay(SERVO_DELAY);
  }
}

void HOME_CloseDoorServo(void)
{
  for(int LocalServo = SERVO_VALUE;LocalServo >= SERVO_INITIAL_VALUE;LocalServo--)
  {
    DoorServo.write(LocalServo);
    delay(SERVO_DELAY);
  }
}

void Home_StartAlarmSystem(void)
{
  digitalWrite(RED_LED, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
}

void Home_StopAlarmSystem(void)
{
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
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
        digitalWrite(RELAY_PIN, HIGH);
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
  if(Global_FlameState == FLAME_DETECTED)
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
  if(Global_GasLevel > UP_GAS_THRESHOLD)
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
  if(Global_MotionState == MOTION_DETECTED)
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
  if(Global_TemperatureValue > UP_TEMPERATURE_THRESHOLD)
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
  if(Global_LightLevel > LIGHT_LEVEL_THRESHOLD)
  {
    digitalWrite(RELAY_PIN, HIGH);
  }
  else
  {
    digitalWrite(RELAY_PIN, LOW);
  }
}

void Home_RunFan(void)
{
  digitalWrite(FAN_MOTOR_PIN_1, HIGH);
  digitalWrite(FAN_MOTOR_PIN_2, LOW);  
  analogWrite(FAN_MOTOR_SPEED_PIN, MOTOR_SPEED_VALUE_1);
}

void Home_RunAirSuction(void)
{
  digitalWrite(FAN_MOTOR_PIN_1, LOW);
  digitalWrite(FAN_MOTOR_PIN_2, HIGH);  
  digitalWrite(AIR_SUCTION_MOTOR_PIN_1,  LOW); 
  digitalWrite(AIR_SUCTION_MOTOR_PIN_2,  HIGH);
  analogWrite(FAN_MOTOR_SPEED_PIN, MOTOR_SPEED_VALUE_1);
  analogWrite(AIR_SUCTION_MOTOR_SPEED_PIN,  MOTOR_SPEED_VALUE_1);
}

void Home_StopFan(void)
{
  digitalWrite(FAN_MOTOR_PIN_1, LOW);
  digitalWrite(FAN_MOTOR_PIN_2, LOW);  
  analogWrite(FAN_MOTOR_SPEED_PIN, MOTOR_SPEED_VALUE_0);
}

void Home_StopAirSuction(void)
{
  digitalWrite(AIR_SUCTION_MOTOR_PIN_1,  LOW); 
  digitalWrite(AIR_SUCTION_MOTOR_PIN_2,  LOW);
  analogWrite(AIR_SUCTION_MOTOR_SPEED_PIN,  MOTOR_SPEED_VALUE_0);
}
