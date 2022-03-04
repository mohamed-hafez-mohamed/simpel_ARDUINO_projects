#include <Arduino.h>
#line 1 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
/*******************************************************************************
* Title                 :   LineFollower_Robot 
* Filename              :   LineFollower_Robot.ino
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
/** @file LineFollower_Robot.ino
 *  @brief This is the source file to make the robot follow the line. 
 */
/******************************************************************************
* Includes
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
// Right Motor Pins
// Speed Control Pin
#define  RIGHT_MOTOR_SPEED_PIN       3
// Direction Control Pin
#define  RIGHT_MOTOR_PIN_1           2
#define  RIGHT_MOTOR_PIN_2           4
// Left Motor Pins 
// Speed Control Pin    
#define  LEFT_MOTOR_SPEED_PIN        5
// Direction Control Pin
#define  LEFT_MOTOR_PIN_1            6
#define  LEFT_MOTOR_PIN_2            7
// Delay for motors
#define  MOTOR_DELAY                 100
// Sensors Pins
#define  RIGHT_LIGHT_SENSOR_PIN      A0
#define  LEFT_LIGHT_SENSOR_PIN       A1
// Speed Value
#define ROBOT_SPEED_VALUE_0          0
#define ROBOT_SPEED_VALUE_1          100
// Initialization values
#define INITIALIZATION_VALUE         0
#define SENSOR_INITIALIZATION_VALUE  SENSORS_READ_BLACK
#define SENSORS_READ_BLACK           HIGH
#define SENSORS_READ_WHITE           LOW
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable and Objects Definitions
*******************************************************************************/
// Global variables to read ir sensors
int Global_RightIrSensorValue; 
int Global_LeftIrSensorValue;
/******************************************************************************
* Function Prototypes
*******************************************************************************/
void Robot_InitializeRobot(void);
void Robot_SetRobotMotions(void);
void Robot_MoveRobotForward(void);
void Robot_MoveRobotRight(void);
void Robot_MoveRobotLeft(void);
void Robot_StopRobot(void);
/******************************************************************************
* Function Definitions
*******************************************************************************/

#line 80 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
void setup();
#line 88 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
void loop();
#line 80 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
void setup() 
{
  // Variable initialization
  Global_RightIrSensorValue  = SENSOR_INITIALIZATION_VALUE; 
  Global_LeftIrSensorValue   = SENSOR_INITIALIZATION_VALUE;
  // Initialize robot pins
  Robot_InitializeRobot();
}
void loop()
{ 
  // Read sensor values
  Global_RightIrSensorValue  = digitalRead(RIGHT_LIGHT_SENSOR_PIN);
  Global_LeftIrSensorValue   = digitalRead(LEFT_LIGHT_SENSOR_PIN);
  // Set robot motions.
  Robot_SetRobotMotions();
}

void Robot_InitializeRobot(void)
{
  for(int LocalPinsCounter = RIGHT_MOTOR_PIN_1;LocalPinsCounter <= LEFT_MOTOR_PIN_2;LocalPinsCounter++)
  {
    pinMode(LocalPinsCounter, OUTPUT);
  }
  pinMode(RIGHT_LIGHT_SENSOR_PIN, INPUT);
  pinMode(LEFT_LIGHT_SENSOR_PIN, INPUT);
}

void Robot_SetRobotMotions(void)
{
  // Motion Cases
  // Case 1: Both sensors are not on the black line...Move Forward
  if(Global_RightIrSensorValue == SENSORS_READ_WHITE && Global_LeftIrSensorValue == SENSORS_READ_WHITE)
  {
    Robot_MoveRobotForward();
  }
  // Case 2: Right sensor is on the black line...Move Right
  else if(Global_RightIrSensorValue == SENSORS_READ_BLACK && Global_LeftIrSensorValue == SENSORS_READ_WHITE)
  {
    Robot_MoveRobotRight();
  }
  // Case 3: Left sensor is on the black line...Move Left
  else if(Global_RightIrSensorValue == SENSORS_READ_WHITE && Global_LeftIrSensorValue == SENSORS_READ_BLACK)
  {
    Robot_MoveRobotLeft();
  }
  else 
  {
    Robot_StopRobot();
  }
}

void Robot_MoveRobotForward(void)
{
  digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN_2, LOW);  
  digitalWrite(LEFT_MOTOR_PIN_1,  HIGH); 
  digitalWrite(LEFT_MOTOR_PIN_2,  LOW);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, ROBOT_SPEED_VALUE_1);
  analogWrite(LEFT_MOTOR_SPEED_PIN,  ROBOT_SPEED_VALUE_1);
}

void Robot_MoveRobotRight(void)
{
  digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN_2, LOW);  
  digitalWrite(LEFT_MOTOR_PIN_1,  HIGH); 
  digitalWrite(LEFT_MOTOR_PIN_2,  LOW);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, ROBOT_SPEED_VALUE_0);
  analogWrite(LEFT_MOTOR_SPEED_PIN,  ROBOT_SPEED_VALUE_1);
}

void Robot_MoveRobotLeft(void)
{
  digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN_2, LOW);  
  digitalWrite(LEFT_MOTOR_PIN_1,  LOW); 
  digitalWrite(LEFT_MOTOR_PIN_2,  LOW);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, ROBOT_SPEED_VALUE_1);
  analogWrite(LEFT_MOTOR_SPEED_PIN,  ROBOT_SPEED_VALUE_0);
}

void Robot_StopRobot(void)
{
  digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN_2, LOW);  
  digitalWrite(LEFT_MOTOR_PIN_1,  LOW); 
  digitalWrite(LEFT_MOTOR_PIN_2,  LOW);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, ROBOT_SPEED_VALUE_0);
  analogWrite(LEFT_MOTOR_SPEED_PIN,  ROBOT_SPEED_VALUE_0);
}
