# 1 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
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
# 12 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
/*************** SOURCE REVISION LOG *****************************************

*

*    Date    Version   Author          Description 

*  17/01/22   1.0.0   Mohamed Hafez   Initial Release.

*

*******************************************************************************/
# 18 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
/** @file LineFollower_Robot.ino

 *  @brief This is the source file to make the robot follow the line. 

 */
# 21 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
/******************************************************************************

* Includes

*******************************************************************************/
# 25 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
/******************************************************************************

* Module Preprocessor Constants

*******************************************************************************/
# 28 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
// Right Motor Pins
// Speed Control Pin

// Direction Control Pin


// Left Motor Pins 
// Speed Control Pin    

// Direction Control Pin


// Delay for motors

// Sensors Pins


// Speed Value


// Initialization values




/******************************************************************************

* Module Preprocessor Macros

*******************************************************************************/
# 57 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
/******************************************************************************

* Module Typedefs

*******************************************************************************/
# 61 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
/******************************************************************************

* Module Variable and Objects Definitions

*******************************************************************************/
# 64 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
// Global variables to read ir sensors
int Global_RightIrSensorValue;
int Global_LeftIrSensorValue;
/******************************************************************************

* Function Prototypes

*******************************************************************************/
# 70 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
void Robot_InitializeRobot(void);
void Robot_SetRobotMotions(void);
void Robot_MoveRobotForward(void);
void Robot_MoveRobotRight(void);
void Robot_MoveRobotLeft(void);
void Robot_StopRobot(void);
/******************************************************************************

* Function Definitions

*******************************************************************************/
# 80 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\01- Simple Line Follower Robot\\Line follower code\\LineFollower_Robot.ino"
void setup()
{
  // Variable initialization
  Global_RightIrSensorValue = 0x1;
  Global_LeftIrSensorValue = 0x1;
  // Initialize robot pins
  Robot_InitializeRobot();
}
void loop()
{
  // Read sensor values
  Global_RightIrSensorValue = digitalRead(A0);
  Global_LeftIrSensorValue = digitalRead(A1);
  // Set robot motions.
  Robot_SetRobotMotions();
}

void Robot_InitializeRobot(void)
{
  for(int LocalPinsCounter = 2;LocalPinsCounter <= 7;LocalPinsCounter++)
  {
    pinMode(LocalPinsCounter, 0x1);
  }
  pinMode(A0, 0x0);
  pinMode(A1, 0x0);
}

void Robot_SetRobotMotions(void)
{
  // Motion Cases
  // Case 1: Both sensors are not on the black line...Move Forward
  if(Global_RightIrSensorValue == 0x0 && Global_LeftIrSensorValue == 0x0)
  {
    Robot_MoveRobotForward();
  }
  // Case 2: Right sensor is on the black line...Move Right
  else if(Global_RightIrSensorValue == 0x1 && Global_LeftIrSensorValue == 0x0)
  {
    Robot_MoveRobotRight();
  }
  // Case 3: Left sensor is on the black line...Move Left
  else if(Global_RightIrSensorValue == 0x0 && Global_LeftIrSensorValue == 0x1)
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
  digitalWrite(2, 0x1);
  digitalWrite(4, 0x0);
  digitalWrite(6, 0x1);
  digitalWrite(7, 0x0);
  analogWrite(3, 100);
  analogWrite(5, 100);
}

void Robot_MoveRobotRight(void)
{
  digitalWrite(2, 0x0);
  digitalWrite(4, 0x0);
  digitalWrite(6, 0x1);
  digitalWrite(7, 0x0);
  analogWrite(3, 0);
  analogWrite(5, 100);
}

void Robot_MoveRobotLeft(void)
{
  digitalWrite(2, 0x1);
  digitalWrite(4, 0x0);
  digitalWrite(6, 0x0);
  digitalWrite(7, 0x0);
  analogWrite(3, 100);
  analogWrite(5, 0);
}

void Robot_StopRobot(void)
{
  digitalWrite(2, 0x0);
  digitalWrite(4, 0x0);
  digitalWrite(6, 0x0);
  digitalWrite(7, 0x0);
  analogWrite(3, 0);
  analogWrite(5, 0);
}
