# 1 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
/*******************************************************************************

* Title                 :   RC_Car_Mobile 

* Filename              :   RC_Car_Mobile.ino

* Author                :   Mohamed Hafez

* Origin Date           :   11/01/2022

* Version               :   1.0.0

* Compiler              :   TODO: COMPILER GOES HERE

* Target                :   arduino uno 

* Notes                 :   None 

*

*****************************************************************************/
# 12 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
/*************** SOURCE REVISION LOG *****************************************

*

*    Date    Version   Author          Description 

*  11/01/22   1.0.0   Mohamed Hafez   Initial Release.

*

*******************************************************************************/
# 18 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
/** @file RC_Car_Mobile.ino

 *  @brief This is the source file to control the arm robot with five dgrees of freedom. 

 */
# 21 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
/******************************************************************************

* Includes

*******************************************************************************/
# 25 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
/******************************************************************************

* Module Preprocessor Constants

*******************************************************************************/
# 51 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
/******************************************************************************

* Module Preprocessor Macros

*******************************************************************************/
# 55 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
/******************************************************************************

* Module Typedefs

*******************************************************************************/
# 59 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
/******************************************************************************

* Module Variable and Objects Definitions

*******************************************************************************/
# 63 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
/***************************Variables Definition******************************/
long int Global_Distance;
long int Global_WaveDuration;
/******************************************************************************

* Function Prototypes

*******************************************************************************/
# 69 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
void Robot_InitializeRobotPins(void);
void Robot_CalculateDistance(void);
void Robot_SetRobotMotions(void);
void Robot_MoveRobotForward(void);
void Robot_MoveRobotBackward(void);
void Robot_MoveRobotRight(void);
void Robot_MoveRobotLeft(void);
void Robot_StopRobot(void);
/******************************************************************************

* Function Definitions

*******************************************************************************/
# 81 "e:\\01- ST Smart\\01- Mid 2022 Season\\04- Projects\\03- Simple Avoided Obstacles Roboot\\Avoiding obstacles code\\Avoiding_Obstacles.ino"
void setup()
{
  // Initialize distance variable
  Global_Distance = 0;
  // Initialize duration variable
  Global_WaveDuration = 0;
  // Attach dc motors to control pins
  Robot_InitializeRobotPins();
}
void loop()
{
  // Caculate distance
  Robot_CalculateDistance();
  // Robot motiion
  Robot_SetRobotMotions();
}
/*****************************************Auxlariy functions**********************************/
void Robot_InitializeRobotPins(void)
{
  for(int Local_PinsCounter = 2 /* Right dc motor forward  pin */;Local_PinsCounter <= 8 /* Ultrasonic trigger pin*/;Local_PinsCounter++)
  {
    pinMode(Local_PinsCounter, 0x1);
  }
  pinMode(9 /* Ultrasonic echo    pin*/, 0x0);
}

void Robot_CalculateDistance(void)
{
  pinMode(8 /* Ultrasonic trigger pin*/, 0x0);
  delayMicroseconds(2);
  pinMode(8 /* Ultrasonic trigger pin*/, 0x1);
  delayMicroseconds(10);
  pinMode(8 /* Ultrasonic trigger pin*/, 0x0);
  Global_WaveDuration = pulseIn(9 /* Ultrasonic echo    pin*/, 0x1);
  Global_Distance = (Global_WaveDuration / 2) * .0343;
}

void Robot_SetRobotMotions(void)
{
  if(Global_Distance < 20)
  {
    Robot_StopRobot();
    delay(200 /* Delay for motors*/);
    Robot_MoveRobotBackward();
    delay(500);
    Robot_MoveRobotRight();
    delay(500);
  }
  else
  {
    Robot_MoveRobotForward();
  }
}

void Robot_MoveRobotForward(void)
{
  digitalWrite(2 /* Right dc motor forward  pin */, 0x1);
  digitalWrite(4 /* Right dc motor backword pin*/,0x0);
  digitalWrite(7 /* Left dc motor backward pin*/, 0x1);
  digitalWrite(6 /* Left dc motor forward  pin */, 0x0);
  analogWrite(3 /* Right dc motor speed    pin */, 150 /* Motor speed*/);
  analogWrite(5 /* Left dc motor speed    pin*/, 150 /* Motor speed*/);
}

void Robot_MoveRobotBackward(void)
{
  digitalWrite(2 /* Right dc motor forward  pin */, 0x0);
  digitalWrite(4 /* Right dc motor backword pin*/, 0x1);
  digitalWrite(7 /* Left dc motor backward pin*/, 0x0);
  digitalWrite(6 /* Left dc motor forward  pin */, 0x1);
  analogWrite(3 /* Right dc motor speed    pin */, 150 /* Motor speed*/);
  analogWrite(5 /* Left dc motor speed    pin*/, 150 /* Motor speed*/);
}

void Robot_MoveRobotRight(void)
{
  digitalWrite(2 /* Right dc motor forward  pin */, 0x0);
  digitalWrite(4 /* Right dc motor backword pin*/, 0x0);
  digitalWrite(7 /* Left dc motor backward pin*/, 0x1);
  digitalWrite(6 /* Left dc motor forward  pin */, 0x0);
  analogWrite(3 /* Right dc motor speed    pin */, 0 /* Motor speed for stop*/);
  analogWrite(5 /* Left dc motor speed    pin*/, 150 /* Motor speed*/);
}

void Robot_MoveRobotLeft(void)
{
  digitalWrite(2 /* Right dc motor forward  pin */, 0x1);
  digitalWrite(4 /* Right dc motor backword pin*/, 0x0);
  digitalWrite(7 /* Left dc motor backward pin*/, 0x0);
  digitalWrite(6 /* Left dc motor forward  pin */, 0x0);
  analogWrite(3 /* Right dc motor speed    pin */, 150 /* Motor speed*/);
  analogWrite(5 /* Left dc motor speed    pin*/, 0 /* Motor speed for stop*/);
}

void Robot_StopRobot(void)
{
  digitalWrite(2 /* Right dc motor forward  pin */, 0x0);
  digitalWrite(4 /* Right dc motor backword pin*/, 0x0);
  digitalWrite(7 /* Left dc motor backward pin*/, 0x0);
  digitalWrite(6 /* Left dc motor forward  pin */, 0x0);
  analogWrite(3 /* Right dc motor speed    pin */, 0 /* Motor speed for stop*/);
  analogWrite(5 /* Left dc motor speed    pin*/, 0 /* Motor speed for stop*/);
}
