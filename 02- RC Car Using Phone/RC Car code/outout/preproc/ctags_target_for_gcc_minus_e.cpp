# 1 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
/*******************************************************************************

* Title                 :   RC_Car_Mobile 

* Filename              :   RcCar.ino

* Author                :   Mohamed Hafez

* Origin Date           :   11/01/2022

* Version               :   1.0.0

* Compiler              :   TODO: COMPILER GOES HERE

* Target                :   arduino uno 

* Notes                 :   None 

*

*****************************************************************************/
# 12 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
/*************** SOURCE REVISION LOG *****************************************

*

*    Date    Version   Author          Description 

*  11/01/22   1.0.0   Mohamed Hafez   Initial Release.

*

*******************************************************************************/
# 18 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
/** @file RC_Car_Mobile.ino

 *  @brief This is the source file to control the arm robot with five dgrees of freedom. 

 */
# 21 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
/******************************************************************************

* Includes

*******************************************************************************/
# 25 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
/******************************************************************************

* Module Preprocessor Constants

*******************************************************************************/
# 46 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
/******************************************************************************

* Module Preprocessor Macros

*******************************************************************************/
# 50 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
/******************************************************************************

* Module Typedefs

*******************************************************************************/
# 54 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
/******************************************************************************

* Module Variable and Objects Definitions

*******************************************************************************/
# 58 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
/***************************Variables Definition******************************/
char Global_BluetoothData; //data from bluetooth  

/******************************************************************************

* Function Prototypes

*******************************************************************************/
# 64 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
void Robot_InitializeDcMotors(void);
void Robot_SetDcMotorsSpeed(void);
void Robot_SetRobotMotions(void);
void Robot_MoveRobotForward(void);
void Robot_MoveRobotBackward(void);
void Robot_MoveRobotRight(void);
void Robot_MoveRobotLeft(void);
void Robot_StopRobot(void);
/******************************************************************************

* Function Definitions

*******************************************************************************/
# 76 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
void setup()
{
  // Initialize bluetooth data variable
  Global_BluetoothData = '\0';
  // Attach dc motors to control pins
  Robot_InitializeDcMotors();
  // Initialize baudrate
  Serial.begin(9600 /* Default baudrate*/);
  // Wait time for bluetooth data
  delay(10);
}
void loop()
{
  if (Serial.available() > 0)
  {
    Global_BluetoothData = Serial.read();
  }
  // Robot motiion
  Robot_SetRobotMotions();
}
/*****************************************Auxlariy functions**********************************/
void Robot_InitializeDcMotors(void)
{
  for(int Local_PinsCounter = 2 /* Right dc motor forward  */;Local_PinsCounter <= 7 /* Left  dc motor backward */;Local_PinsCounter++)
  {
    pinMode(Local_PinsCounter, 0x1);
  }
}

void Robot_SetRobotMotions(void)
{
  if(Global_BluetoothData == 'F')
  {
    Robot_MoveRobotForward();
  }
  else if(Global_BluetoothData == 'B')
  {
    Robot_MoveRobotBackward();
  }
  else if(Global_BluetoothData == 'R')
  {
    Robot_MoveRobotRight();
  }
  else if(Global_BluetoothData == 'L')
  {
    Robot_MoveRobotLeft();
  }
  else if(Global_BluetoothData == 'S')
  {
    Robot_StopRobot();
  }
}

void Robot_MoveRobotForward(void)
{
  digitalWrite(2 /* Right dc motor forward  */, 0x1);
  digitalWrite(4 /* Right dc motor backword */,0x0);
  digitalWrite(6 /* Left  dc motor forward  */, 0x1);
  digitalWrite(7 /* Left  dc motor backward */, 0x0);
  analogWrite(3 /* Right dc motor speed*/, 200 /* Motor speed*/);
  analogWrite(5 /* Left  dc motor speed*/, 200 /* Motor speed*/);
  delay(20 /* Delay for motors to move*/);
}

void Robot_MoveRobotBackward(void)
{
  digitalWrite(2 /* Right dc motor forward  */, 0x0);
  digitalWrite(4 /* Right dc motor backword */, 0x1);
  digitalWrite(6 /* Left  dc motor forward  */, 0x0);
  digitalWrite(7 /* Left  dc motor backward */, 0x1);
  analogWrite(3 /* Right dc motor speed*/, 200 /* Motor speed*/);
  analogWrite(5 /* Left  dc motor speed*/, 200 /* Motor speed*/);
  delay(20 /* Delay for motors to move*/);
}

void Robot_MoveRobotRight(void)
{
  digitalWrite(2 /* Right dc motor forward  */, 0x0);
  digitalWrite(4 /* Right dc motor backword */, 0x0);
  digitalWrite(6 /* Left  dc motor forward  */, 0x1);
  digitalWrite(7 /* Left  dc motor backward */, 0x0);
  analogWrite(3 /* Right dc motor speed*/, 0 /* Motor speed for stop*/);
  analogWrite(5 /* Left  dc motor speed*/, 200 /* Motor speed*/);
  delay(20 /* Delay for motors to move*/);
}

void Robot_MoveRobotLeft(void)
{
  digitalWrite(2 /* Right dc motor forward  */, 0x1);
  digitalWrite(4 /* Right dc motor backword */, 0x0);
  digitalWrite(6 /* Left  dc motor forward  */, 0x0);
  digitalWrite(7 /* Left  dc motor backward */, 0x0);
  analogWrite(3 /* Right dc motor speed*/, 200 /* Motor speed*/);
  analogWrite(5 /* Left  dc motor speed*/, 0 /* Motor speed for stop*/);
  delay(20 /* Delay for motors to move*/);
}

void Robot_StopRobot(void)
{
  digitalWrite(2 /* Right dc motor forward  */, 0x0);
  digitalWrite(4 /* Right dc motor backword */, 0x0);
  digitalWrite(6 /* Left  dc motor forward  */, 0x0);
  digitalWrite(7 /* Left  dc motor backward */, 0x0);
  analogWrite(3 /* Right dc motor speed*/, 0 /* Motor speed for stop*/);
  analogWrite(5 /* Left  dc motor speed*/, 0 /* Motor speed for stop*/);
}
