#include <Arduino.h>
#line 1 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
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
/*************** SOURCE REVISION LOG *****************************************
*
*    Date    Version   Author          Description 
*  11/01/22   1.0.0   Mohamed Hafez   Initial Release.
*
*******************************************************************************/
/** @file RC_Car_Mobile.ino
 *  @brief This is the source file to control the arm robot with five dgrees of freedom. 
 */
/******************************************************************************
* Includes
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define BLUETOOTH_TX_PIN               0    // TX pin of bluetooth module conected to RX pin of arduino
#define BLUETOOTH_RX_PIN               1    // RX pin of bluetooth module conected to TX pin of arduino
 

#define RIGHT_DC_MOTOR_SPEED_PIN       3    // Right dc motor speed
#define RIGHT_DC_MOTOR_FORWARD_PIN     2    // Right dc motor forward  
#define RIGHT_DC_MOTOR_BACKWARD_PIN    4    // Right dc motor backword 
#define LEFT_DC_MOTOR_SPEED_PIN        5    // Left  dc motor speed
#define LEFT_DC_MOTOR_FORWARD_PIN      6    // Left  dc motor forward  
#define LEFT_DC_MOTOR_BACKWARD_PIN     7    // Left  dc motor backward 

#define TIME_DELAY_FOR_MOTORS          20   // Delay for motors to move
#define MOTOR_SPEED                    200  // Motor speed
#define MOTOR_SPEED_FOR_STOP           0    // Motor speed for stop
#define TIME_DELAY_FOR_DATA            10
#define DEFAULT_BAUDRATE               9600 // Default baudrate
#define DEFAULT_SPEED                  255
#define NO_DATA                        '\0'
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable and Objects Definitions
*******************************************************************************/

/***************************Variables Definition******************************/
char Global_BluetoothData;               //data from bluetooth  
 
/******************************************************************************
* Function Prototypes
*******************************************************************************/
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

#line 76 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
void setup();
#line 87 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
void loop();
#line 76 "e:\\01- My Work\\01- ST Smart work\\01- Mid 2022 Season\\04- Projects\\02- RC Car Using Phone\\RC Car code\\RcCar_Robot.ino"
void setup() 
{
  // Initialize bluetooth data variable
  Global_BluetoothData = NO_DATA;
  // Attach dc motors to control pins
  Robot_InitializeDcMotors();
  // Initialize baudrate
  Serial.begin(DEFAULT_BAUDRATE);
  // Wait time for bluetooth data
  delay(TIME_DELAY_FOR_DATA); 
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
  for(int Local_PinsCounter = RIGHT_DC_MOTOR_FORWARD_PIN;Local_PinsCounter <= LEFT_DC_MOTOR_BACKWARD_PIN;Local_PinsCounter++)
  {
    pinMode(Local_PinsCounter, OUTPUT);
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
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN,LOW);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, LOW);
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, MOTOR_SPEED);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  MOTOR_SPEED);
  delay(TIME_DELAY_FOR_MOTORS);
}

void Robot_MoveRobotBackward(void)
{
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, HIGH);
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, MOTOR_SPEED);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  MOTOR_SPEED);
  delay(TIME_DELAY_FOR_MOTORS);
}

void Robot_MoveRobotRight(void)
{
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, LOW);
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, MOTOR_SPEED_FOR_STOP);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  MOTOR_SPEED);
  delay(TIME_DELAY_FOR_MOTORS);
}

void Robot_MoveRobotLeft(void)
{
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, LOW);
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, MOTOR_SPEED);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  MOTOR_SPEED_FOR_STOP);
  delay(TIME_DELAY_FOR_MOTORS);
}

void Robot_StopRobot(void)
{
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, LOW);
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, MOTOR_SPEED_FOR_STOP);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN, MOTOR_SPEED_FOR_STOP);
}
