/*******************************************************************************
* Title                 :   Avoiding Obstacles Car 
* Filename              :   Avoiding_Obstacles.ino
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
/** @file Avoiding_Obstacles.ino
 *  @brief This is the source file make an avoiding robot. 
 */
/******************************************************************************
* Includes
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/


#define RIGHT_DC_MOTOR_SPEED_PIN       3    // Right dc motor speed    pin 
#define RIGHT_DC_MOTOR_FORWARD_PIN     2    // Right dc motor forward  pin 
#define RIGHT_DC_MOTOR_BACKWARD_PIN    4    // Right dc motor backword pin

#define LEFT_DC_MOTOR_SPEED_PIN        5    // Left dc motor speed    pin
#define LEFT_DC_MOTOR_BACKWARD_PIN     6    // Left dc motor forward  pin 
#define LEFT_DC_MOTOR_FORWARD_PIN      7    // Left dc motor backward pin

#define ULTRASONIC_TRIGGER_PIN         8    // Ultrasonic trigger pin
#define ULTRASONIC_ECHO_PIN            9    // Ultrasonic echo    pin

#define TIME_DELAY_FOR_MOTORS          200  // Delay for motors
#define TIME_DELAY_FOR_MOVE            500
#define MOTOR_SPEED                    150  // Motor speed
#define MOTOR_SPEED_FOR_STOP           0    // Motor speed for stop

#define INITIALIZE_WITH_ZERO           0
#define LOW_PULSE_DURATION             2
#define HIGH_PULSE_DURATION            10
#define CONSTANT_VALUE                 .0343
#define LIMIT_DISTANCE                 20
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
long int Global_Distance;
long int Global_WaveDuration;
/******************************************************************************
* Function Prototypes
*******************************************************************************/
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

void setup() 
{
  // Initialize distance variable
  Global_Distance     = INITIALIZE_WITH_ZERO;
  // Initialize duration variable
  Global_WaveDuration = INITIALIZE_WITH_ZERO;
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
  for(int Local_PinsCounter = RIGHT_DC_MOTOR_FORWARD_PIN;Local_PinsCounter <= ULTRASONIC_TRIGGER_PIN;Local_PinsCounter++)
  {
    pinMode(Local_PinsCounter, OUTPUT);
  }
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
}

void Robot_CalculateDistance(void)
{
  pinMode(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(LOW_PULSE_DURATION);
  pinMode(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(HIGH_PULSE_DURATION);
  pinMode(ULTRASONIC_TRIGGER_PIN, LOW);
  Global_WaveDuration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  Global_Distance     = (Global_WaveDuration / 2) * CONSTANT_VALUE;
}

void Robot_SetRobotMotions(void)
{
  if(Global_Distance < LIMIT_DISTANCE)
  {
    Robot_StopRobot();
    delay(TIME_DELAY_FOR_MOTORS);
    Robot_MoveRobotBackward();
    delay(TIME_DELAY_FOR_MOVE);
    Robot_MoveRobotRight();
    delay(TIME_DELAY_FOR_MOVE);
  }
  else
  {
    Robot_MoveRobotForward();
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
}

void Robot_MoveRobotBackward(void)
{
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, HIGH);
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, MOTOR_SPEED);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  MOTOR_SPEED);
}

void Robot_MoveRobotRight(void)
{
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, LOW);
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, MOTOR_SPEED_FOR_STOP);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  MOTOR_SPEED);
}

void Robot_MoveRobotLeft(void)
{
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, LOW);
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, MOTOR_SPEED);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  MOTOR_SPEED_FOR_STOP);
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