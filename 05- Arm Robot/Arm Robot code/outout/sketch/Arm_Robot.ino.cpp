#include <Arduino.h>
#line 1 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
/*******************************************************************************
* Title                 :   Arm_Robot 
* Filename              :   Arm_Robot.ino
* Author                :   Mohamed Hafez
* Origin Date           :   11/01/2022
* Version               :   1.0.0
* Compiler              :   TODO: COMPILER GOES HERE
* Target                :    arduino mega 
* Notes                 :   None 
*
*****************************************************************************/
/*************** SOURCE REVISION LOG *****************************************
*
*    Date    Version   Author          Description 
*  11/01/22   1.0.0   Mohamed Hafez   Initial Release.
*
*******************************************************************************/
/** @file Arm_Robot.c
 *  @brief This is the source file to control the arm robot with five dgrees of freedom. 
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <Servo.h>
#include <SoftwareSerial.h>
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define BLUETOOTH_TX_PIN                 A0   //TX pin of bluetooth module
#define BLUETOOTH_RX_PIN                 A1   //RX pin of bluetooth module
#define WAIST_SERVO_CONTROL_PIN          44   //waist Servo control pin
#define SHOULDER_SERVO_CONTROL_PIN       45   //shoulder Servo control pin
#define ELBOW_SERVO_CONTROL_PIN          46   //elbow Servo control pin
#define WRIST_ROLL_SERVO_CONTROL_PIN     5    //wrist roll Servo control pin
#define WRIST_PITCH_SERVO_CONTROL_PIN    13   //wrist pitch Servo control pin
#define GRIPPER_SERVO_CONTROL_PIN        10   //gripper Servo control pin
#define RIGHT_DC_MOTOR_SPEED_PIN         3    //right dc motor speed 
#define LEFT_DC_MOTOR_SPEED_PIN          11   //left dc motor speed
#define RIGHT_DC_MOTOR_FORWARD_PIN       2    //forward  dc right motor
#define RIGHT_DC_MOTOR_BACKWARD_PIN      4    //backword dc right motor
#define LEFT_DC_MOTOR_FORWARD_PIN        7    //forward dc left motor
#define LEFT_DC_MOTOR_BACKWARD_PIN       8    //backward dc left motor
#define TIME_DELAY_FOR_MOTORS            20   //delay for motors t
#define TIME_DELAY_BLUETOOTH             1    //delay for bluetooth (data streaming)
#define TIME_DELAY_FOR_DATA              10
#define DEFAULT_BAUDRATE                 9600 //default baudrate
#define INITIAL_SPEED                    190
#define MAX_SPEED                        255
#define MIN_SPEED                        100
#define MED_SPEED                        150 
#define INITIAL_SERVO_WAIST_ANGLE        90   //waist servo angle
#define INITIAL_SERVO_SHOULDER_ANGLE     150  //shoulder servo angle
#define INITIAL_SERVO_ELBOW_ANGLE        35   //elbow servo angle
#define INITIAL_SERVO_WRIST_ROLL_ANGLE   140  //wrist roll servo angle
#define INITIAL_SERVO_WRIST_PITCH_ANGLE  85   //wrist pitch servo angle
#define INITIAL_SERVO_GRIPPER_ANGLE      80   //gripper servo angle
#define SERVO_MOTOR_CHARACTER            2
#define SERVO_MOTOR_NUMBER               5
#define SERVO_WAIST_INDEX                0
#define SERVO_SHOULDER_INDEX             1
#define SERVO_ELBOW_INDEX                2
#define SERVO_WRIST_ROLL_INDEX           3
#define SERVO_WRIST_PITCH_INDEX          4
#define SERVO_GRIPPER_INDEX              5
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable and Objects Definitions
*******************************************************************************/
/***************************Objectives Definition******************************/
// Bluetooth
SoftwareSerial StRobot(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN); // Arduino(RX, Tx) Bluetooth(Tx, Rx)
// Servo Motors
Servo WaistRotationServo;             //servo motor for waist
Servo ShoulderRotationServo;          //servo motor for shoulder
Servo ElbowRotationServo;             //servo motor for elbow
Servo WristRollRotationServo;         //servo motor for wrist roll
Servo WristPitchRotationServo;        //servo motor for wrist pitch
Servo GripperRotationServo;           //servo motor for gripper


/***************************Variables Definition******************************/
String Global_BluetoothData;               //data from bluetooth

int    Global_DcMotorsSpeed;               //speed of dc motors  

int    Global_PreviousPosition[SERVO_MOTOR_NUMBER]; //previous position of servo motor
int    Global_CurrentPosition[SERVO_MOTOR_NUMBER];  //current  position of servo motor
/******************************************************************************
* Function Prototypes
*******************************************************************************/
void Robot_InitializeDcMotors(void);
void Robot_InitializeServoMotors(void);
void Robot_SetDcMotorsSpeed(void);
void Robot_SetRobotMotions(void);
void Robot_MoveRobotForward(void);
void Robot_MoveRobotBackward(void);
void Robot_MoveRobotRight(void);
void Robot_MoveRobotLeft(void);
void Robot_StopRobot(void);
void Robot_SetArmPositions(void);
/******************************************************************************
* Function Definitions
*******************************************************************************/

#line 112 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
void setup();
#line 134 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
void loop();
#line 112 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
void setup() 
{
  Global_BluetoothData = "\0";
  //initial value for dc motor   
  Global_DcMotorsSpeed = INITIAL_SPEED;
  //initial value for servo positions
  Global_PreviousPosition[SERVO_WAIST_INDEX]       = INITIAL_SERVO_WAIST_ANGLE;
  Global_PreviousPosition[SERVO_SHOULDER_INDEX]    = INITIAL_SERVO_SHOULDER_ANGLE;
  Global_PreviousPosition[SERVO_ELBOW_INDEX]       = INITIAL_SERVO_ELBOW_ANGLE;
  Global_PreviousPosition[SERVO_WRIST_ROLL_INDEX]  = INITIAL_SERVO_WRIST_ROLL_ANGLE;
  Global_PreviousPosition[SERVO_WRIST_PITCH_INDEX] = INITIAL_SERVO_WRIST_PITCH_ANGLE;
  Global_PreviousPosition[SERVO_GRIPPER_INDEX]     = INITIAL_SERVO_GRIPPER_ANGLE;
  // attach dc motors to control pins
  Robot_InitializeDcMotors();
  // attach servo motors to control pins and initialize it
  Robot_InitializeServoMotors();
  //initialize bluetooth
  Serial.begin(DEFAULT_BAUDRATE);
  StRobot.begin(DEFAULT_BAUDRATE);
  StRobot.setTimeout(TIME_DELAY_BLUETOOTH); 
  delay(TIME_DELAY_FOR_DATA); //wait time for bluetooth data
}
void loop()
{ 

 if (StRobot.available() > 0)
 {
    Global_BluetoothData = StRobot.readString();
    // robot speed
    Robot_SetDcMotorsSpeed(); 
    //robot motiion
    Robot_SetRobotMotions();
    //control arm
    Robot_SetArmPositions();
  }
}
/*****************************************Auxlariy functions**********************************/
void Robot_InitializeDcMotors(void)
{
  pinMode(RIGHT_DC_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(LEFT_DC_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_DC_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_DC_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(LEFT_DC_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_DC_MOTOR_BACKWARD_PIN, OUTPUT);
}

void Robot_InitializeServoMotors(void)
{
  WaistRotationServo.attach(WAIST_SERVO_CONTROL_PIN);
  WaistRotationServo.write(Global_PreviousPosition[SERVO_WAIST_INDEX]);
  ShoulderRotationServo.attach(SHOULDER_SERVO_CONTROL_PIN);
  ShoulderRotationServo.write(Global_PreviousPosition[SERVO_SHOULDER_INDEX]);
  ElbowRotationServo.attach(ELBOW_SERVO_CONTROL_PIN);
  ElbowRotationServo.write(Global_PreviousPosition[SERVO_ELBOW_INDEX]);
  WristRollRotationServo.attach(WRIST_ROLL_SERVO_CONTROL_PIN);
  WristRollRotationServo.write(Global_PreviousPosition[SERVO_WRIST_ROLL_INDEX]);
  WristPitchRotationServo.attach(WRIST_PITCH_SERVO_CONTROL_PIN);
  WristPitchRotationServo.write(Global_PreviousPosition[SERVO_WRIST_PITCH_INDEX]);
  GripperRotationServo.attach(GRIPPER_SERVO_CONTROL_PIN);
  GripperRotationServo.write(Global_PreviousPosition[SERVO_GRIPPER_INDEX]);
}

void Robot_SetDcMotorsSpeed(void)
{
  //-------dc motors speed control------------
  if(Global_BluetoothData.startsWith("x")) Global_DcMotorsSpeed = MAX_SPEED;  
  if(Global_BluetoothData.startsWith("d")) Global_DcMotorsSpeed = MED_SPEED; 
  if(Global_BluetoothData.startsWith("n")) Global_DcMotorsSpeed = MIN_SPEED;
}

void Robot_SetRobotMotions(void)
{
  if(Global_BluetoothData.startsWith("f"))
  { 
    //forward motion
    Robot_MoveRobotForward();
  }
  if(Global_BluetoothData.startsWith("r"))
  {
    //right motion
    Robot_MoveRobotRight();
  }
  if(Global_BluetoothData.startsWith("l"))
  { 
    //left motion
    Robot_MoveRobotLeft();
  } 
  if(Global_BluetoothData.startsWith("b"))
  { 
    //backward motion
    Robot_MoveRobotBackward();
  }
  if(Global_BluetoothData.startsWith("s"))
  { 
    //stop
    Robot_StopRobot();
  }
}

void Robot_MoveRobotForward(void)
{
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, Global_DcMotorsSpeed);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  Global_DcMotorsSpeed);
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN,LOW);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, LOW);
  delay(TIME_DELAY_FOR_MOTORS);
}

void Robot_MoveRobotBackward(void)
{
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, Global_DcMotorsSpeed);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  Global_DcMotorsSpeed);
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, HIGH);
  delay(TIME_DELAY_FOR_MOTORS);
}

void Robot_MoveRobotRight(void)
{
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, Global_DcMotorsSpeed);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  Global_DcMotorsSpeed);
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, HIGH);
  delay(TIME_DELAY_FOR_MOTORS);
}

void Robot_MoveRobotLeft(void)
{
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, Global_DcMotorsSpeed);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN,  Global_DcMotorsSpeed);
  digitalWrite(RIGHT_DC_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_DC_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_DC_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_DC_MOTOR_BACKWARD_PIN, LOW);
  delay(TIME_DELAY_FOR_MOTORS);
}

void Robot_StopRobot(void)
{
  analogWrite(RIGHT_DC_MOTOR_SPEED_PIN, LOW);
  analogWrite(LEFT_DC_MOTOR_SPEED_PIN, LOW);
}

void Robot_SetArmPositions(void)
{
  if(Global_BluetoothData.startsWith("wa"))
  {  
    //waist servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[SERVO_WAIST_INDEX] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[SERVO_WAIST_INDEX] > Global_PreviousPosition[SERVO_WAIST_INDEX])
    {
      for(int i = Global_PreviousPosition[SERVO_WAIST_INDEX]; i <= Global_CurrentPosition[SERVO_WAIST_INDEX]; i++)
      {
        //write servo motor position
        WaistRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[SERVO_WAIST_INDEX]; i >= Global_CurrentPosition[SERVO_WAIST_INDEX]; i--)
      {
        //write servo motor position
        WaistRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousPosition[SERVO_WAIST_INDEX] = Global_CurrentPosition[SERVO_WAIST_INDEX];
  }

  if(Global_BluetoothData.startsWith("sh"))
  {  
    //shoulder servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[SERVO_SHOULDER_INDEX] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[SERVO_SHOULDER_INDEX] > Global_PreviousPosition[SERVO_SHOULDER_INDEX])
    {
      for(int i = Global_PreviousPosition[SERVO_SHOULDER_INDEX]; i <= Global_CurrentPosition[SERVO_SHOULDER_INDEX]; i++)
      {
        //write servo motor position
        ShoulderRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[SERVO_SHOULDER_INDEX]; i >= Global_CurrentPosition[SERVO_SHOULDER_INDEX]; i--)
      {
        //write servo motor position
        ShoulderRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousPosition[SERVO_SHOULDER_INDEX] = Global_CurrentPosition[SERVO_SHOULDER_INDEX];
  }

  if(Global_BluetoothData.startsWith("el"))
  {  
    //elbow servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[SERVO_ELBOW_INDEX] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[SERVO_ELBOW_INDEX] > Global_PreviousPosition[SERVO_ELBOW_INDEX])
    {
      for(int i = Global_PreviousPosition[SERVO_ELBOW_INDEX]; i <= Global_CurrentPosition[SERVO_ELBOW_INDEX]; i++)
      {
        //write servo motor position
        ElbowRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[SERVO_ELBOW_INDEX]; i >= Global_CurrentPosition[SERVO_ELBOW_INDEX]; i--)
      {
        //write servo motor position
        ElbowRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousPosition[SERVO_ELBOW_INDEX] = Global_CurrentPosition[SERVO_ELBOW_INDEX];
  }

  if(Global_BluetoothData.startsWith("wr"))
  {  
    //wrist roll servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[SERVO_WRIST_ROLL_INDEX] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[SERVO_WRIST_ROLL_INDEX] > Global_PreviousPosition[SERVO_WRIST_ROLL_INDEX])
    {
      for(int i = Global_PreviousPosition[SERVO_WRIST_ROLL_INDEX]; i <= Global_CurrentPosition[SERVO_WRIST_ROLL_INDEX]; i++)
      {
        //write servo motor position
        WristRollRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[SERVO_WRIST_ROLL_INDEX]; i >= Global_CurrentPosition[SERVO_WRIST_ROLL_INDEX]; i--)
      {
        //write servo motor position
        WristRollRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousPosition[SERVO_WRIST_ROLL_INDEX] = Global_CurrentPosition[SERVO_WRIST_ROLL_INDEX];
  }

  if(Global_BluetoothData.startsWith("wp"))
  {  
    //wrist pitch servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[SERVO_WRIST_PITCH_INDEX] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[SERVO_WRIST_PITCH_INDEX] > Global_PreviousPosition[SERVO_WRIST_PITCH_INDEX])
    {
      for(int i =  Global_PreviousPosition[SERVO_WRIST_PITCH_INDEX]; i <= Global_CurrentPosition[SERVO_WRIST_PITCH_INDEX]; i++)
      {
        //write servo motor position
        WristPitchRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i =  Global_PreviousPosition[SERVO_WRIST_PITCH_INDEX]; i >= Global_CurrentPosition[SERVO_WRIST_PITCH_INDEX]; i--)
      {
        //write servo motor position
        WristPitchRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
     Global_PreviousPosition[SERVO_WRIST_PITCH_INDEX] = Global_CurrentPosition[SERVO_WRIST_PITCH_INDEX];
  }

  if(Global_BluetoothData.startsWith("gr"))
  {  
    //gripper servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[SERVO_GRIPPER_INDEX] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[SERVO_GRIPPER_INDEX] > Global_PreviousPosition[SERVO_GRIPPER_INDEX])
    {
      for(int i = Global_PreviousPosition[SERVO_GRIPPER_INDEX]; i <= Global_CurrentPosition[SERVO_GRIPPER_INDEX]; i++)
      {
        //write servo motor position
        GripperRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[SERVO_GRIPPER_INDEX]; i >= Global_CurrentPosition[SERVO_GRIPPER_INDEX]; i--)
      {
        //write servo motor position
        GripperRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousPosition[SERVO_GRIPPER_INDEX] = Global_CurrentPosition[SERVO_GRIPPER_INDEX];
  }
  
}
