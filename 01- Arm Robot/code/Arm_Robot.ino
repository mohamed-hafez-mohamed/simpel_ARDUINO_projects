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
#define BLUETOOTH_TX_PIN               A0   //TX pin of bluetooth module
#define BLUETOOTH_RX_PIN               A1   //RX pin of bluetooth module
#define WAIST_SERVO_CONTROL_PIN        44   //waist Servo control pin
#define SHOULDER_SERVO_CONTROL_PIN     45   //shoulder Servo control pin
#define ELBOW_SERVO_CONTROL_PIN        46   //elbow Servo control pin
#define WRIST_ROLL_SERVO_CONTROL_PIN   5    //wrist roll Servo control pin
#define WRIST_PITCH_SERVO_CONTROL_PIN  13   //wrist pitch Servo control pin
#define GRIPPER_SERVO_CONTROL_PIN      10   //gripper Servo control pin
#define RIGHT_DC_MOTOR_SPEED_PIN       3    //right dc motor speed 
#define LEFT_DC_MOTOR_SPEED_PIN        11   //left dc motor speed
#define RIGHT_DC_MOTOR_FORWARD_PIN     2    //forward  dc right motor
#define RIGHT_DC_MOTOR_BACKWARD_PIN    4    //backword dc right motor
#define LEFT_DC_MOTOR_FORWARD_PIN      7    //forward dc left motor
#define LEFT_DC_MOTOR_BACKWARD_PIN     8    //backward dc left motor
#define TIME_DELAY_FOR_MOTORS          20   //delay for motors t
#define TIME_DELAY_BLUETOOTH           1    //delay for bluetooth (data streaming)
#define TIME_DELAY_FOR_DATA            10
#define DEFAULT_BAUDRATE               9600 //default baudrate
#define MAX_SPEED                      255
#define MIN_SPEED                      100
#define MED_SPEED                      150 
#define SERVO_MOTOR_CHARACTER          2
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
 
int    Global_CurrentWaistPosition;        //current  position of waist servo motor
int    Global_CurrentShoulderPosition;     //current  position of shoulder servo motor
int    Global_CurrentElbowPosition;        //current  position of elbow servo motor
int    Global_CurrentWristRollPosition;    //current  position of wrist roll servo motor
int    Global_CurrentWristPitchPosition;   //current  position of wrist pitch servo motor
int    Global_CurrentGripperPosition;      //current  position of gripper servo motor
int    Global_PreviousWaistPosition;       //Previous position of waist servo motor
int    Global_PreviousShoulderPosition;    //Previous position of shoulder servo motor
int    Global_PreviousElbowPosition;       //Previous position of elbow servo motor
int    Global_PreviousWristRollPosition;   //Previous position of wrist roll servo motor
int    Global_PreviousWristPitchPosition;  //Previous position of wrist pitch servo motor
int    Global_PreviousGripperPosition;     //Previous position of gripper servo motor
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

void setup() 
{
  Global_BluetoothData = "\0";
  //initial value for dc motor   
  Global_DcMotorsSpeed = 190;
  //initial value for servo positions  
  Global_PreviousWaistPosition      = 90;     //angle of waist servo motor
  Global_PreviousShoulderPosition   = 150;    //angle of shoulder servo motor
  Global_PreviousElbowPosition      = 35;     //angle of elbow servo motor
  Global_PreviousWristRollPosition  = 140;    //angle of wrist roll servo motor
  Global_PreviousWristPitchPosition = 85;     //angle of wrist pitch servo motor
  Global_PreviousGripperPosition    = 80;     //angle of gripper servo motor
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
  WaistRotationServo.write(Global_PreviousWaistPosition);
  ShoulderRotationServo.attach(SHOULDER_SERVO_CONTROL_PIN);
  ShoulderRotationServo.write(Global_PreviousShoulderPosition);
  ElbowRotationServo.attach(ELBOW_SERVO_CONTROL_PIN);
  ElbowRotationServo.write(Global_PreviousElbowPosition);
  WristRollRotationServo.attach(WRIST_ROLL_SERVO_CONTROL_PIN);
  WristRollRotationServo.write(Global_PreviousWristRollPosition);
  WristPitchRotationServo.attach(WRIST_PITCH_SERVO_CONTROL_PIN);
  WristPitchRotationServo.write(Global_PreviousWristPitchPosition);
  GripperRotationServo.attach(GRIPPER_SERVO_CONTROL_PIN);
  GripperRotationServo.write(Global_PreviousGripperPosition);
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
    Global_CurrentWaistPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentWaistPosition > Global_PreviousWaistPosition)
    {
      for(int i = Global_PreviousWaistPosition; i <= Global_CurrentWaistPosition; i++)
      {
        //write servo motor position
        WaistRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousWaistPosition; i >= Global_CurrentWaistPosition; i--)
      {
        //write servo motor position
        WaistRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousWaistPosition = Global_CurrentWaistPosition;
  }

  if(Global_BluetoothData.startsWith("sh"))
  {  
    //shoulder servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentShoulderPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentShoulderPosition > Global_PreviousShoulderPosition)
    {
      for(int i = Global_PreviousShoulderPosition; i <= Global_CurrentShoulderPosition; i++)
      {
        //write servo motor position
        ShoulderRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousShoulderPosition; i >= Global_CurrentShoulderPosition; i--)
      {
        //write servo motor position
        ShoulderRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousShoulderPosition = Global_CurrentShoulderPosition;
  }

  if(Global_BluetoothData.startsWith("el"))
  {  
    //elbow servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentElbowPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentElbowPosition > Global_PreviousElbowPosition)
    {
      for(int i = Global_PreviousElbowPosition; i <= Global_CurrentElbowPosition; i++)
      {
        //write servo motor position
        ElbowRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousElbowPosition; i >= Global_CurrentElbowPosition; i--)
      {
        //write servo motor position
        ElbowRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousElbowPosition = Global_CurrentElbowPosition;
  }

  if(Global_BluetoothData.startsWith("wr"))
  {  
    //wrist roll servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentWristRollPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentWristRollPosition > Global_PreviousWristRollPosition)
    {
      for(int i = Global_PreviousWristRollPosition; i <= Global_CurrentWristRollPosition; i++)
      {
        //write servo motor position
        WristRollRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousWristRollPosition; i >= Global_CurrentWristRollPosition; i--)
      {
        //write servo motor position
        WristRollRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousWristRollPosition = Global_CurrentWristRollPosition;
  }

  if(Global_BluetoothData.startsWith("wp"))
  {  
    //wrist pitch servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentWristPitchPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentWristPitchPosition > Global_PreviousWristPitchPosition)
    {
      for(int i = Global_PreviousWristPitchPosition; i <= Global_CurrentWristPitchPosition; i++)
      {
        //write servo motor position
        WristPitchRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousWristPitchPosition; i >= Global_CurrentWristPitchPosition; i--)
      {
        //write servo motor position
        WristPitchRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousWristPitchPosition = Global_CurrentWristPitchPosition;
  }

  if(Global_BluetoothData.startsWith("gr"))
  {  
    //gripper servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(SERVO_MOTOR_CHARACTER, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentGripperPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentGripperPosition > Global_PreviousGripperPosition)
    {
      for(int i = Global_PreviousGripperPosition; i <= Global_CurrentGripperPosition; i++)
      {
        //write servo motor position
        GripperRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    else
    {
      for(int i = Global_PreviousGripperPosition; i >= Global_CurrentGripperPosition; i--)
      {
        //write servo motor position
        GripperRotationServo.write(i);
        delay(TIME_DELAY_FOR_MOTORS);
      }
    }
    Global_PreviousGripperPosition = Global_CurrentGripperPosition;
  }
  
}