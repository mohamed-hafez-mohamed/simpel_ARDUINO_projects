# 1 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
/*******************************************************************************

* Title                 :   Arm_Robot 

* Filename              :   Arm_Robot.ino

* Author                :   Mohamed Hafez

* Origin Date           :   11/01/2022

* Version               :   1.0.0

* Compiler              :   TODO: COMPILER GOES HERE

* Target                :   TODO: arduino mega 

* Notes                 :   None 

*

*****************************************************************************/
# 12 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
/*************** SOURCE REVISION LOG *****************************************

*

*    Date    Version   Author          Description 

*  10/01/21   1.0.0   Mohamed Hafez   Initial Release.

*

*******************************************************************************/
# 18 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
/** @file Arm_Robot.c

 *  @brief This is the source file to control the arm robot with five dgrees of freedom. 

 */
# 21 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
/******************************************************************************

* Includes

*******************************************************************************/
# 24 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
# 25 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino" 2
# 26 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino" 2
/******************************************************************************

* Module Preprocessor Constants

*******************************************************************************/
# 51 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
/******************************************************************************

* Module Preprocessor Macros

*******************************************************************************/
# 55 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
/******************************************************************************

* Module Typedefs

*******************************************************************************/
# 59 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
/******************************************************************************

* Module Variable and Objects Definitions

*******************************************************************************/
# 62 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
/***************************Objectives Definition******************************/
// Bluetooth
SoftwareSerial StRobot(A1 /*RX pin of bluetooth module*/, A0 /*TX pin of bluetooth module*/); // RX, TX
// Servo Motors
Servo WaistRotationServo; //servo motor for waist
Servo ShoulderRotationServo; //servo motor for shoulder
Servo ElbowRotationServo; //servo motor for elbow
Servo WristRollRotationServo; //servo motor for wrist roll
Servo WristPitchRotationServo; //servo motor for wrist pitch
Servo GripperRotationServo; //servo motor for gripper


/***************************Variables Definition******************************/
String Global_BluetoothData; //data from bluetooth

int Global_DcMotorsSpeed; //speed of dc motors  

int Global_CurrentWaistPosition; //current  position of waist servo motor
int Global_CurrentShoulderPosition; //current  position of shoulder servo motor
int Global_CurrentElbowPosition; //current  position of elbow servo motor
int Global_CurrentWristRollPosition; //current  position of wrist roll servo motor
int Global_CurrentWristPitchPosition; //current  position of wrist pitch servo motor
int Global_CurrentGripperPosition; //current  position of gripper servo motor
int Global_PreviousWaistPosition; //Previous position of waist servo motor
int Global_PreviousShoulderPosition; //Previous position of shoulder servo motor
int Global_PreviousElbowPosition; //Previous position of elbow servo motor
int Global_PreviousWristRollPosition; //Previous position of wrist roll servo motor
int Global_PreviousWristPitchPosition; //Previous position of wrist pitch servo motor
int Global_PreviousGripperPosition; //Previous position of gripper servo motor
/******************************************************************************

* Function Prototypes

*******************************************************************************/
# 94 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
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
# 108 "e:\\02- Embedded Systems\\09- ST Smart\\01- Arm Robot\\Arm_Robot.ino"
void setup()
{
  Global_BluetoothData = "\0";
  //initial value for dc motor   
  Global_DcMotorsSpeed = 190;
  //initial value for servo positions  
  Global_PreviousWaistPosition = 90; //angle of waist servo motor
  Global_PreviousShoulderPosition = 150; //angle of shoulder servo motor
  Global_PreviousElbowPosition = 35; //angle of elbow servo motor
  Global_PreviousWristRollPosition = 140; //angle of wrist roll servo motor
  Global_PreviousWristPitchPosition = 85; //angle of wrist pitch servo motor
  Global_PreviousGripperPosition = 80; //angle of gripper servo motor
  // attach servo motors to control pins and initialize it
  Robot_InitializeDcMotors();
  // attach dc motors to control pins
  Robot_InitializeServoMotors();
  //initialize bluetooth
  Serial.begin(9600 /*default baudrate*/);
  StRobot.begin(9600 /*default baudrate*/);
  StRobot.setTimeout(1 /*delay for bluetooth (data streaming)*/);
  delay(10); //wait time for bluetooth data
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
  WaistRotationServo.attach(44 /*waist Servo control pin*/);
  WaistRotationServo.write(Global_PreviousWaistPosition);
  ShoulderRotationServo.attach(45 /*shoulder Servo control pin*/);
  ShoulderRotationServo.write(Global_PreviousShoulderPosition);
  ElbowRotationServo.attach(46 /*elbow Servo control pin*/);
  ElbowRotationServo.write(Global_PreviousElbowPosition);
  WristRollRotationServo.attach(5 /*wrist roll Servo control pin*/);
  WristRollRotationServo.write(Global_PreviousWristRollPosition);
  WristPitchRotationServo.attach(13 /*wrist pitch Servo control pin*/);
  WristPitchRotationServo.write(Global_PreviousWristPitchPosition);
  GripperRotationServo.attach(10 /*gripper Servo control pin*/);
  GripperRotationServo.write(Global_PreviousGripperPosition);
}

void Robot_InitializeServoMotors(void)
{
  pinMode(3 /*right dc motor speed */, 0x1);
  pinMode(11 /*left dc motor speed*/, 0x1);
  pinMode(2 /*forward  dc right motor*/, 0x1);
  pinMode(4 /*backword dc right motor*/, 0x1);
  pinMode(7 /*forward dc left motor*/, 0x1);
  pinMode(8 /*backward dc left motor*/, 0x1);
}

void Robot_SetDcMotorsSpeed(void)
{
  //-------dc motors speed control------------
  if(Global_BluetoothData.startsWith("x")) Global_DcMotorsSpeed = 255;
  if(Global_BluetoothData.startsWith("d")) Global_DcMotorsSpeed = 150;
  if(Global_BluetoothData.startsWith("n")) Global_DcMotorsSpeed = 100;
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
  analogWrite(3 /*right dc motor speed */, Global_DcMotorsSpeed);
  analogWrite(11 /*left dc motor speed*/, Global_DcMotorsSpeed);
  digitalWrite(2 /*forward  dc right motor*/, 0x1);
  digitalWrite(4 /*backword dc right motor*/,0x0);
  digitalWrite(7 /*forward dc left motor*/, 0x1);
  digitalWrite(8 /*backward dc left motor*/, 0x0);
  delay(20 /*delay for motors t*/);
}

void Robot_MoveRobotBackward(void)
{
  analogWrite(3 /*right dc motor speed */, Global_DcMotorsSpeed);
  analogWrite(11 /*left dc motor speed*/, Global_DcMotorsSpeed);
  digitalWrite(2 /*forward  dc right motor*/, 0x0);
  digitalWrite(4 /*backword dc right motor*/, 0x1);
  digitalWrite(7 /*forward dc left motor*/, 0x0);
  digitalWrite(8 /*backward dc left motor*/, 0x1);
  delay(20 /*delay for motors t*/);
}

void Robot_MoveRobotRight(void)
{
  analogWrite(3 /*right dc motor speed */, Global_DcMotorsSpeed);
  analogWrite(11 /*left dc motor speed*/, Global_DcMotorsSpeed);
  digitalWrite(2 /*forward  dc right motor*/, 0x1);
  digitalWrite(4 /*backword dc right motor*/, 0x0);
  digitalWrite(7 /*forward dc left motor*/, 0x0);
  digitalWrite(8 /*backward dc left motor*/, 0x1);
  delay(20 /*delay for motors t*/);
}

void Robot_MoveRobotLeft(void)
{
  analogWrite(3 /*right dc motor speed */, Global_DcMotorsSpeed);
  analogWrite(11 /*left dc motor speed*/, Global_DcMotorsSpeed);
  digitalWrite(2 /*forward  dc right motor*/, 0x0);
  digitalWrite(4 /*backword dc right motor*/, 0x1);
  digitalWrite(7 /*forward dc left motor*/, 0x1);
  digitalWrite(8 /*backward dc left motor*/, 0x0);
  delay(20 /*delay for motors t*/);
}

void Robot_StopRobot(void)
{
  analogWrite(3 /*right dc motor speed */, 0x0);
  analogWrite(11 /*left dc motor speed*/, 0x0);
}

void Robot_SetArmPositions(void)
{
  if(Global_BluetoothData.startsWith("wa"))
  {
    //waist servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentWaistPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentWaistPosition > Global_PreviousWaistPosition)
    {
      for(int i = Global_PreviousWaistPosition; i <= Global_CurrentWaistPosition; i++)
      {
        //write servo motor position
        WaistRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousWaistPosition; i >= Global_CurrentWaistPosition; i--)
      {
        //write servo motor position
        WaistRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousWaistPosition = Global_CurrentWaistPosition;
  }

  if(Global_BluetoothData.startsWith("sh"))
  {
    //shoulder servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentShoulderPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentShoulderPosition > Global_PreviousShoulderPosition)
    {
      for(int i = Global_PreviousShoulderPosition; i <= Global_CurrentShoulderPosition; i++)
      {
        //write servo motor position
        ShoulderRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousShoulderPosition; i >= Global_CurrentShoulderPosition; i--)
      {
        //write servo motor position
        ShoulderRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousShoulderPosition = Global_CurrentShoulderPosition;
  }

  if(Global_BluetoothData.startsWith("el"))
  {
    //elbow servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentElbowPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentElbowPosition > Global_PreviousElbowPosition)
    {
      for(int i = Global_PreviousElbowPosition; i <= Global_CurrentElbowPosition; i++)
      {
        //write servo motor position
        ElbowRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousElbowPosition; i >= Global_CurrentElbowPosition; i--)
      {
        //write servo motor position
        ElbowRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousElbowPosition = Global_CurrentElbowPosition;
  }

  if(Global_BluetoothData.startsWith("wr"))
  {
    //wrist roll servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentWristRollPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentWristRollPosition > Global_PreviousWristRollPosition)
    {
      for(int i = Global_PreviousWristRollPosition; i <= Global_CurrentWristRollPosition; i++)
      {
        //write servo motor position
        WristRollRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousWristRollPosition; i >= Global_CurrentWristRollPosition; i--)
      {
        //write servo motor position
        WristRollRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousWristRollPosition = Global_CurrentWristRollPosition;
  }

  if(Global_BluetoothData.startsWith("wp"))
  {
    //wrist pitch servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentWristPitchPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentWristPitchPosition > Global_PreviousWristPitchPosition)
    {
      for(int i = Global_PreviousWristPitchPosition; i <= Global_CurrentWristPitchPosition; i++)
      {
        //write servo motor position
        WristPitchRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousWristPitchPosition; i >= Global_CurrentWristPitchPosition; i--)
      {
        //write servo motor position
        WristPitchRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousWristPitchPosition = Global_CurrentWristPitchPosition;
  }

  if(Global_BluetoothData.startsWith("gr"))
  {
    //gripper servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentGripperPosition = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentGripperPosition > Global_PreviousGripperPosition)
    {
      for(int i = Global_PreviousGripperPosition; i <= Global_CurrentGripperPosition; i++)
      {
        //write servo motor position
        GripperRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousGripperPosition; i >= Global_CurrentGripperPosition; i--)
      {
        //write servo motor position
        GripperRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousGripperPosition = Global_CurrentGripperPosition;
  }

}
