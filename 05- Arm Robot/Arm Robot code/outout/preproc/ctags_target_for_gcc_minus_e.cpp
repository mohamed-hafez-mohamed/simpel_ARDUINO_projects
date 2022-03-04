# 1 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
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
# 12 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
/*************** SOURCE REVISION LOG *****************************************

*

*    Date    Version   Author          Description 

*  11/01/22   1.0.0   Mohamed Hafez   Initial Release.

*

*******************************************************************************/
# 18 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
/** @file Arm_Robot.c

 *  @brief This is the source file to control the arm robot with five dgrees of freedom. 

 */
# 21 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
/******************************************************************************

* Includes

*******************************************************************************/
# 24 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
# 25 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino" 2
# 26 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino" 2
/******************************************************************************

* Module Preprocessor Constants

*******************************************************************************/
# 65 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
/******************************************************************************

* Module Preprocessor Macros

*******************************************************************************/
# 69 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
/******************************************************************************

* Module Typedefs

*******************************************************************************/
# 73 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
/******************************************************************************

* Module Variable and Objects Definitions

*******************************************************************************/
# 76 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
/***************************Objectives Definition******************************/
// Bluetooth
SoftwareSerial StRobot(A1 /*RX pin of bluetooth module*/, A0 /*TX pin of bluetooth module*/); // Arduino(RX, Tx) Bluetooth(Tx, Rx)
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

int Global_PreviousPosition[5]; //previous position of servo motor
int Global_CurrentPosition[5]; //current  position of servo motor
/******************************************************************************

* Function Prototypes

*******************************************************************************/
# 98 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
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
# 112 "e:\\02- Embedded Systems\\00- ST Smart\\04- Arm Robot\\code\\Arm_Robot.ino"
void setup()
{
  Global_BluetoothData = "\0";
  //initial value for dc motor   
  Global_DcMotorsSpeed = 190;
  //initial value for servo positions
  Global_PreviousPosition[0] = 90 /*waist servo angle*/;
  Global_PreviousPosition[1] = 150 /*shoulder servo angle*/;
  Global_PreviousPosition[2] = 35 /*elbow servo angle*/;
  Global_PreviousPosition[3] = 140 /*wrist roll servo angle*/;
  Global_PreviousPosition[4] = 85 /*wrist pitch servo angle*/;
  Global_PreviousPosition[5] = 80 /*gripper servo angle*/;
  // attach dc motors to control pins
  Robot_InitializeDcMotors();
  // attach servo motors to control pins and initialize it
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
  pinMode(3 /*right dc motor speed */, 0x1);
  pinMode(11 /*left dc motor speed*/, 0x1);
  pinMode(2 /*forward  dc right motor*/, 0x1);
  pinMode(4 /*backword dc right motor*/, 0x1);
  pinMode(7 /*forward dc left motor*/, 0x1);
  pinMode(8 /*backward dc left motor*/, 0x1);
}

void Robot_InitializeServoMotors(void)
{
  WaistRotationServo.attach(44 /*waist Servo control pin*/);
  WaistRotationServo.write(Global_PreviousPosition[0]);
  ShoulderRotationServo.attach(45 /*shoulder Servo control pin*/);
  ShoulderRotationServo.write(Global_PreviousPosition[1]);
  ElbowRotationServo.attach(46 /*elbow Servo control pin*/);
  ElbowRotationServo.write(Global_PreviousPosition[2]);
  WristRollRotationServo.attach(5 /*wrist roll Servo control pin*/);
  WristRollRotationServo.write(Global_PreviousPosition[3]);
  WristPitchRotationServo.attach(13 /*wrist pitch Servo control pin*/);
  WristPitchRotationServo.write(Global_PreviousPosition[4]);
  GripperRotationServo.attach(10 /*gripper Servo control pin*/);
  GripperRotationServo.write(Global_PreviousPosition[5]);
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
    Global_CurrentPosition[0] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[0] > Global_PreviousPosition[0])
    {
      for(int i = Global_PreviousPosition[0]; i <= Global_CurrentPosition[0]; i++)
      {
        //write servo motor position
        WaistRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[0]; i >= Global_CurrentPosition[0]; i--)
      {
        //write servo motor position
        WaistRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousPosition[0] = Global_CurrentPosition[0];
  }

  if(Global_BluetoothData.startsWith("sh"))
  {
    //shoulder servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[1] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[1] > Global_PreviousPosition[1])
    {
      for(int i = Global_PreviousPosition[1]; i <= Global_CurrentPosition[1]; i++)
      {
        //write servo motor position
        ShoulderRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[1]; i >= Global_CurrentPosition[1]; i--)
      {
        //write servo motor position
        ShoulderRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousPosition[1] = Global_CurrentPosition[1];
  }

  if(Global_BluetoothData.startsWith("el"))
  {
    //elbow servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[2] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[2] > Global_PreviousPosition[2])
    {
      for(int i = Global_PreviousPosition[2]; i <= Global_CurrentPosition[2]; i++)
      {
        //write servo motor position
        ElbowRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[2]; i >= Global_CurrentPosition[2]; i--)
      {
        //write servo motor position
        ElbowRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousPosition[2] = Global_CurrentPosition[2];
  }

  if(Global_BluetoothData.startsWith("wr"))
  {
    //wrist roll servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[3] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[3] > Global_PreviousPosition[3])
    {
      for(int i = Global_PreviousPosition[3]; i <= Global_CurrentPosition[3]; i++)
      {
        //write servo motor position
        WristRollRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[3]; i >= Global_CurrentPosition[3]; i--)
      {
        //write servo motor position
        WristRollRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousPosition[3] = Global_CurrentPosition[3];
  }

  if(Global_BluetoothData.startsWith("wp"))
  {
    //wrist pitch servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[4] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[4] > Global_PreviousPosition[4])
    {
      for(int i = Global_PreviousPosition[4]; i <= Global_CurrentPosition[4]; i++)
      {
        //write servo motor position
        WristPitchRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[4]; i >= Global_CurrentPosition[4]; i--)
      {
        //write servo motor position
        WristPitchRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
     Global_PreviousPosition[4] = Global_CurrentPosition[4];
  }

  if(Global_BluetoothData.startsWith("gr"))
  {
    //gripper servo
    //extract servo position from bluetooth data as servo name conatenated with position in the 
    //incoming bluetooth data
    String Local_ServoData = Global_BluetoothData.substring(2, Global_BluetoothData.length());
    //convert extracted character to integer
    Global_CurrentPosition[5] = Local_ServoData.toInt();
    //use for loop to control servo motor movement speed
    if(Global_CurrentPosition[5] > Global_PreviousPosition[5])
    {
      for(int i = Global_PreviousPosition[5]; i <= Global_CurrentPosition[5]; i++)
      {
        //write servo motor position
        GripperRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    else
    {
      for(int i = Global_PreviousPosition[5]; i >= Global_CurrentPosition[5]; i--)
      {
        //write servo motor position
        GripperRotationServo.write(i);
        delay(20 /*delay for motors t*/);
      }
    }
    Global_PreviousPosition[5] = Global_CurrentPosition[5];
  }

}
