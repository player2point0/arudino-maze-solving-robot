//Motor pins
#define R_MOTOR_ENABLE 5
#define L_MOTOR_ENABLE 6
#define R_MOTOR_1 7
#define R_MOTOR_2 8
#define L_MOTOR_1 9
#define L_MOTOR_2 11
//motor speed range
#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 130
//ultrasonic
#include <Ultrasonic.h>
Ultrasonic leftSensor(12, 10, 10000UL);
Ultrasonic frontSensor(A5, A4, 10000UL);
//infrared
#define IR_SENSOR A0

int slowSpeed = 135;
int normalSpeed = 190;
int leftMotorMulti = 1;
int rightMotorMulti = 1;
int updateDelay = 15;

//add a robot length var

void setup() 
{
  Serial.begin(9600);
  //pin setup
  pinMode(L_MOTOR_1, OUTPUT);
  pinMode(L_MOTOR_2, OUTPUT);
  pinMode(R_MOTOR_1, OUTPUT);
  pinMode(R_MOTOR_2, OUTPUT);
  pinMode(L_MOTOR_ENABLE, OUTPUT);
  pinMode(R_MOTOR_ENABLE, OUTPUT);
  pinMode(IR_SENSOR, INPUT);
}

void loop() 
{
  mazeSolve();
}

void mazeSolve()
{
  int wallLeft = digitalRead(IR_SENSOR);
  int front = frontSensor.read();
  int left = leftSensor.read();
  
  //check for no wall left
  if(wallLeft == 1)
  {
    //pivot left
    sprintFoward(5);
    delay(updateDelay);
    pivotRobot(-70);
    delay(updateDelay);
    sprintFoward(20);
  }

  else if (front > 20)
  {
    wallFollowFoward(2);
  }

  else
  {
    //rotate right 
    rotateRobot(90);
    
    //check if dead end using front sensor
    delay(updateDelay);
    front = frontSensor.read();
    
    if (front > 20)
    {
      sprintFoward(20);
    }
  }

  delay(updateDelay);
}

void pivotRobot(float angle)
{
  float fullCircle = 1500;
  float percentage = angle / 360;

  if(angle < 0)
  {
    //left
    driveRightMotor(slowSpeed);
  }

  else
  { 
    //right
    driveLeftMotor(slowSpeed); 
  }
  
  delay(abs(fullCircle * percentage));
  
  stopMotors();  
}

void followWall()
{
  int distanceLeft = leftSensor.read();
  
  if(distanceLeft > 25)
  {
    //go left
    driveLeftMotor(slowSpeed);
    driveRightMotor(normalSpeed);
    delay(10);
    stopMotors();
  }
  
  else if(distanceLeft < 15)
  {
    //go right
    driveLeftMotor(normalSpeed);
    driveRightMotor(slowSpeed);
    delay(10);
    stopMotors();
  }

  else 
  {
    driveRightMotor(normalSpeed);
    driveLeftMotor(normalSpeed);
    delay(10);
    stopMotors();
  }
  
  delay(updateDelay);
}

void sprintFoward(int callCount)
{
  
  for(int i = 0;i<callCount;i++)
  {
      int front = frontSensor.read();

      if (front > 20)
      {
        driveRightMotor(normalSpeed);
        driveLeftMotor(normalSpeed);
        delay(10);       
      }

      else break;
      
      stopMotors();    
  }
}

void wallFollowFoward(int callCount)
{
  //could change so the robot moves one bodyLenght
  //could change callCount to bodyLength
  
  for(int i = 0;i<callCount;i++)
  {
    followWall();  
  }
}

void rotateRobot(float angle)
{
  float fullCircle = 800;
  float percentage = angle / 360;
  
  if(angle < 0)
  {
    driveRightMotor(slowSpeed);
    driveLeftMotor(-slowSpeed);
  }

  else
  { 
    driveRightMotor(-slowSpeed);
    driveLeftMotor(slowSpeed); 
  }
  
  delay(abs(fullCircle * percentage));
  stopMotors();  
}

void stopMotors()
{
  driveRightMotor(0);
  driveLeftMotor(0);
}

void driveRightMotor(int speed)
{
  speed *= rightMotorMulti;
  
  if(abs(speed) < MIN_MOTOR_SPEED)
  {
    analogWrite(R_MOTOR_ENABLE, 0);
    digitalWrite(R_MOTOR_1, LOW);
    digitalWrite(R_MOTOR_2, LOW);
  }

  else
  {
    analogWrite(R_MOTOR_ENABLE, abs(speed));
    
    if(speed > 0)
    {
      //drive foward
      digitalWrite(R_MOTOR_1, LOW);
      digitalWrite(R_MOTOR_2, HIGH);
    }
    else
    {
      //drive in reverse
      digitalWrite(R_MOTOR_1, HIGH);
      digitalWrite(R_MOTOR_2, LOW);
    }
  }
}

void driveLeftMotor(int speed)
{
  speed *= leftMotorMulti;
  
  if(abs(speed) < MIN_MOTOR_SPEED)
  {
    analogWrite(L_MOTOR_ENABLE, 0);
    digitalWrite(L_MOTOR_1, LOW);
    digitalWrite(L_MOTOR_2, LOW);
  }
  else
  {
    analogWrite(L_MOTOR_ENABLE, abs(speed));
    
    if(speed > 0)
    {
      //drive foward
      digitalWrite(L_MOTOR_1, HIGH);
      digitalWrite(L_MOTOR_2, LOW);
    }
    else
    {
      //drive in reverse
      digitalWrite(L_MOTOR_1, LOW);
      digitalWrite(L_MOTOR_2, HIGH);
    }
  }
}
