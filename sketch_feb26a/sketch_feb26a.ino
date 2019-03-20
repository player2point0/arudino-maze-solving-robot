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
//encoders
#include <Encoder.h>
Encoder leftEncoder(3, 4);
Encoder rightEncoder(2, 10);
volatile long leftEncoderStore, rightEncoderStore;
volatile int rightMotorSpeed, leftMotorSpeed;
bool flag = false;

int slowSpeed = -235;
int normalSpeed = -390;
int updateDelay = 15;

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

  //https://learn.adafruit.com/multi-tasking-the-arduino-part-2/timers
  //https://www.teachmemicro.com/arduino-timer-interrupt-tutorial/
  //https://arduino.stackexchange.com/questions/30968/how-do-interrupts-work-on-the-arduino-uno-and-similar-boards
  OCR0A = 0xAF; // set the compare register A for timer0
  TIMSK0 |= _BV(OCIE0A);  //enable the compare interrupt A for timer 0
}

// interrupt service routine called when timer0 compare A interrupt flag set
//called on timer0 so every 1ms
ISR(TIMER0_COMPA_vect) 
{
  if (flag) {
    //leftMotor
    long encoderReading = leftEncoder.read();
    int measuredSpeed = abs(leftEncoderStore - encoderReading) * 15;//in rpm
    leftEncoderStore = encoderReading;
    if (measuredSpeed > leftMotorSpeed)
      digitalWrite(L_MOTOR_ENABLE, LOW);
    else
      digitalWrite(L_MOTOR_ENABLE, HIGH);
  }
  
  else
  {
    //rightMotor
    long encoderReading = rightEncoder.read();
    long measuredSpeed = abs(rightEncoderStore - encoderReading) * 15;//in rpm
    rightEncoderStore = encoderReading;
    if (measuredSpeed > rightMotorSpeed)
      digitalWrite(R_MOTOR_ENABLE, LOW);
    else
      digitalWrite(R_MOTOR_ENABLE, HIGH);  
  }

  flag = !flag;
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

void stopMotors()
{
  driveRightMotor(0);
  driveLeftMotor(0);
}

//MOTOR DRIVES
void driveLeftMotor(int speed)
{
  noInterrupts();
  leftMotorSpeed = abs(speed);
  interrupts();

  if (leftMotorSpeed < MIN_MOTOR_SPEED) 
  {  
    //analogWrite(L_MOTOR_ENABLE, 0);
    digitalWrite(L_MOTOR_1, LOW);
    digitalWrite(L_MOTOR_2, LOW);  
  }
  else 
  {
    //analogWrite(L_MOTOR_ENABLE, abs(speed));
    if (speed > 0)
    {
      digitalWrite(L_MOTOR_1, HIGH);
      digitalWrite(L_MOTOR_2, LOW);
    }
    else
    {
      digitalWrite(L_MOTOR_1, LOW);
      digitalWrite(L_MOTOR_2, HIGH);
    }
  }
}
void driveRightMotor(int speed)
{
  noInterrupts();
  rightMotorSpeed = abs(speed);
  interrupts();

  if (rightMotorSpeed < MIN_MOTOR_SPEED) 
  { 
    //analogWrite(R_MOTOR_ENABLE, 0);
    digitalWrite(R_MOTOR_1, LOW);
    digitalWrite(R_MOTOR_2, LOW);  
  }
  else 
  {
    //analogWrite(R_MOTOR_ENABLE, abs(speed));
    if (speed > 0)
    {
      digitalWrite(R_MOTOR_1, LOW);
      digitalWrite(R_MOTOR_2, HIGH);
    }
    else 
    {
      digitalWrite(R_MOTOR_1, HIGH);
      digitalWrite(R_MOTOR_2, LOW);
    }
  }
}