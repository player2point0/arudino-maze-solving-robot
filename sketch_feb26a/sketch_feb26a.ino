#include <Encoder.h>

//ultrasonic
#include <Ultrasonic.h>
Ultrasonic leftSensor(A3, A2, 10000UL);
Ultrasonic frontSensor(A5, A4, 10000UL);

//Motor pins
#define L_MOTOR_ENABLE 6
#define R_MOTOR_ENABLE 5
#define L_MOTOR_1 11
#define L_MOTOR_2 9
#define R_MOTOR_1 8
#define R_MOTOR_2 7
#define MIN_MOTOR_SPEED 1
#define MAX_MOTOR_SPEED 255

Encoder leftEncoder(3, 4);
Encoder rightEncoder(2, 10);
volatile long leftEncoderStore, rightEncoderStore;
volatile int rightMotorSpeed, leftMotorSpeed;
bool flag = false;

void setup() {
  Serial.begin(9600);
  //setup motors
  pinMode(L_MOTOR_1, OUTPUT);
  pinMode(L_MOTOR_2, OUTPUT);
  pinMode(R_MOTOR_1, OUTPUT);
  pinMode(R_MOTOR_2, OUTPUT);
  pinMode(L_MOTOR_ENABLE, OUTPUT);
  pinMode(R_MOTOR_ENABLE, OUTPUT);
  
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
  //bruteForce();
  //mazeSolve();
  followLeftWall();
}

void followLeftWall()
{
  delay(50);  
  int distanceLeft = leftSensor.read();
  delay(50);
  int distanceForward = frontSensor.read();
  
  int normalSpeed = 160;
  int mediumSpeed = 120;//normalSpeed / 2;//120
  int slowSpeed = 70;//normalSpeed / 3;//70

  if(distanceLeft > 25)
  {
    //go left
    driveLeftMotor(slowSpeed);
    driveRightMotor(normalSpeed);
  }
  
  else if(distanceLeft < 22 && distanceLeft > 18)
  {
    //slight correction right
    driveLeftMotor(normalSpeed);
    driveRightMotor(mediumSpeed);  
  }
  
  else if(distanceLeft < 18 && distanceForward < 18)
  {
    stopMotors();
    delay(50);
    acceleratedRotation(90, 200);
    //rotateRobot(90, 50); 
  }

  else if(distanceLeft < 15 || distanceForward < 30)
  {
    //go right
    driveLeftMotor(normalSpeed);
    driveRightMotor(slowSpeed);
  }

  else 
  {
    //sprintForward(normalSpeed, 1000);
    driveLeftMotor(normalSpeed);
    driveRightMotor(normalSpeed);
  }
}

void sprintForward(float driveSpeed, float distance)
{
  //add acceleration
  driveLeftMotor(driveSpeed);
  driveRightMotor(driveSpeed);

  long startDistance = (abs(leftEncoder.read()) + abs(rightEncoder.read())) / 2;
  long totalDistance = 0;
  int front = 100;

  while(totalDistance < distance && front > 20)
  {
      //record distance travelled
      float avgDistance = (abs(leftEncoder.read()) + abs(rightEncoder.read())) / 2;

      totalDistance = avgDistance - startDistance;

      front = frontSensor.read();

      delay(10);
  }

  stopMotors();
}

void acceleratedRotation(float angle, float driveSpeed)
{
  float accelerationAngle = 30;
  float decelerationAngle = 30;
  float accelerationSpeed = driveSpeed * 0.5;
  float decelerationSpeed = driveSpeed * 0.5;

  if((accelerationAngle + decelerationAngle) >= angle)
  {
    rotateRobot(angle, accelerationSpeed);  
  }
  
  rotateRobot(accelerationAngle, accelerationSpeed);
  delay(5);
  rotateRobot(angle - (accelerationAngle + decelerationAngle), driveSpeed);
  delay(5);
  rotateRobot(decelerationAngle, decelerationSpeed);
}

void rotateRobot(float angle, int driveSpeed)
{
  float fullTurnDistance = 6000;
  float percentage = abs(angle / 360);
  long turnDistance = 0;
  long intialDistance = abs(leftEncoder.read());
 
  if(angle < 0)
  {
    driveRightMotor(driveSpeed);
    driveLeftMotor(-driveSpeed);
  }
  else
  { 
    driveRightMotor(-driveSpeed);
    driveLeftMotor(driveSpeed); 
  }

  while(turnDistance < (fullTurnDistance * percentage))
  {
      long avgDistance = abs(leftEncoder.read());

      turnDistance = abs(avgDistance - intialDistance);
  }
  
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
