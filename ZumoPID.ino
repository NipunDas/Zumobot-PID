#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;

double currentDistance = 0;
double target = 30;
const double kP = 13;
const double kI = 5.5;
const double kD = 1.5;
double error;
int power;
int dt = 0;
double dx;
unsigned long previousTime = 0;
double errorIntegral = 0;
double previousError = target;
double derivative;

//setup method
void setup() {
 Serial.begin(115200);
 motors.setSpeeds(0, 0);
 encoders.init();
 lcd.clear();
 delay(1000);
}

//turns the robot to the right
void spinRight(int motorSpeed) {
  motors.setSpeeds(motorSpeed, -motorSpeed);
}

//turns the robot to the left
void spinLeft(int motorSpeed) {
  motors.setSpeeds(-motorSpeed, motorSpeed);
}

//moves the robot forward (negative motor speed to move backwards)
void goForward(int motorSpeed){
  motors.setSpeeds(motorSpeed*1.05, motorSpeed);
}

//stops the robot
void stopRobot() {
   motors.setSpeeds(0, 0);
}

//returns number of encoder ticks
double getDistance() {
  
  return (encoders.getCountsLeft() + encoders.getCountsRight())/403.0;
}

//resets the encoder tick count
void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void loop() {
  //updating encoder count and error variable
  currentDistance = getDistance();
  error = target - currentDistance;

  //calculating dx and dt and errorIntegral and derivative
  dx = error - previousError;
  dt = (int) (micros() - previousTime);
  errorIntegral += (error * dt)/1000000.0;
  derivative = (dx/dt) * 1000000.0;
  
  //moving the robot and stopping it based on encoder count
  if (error > 0.1) {
    power = (int) (kP * error) + (int) (kI * errorIntegral)+ (int) (kD * derivative);
    goForward(power);
    
    //updating previous time and previous distance
    previousTime = micros();
    previousError = target - getDistance();
  } else {
    stopRobot();
  }

  //printing out distance from encoder values
  lcd.gotoXY(0, 0);
  lcd.print(currentDistance);
  lcd.print(" ");
}
