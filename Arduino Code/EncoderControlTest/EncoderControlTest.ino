#define LEFT_LINE A0
#define RIGHT_LINE A1
#define MID_LINE A2
#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3


//+++ENABLING TIMERS+++//
#define USING_16MHZ     true
#define USING_8MHZ      false
#define USING_250KHZ    false
#define USE_TIMER_0     false
#define USE_TIMER_1     true
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define TIMER1_INTERVAL_MS    1000
#include <megaAVR_TimerInterrupt.h>


//+++DEFINING MOTORSHIELD+++//
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select motor ports M1 and M2
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);


//+++VARIABLES FOR PID CONTROL+++//
const int threshold = 0;

const int Kp = 10;
const int Ki = 0.05;
const int Kd = 0;

unsigned int prev_err=0;
unsigned int lintegral=0;
bool pid_active = false;

int target_ticks = 0;
int left_ticks = 0;
int right_ticks = 0;

int left_motor_speed = 0;
int right_motor_speed = 0;

// Dimensions in mm
const float wheel_radius = 34.925;
const float base_radius = 237;


void setup() {
  Serial.begin(115200);

  //+++SETTING UP ENCODER PIN INTERRUPTS+++//
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  attachInterrupt(LEFT_ENCODER, left_interrupt_routine, CHANGE);
  attachInterrupt(RIGHT_ENCODER, right_interrupt_routine, CHANGE);

  //+++SETTING UP TIMER INTERRUPTS+++//
  ITimer1.init();
  if (ITimer1.attachInterruptInterval(100, timer_interrupt_routine)){ // Attach update_pid with interval 100ms
    Serial.println("Starting  ITimer1 OK, millis() = " + String(millis()));
  }else {
    Serial.println("Can't set ITimer1. Select another freq. or timer");
  }

  //+++SETTING UP MOTORSHIELD+++//
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");


  LeftMotor->setSpeed(0);
  RightMotor->setSpeed(0);
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);

  move(100);


}

void loop() {
  delay(100);
  Serial.println("Left Ticks: ");Serial.println(left_ticks);
  Serial.println("Right Ticks: ");Serial.println(right_ticks);
  Serial.println("Left Speed: ");Serial.println(left_motor_speed);
  Serial.println("Right Speed: ");Serial.println(right_motor_speed);

  // Stop each motor if it's ahead of the other
  if (left_ticks > right_ticks){
    LeftMotor->setSpeed(0);
  } else {
    LeftMotor->setSpeed(left_motor_speed);
  }
  if (right_ticks > left_ticks){
    RightMotor->setSpeed(0);
  } else {
    RightMotor->setSpeed(right_motor_speed);  
  }
}


void move(int distance){ //units in mm
  target_ticks = abs((int)distance/(2*PI*wheel_radius)*96);
  left_ticks = 0;
  right_ticks = 0;
  Serial.print("Target Ticks: ");Serial.println(target_ticks);
  LeftMotor->setSpeed(0);
  RightMotor->setSpeed(0);
  if (distance > 0){
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
  } else {
    LeftMotor->run(BACKWARD);
    RightMotor->run(BACKWARD);
  }
  pid_active = true;
}

void pivot(int angle){ //units in 0.1deg
  return;
}

void update_pid(){
  if (!pid_active){
    return;
  }
  int error = target_ticks - left_ticks;
  int integral = integral+(prev_err+error)/2; // trapezoidal integration
  int derivative = prev_err-error; //instantaneous gradient
  if (error < threshold){
    target_ticks = 0;
    left_ticks = 0;
    right_ticks = 0;//remove
    left_motor_speed=0;
    right_motor_speed=0;    
    pid_active = false;
    return;
  }
  int output = (int) Kp*error+Ki*integral+Kd*derivative;
  left_motor_speed = constrain(output, 0, 255);
  right_motor_speed = constrain(output, 0, 255);//remove
  prev_err = error;
  return;
}

void left_interrupt_routine(){
  left_ticks++;
}
void right_interrupt_routine(){
  right_ticks++;
}

void timer_interrupt_routine(){
  update_pid();
}