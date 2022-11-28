#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3
#define LEFT_LINE 4
#define RIGHT_LINE 5
#define LEFT_CHECK 6
#define RIGHT_CHECK 7
#define LEFT_SERVO 9
#define RIGHT_SERVO 10

long timey = 0;

//+++DEFINING MOTORSHIELD+++//
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select motor ports M1 and M2
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);

//+++DEFINING LEFT FACING ULTRASONIC SENSOR+++//
#include <HCSR04.h>
//+++LEFTHAND ULTRASONIC+++//
const byte triggerPin = 13;
const byte echoPin = 12;
UltraSonicDistanceSensor distanceSensorLeft(triggerPin, echoPin);
//+++FRONT ULTRASONIC+++//
UltraSonicDistanceSensor distanceSensorFront(11, 8);

//+++DEFINING SERVOMOTORS+++//
#include <Servo.h>
Servo left_servo;
Servo right_servo;

//+++GLOBAL VARIABLES+++//
int target_ticks = 0;
int left_ticks = 0;
int right_ticks = 0;
int state = 0;
int lap = 0;
int pos = 0;
bool encoder_running = false;
bool line_running = false;
bool in_tunnel = false;
bool high_density = false;

//+++CONSTANT VARIABLES+++//
const float wheel_radius = 34.85; //mm
const float wheel_sep = 240; //mm
const int line_speed = 4090;

void setup() {
  //Serial.begin(115200);

  //+++SETTING UP LINE FOLLOW PINS+++//
  pinMode(LEFT_LINE, INPUT_PULLUP);
  pinMode(RIGHT_LINE, INPUT_PULLUP);
  pinMode(LEFT_CHECK, INPUT_PULLUP);
  pinMode(RIGHT_CHECK, INPUT_PULLUP);
 
  //+++SETTING UP ENCODER PIN INTERRUPTS+++//
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  attachInterrupt(LEFT_ENCODER, left_interrupt_routine, CHANGE);
  attachInterrupt(RIGHT_ENCODER, right_interrupt_routine, CHANGE);

  //+++SETTING UP MOTORSHIELD+++//
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  LeftMotor->setSpeedFine(0);
  RightMotor->setSpeedFine(0);
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);

  //+++SETTING UP SERVOMOTOR+++//
  left_servo.attach(LEFT_SERVO);
  right_servo.attach(RIGHT_SERVO);
  pickup();
  delay(100);
  left_servo.write(20);
  right_servo.write(160);
  delay(100);

  pinMode(A3, OUTPUT);
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  
  digitalWrite(A3, HIGH);
  delay(2000);
  digitalWrite(A3, LOW);
  delay(2000);

}
 
void loop() {
   switch (state) {
     case 0:
     //Moving straight out of the starting box//
      delay(1000);
       enc_move(333);
       state++;
       break;

     case 1:
     //Turning right onto the main line//
       if (!encoder_running){
        enc_pivot(900);
        state++;
       }
       break;
    
     case 2:
     //Following the line//
       if (!encoder_running){
         line_start();
         state++;
       }
       break;
    
     case 3:
     //Skipping any junctions
      if (digitalRead(RIGHT_CHECK)==0 || digitalRead(LEFT_CHECK)==0){
        LeftMotor->run(FORWARD);
        RightMotor->run(FORWARD);
        LeftMotor->setSpeedFine(4000);
        RightMotor->setSpeedFine(4000);
        delay(800);
        LeftMotor->setSpeedFine(3500);
        RightMotor->setSpeedFine(3500);
      }

      // Stopping to test the block after XXX ticks travelled
      if (left_ticks>2100){
        LeftMotor->setSpeedFine(0);
        RightMotor->setSpeedFine(0);
        int block_checking = 0;
        for (int i=0; i<3; i++){
          if (check_block()){
            block_checking++;
          }
          delay(500);
        }
        if (block_checking > 1){
          Serial.println("Low-Density");
          digitalWrite(0, HIGH);
        } else {
          Serial.println("High-Density");
          digitalWrite(1, HIGH);
        }
        delay(5000);
        digitalWrite(0, LOW);
        digitalWrite(1, LOW);
        pickup();
        line_start();
        state++;
      }
      break;

    case 4:
      // Skipping any Junctions.
      if (digitalRead(RIGHT_CHECK)==0 || digitalRead(LEFT_CHECK)==0){
        LeftMotor->run(FORWARD);
        RightMotor->run(FORWARD);
        LeftMotor->setSpeedFine(4000);
        RightMotor->setSpeedFine(4000);
        delay(800);
        LeftMotor->setSpeedFine(3500);
        RightMotor->setSpeedFine(3500);
      }
      // Wall align
      if (left_ticks>500) {
        line_running = false;
        enc_move(600);
        state+=2;
      }
      break;

    case 5:
    //Drive straight for XX mm exiting the tunnel
      state++;
      break;

    case 6:
    //Pivot 90 degrees
      if (encoder_running == false){
        enc_pivot(-900);
        state++;
      }
      break;

    case 7:
    //Align back against wall and then drive straight for XX mm
      if (encoder_running == false){
        LeftMotor->run(BACKWARD);
        RightMotor->run(BACKWARD);
        LeftMotor->setSpeedFine(3000);
        RightMotor->setSpeedFine(3000);
        delay(800);
        LeftMotor->setSpeedFine(0);
        RightMotor->setSpeedFine(0);
        enc_move(400);
        state++;
      }
      break;

    case 8:
      //Either do a 180 pivot or move forwards by XXX mm
      if (encoder_running == false){
        if (high_density){
          enc_move(1000);
        } else {
          enc_pivot(1800);
        }
        state++;
      }
      break;

    case 9:
      //Drop the block and reverse into parking
      if (encoder_running == false){
        drop();
        enc_move(-1000);
        state++;
      }
      break;

     default:
       Serial.print("State not written - State ");Serial.println(state);
   }

  Serial.println(state);
  if (line_running){
    line_update();
  } else if (encoder_running){
    enc_update();
  }
}
 

//+++LINE FOLLOW FUNCTIONS+++///

void line_start(){
  Serial.println("Line Starting");
  line_running = true;
  LeftMotor->setSpeedFine(3500);
  RightMotor->setSpeedFine(3500);
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);
  left_ticks = 0;
  right_ticks = 0;
}

void line_update(){
  delay(200);
  if(digitalRead(LEFT_LINE)==0 && digitalRead(RIGHT_LINE)==1) {
    //turn left
    LeftMotor->run(BACKWARD);
    RightMotor->run(FORWARD);
  }
  else if(digitalRead(LEFT_LINE)==1 && digitalRead(RIGHT_LINE)==0){
    //turn right
    LeftMotor->run(FORWARD);
    RightMotor->run(BACKWARD);
  }
  else if(digitalRead(LEFT_LINE)==1 && digitalRead(RIGHT_LINE)==1){
    //keep moving
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
  }
}

//+++FUNCTION FOR WHEN IN THE TUNNEL+++//
void tunnel_movement() {
  while (in_tunnel) {
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
    if (distanceSensorLeft.measureDistanceCm() * 10.026 - 10.515 < 80) {
      LeftMotor->run(FORWARD);
      RightMotor->run(BACKWARD);
    }
    else if (distanceSensorLeft.measureDistanceCm() * 10.026 - 10.515 > 90) {
      LeftMotor->run(BACKWARD);
      RightMotor->run(FORWARD);
    }
    /*else if (sensor.GetDistance() > 40) {
      in_tunnel = false;
    }
    */
    else if(digitalRead(LEFT_LINE) == 0 || digitalRead(RIGHT_LINE) == 0 ||  digitalRead(LEFT_CHECK) == 0 || digitalRead(RIGHT_CHECK) == 0) {
      in_tunnel = false;
    }
    if ((left_ticks+right_ticks)/2 > 200){
      in_tunnel = false;
    }
    else {
      LeftMotor->run(FORWARD);
      RightMotor->run(FORWARD);
    }
  }
}


//+++ENCODER FUNCTIONS+++///

void enc_move(int distance){ //units in mm
  encoder_running = true;
  target_ticks = abs((int)distance/(2*PI*wheel_radius)*96);
  left_ticks = 0;
  right_ticks = 0;
  Serial.print("Target Ticks: ");Serial.println(target_ticks);
  LeftMotor->setSpeedFine(4090);
  RightMotor->setSpeedFine(4090);
  if (distance > 0){
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
  } else {
    LeftMotor->run(BACKWARD);
    RightMotor->run(BACKWARD);
  }
}
 
void enc_pivot(int angle){ //units in 0.1deg
  encoder_running = true;  
  target_ticks = abs((int)(angle * wheel_sep * PI / 3600)/(2*PI*wheel_radius)*96);
  left_ticks = 0;
  right_ticks = 0;
  Serial.print("Target Ticks: ");Serial.println(target_ticks);
  LeftMotor->setSpeedFine(4090);
  RightMotor->setSpeedFine(4090);
  if (angle > 0) {
    LeftMotor->run(FORWARD);
    RightMotor->run(BACKWARD);
  } else {
    LeftMotor->run(BACKWARD);
    RightMotor->run(FORWARD);
  }
}

void enc_update(){
  //  Update motor speed proportionally 
  int left_motor_speed = constrain(1400+(target_ticks-left_ticks)*75, 0, 4090);
  int right_motor_speed = constrain(1400+(target_ticks-right_ticks)*75, 0, 4090);

  // Slow each motor if it's ahead of the other
  if (left_ticks > right_ticks){
    LeftMotor->setSpeedFine((int)left_motor_speed*0.8);
  } else {
    LeftMotor->setSpeedFine(left_motor_speed);
  }
  if (right_ticks > left_ticks){
    RightMotor->setSpeedFine((int)right_motor_speed*0.8);
  } else {
    RightMotor->setSpeedFine(right_motor_speed);  
  }

  // Stop both motors at the end
  if (left_ticks >= target_ticks && right_ticks >= target_ticks){
    encoder_running = false;
    LeftMotor->setSpeedFine(0);
    RightMotor->setSpeedFine(0);
    LeftMotor->run(RELEASE);
    RightMotor->run(RELEASE);
    target_ticks = 0;
    left_ticks = 0;
    right_ticks = 0;
  }
}

void left_interrupt_routine(){
  left_ticks++;
}
void right_interrupt_routine(){
  right_ticks++;
}




//+++BLOCK FUNCTIONS+++//
void pickup() {
  left_servo.write(80);
  right_servo.write(100);
}

void drop() {
  left_servo.write(45);
  right_servo.write(135);
}

int check_block(){ //returns true for low-density block
  pickup();
  int average = 0;
  for (int i=0; i<100; i++){
    int us_val = distanceSensorFront.measureDistanceCm();
    if (us_val < 0){
      average += 100;
    } else {
      average += us_val;
    }
    delay(10);
  }
  average = average/100;
  if (digitalRead(A2)==HIGH){
    return true;
  } else {
    return false;
  }
  Serial.println(average);
  drop();
  return (average<30);
}
