#include <Adafruit_MotorShield.h>
#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3
#define lefts A0
#define middles A1
#define rights A2

int target_ticks = 0;
int left_ticks = 0;
int right_ticks = 0;
int cross = 0;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - Navigation test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  pinMode(lefts,INPUT);
  pinMode(rights, INPUT);
  pinMode(middles, INPUT);

  // Set the speed to start, from 0 (off) to 255 (max speed)
  startmotion();
  LeftMotor->setSpeed(200);
  RightMotor->setSpeed(200);

  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  attachInterrupt(LEFT_ENCODER, left_interrupt_routine, CHANGE);
  attachInterrupt(RIGHT_ENCODER, right_interrupt_routine, CHANGE);

}

void loop() {

  delay(100);
  update_enc();

  Serial.println(analogRead(lefts));
  Serial.println(analogRead(middles));
  Serial.println(analogRead(rights));

  if((analogRead(lefts)<=100) && (!analogRead(rights)<=100)) {
    //turn left
    LeftMotor->run(BACKWARD);
    RightMotor->run(FORWARD);
  }

  else if(!analogRead(lefts)<=100 && analogRead(rights)<=100){
    //turn right
    LeftMotor->run(FORWARD);
    RightMotor->run(BACKWARD);
  }

  else if(!analogRead(lefts)<=100 && !analogRead(rights)<=100){
    //stop
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
  }

  else if(/*DISTANCESENSOR < CERTAIN THRESHOLD*/) {
    move(500);
    LeftMotor->setSpeed(200);
    RightMotor->setSpeed(200);
  }

  else if(/*Far left detects line but far right doesnt and cross = 0*/) {
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    move(200);
    cross += 1; """How do we get this to just go up by one each time"""
    pickup();
    LeftMotor->setSpeed(200);
    RightMotor->setSpeed(200);
  }

  else if(/*Far left and far right detect line and cross = 1*/) {
    cross += 1;
  }

  else if(/*Far right detects line but far left doesnt and cross = 2*/) {
    cross += 1;
  }

  else if(/*Far right detects line and left doesnt and current density low and cross = 3*/) {
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    pivot(-900);
    move(100);
    drop();
    pivot(1800);
    move(100);
    pivot(-900);
    LeftMotor->setSpeed(200);
    RightMotor->setSpeed(200);
    cross += 1;
  }

  else if(/*Far right detects line and left doesnt and current density is high and cross ==3 or 4*/) {
    move(50);
    cross += 1;
    LeftMotor->setSpeed(200);
    RightMotor->setSpeed(200);
  }
  
  else if(/*Far right detects line and left doesnt and current density is high and cross == 5*/) {
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    pivot(-900);
    move(100);
    drop();
    pivot(1800);
    move(100);
    pivot(-900);
    LeftMotor->setSpeed(200);
    RightMotor->setSpeed(200);
    cross += 1;
  }
  else if(/*Far right detects line and left doesnt and current density is low and cross ==4 or 5*/) {
    move(100);
    LeftMotor->setSpeed(200);
    RightMotor->setSpeed(200);
    cross += 1;
  }

  //We need a way of detecting which block we have 
  //just carried/are carrying so we know which crossroads to cross over
  //and which to turn. 
  //Should do this by having a value of current density which updates
  //each time a block is picked up
}

void startmotion() {
  move(400);
  pivot(-900);
}

void pickup() {
  //Function to pick up block
  //Move servos inwards
  //Detect current required
  //Put into category of block dependent on amount of current drawn
}

void drop() {
  //Function to drop block
  //Move servos outwards
}

void move() {int distance) {
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

void pivot(int angle){ //units in 0.1deg
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

void update_enc(){
  //  Update motor speed proportionally 
  int left_motor_speed = constrain(1000+(target_ticks-left_ticks)*75, 0, 4090);
  int right_motor_speed = constrain(1000+(target_ticks-right_ticks)*75, 0, 4090);

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
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
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
