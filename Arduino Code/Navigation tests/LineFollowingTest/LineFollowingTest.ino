
#include <Adafruit_MotorShield.h>
#define lefts A0
#define rights A1

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

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

  // Set the speed to start, from 0 (off) to 255 (max speed)
  LeftMotor->setSpeed(200);
  RightMotor->setSpeed(200);
  pinMode(lefts,INPUT);
  pinMode(rights, INPUT);
}

void loop() {
  Serial.println(analogRead(lefts));
  Serial.println(analogRead(rights));

  // if (analogRead(lefts)<=100 && analogRead(rights)<=100) {
  //   LeftMotor->run(RELEASE);
  //   RightMotor->run(RELEASE);
  // }

  if((analogRead(lefts)<=100) && (!analogRead(rights)<=100)) {
    LeftMotor->run(FORWARD);
    RightMotor->run(BACKWARD);
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
}