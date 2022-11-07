#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz (the default parameter is 1600)
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  AFMS.begin();
}

void loop() {
  myMotor1->run(FORWARD);
  myMotor1->setSpeed(255);
  delay(1000);
  myMotor1->run(RELEASE);
  delay(1000);
  Serial.println("Pausing");
}