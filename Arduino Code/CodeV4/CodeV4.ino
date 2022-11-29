#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3
#define LEFT_LINE 4
#define RIGHT_LINE 5
#define LEFT_CHECK 6
#define RIGHT_CHECK 7
#define LEFT_SERVO 9
#define RIGHT_SERVO 10
#define FRONT_ECHO 8
#define FRONT_TRIG 11
#define LEFT_ECHO 12
#define LEFT_TRIG 13
#define IR_TUNNEL A2
#define BLOCK_LDR A0
#define RED_LED 1
#define YELLOW_LED A3
#define GREEN_LED 0

//+++DEFINING MOTORSHIELD+++//
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select motor ports M1 and M2
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);

//+++DEFINING ULTRASONIC+++//
#include <HCSR04.h>
UltraSonicDistanceSensor distanceSensorLeft(LEFT_TRIG, LEFT_ECHO);
UltraSonicDistanceSensor distanceSensorFront(FRONT_TRIG, FRONT_ECHO);

//+++DEFINING SERVOMOTORS+++//
#include <Servo.h>
Servo left_servo;
Servo right_servo;

//+++GLOBAL VARIABLES+++//
int target_ticks = 0;
int left_ticks = 0;
int right_ticks = 0;
int state = 0;
int laps = 0;
int pos = 0;
bool encoder_running = false;
bool line_running = false;
bool tunnel_running = false;
bool is_high_density = false;

//+++CONSTANT VARIABLES+++//
const float wheel_radius = 34.85; //mm
const float wheel_sep = 225; //mm
const int line_speed = 4090;

void setup() {
  //Serial.begin(115200);

  pinMode(IR_TUNNEL, INPUT_PULLUP);
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

  //+++SETTING UP LED PINS+++//
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  
  digitalWrite(YELLOW_LED, HIGH);
  delay(1000);
  digitalWrite(YELLOW_LED, LOW);
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
      //Skipping red junction//
      if (left_ticks>200 && digitalRead(RIGHT_CHECK)==0 && digitalRead(LEFT_CHECK)==1){
        line_skip_junction();
        line_start();
        state++;
      }
      break;

    case 4:
      //Skipping 1st block junction//
      if (left_ticks>720 && digitalRead(RIGHT_CHECK)==1 && digitalRead(LEFT_CHECK)==0){
        line_skip_junction();
        line_start();
        halfopen();
        state++;
      }
      break;

    case 5:
      //Stop at 2nd block junction and test block//
      if (left_ticks>80 && (digitalRead(RIGHT_CHECK)==0 || digitalRead(LEFT_CHECK)==0)){
        LeftMotor->setSpeedFine(0);
        RightMotor->setSpeedFine(0);
        check_block();
        pickup();
        line_start();
        state++;
      }
      break;

    case 6:
      //Skip 3rd block junction
      if (left_ticks>80 && digitalRead(RIGHT_CHECK)==1 && digitalRead(LEFT_CHECK)==0){
        digitalWrite(GREEN_LED,HIGH);
        line_skip_junction();
        line_start();
        state++;
      }
      break;

    case 7:
      // Enter the tunnel mode based on IR reading
      if (digitalRead(IR_TUNNEL)==0){
        line_running = false;
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(RED_LED,HIGH);
        tunnel_start();
        state++;
      }
      break;

    case 8:
      // Exits the tunnel mode based on IR reading//
      if (left_ticks>30 && digitalRead(IR_TUNNEL)==1){
        line_skip_junction();
        digitalWrite(RED_LED, LOW);
        tunnel_running = false;
        line_start();
        if (is_high_density){
          state = 20;
        } else {
          state = 10;
        }
      }
      break;

//++++++++++++SCORING IN GREEN (1_ states)++++++++++++//
    case 10:
      // Turn on Green Junction
      if (left_ticks>180 && digitalRead(RIGHT_CHECK)==0 && digitalRead(LEFT_CHECK)==1){
        line_running = false;
        enc_pivot(700);   
        state++;
      }
      break;

    case 11:
      // Enter green
      if (encoder_running == false){
        drop();
        enc_move(200);   
        state++;
      }
      break;

    case 12:
      // Leave green
      if (encoder_running == false){
        enc_move(-200);   
        state++;
      }
      break;

    case 13:
      // Turn back to line
      if (encoder_running == false){
        enc_pivot(-700);
        state++;
      }
      break;

    case 14:
      // Continue line-following
      if (encoder_running == false){
        line_skip_junction();
        line_start();
        state++;
      }
      break;

    case 15:
      // EITHER skip white junction and continue to red OR jump to state 17 to park
      if (left_ticks>150 && digitalRead(RIGHT_CHECK)==0 && digitalRead(LEFT_CHECK)==1){
        if (laps == 1){
          enc_move(115);
          state = 17;
        }
        else {
          line_skip_junction();
          line_start();
          state++;
        }
      }
      break;
    
    case 16:
      // Second lap pick up block
      if (left_ticks>200 && digitalRead(RIGHT_CHECK)==0 && digitalRead(LEFT_CHECK)==1){
        line_skip_junction();
        line_start();
        state = 30;
        laps++;
      }
      break;
    
    case 17:
      // Park in
      if (encoder_running==false){
        enc_pivot(-900);
        state=99;
      }
      break;

//++++++++++++SCORING IN RED (2_ states)++++++++++++//
    case 20:
      // Skip green bin junction 
      if (left_ticks>180 && digitalRead(RIGHT_CHECK)==0 && digitalRead(LEFT_CHECK)==1){
        line_skip_junction();
        line_start();
        state++;
      }

    case 21:
      // Skip starting bin junction
      if (left_ticks>200 && digitalRead(RIGHT_CHECK)==0 && digitalRead(LEFT_CHECK)==1){
        line_skip_junction();
        line_start();
        state++;
      }
    
    case 22:
      if (left_ticks>200 && digitalRead(RIGHT_CHECK)==0 && digitalRead(LEFT_CHECK)==1){
        line_running = false;
        enc_pivot(700);   
        state++;
      }
      break;

    case 23:
      // Enter red
      if (encoder_running == false){
        drop();
        enc_move(200);   
        state++;
      }
      break;

    case 24:
      // Leave red
      if (encoder_running == false){
        enc_move(-200);   
        state++;
      }
      break;

    case 25:
      // EITHER turn back to line OR turn the other way to park
      if (encoder_running == false){
        if (laps == 1){
          enc_pivot(1150);
          state = 27;
        } else {
          enc_pivot(-650);
          state++;
        }
      }
      break;

    case 26:
      // Second lap pick up block
      if (encoder_running == false){
        line_skip_junction();
        line_start();
        state = 30;
        laps++;
      }
      break;
    
    case 27:
      // Follow line back to white junction
      if (encoder_running == false){
        line_skip_junction();
        line_start();
        state++;
      }
      break;

    case 28:
      // Move forwards
      if (left_ticks>150 && digitalRead(RIGHT_CHECK)==1 && digitalRead(LEFT_CHECK)==0){
        enc_move(115);
        state++;
      }
      break;
    
    case 29:
      // Pivot on white junction
      if (encoder_running == false){
        enc_pivot(900);
        state=99;
      }
      break;

//++++++++++++FINDING BLOCK LAP 1 (3_ states)++++++++++++//
    case 30:
      //Pivot on first block junction
      if (left_ticks>720 && digitalRead(RIGHT_CHECK)==1 && digitalRead(LEFT_CHECK)==0){
        enc_pivot(-600);
        state++;
      }
      break;
    
    case 31:
      //Move forwards to grab the block
      if (encoder_running == false){
        halfopen();
        enc_move(150);
        state++;
      }
      break;

    case 32:
      //Identify and pick up the block
      if (encoder_running == false){
        pickup();
        check_block();
        state++;
      }
      break;

    case 33:
      //Reverse to the line
      if (encoder_running == false){
        enc_move(-150);
        state++;
      }
      break;

    case 34:
      //Pivot back onto line
      if (encoder_running == false){
        enc_pivot(600);
        state++;
      }
      break;

    case 35:
      // Continue line following to 2nd block junction
      if (encoder_running == false){
        line_skip_junction();
        line_start();
        state++;
      }
      break;


    case 36:
      //Skip 2nd block junction//
      if (left_ticks>80 && (digitalRead(RIGHT_CHECK)==0 || digitalRead(LEFT_CHECK)==0)){
        line_skip_junction();
        line_start();
        state++;
      }
      break;
    
    case 37:
      //Skip 3rd block junction//
      if (left_ticks>80 && digitalRead(RIGHT_CHECK)==1 && digitalRead(LEFT_CHECK)==0){
        line_skip_junction();
        line_start();
        state=7;
      }
      break;

    case 98:
      //Park in 
      if (encoder_running==false){
        while(true){
          pickup();
          delay(500);
          drop();
          delay(500);
        }
        state--;
      }
      break;

    case 99:
      //Park in 
      if (encoder_running==false){
        enc_move(-400);
        state--;
      }
      break;

//++++++++++++TESTING++++++++++++//
    case 100:
      // Testing only
      LeftMotor->run(FORWARD);
      RightMotor->run(FORWARD);
      LeftMotor->setSpeedFine(3340);
      RightMotor->setSpeedFine(4090);
      break;

    case 101:
      check_block();
      break;
  
    default:
      Serial.print("State not written - State ");Serial.println(state);
   }

   


  //Serial.println(state);
  //Serial.print("LEFT TICKS: ");Serial.println(left_ticks);
  //Serial.print("RIGHT TICKS: ");Serial.println(right_ticks);
  blink();
  if (line_running){
    line_update();
  } else if (encoder_running){
    enc_update();
  } else if (tunnel_running){
    tunnel_update();
  }
}





//+++LINE FOLLOW FUNCTIONS+++///

void line_start(){
  Serial.println("Line Starting");
  line_running = true;
  encoder_running = false;
  tunnel_running = false;
  LeftMotor->setSpeedFine(3340);
  RightMotor->setSpeedFine(4090);
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);
  left_ticks = 0;
  right_ticks = 0;
}

void line_update(){
  if(digitalRead(LEFT_LINE)==0 && digitalRead(RIGHT_LINE)==1) {
    //turn left
    LeftMotor->run(RELEASE);
    RightMotor->run(FORWARD);
  }
  else if(digitalRead(LEFT_LINE)==1 && digitalRead(RIGHT_LINE)==0){
    //turn right
    LeftMotor->run(FORWARD);
    RightMotor->run(RELEASE);
  }
  else if(digitalRead(LEFT_LINE)==1 && digitalRead(RIGHT_LINE)==1){
    //keep moving
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
  }
}

void line_skip_junction(){
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);
  LeftMotor->setSpeedFine(3340);
  RightMotor->setSpeedFine(4090);
  delay(900);
}




//+++TUNNEL FUNCTIONS+++//
void tunnel_start(){
  LeftMotor->setSpeedFine(3340);
  RightMotor->setSpeedFine(4090);
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);
  left_ticks = 0;
  right_ticks = 0;
  line_skip_junction();
  line_skip_junction();
  line_running = false;
  encoder_running = false;
  tunnel_running = true;
}

void tunnel_update() { // average distance is 80
  if (distanceSensorLeft.measureDistanceCm() * 10.026 - 10.515 < 70) {
    LeftMotor->run(FORWARD);
    RightMotor->run(RELEASE);
  } else if (distanceSensorLeft.measureDistanceCm() * 10.026 - 10.515 > 90) {
    LeftMotor->run(RELEASE);
    RightMotor->run(FORWARD);
  } else {
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
  }
}




//+++ENCODER FUNCTIONS+++///

void enc_move(int distance){ //units in mm
  line_running = false;
  encoder_running = true;
  tunnel_running = false;
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
  line_running = false;
  encoder_running = true;
  tunnel_running = false;
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
  int left_motor_speed = constrain(1800+(target_ticks-left_ticks)*75, 0, 4090);
  int right_motor_speed = constrain(1800+(target_ticks-right_ticks)*75, 0, 4090);

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
  left_servo.write(20);
  right_servo.write(160);
}

void halfopen(){
  left_servo.write(45);
  right_servo.write(135);
}

bool check_block(){ //returns true for high-density block
  pickup();
  int us_average = 0;
  for (int i=0; i<100; i++){
    int us_val = distanceSensorFront.measureDistanceCm();
    if (us_val < 0){
      us_average += 100;

    } else {
      us_average += us_val;
    }
    delay(10);
  }
  us_average = us_average/100;
  if (us_average<20){ //ULTRASONIC THRESHOLD
    Serial.println("US LOW");
    is_high_density = false;
  } else {
    long ldr_average = 0;
    for (int i=0; i<100; i++){
      int ldr_val = analogRead(BLOCK_LDR);
      ldr_average += ldr_val;      
    }
    ldr_average = ldr_average / 100;
    if (ldr_average<930){ //LDR THRESHOLD
      Serial.println("LDR LOW");
      is_high_density = false;
    } else {
      Serial.println("LDR HIGH");
      is_high_density = true;
    }
  }  
  if (is_high_density){
    digitalWrite(RED_LED, HIGH);
    delay(5000);
    digitalWrite(RED_LED, LOW);
  } else {
    digitalWrite(GREEN_LED, HIGH);
    delay(5000);
    digitalWrite(GREEN_LED, LOW);
  }
  return is_high_density;
}




//Debugging functions
unsigned long previousMillis = 0;
int interval = 250;
bool LED_state = LOW;
void blink(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    if (LED_state == LOW){
      LED_state = HIGH;      
    } else{
      LED_state = LOW;
    }
    digitalWrite(YELLOW_LED, LED_state);
  }  
}

