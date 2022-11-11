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

#define LEFT_LINE A0
#define RIGHT_LINE A1
#define MID_LINE A2
#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3

//+++DEFINING MOTORSHIELD+++//
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select motor ports M1 and M2
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);

class EncoderControl
{
  public:
    EncoderControl(Adafruit_DCMotor* m_left, Adafruit_DCMotor* m_right);
    void begin(int left_encoder_pin, int right_encoder_pin);
    void move(int distance);
    void pivot(int angle);
    void encoder_pid_drive();
    void encoder_pid_update();
    static void left_interrupt_routine();
    static void right_interrupt_routine();
    static int get_left_ticks();
    static int get_right_ticks();
    int get_left_speed();
    int get_right_speed();

  private:
    Adafruit_DCMotor* left_motor;
    Adafruit_DCMotor* right_motor;
    const int Kp_left = 10;
    const int Ki_left = 0.1;
    const int Kd_left = -0.1;
    const int Kp_right = 3;
    const int Ki_right = 0.1;
    const int Kd_right = -0.1;
    const int wheel_rad = 34.75;
    const int wheel_sep = 238;
    const int threshold = 5;
    int target_ticks;
    //TICKS
    static int left_ticks;
    static int right_ticks;
    //PREVIOUS VAL
    int left_previous;
    int right_previous;
    //INTEGRAL VAL
    int left_integral;
    int right_integral;
    //ACTIVE FLAG
    bool left_pid_active;
    bool right_pid_active;
    //MOTOR SPEEDS
    int left_motor_speed;
    int right_motor_speed;
};

int EncoderControl::left_ticks=0;
int EncoderControl::right_ticks=0;

EncoderControl::EncoderControl(Adafruit_DCMotor* m_left, Adafruit_DCMotor* m_right){
  left_motor = m_left;
  right_motor = m_right;
}

void EncoderControl::begin(int left_encoder_pin, int right_encoder_pin){
  pinMode(left_encoder_pin, INPUT);
  pinMode(right_encoder_pin, INPUT);
  attachInterrupt(left_encoder_pin, left_interrupt_routine, CHANGE);
  attachInterrupt(right_encoder_pin, right_interrupt_routine, CHANGE);
}

void EncoderControl::move(int distance){ //units in mm
  target_ticks = abs((int)distance/(2*PI*wheel_rad)*96);
  left_ticks = 0;
  right_ticks = 0;
  Serial.print("Target Ticks: ");Serial.println(target_ticks);
  left_motor->setSpeed(0);
  right_motor->setSpeed(0);
  if (distance > 0){
    left_motor->run(FORWARD);
    right_motor->run(FORWARD);
  } else {
    left_motor->run(BACKWARD);
    right_motor->run(BACKWARD);
  }
  left_pid_active = true;
  right_pid_active = true;
}

void EncoderControl::pivot(int angle){ //units in 0.1deg
  target_ticks = abs((int)(angle * wheel_sep * PI / 360)/(2*PI*wheel_rad)*96);
  left_ticks = 0;
  right_ticks = 0;
  Serial.print("Target Ticks: ");Serial.println(target_ticks);
  LeftMotor->setSpeed(0);
  RightMotor->setSpeed(0);
  if (angle > 0) {
    LeftMotor->run(FORWARD);
    RightMotor->run(BACKWARD);
  } else {
    LeftMotor->run(BACKWARD);
    RightMotor->run(FORWARD);
  }
  left_pid_active = true;
  right_pid_active = true;
}

void EncoderControl::encoder_pid_drive(){
  left_motor->setSpeed(left_motor_speed);
  right_motor->setSpeed(right_motor_speed);
  return;
}

void EncoderControl::encoder_pid_update(){
  if (!left_pid_active && !right_pid_active){
    Serial.println("inactive");
    target_ticks = 0;
    //right_motor_speed = 0;
    //left_motor_speed = 0;
    return;
  }
  // error between current ticks and target ticks
  int left_error = target_ticks - left_ticks;
  int right_error = target_ticks - right_ticks;

  if (left_error < threshold||!left_pid_active){
    left_ticks = 0;
    left_motor_speed=0;
    left_pid_active = false;
    Serial.println("left inactive");
  } else {
    // trapezoidal integration
    int left_integral = left_integral+(left_previous+left_error)/2; 
    // instantaneous gradient
    int left_derivative = left_previous-left_error; 
    // PID output
    uint32_t output = (uint32_t) 30 + Kp_left*left_error+Ki_left*left_integral+Kd_left*left_derivative;
    left_motor_speed = constrain(output, 30, 250);    
  }

  if (right_error < threshold || !right_pid_active){
    right_ticks = 0;
    right_motor_speed=0;
    right_pid_active = false;
    Serial.println("right inactive");
  } else {
    // trapezoidal integration
    int right_integral = right_integral+(right_previous+right_error)/2; 
    // instantaneous gradient
    int right_derivative = right_previous-right_error; 
    // PID output
    uint32_t output = (uint32_t) 30 + Kp_right*right_error+Ki_right*right_integral+Kd_right*right_derivative;
    right_motor_speed = constrain(output, 30, 250);
  }
  
  left_previous = left_error;
  right_previous = right_error;
  return;
}

static void EncoderControl::left_interrupt_routine(){
  left_ticks++;
}

static void EncoderControl::right_interrupt_routine(){
  right_ticks++;
}

static int EncoderControl::get_left_ticks(){
  return left_ticks;
}

static int EncoderControl::get_right_ticks(){
  return right_ticks;
}

int EncoderControl::get_left_speed(){
  return left_motor_speed;
}

int EncoderControl::get_right_speed(){
  return right_motor_speed;
}

EncoderControl encoder_control = EncoderControl(LeftMotor, RightMotor);





void setup() {
  Serial.begin(115200);

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

  //+++EncoderControl+++//
  encoder_control.begin(LEFT_ENCODER, RIGHT_ENCODER);
  encoder_control.pivot(900);
}

void loop() {
  delay(100);
  encoder_control.encoder_pid_drive();
  Serial.print("Left Ticks: ");Serial.println(encoder_control.get_left_ticks());
  Serial.print("Right Ticks: ");Serial.println(encoder_control.get_right_ticks());
  Serial.print("Left Speed: ");Serial.println(encoder_control.get_left_speed());
  Serial.print("Right Speed: ");Serial.println(encoder_control.get_right_speed());
}

void timer_interrupt_routine(){
    encoder_control.encoder_pid_update();
}