const int interrupt_pin_1 = 2;
const int interrupt_pin_2 = 3;
const int detector_pin = A0;
int counter_1 = 0;
int counter_2 = 0;


void setup() {
  Serial.begin(115200);
  pinMode(interrupt_pin_1, INPUT);
  pinMode(interrupt_pin_2, INPUT);
  pinMode(detector_pin, INPUT);
  attachInterrupt(interrupt_pin_1, interrupt_routine_1, CHANGE);
  attachInterrupt(interrupt_pin_2, interrupt_routine_2, CHANGE);

  //attachInterrupt(digitalPinToInterrupt(interrupt_pin_1), interrupt_routine_1, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(interrupt_pin_2), interrupt_routine_2, CHANGE);
}

void loop() {
  delay(100);
  //Serial.println(analogRead(detector_pin));
  Serial.println(counter_1);
  //Serial.println(digitalRead(interrupt_pin_1));
  Serial.println(counter_2);
  //Serial.println(digitalRead(interrupt_pin_2));
}

void interrupt_routine_1(){
  counter_1++;
}

void interrupt_routine_2(){
  counter_2++;
}