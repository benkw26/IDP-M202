const int interrupt_pin = 2;
const int detector_pin = A0;
int counter = 0;

void setup() {
  Serial.begin(115200);
  pinMode(interrupt_pin, INPUT);
  pinMode(detector_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), interrupt_routine, CHANGE);
}

void loop() {
  delay(100);
  //Serial.println(analogRead(detector_pin));
  Serial.println(counter);
  Serial.println(digitalRead(interrupt_pin));
}

void interrupt_routine(){
  counter++;
}