void setup() {
  Serial.begin(9600);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
}

void loop() {
  Serial.print("left: ");Serial.println(digitalRead(4));
  Serial.print("right: ");Serial.println(digitalRead(5));
  Serial.print("farleft: ");Serial.println(digitalRead(6));
  Serial.print("farright: ");Serial.println(digitalRead(7));
  delay(200);
}
