#define lefts A0
#define middle A1
#define rights A2
void setup() {
  Serial.begin(9600);
  pinMode(lefts, INPUT);
  pinMode(middle, INPUT);
  pinMode(rights, INPUT);
}

void loop() {
  Serial.print("LEFTS: ");
  Serial.println(analogRead(lefts));
  Serial.print("MIDDLE: ");
  Serial.println(analogRead(middle));
  Serial.print("RIGHTS: ");
  Serial.println(analogRead(rights));
  delay(200);
}
