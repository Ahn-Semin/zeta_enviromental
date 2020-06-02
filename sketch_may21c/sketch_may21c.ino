
const byte interruptPin = 2;
volatile byte state = LOW;

void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}

void loop() {
  ;
}

void blink() {
  static int cnt;
  Serial.println(cnt++);
}
