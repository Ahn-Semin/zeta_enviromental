#define ADC_span                  (float)1024.0
#define ADC_ref                   1.1
#define CO_full_range             500
#define CO_Vout_span              1.6f

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReference(INTERNAL1V1);
  pinMode(A0,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t analog = 0;
  char data_str[10] = {0};
  static float pre = 0;
  float val = 0;
  //Serial.print("current voltage = ");Serial.print( (float)analog*5/1024 );Serial.println("V");
  val = ((float)analogRead(A0) * ADC_ref / ADC_span);
  //Serial.print("current voltage = ");Serial.print( );Serial.println("V");// * CO_full_range / CO_Vout_span );Serial.println("ppm");
  Serial.println(val - pre,7);
  pre = val;
  delay(1000);
}
