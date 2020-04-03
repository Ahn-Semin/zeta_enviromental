void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
int Rn_nSv;
int temp_celcius;
int hum_RHp;
int NO2_ppb;

void loop() {
  // put your main code here, to run repeatedly:
  String temp_str;
  int cIndex = 0;
  char NO2_rec_data[] = "1234, 5678, 9101112, 131415, 16";
  temp_str = String(NO2_rec_data);
  cIndex = temp_str.indexOf(',');
  temp_str = temp_str.substring(cIndex+1);
  cIndex = temp_str.indexOf(',');
  temp_str = temp_str.substring(0, cIndex);
  NO2_ppb = temp_str.toInt();
  Serial.println(NO2_ppb);
  delay(3000);
}
