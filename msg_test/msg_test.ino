void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  char data_str[]="{\"command\":\"SENSOR_DATA\", \"sid\":\"XefiGyJeTYMpWNNSViBJW6pSI3yp7IKs\", \"data\":0000}";
  const int str_len = sizeof(data_str);
  char data[] = "1234";
  const int data_len = sizeof(data)-1;
  for(int i=data_len;i>0;i--){
    data_str[str_len-i-2] = data[data_len-i];
  }
  Serial.println(data_str);
  Serial.println(sizeof(data_str));
  delay(3000);
}
