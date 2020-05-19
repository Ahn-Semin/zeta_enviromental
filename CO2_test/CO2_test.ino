//sensor vars
uint32_t CO2 = 0;
char rxByte[11];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(38400,SERIAL_8N1);  //38400 baud rate, 8bit, no parity bit, 1 stop bit
  Serial.println("setup finish");
}

void loop() {
  if(Serial1.available()){
    int no =  Serial1.readBytes(rxByte,10);
    //rxByte[no] = '\0';
    //CO2 = (uint32_t)rxByte;
    CO2 = Byte2int(rxByte);
    Serial.print("current CO2 :");Serial.println(CO2);
  }

}

uint32_t Byte2int(char* Byte){
  char temp[4];
  for(int i=0;i<4;i++){
    temp[i] = Byte[i];
  }
  return atoi(temp);
}
