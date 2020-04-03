#include <SPI.h>
#include <WiFiNINA.h>

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "zetaA";       // WiFi의 SSID 입니다.
char pass[] = "12345678"; // 비번입니다.
int status = WL_IDLE_STATUS;     // the Wifi radio's status
//int keyIndex = 0; 


const char* server = "15.164.221.85"; //Zeta server
const int port = 5543;

WiFiClient client;
void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    Serial.println(status);
    // wait 10 seconds for connection:
    delay(5000);
  }

  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");
  
}

void loop() {

WiFiClient client;
  char data_str[]="{\"command\":\"SENSOR_DATA\", \"sid\":\"XefiGyJeTYMpWNNSViBJW6pSI3yp7IKs\", \"data\":0000}";
  const int str_len = sizeof(data_str);
  char data[] = "1234";
  const int data_len = sizeof(data)-1;
  for(int i=data_len;i>0;i--){
    data_str[str_len-i-2] = data[data_len-i];
  }
  //서버의 주소와 포트번호를 적어준다.

  if (!client.connect(server, port)) {
    Serial.println("connection failed");//서버 접속에 실패
    
    return;
  }

  else{
    //서버로 보낼 정보를 구성해 보내고 받는다.
   
     
     client.write(data_str);

     String recevbline = client.readStringUntil('\r');
     Serial.println(recevbline);
  }

  //client.stop();
  Serial.println("Waiting…");
  client.flush();
  client.stop();
  delay(3000);  // 
}
