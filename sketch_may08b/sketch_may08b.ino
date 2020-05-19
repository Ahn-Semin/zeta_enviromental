
#include <ArduinoBLE.h>

 // BLE Battery Service
BLEService batteryService("180F");

// BLE Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",BLERead | BLENotify); 

int oldBatteryLevel = 0;  
long previousMillis = 0;  

void setup() {
  Serial.begin(9600);    
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); 

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  BLE.setLocalName("BatteryMonitor");
  BLE.setAdvertisedService(batteryService);
  batteryService.addCharacteristic(batteryLevelChar); 
  BLE.addService(batteryService); 
  batteryLevelChar.writeValue(oldBatteryLevel); 
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateBatteryLevel();
      }
    }
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateBatteryLevel() {

  int battery = analogRead(A0);
  int batteryLevel = map(battery, 0, 1023, 0, 100);

  if (batteryLevel != oldBatteryLevel) {    
    Serial.print("Battery Level % is now: "); 
    Serial.println(batteryLevel);
    batteryLevelChar.writeValue(batteryLevel); 
    oldBatteryLevel = batteryLevel;          
  }
}
