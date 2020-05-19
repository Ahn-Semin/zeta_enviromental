 /* 
  *       Author         Ahn semin
  *       Created        2020.04.09
  *       Last modified  2020.04.09
  *       Description    RS485 to ELD2 servo motor drive
  */

#include <CRC16_modbus.h>

// pin config
#define SerialTxControl 10   //RS485 제어 접점 핀 arduino pin 10
#define RS485Transmit    HIGH
#define RS485Receive     LOW
#define RS485_baudrate   38400

// Command
#define CMD_ELD2_SlaveID          0x10
#define CMD_FC_Send1Word          0x06
#define CMD_FC_SendNWord          0x10
  // PR trigger command
#define CMD_PRT_AddressHigh       0x60  // PR trigger address
#define CMD_PRT_AddressLow        0x02
#define CMD_PRT_HighZero          0x00
enum PRT_POSITION{
  CMD_PRT_P0                    = 0x10,
  CMD_PRT_P1,
  CMD_PRT_P2,
  CMD_PRT_P3,
  CMD_PRT_P4,
  CMD_PRT_P5,
  CMD_PRT_P6,
  CMD_PRT_P7,
};
#define CMD_PRT_Homing            0x20
#define CMD_PRT_EStop             0x40

#define CMD_PRn_AddressStart      0x62  // PR9.00 = 0x6200
#define CMD_PRn_Address00         0x00
#define PRn_AddressOffset         0x08


HardwareSerial* RS485_Serial = &Serial1;
uint8_t send_buffer[8] = {CMD_ELD2_SlaveID, CMD_FC_Send1Word, CMD_PRT_AddressHigh,
                    CMD_PRT_AddressLow, CMD_PRT_HighZero, 0x00,};
uint8_t buffer_sub[6] = {CMD_ELD2_SlaveID, CMD_FC_Send1Word, CMD_PRT_AddressHigh,
                    CMD_PRT_AddressLow, CMD_PRT_HighZero, 0x00};
union {
  uint16_t  CRC_code;
  uint8_t   CRC_byte[2];
  }CRC;

int CRC_code = 0;
void setup(void) {
  Serial.begin(9600);
  Serial1.begin(RS485_baudrate,8E1);
  pinMode(SerialTxControl, OUTPUT);  
  delay(100);
  digitalWrite(SerialTxControl, RS485Transmit);  
}

void loop(void) {
  send_buffer[5] = CMD_PRT_P0;
  buffer_sub[5] = CMD_PRT_P0;
  CRC.CRC_code = CRC16_MODBUS(buffer_sub,sizeof(buffer_sub));
  send_buffer[6] = CRC.CRC_byte[0];
  send_buffer[7] = CRC.CRC_byte[1];
  for(int i = 0; i<8; i++){
    Serial.print(send_buffer[i],HEX); Serial.print(" ");
  }
  Serial.println("");

  Serial1.write(send_buffer,8);
  delay(1000);
  send_buffer[5] = CMD_PRT_P1;
  buffer_sub[5] = CMD_PRT_P1;
  CRC.CRC_code = CRC16_MODBUS(buffer_sub,sizeof(buffer_sub));
  send_buffer[6] = CRC.CRC_byte[0];
  send_buffer[7] = CRC.CRC_byte[1];
  Serial1.write(send_buffer,8);
  for(int i = 0; i<8; i++){
    Serial.print(send_buffer[i],HEX); Serial.print(" ");
  }
  Serial.println("");
  delay(1000);
  
}

  
