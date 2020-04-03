#include <SoftwareSerial.h>
#include <Timer.h> // https://github.com/JChristensen/Timer
#include <mthread.h> // https://github.com/jlamothe/mthread
/* Thread class for receive UART */
class FooThread : public Thread
{
public: FooThread(int id);
protected: bool loop();
private: int id;
};
FooThread::FooThread(int id){
this->id = id;
}
/* Command */
enum {
cmd_GAMMA_RESULT_QUERY = 0x44, // D, Read measuring value(10min avg, 1min update)
cmd_GAMMA_RESULT_QUERY_1MIN = 0x4D, // M, Read measuring value(1min avg, 1min update)
cmd_GAMMA_PROC_TIME_QUERY = 0x54, // T, Read measuring time
cmd_GAMMA_MEAS_QUERY = 0x53, // S, Read status
cmd_GAMMA_FW_VERSION_QUERY = 0x46, // F, Read firmware version
cmd_GAMMA_VIB_STATUS_QUERY = 0x56, // V, Response vibration status
cmd_GAMMA_RESET = 0x52, // R, Reset
cmd_GAMMA_AUTO_SEND = 0x55, cmd_GAMMA_ALL_QUERY = 0x41, // A, Read all data
// U, Set Auto_send status
};
/* Pin map
:
Digital 2 - RX (to Gamma Sensor TX), Digital 3 - TX (to Gamma Sensor RX)
*/
SoftwareSerial mySerial(2, 3);
Timer ts1;
char rec_data[50]; // Array for received command
bool request_flag = true; // enable or disable send command automatically
void setup() {
Serial.begin(9600); // PC - Arduino
mySerial.begin(19200); // Arduino - Gamma Sensor
ts1.every(1000, RequestData); // Request to Gamma Sensor
Gamma_INIT();
}
bool FooThread::loop(){
if(id == 1) RecUartData(); // Thread 1 - for Receive Data
else ts1.update(); // Thread 2 - for Timers
return true;
}
/* Gamma Sensor Initialize */
void Gamma_INIT() {
main_thread_list->add_thread(new FooThread(1));
Read_FW(); // Read FW version
Reset(); // Reset
// Add Thread 1 for UART Response
if(request_flag) main_thread_list->add_thread(new FooThread(2));
// Add Thread 2 for Timers
Serial.println("====================================================");
}
/* Meawurement Reset */
void Reset(){
byte send_data[6] = {0x02, cmd_GAMMA_RESET, ':', '1', 0x0D, 0x0A};
mySerial.write(send_data, 6);
Serial.println("Reset.");
delay(100);
}
/* Read Firmware */
void Read_FW(){
byte send_data[6] = {0x02, cmd_GAMMA_FW_VERSION_QUERY, ':', '?', 0x0D, 0x0A};
mySerial.write(send_data, 6);
delay(100);
}
/* Read all data (automatically) */
void RequestData(){
byte send_data[6] = {0x02, cmd_GAMMA_ALL_QUERY, 0x3A, 0x3F, 0x0D, 0x0A};
mySerial.write(send_data, 6);
}
void RecUartData(){
int rec_size = mySerial.available();
if (rec_size > 0) {
for (int i = 0; i < rec_size; i++)
{
rec_data[i] = mySerial.read();
Serial.print(rec_data[i]);
}
}
}
