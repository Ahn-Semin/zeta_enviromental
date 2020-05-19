/* 
  *       Author         Ahn semin
  *       Created        2020.05.15
  *       Last modified  2020.05.15 18:11
  *       Description    US get distance & send to COM ROS installed.
  */
  
#ifndef _ZETA_US_H_
#define _ZETA_US_H_
#include <NewPing.h>
#include <Timer.h> // https://github.com/JChristensen/Timer
#include <mthread.h> // https://github.com/jlamothe/mthread
#include <ros.h>
#include <sensor_msgs/Range.h>

#define NUM_THREAD          2

#define SERIAL_SPEED        115200
#define BUFFER_SIZE         256

#define NUM_SONAR           7
#define MAX_DISTANCE        200 // cm full range 150
#define MIN_DISTANCE        3  // cm
#define FOV                 0.3665f // rad
#define SONAR_SPEED         0.0343f  // cm/us

#define CallbackPeriod      1000 // ms
#define ONE_SEC             1000 // ms

// pin configuration
const int trigPin[NUM_SONAR] = {2,3,4,5,6,7,8};
const int echoPin[NUM_SONAR] = {2,3,4,5,6,7,8};

// ros variables
ros::NodeHandle nh;
sensor_msgs::Range US_msg;
ros::Publisher US_pub("sonar", &US_msg);
bool rosPublishFlag;

// processing variables
class FooThread : public Thread{
    public: FooThread(int id);
    protected: bool loop();
    private: int id;
};

FooThread::FooThread(int id){
  this->id = id;
}

Timer tTimer;

// US variables
class USDistance {
  public:
    USDistance() {
      for(int i = 0; i < NUM_SONAR; i++) this->distance[i] = 0;
    }
    void getDistance();
    void showDistance();
    float distance[NUM_SONAR];
};

USDistance US;

NewPing sonar[NUM_SONAR] = {
  NewPing(trigPin[0],echoPin[0],MAX_DISTANCE),
  NewPing(trigPin[1],echoPin[1],MAX_DISTANCE),
  NewPing(trigPin[2],echoPin[2],MAX_DISTANCE),
  NewPing(trigPin[3],echoPin[3],MAX_DISTANCE),
  NewPing(trigPin[4],echoPin[4],MAX_DISTANCE),
  NewPing(trigPin[5],echoPin[5],MAX_DISTANCE),
  NewPing(trigPin[6],echoPin[6],MAX_DISTANCE),
};

// fucntions forward-declaration 
void showDistance();
void getDistance();
#endif
//EOF