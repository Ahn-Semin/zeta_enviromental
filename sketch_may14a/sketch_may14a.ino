 /* 
  *       Author         Ahn semin
  *       Created        2020.05.15
  *       Last modified  2020.05.15 11:46
  *       Description    US get distance & send to ROS installed COM.
  */
  
#include <NewPing.h>
#include <Timer.h> // https://github.com/JChristensen/Timer
#include <mthread.h> // https://github.com/jlamothe/mthread
#include <ros.h>
#include <std_msgs/String.h>

#define DEBUG               1
#define Number_of_Thread    0  //

#define BUFFER_SIZE         255

#define MAX_DISTANCE        80 // cm
#define MIN_DISTANCE        3  // cm
#define SERIAL_SPEED        115200
#define NUM_SONAR           7

#define CallbackPeriod      1000 // ms
#define ONE_SEC             1000 // ms
class FooThread : public Thread{
    public: FooThread(int id);
    protected: bool loop();
    private: int id;
};

// ros variables
ros::NodeHandle nh;
std_msgs::String US_msg;
ros::Publisher US_pub("US", &US_msg);
bool rosPublishFlag = false;


// 
class USDistance {
  public:
    USDistance() {
      for(int i = 0; i < NUM_SONAR; i++) this->distance[i] = 0;
    }
    void getDistance();
    void showDistance();
    uint32_t distance[NUM_SONAR];
};


Timer tTimer;

USDistance US;
const int trigPin[NUM_SONAR] = {2,3,4,5,6,7,8};
const int echoPin[NUM_SONAR] = {2,3,4,5,6,7,8};

NewPing sonar[NUM_SONAR] = {
  NewPing(trigPin[0],echoPin[0],MAX_DISTANCE),
  NewPing(trigPin[1],echoPin[1],MAX_DISTANCE),
  NewPing(trigPin[2],echoPin[2],MAX_DISTANCE),
  NewPing(trigPin[3],echoPin[3],MAX_DISTANCE),
  NewPing(trigPin[4],echoPin[4],MAX_DISTANCE),
  NewPing(trigPin[5],echoPin[5],MAX_DISTANCE),
  NewPing(trigPin[6],echoPin[6],MAX_DISTANCE),
};

void showDistance();
void getDistance();


void setup() {
  int i = 1;
  char temp[] = "";
  #if DEBUG
  Serial.begin(SERIAL_SPEED);
  delay(ONE_SEC);
  #endif
  nh.getHardware()->setBaud(SERIAL_SPEED);
  nh.initNode();
  nh.advertise(US_pub);
  delay(ONE_SEC);
  US_msg.data = temp;
  US_pub.publish(&US_msg);
  nh.spinOnce();
  delay(ONE_SEC);
  main_thread_list->add_thread(new FooThread(i++));
  tTimer.every(CallbackPeriod*0.1, requestPublish);
  tTimer.every(CallbackPeriod*0.1, getDistance);
  tTimer.every(CallbackPeriod, showDistance);
  main_thread_list->add_thread(new FooThread(i++));
  delay(ONE_SEC*0.5);
  nh.loginfo("arduino setting finish");
}





bool FooThread::loop() {
  switch(id) {
    case 1:
      if(( nh.connected() && rosPublishFlag )) {
        USPublish();
      } else nh.spinOnce();
      break;
    case 2:
      tTimer.update();
      break;
  }
  return true;
}


void USPublish() {
  char send_msg[BUFFER_SIZE] = "";
  sprintf(send_msg,"%lu|%lu|%lu|%lu|%lu|%lu|%lu|",
  US.distance[0], US.distance[1], US.distance[2], US.distance[3],
  US.distance[4], US.distance[5], US.distance[6]);
  nh.loginfo(send_msg);
  US_msg.data = send_msg;
  US_pub.publish(&US_msg);
  nh.spinOnce();
  US_msg.data = "";
  rosPublishFlag = false;
}


void requestPublish() {
  rosPublishFlag = true;
}

FooThread::FooThread(int id){
  this->id = id;
}


void USDistance::getDistance() {
  for(int i = 0; i < NUM_SONAR; i++) {
    distance[i] = sonar[i].ping_cm();
    if(distance[i] < MIN_DISTANCE) distance[i] = MAX_DISTANCE;
  }
}
void USDistance::showDistance() {
  Serial.println("");
  for(int i = 0; i < NUM_SONAR; i++) {
    Serial.print("#");Serial.print(i+1);Serial.print(": ");Serial.print(US.distance[i]);Serial.println("[cm]");
  }
}

void getDistance() {
  US.getDistance();
}
void showDistance() {
  US.showDistance();
}
// EOF
