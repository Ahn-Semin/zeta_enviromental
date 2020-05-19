/* 
  *       Author         Ahn semin
  *       Created        2020.05.15
  *       Last modified  2020.05.15 18:11
  *       Description    US get distance & send to COM ROS installed.
  */
#define DEBUG               false

#include "ZETA_US.h"

void setup() {
  int i = 1;
  char temp[] = "";
  #if DEBUG
  Serial.begin(SERIAL_SPEED);
  delay(ONE_SEC);
  #endif
  rosPublishFlag = false;
  nh.getHardware()->setBaud(SERIAL_SPEED);
  nh.initNode();
  nh.advertise(US_pub);
  delay(ONE_SEC);
  US_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  US_msg.field_of_view = FOV;  // 21[deg]. The size of the arc that the distance reading is valid for.
  US_msg.min_range = MIN_DISTANCE*0.01; // cm2m
  US_msg.max_range = MAX_DISTANCE*0.01;
  US_pub.publish(&US_msg);
  nh.spinOnce();
  delay(ONE_SEC);
  main_thread_list->add_thread(new FooThread(i++));
  tTimer.every(CallbackPeriod*0.05, requestPublish);
  tTimer.every(CallbackPeriod*0.05, getDistance);
  #if DEBUG
  tTimer.every(CallbackPeriod, showDistance);
  #endif
  main_thread_list->add_thread(new FooThread(i++));
  delay(ONE_SEC*0.5);
  nh.loginfo("Arduino setting finished");
}




bool FooThread::loop() {
  switch(id) {
    case 1:
      if(( nh.connected() && rosPublishFlag )) {
        USPublish();
      } else nh.spinOnce(); // flush rosserial buffer
      break;
    case 2:
      tTimer.update();
      break;
  }
  return true;
}




void USPublish() {
  char frameid[] = "sonar0";
  String temp = "sonar0";
  for(int i = 0; i < 4; i++) {
    temp = "sonar" + String(i+1);
    temp.toCharArray(frameid,sizeof(frameid));
    US_msg.range = (float)US.distance[i]*0.01;
    US_msg.header.frame_id = frameid;
    US_pub.publish(&US_msg);
    nh.spinOnce();
    
  }
  
  rosPublishFlag = false;
}


void requestPublish() {
  rosPublishFlag = true;
}


void USDistance::getDistance() {
  for(int i = 0; i < NUM_SONAR; i++) {
    distance[i] = sonar[i].ping()*SONAR_SPEED/2;
    //if(distance[i] < MIN_DISTANCE) distance[i] = MAX_DISTANCE;
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
