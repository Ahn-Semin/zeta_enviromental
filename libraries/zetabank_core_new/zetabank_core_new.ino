/*******************************************************************************
  Copyright 2018 SEOULTECH CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Seo, Jae Yong */

#include "zetabank_core_config_new.h"

/*******************************************************************************
  Setup function
*******************************************************************************/

void setup()
{
    
  // Setting ultrasonic sensor distance
  if(ultra_set)
    ultrasonic_dist_setting();
    
  ///// Init motor
  init_MotorController();    

  // Init LED
  init_led();


  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(teleop_sub);
  nh.subscribe(led_sub);
  nh.subscribe(reset_sub);
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(sonar_pub);
  nh.advertise(sensor_state_pub);
  
  // Debugging
  nh.advertise(right_vel_pub);
  nh.advertise(left_vel_pub);
  nh.advertise(debug_pub);
  /////

  nh.advertise(pub_rx);

  tf_broadcaster.init(nh);

  // Reset velocity
  resetGoalVelocity();
  
  // Setting for ultrasonic sensors
  ultrasonic_setup();

  // Setting for IMU
  sensors.init();

  // Initialization for LED, voltage check, Button in OpenCR, which the sources exist in the zetabank_ros_lib
  diagnosis.init();

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  SerialBT2.begin(57600);

  prev_update_time = millis();
  
  // For voltage check
  vel_flg = true;
  teleop_flg = false;

  
}

/*******************************************************************************
  Loop function
*******************************************************************************/
void loop()
{
  uint32_t t = millis();

  // if the connection of ROS communication with PC is disconnected, it will skip.
  if (nh.connected() )
  {
    updateTime();
    updateVariable();

    if ((t - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))  
    {
      // Subscribe cmd_vel and control motors
      updateGoalVelocity();      
      check_vel_safety(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
      controlMotor(WHEEL_SEPARATION, goal_velocity);

      tTime[0] = t;
    }
    if ((t - tTime[1]) >= (1000 / CMD_VEL_PUBLISH_PERIOD))
    {
      // subscribe sensors embedded in OpenCR and update odometry infomation.
      publishSensorStateMsg();
      tTime[1] = t;
    }
    if ((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
    {
      
       get_motor_velocity_data();
      //get_motor_position_data();
      receive_motor_data();  
      publishDriveInformation();
                
      
      tTime[2] = t;
    }
    if ((t - tTime[3]) >= (1000 / IMU_SONAR_PUBLISH_PERIOD))
    {
      //update sonar
      if(!ultra_set)
      {
        ultrasonic_check();     
      }
        
      publishImuMsg();
      tTime[3] = t;
    }
    if ((t - tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_PERIOD))
    {
      publishVersionInfoMsg();
      publishBatteryStateMsg();
      teleop_flg = false;
        
      // Show LED status
      diagnosis.showLedStatus(nh.connected());

      tTime[4] = t;
    }

    // Update the IMU unit
    sensors.updateIMU();

    // Start Gyro Calibration after ROS connection
    updateGyroCali();
    
    // Send log message after ROS connection
    sendLogMsg();

  }
  else
  {
    reset_cnt ++;
    
    network_disconnect();
    digitalWrite(BDPIN_LED_USER_1, LOW);
    digitalWrite(BDPIN_LED_USER_2, LOW);
    digitalWrite(BDPIN_LED_USER_3, LOW);
    digitalWrite(BDPIN_LED_USER_4, LOW);
    delay(100);                   // Wait for 0.1 second
    digitalWrite(BDPIN_LED_USER_1, HIGH);
    digitalWrite(BDPIN_LED_USER_2, HIGH);
    digitalWrite(BDPIN_LED_USER_3, HIGH);
    digitalWrite(BDPIN_LED_USER_4, HIGH);
    delay(100);                   // Wait for 0.1 second
    
    if(log_flag == true && reset_cnt > 10)
      NVIC_SystemReset();     


  }
  
  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();
  
  // give the serial link time to process
  delay(10);
    
}


/*******************************************************************************
  Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist & cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(
                                      goal_velocity_from_cmd[LINEAR],
                                      (-1) * MAX_LINEAR_VELOCITY,
                                      MAX_LINEAR_VELOCITY
                                    );
  goal_velocity_from_cmd[ANGULAR] = constrain(
                                      goal_velocity_from_cmd[ANGULAR],
                                      (-1) * MAX_ANGULAR_VELOCITY,
                                      MAX_ANGULAR_VELOCITY
                                    );
}

/*******************************************************************************
  Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty & reset_msg)
{
  char log_msg[50];

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);
}

/*******************************************************************************
  Callback function for teleoperation msg
*******************************************************************************/
void teleopCallback(const std_msgs::Bool& bool_msg)
{
  if(bool_msg.data == true)
  {
    teleop_flg = true;
  }
  else
  {
    teleop_flg = false;
  }
}

/*******************************************************************************
  Callback function for LED msg
*******************************************************************************/
void init_led()
{
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
 
}

void ledCallback(const std_msgs::String& str_msg)
{
  const char* total_led = str_msg.data;
  
  static int16_t pwm_out = 0;
  static bool pwm_start_flg = true;
  static bool pwm_status_flg = true;

  //front = total_led[0], rear_bottom= total_led[1], rear_middle= total_led[2], rear_top= total_led[3]
  if( total_led[0] == '1')
  { 
    if(pwm_status_flg == false)
         pwm_start_flg = true;
    
    
    if(pwm_start_flg == true)
    {
      pwm_out = 0;
      pwm_start_flg = false;
    }  

    if(pwm_out > 150)
      pwm_out = pwm_out + 40;
    else 
      pwm_out = pwm_out + 1;
      
    if(pwm_out >= 255)
      pwm_out =255;    
      
      
    analogWrite(9, pwm_out);
    pwm_status_flg = true;
  }
  else
  {
    if(pwm_status_flg == true)
         pwm_start_flg = true;
         
    if(pwm_start_flg == true)
    {
      pwm_out = 255;
      pwm_start_flg = false;
    }  
      
    if(pwm_out > 150)
      pwm_out = pwm_out - 40;
    else 
      pwm_out = pwm_out - 1;
      
    if(pwm_out <= 0)
      pwm_out =0;  
      
    analogWrite(9, pwm_out);
    pwm_status_flg = false;
  }

  digitalWrite(13, LOW);
  
  // rear_bottom, strange
  if( total_led[1] == '1')
  {    
    digitalWrite(12, HIGH);
  }
  else
  {
    digitalWrite(12, LOW);    
  }

  //rear_middle
  if( total_led[2] == '1')
  {    
    digitalWrite(11, HIGH);
  }
  else
  {
    digitalWrite(11, LOW);
  }

  //rear_top
  if( total_led[3] == '1')
  {    
    digitalWrite(10, HIGH);
  }
  else
  {
    digitalWrite(10, LOW);
  }
}



/*******************************************************************************
  Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg                 = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = "imu_link";

  imu_pub.publish(& imu_msg);
}

/*******************************************************************************
  Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  bool dxl_comm_result          = false;

  sensor_state_msg.header.stamp = rosNow();
  sensor_state_msg.battery      = battery_state_msg.voltage;
  
  // Accumulate pulses for pose of odometry 
  get_position();
    
  left_relative_pos_motor = relative_position(left_motor_pos, left_pre_motor_pos);
  right_relative_pos_motor = relative_position(right_motor_pos, right_pre_motor_pos);

  left_pre_motor_pos = (int)(left_motor_pos*1000); //이전 모터 위치 설정
  right_pre_motor_pos = (int)(right_motor_pos*1000);

  
  updateMotorInfo(left_relative_pos_motor, right_relative_pos_motor);

  sensor_state_msg.left_encoder  = left_motor_pos;
  sensor_state_msg.right_encoder = left_motor_pos;

  sensor_state_msg.button        = sensors.checkPushButton();
  //sensor_state_msg.torque = motor_driver.getTorque();
  sensor_state_pub.publish(& sensor_state_msg);
}


/*******************************************************************************
  Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = HARDWARE_VER;
  version_info_msg.software = SOFTWARE_VER;
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(& version_info_msg);
}

/*******************************************************************************
  Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp    = rosNow();
  battery_state_msg.capacity = analogRead(A1); 
  battery_state_msg.design_capacity = analogRead(A1)*0.004882813f; 
  battery_state_msg.voltage         = battery_state_msg.design_capacity * 5.745745746f; //battery volatege
  battery_state_msg.percentage      = (float)(battery_state_msg.voltage * 3.40);


  battery_state_pub.publish(& battery_state_msg);
}


/*******************************************************************************
  Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now  = millis();
  unsigned long step_time = time_now - prev_update_time; // dimension = [msec]

  prev_update_time = time_now;
  ros::Time stamp_now     = rosNow();


   // Get velocity
   get_velocity(); 
      
  // calculate odometry
  calcOdometry((double)(step_time * 0.001)); // dimension = [sec]

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(& odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(& joint_states);


}

/*******************************************************************************
  Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id       = "odom";
  odom.child_frame_id        = "base_link";

  odom.pose.pose.position.x  = odom_pose[0];
  odom.pose.pose.position.y  = odom_pose[1];
  odom.pose.pose.position.z  = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

}


/*******************************************************************************
  Update variable (initialization)
*******************************************************************************/
void updateVariable(void)
{
  static bool variable_flag = false;

  if (nh.connected())
  {
    if (variable_flag == false)
    {
      sensors.initIMU();
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
  Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = micros();
  current_time   = nh.now();
}

/*******************************************************************************
  ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return addMicros(current_time, micros() - current_offset);
}

/*******************************************************************************
  Time Interpolation function
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec,
           nsec;

  sec  = _micros / 1000000 + t.sec;
  nsec = _micros % 1000000 + 1000 * (t.nsec / 1000);

  if (nsec >= 1e9) {
    sec++,
        nsec--;
  }
  return ros::Time(sec, nsec);
}

/*******************************************************************************
  Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(void)
{
  static bool isEnded = false;
  char log_msg[50];

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x    = 0.0;
  odom.pose.pose.position.y    = 0.0;
  odom.pose.pose.position.z    = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x    = 0.0;
  odom.twist.twist.angular.z   = 0.0;
}

void initJointStates(void)
{
  static char * joint_states_name[] =
  {
    "wheel_left_joint",
    "wheel_right_joint"
  };

  joint_states.header.frame_id      = "base_link";
  joint_states.name                 = joint_states_name;

  joint_states.name_length          = WHEEL_NUM;
  joint_states.position_length      = WHEEL_NUM;
  joint_states.velocity_length      = WHEEL_NUM;
  joint_states.effort_length        = WHEEL_NUM;
}

/*******************************************************************************
  Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{

  int32_t current_tick                = 0;
  static int32_t last_tick[WHEEL_NUM] = { 0.0, 0.0};

  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      g_last_diff_tick[index] = 0.0;
      last_tick[index]        = 0.0;
      g_last_rad[index]       = 0.0;

      last_velocity[index]    = 0.0;
    }

    last_tick[LEFT]  = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder     = false;
    return;
  }

  g_last_diff_tick[LEFT] = left_tick; // difference of encoder pulses
  g_last_rad[LEFT]       += PULSE2RAD * (double)g_last_diff_tick[LEFT]; // translate the number of conunt to radian


  g_last_diff_tick[RIGHT] = right_tick; // difference of encoder pulses
  g_last_rad[RIGHT]       += PULSE2RAD * (double)g_last_diff_tick[RIGHT]; // translate the number of conunt to radian

}
/*******************************************************************************
  Update the joint states
*******************************************************************************/
void updateJointStates(void)
{
  joint_states_pos[LEFT]                   = g_last_rad[LEFT];
  joint_states_pos[RIGHT]                  = g_last_rad[RIGHT];

  joint_states_vel[LEFT]                   = last_velocity[LEFT];
  joint_states_vel[RIGHT]                  = last_velocity[RIGHT];

  joint_states.position                    = joint_states_pos;
  joint_states.velocity                    = joint_states_vel;
  joint_states.effort                      = joint_states_eff;
}

/*******************************************************************************
  CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped & odom_tf)
{
  odom_tf.header                  = odom.header;
  odom_tf.child_frame_id          = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
  Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float * orientation;
  float wheel_vel_l, wheel_vel_r;
  static double last_theta = 0.0;
  
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s,
         theta,
         delta_theta;
  float v, w; // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

    
  wheel_vel_l = -left_motor_vel;
  wheel_vel_r = right_motor_vel;

  wheel_l   = wheel_r = 0.0;
  delta_s   = delta_theta = theta = 0.0;
  v         = w = 0.0;
  step_time = 0.0;

  step_time = diff_time; // dimension : [sec]

  if (step_time == 0)
    return false;


  wheel_l = PULSE2RAD * (double)g_last_diff_tick[LEFT];
  wheel_r = PULSE2RAD * (double)g_last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;
  if (isnan(wheel_r))
    wheel_r = 0.0;


  // Get theta info using Imu sensor
  orientation          = sensors.getOrientation(); // IMU
  theta                = atan2f(
    orientation[1] * orientation[2] + orientation[0] * orientation[3],
    0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3]
  );
    
    
 /*
  delta_s              = WHEEL_RADIUS * (wheel_r + wheel_l) * 0.5;
  theta                = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  delta_theta          = theta - last_theta;

  */

  v                    = (wheel_vel_l + wheel_vel_r) / 2.0;
  w                    = get_angle_vel(wheel_vel_l, wheel_vel_r);

  last_velocity[LEFT]  = wheel_vel_l;
  last_velocity[RIGHT] = wheel_vel_r;

  delta_s              = (diff_time * v) ;
  delta_theta          = theta - last_theta;

  
  // compute odometric pose
  odom_pose[0]         += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1]         += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2]         += delta_theta;

    
  // compute odometric instantaneouse velocity
  odom_vel[0]          = v;
  odom_vel[1]          = 0.0;
  odom_vel[2]          = w;

  last_theta           = theta;

  return true;
}

/*******************************************************************************
  Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  // Recieve goal velocity through ros messages
  goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];

}

void check_vel_safety(float trans_velocity, float angular_velocity)
{
  if(teleop_flg == true)
  {
    return ;
  }

  vel_total_same_cnt++;
  
  if( pre_velocity[LINEAR] == trans_velocity &&  pre_velocity[ANGULAR] == angular_velocity)
  {
    vel_same_cnt++;
  }

  if(vel_same_cnt >= 500 && vel_total_same_cnt >= 500)
  {
      char log_msg[10];
      goal_velocity_from_cmd[LINEAR] = 0;
      goal_velocity_from_cmd[ANGULAR] = 0;            
      writeVelocity(0, 0);   
      sprintf(log_msg, "Same_vel");
      nh.loginfo(log_msg);       
      vel_same_cnt = 0;
  }
  else if(vel_total_same_cnt >= 500)
  {
    //For clear the case of having same velocity
    vel_same_cnt = 0;
    vel_total_same_cnt = 0;
  }
  
  pre_velocity[LINEAR]=trans_velocity;
  pre_velocity[ANGULAR]=angular_velocity;
  
}
/*******************************************************************************
  Zetabank
*******************************************************************************/
/*******************************************************************************
  Motor control
*******************************************************************************/
bool controlMotor(const float wheel_separation, float * value)
{

  float wheel_velocity_cmd[2];

  float lin_vel             = value[LINEAR];
  float ang_vel             = value[ANGULAR];

  wheel_velocity_cmd[LEFT]  = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT] = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT], -LINEAR_X_MAX_VELOCITY, LINEAR_X_MAX_VELOCITY);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT], -LINEAR_X_MAX_VELOCITY, LINEAR_X_MAX_VELOCITY);


  writeVelocity((float)wheel_velocity_cmd[LEFT], (float)wheel_velocity_cmd[RIGHT]);

  return true;
}

bool writeVelocity(float left_vel, float right_vel)
{
  float vel[2] =
  {
    left_vel,
    right_vel
  };

  left_vel = -26 * 117.602174 * left_vel;
  right_vel = 26 * 117.602174 * right_vel;

  left_vel_str = String((int)left_vel);
  right_vel_str = String((int)right_vel);
  drv_dxl_tx_enable(TRUE);

  DXL_PORT.write(0x01);
  DXL_PORT.print("SV");
  DXL_PORT.print(left_vel_str);
  DXL_PORT.print(",");
  DXL_PORT.print(right_vel_str);
  DXL_PORT.print("!");
  drv_dxl_tx_enable(FALSE);

}

/*******************************************************************************
  Setting for Ultrasonic sensor
*******************************************************************************/

void ultratimer_tick()
{
  // Set flag to true at each SET_RATE 
  ultra_set_tick=true; 
}

void ultrasonic_setup()
{
  sonar_cnt = 0;
  
  for(int i=0;i<sizeof(echoPin);i++)
  { 
    pinMode(echoPin[i], INPUT);
  }
  
  if(ultra_set)
  {
    Timer.stop(); 
    Timer.setPeriod(SET_RATE); 
    Timer.attachInterrupt(ultratimer_tick); 
    pinMode(setPin,OUTPUT); 
    digitalWrite(setPin, HIGH);  
    delay(1000);
    Timer.start(); 
  }
}

void ultrasonic_pul_cmd()  
{  
  switch (setCount)  
  {  
    case 0 : digitalWrite(setPin, LOW);  // start bit
             break;   
    case 1:  
    case 2: 
    case 3 : 
    case 4 : 
    case 5 : 
    case 6 : 
    case 7 :  
    case 8 : digitalWrite(setPin, setting_50cm[setCount-1]);  
             break; 
    case 9 : digitalWrite(setPin, HIGH);  // end bit
             break;   
    default: digitalWrite(setPin, HIGH);  
             break;  
  }
}  

void ultrasonic_dist_setting()
{
  if(ultra_set_tick)  
  {  
    ultrasonic_pul_cmd();  
    setCount++;
    ultra_set_tick=false; 
  }  
  
  if(setCount > 100)  
  {  
    ultra_set=false;
    digitalWrite(setPin,LOW);
    Timer.stop();
    delay(1000);
  }
}

void ultrasonic_check()
{
    String str="";  
    char bin[8] = "";  

    for(int i = 0; i < sizeof(echoPin);i++)
    { 
      if(digitalRead(echoPin[i]) == LOW)  //{'0':'bl', '2':'l', '3':'fl','4':'f',5':'fr' ,6':'r','7':'br'}
        str = "1" + str; 
      else
        str = "0" + str; 
    } 
    
    str.toCharArray(bin,8); 
    sonar_msg.data = bin;         //echo는 10진수, bin은 2진수문자열
    sonar_pub.publish(&sonar_msg);

    //sonar_cnt++;
    //if(teleop_flg == false)
    //  ultrasonic_safe(bin, sonar_cnt);  
       
    //if(sonar_cnt >= SONAR_SAFETY_CNT)
    //  sonar_cnt = 0;
}

void ultrasonic_safe(char bin[8], unsigned int sonar_cnt)
{
  char buffer[50];
    
  for(int i = 0; i < 7 ;i++)
  {
    if(bin[i] == '1')
    {
      danger_cnt[i] += 1;
    }
  }

  if(sonar_cnt >= SONAR_SAFETY_CNT)
  {
    if( abs(goal_velocity[LINEAR]) > 0.5 ||  abs(goal_velocity[ANGULAR]) > 0.5 )
    {       
        char log_msg[20];
       if(danger_cnt[2] >= SONAR_SAFETY_CNT*0.3 || danger_cnt[3] >= SONAR_SAFETY_CNT*0.3 || danger_cnt[4] >= SONAR_SAFETY_CNT*0.3)
       {
        goal_velocity_from_cmd[LINEAR] = 0;
        goal_velocity_from_cmd[ANGULAR] = 0;
        writeVelocity(0, 0); 
        sprintf(log_msg, "warn %d %d %d",danger_cnt[2],danger_cnt[3],danger_cnt[4]);
       
       }            
       else
        sprintf(log_msg, " %d %d %d",danger_cnt[2],danger_cnt[3],danger_cnt[4]);
       
       
        nh.loginfo(log_msg);
    }
    else if( abs(goal_velocity[LINEAR]) >= 0.3 ||  abs(goal_velocity[ANGULAR]) > 0.3)
    {
        char log_msg[20];
       if(danger_cnt[2] >= SONAR_SAFETY_CNT*0.5 || danger_cnt[3] >= SONAR_SAFETY_CNT*0.5 || danger_cnt[4] >= SONAR_SAFETY_CNT*0.5)
       {
        goal_velocity_from_cmd[LINEAR] = 0;
        goal_velocity_from_cmd[ANGULAR] = 0;
        writeVelocity(0, 0);  
        sprintf(log_msg, "warn %d %d %d",danger_cnt[2],danger_cnt[3],danger_cnt[4]);
       
       }           
       else
        sprintf(log_msg, "%d %d %d",danger_cnt[2],danger_cnt[3],danger_cnt[4]);
       
        
        nh.loginfo(log_msg);    
    }
    else if( abs(goal_velocity[LINEAR]) > 0 ||  abs(goal_velocity[ANGULAR]) > 0)
    {
        char log_msg[20];
       if(danger_cnt[2] >= SONAR_SAFETY_CNT*0.9 || danger_cnt[3] >= SONAR_SAFETY_CNT*0.9 || danger_cnt[4] >= SONAR_SAFETY_CNT*0.9)
       {
        goal_velocity_from_cmd[LINEAR] = 0;
        goal_velocity_from_cmd[ANGULAR] = 0;
        writeVelocity(0, 0);   
        sprintf(log_msg, "warn %d %d %d",danger_cnt[2],danger_cnt[3],danger_cnt[4]);
       
       }           
       else
       {
        sprintf(log_msg, "%d %d %d",danger_cnt[2],danger_cnt[3],danger_cnt[4]);
       }
        
        nh.loginfo(log_msg);  
    }
    
  for(int i = 0; i < 7 ;i++)
  {
      danger_cnt[i] = 0;
    
  }  
    
  }   

}

/*******************************************************************************
  Network disconnection for protecting something
*******************************************************************************/

void network_disconnect()
{
  // Stop the motor
  resetGoalVelocity();
  writeVelocity(0, 0);
 
}

void resetGoalVelocity()
{
  // Reset the goal velocities
  goal_velocity_from_cmd[LINEAR]  = 0.0f;
  goal_velocity_from_cmd[ANGULAR] = 0.0f;
  goal_velocity[LINEAR] = 0;
  goal_velocity[ANGULAR] = 0;

}

/////
void rx_dxl()
{
  //-- DXL -> USB
  int length;
  int i;

  length = DXL_PORT.available();
  if ( length > 0 )
  {
    if ( length > DXL_TX_BUFFER_LENGTH )
    {
      length = DXL_TX_BUFFER_LENGTH;
    }
    for (i = 0; i < length; i++ )
    {
      tx_buffer[i] = DXL_PORT.read();
      rx_msg.data = tx_buffer[i];
      pub_rx.publish(&rx_msg);
    }

    rx_data_cnt += length;
  }
}

/*******************************************************************************
  Update motor information
*******************************************************************************/

void init_MotorController()
{

  DXL_PORT.begin(DXL_BAUD);
  drv_dxl_tx_enable(FALSE);

  delay(3000);

  drv_dxl_tx_enable(TRUE);
  DXL_PORT.write(0x01);
  DXL_PORT.print(PE);
  drv_dxl_tx_enable(FALSE);

  delay(1000);

  drv_dxl_tx_enable(TRUE);
  DXL_PORT.write(0x01);
  DXL_PORT.print(SM);
  drv_dxl_tx_enable(FALSE);

}
/////

/////

uint32_t tran_hexatodecimal(String S) 
{
  // hex to int32_t func

  uint32_t pos = 1;
  uint32_t ans = 0;

  for ( int i = 7 ; i >= 4 ; i--) 
  {
    if (S.charAt(i) < 65 ) 
    {
      int n = S.charAt(i) - '0';
      ans += n * pos;
    }
    else 
    {
      int n = S.charAt(i) - 55;
      ans += n * pos;
    }
    pos *= 16;
  }
  return ans;
}

float relative_position(float current_pos, float previous_pos) 
{ 
  // calc relative pulse func
  pre_relative_dir = relative_dir;
  pre_relative_pos = relative_pos;

  relative_pos = current_pos - previous_pos;

  if (relative_pos > 0) {
    relative_dir = true;
  }
  else if (relative_pos < 0) {
    relative_dir = false;
  }

  if (relative_dir == pre_relative_dir) {
    return relative_pos;
  }

  else {
    return pre_relative_pos;
  }
}

void get_velocity() 
{    
    left_motor_dir = inVelData[3];
    right_motor_dir = inVelData[12];
    
    for(int i = 0; i < 7; i++)
    {
      left_motor_vel_str[i] = inVelData[i+4];
      right_motor_vel_str[i] = inVelData[i+13];
    }    
    left_motor_vel_str[7] = '\0';
    right_motor_vel_str[7] = '\0';
        
  //left_motor_vel = Str_to_Long(left_motor_vel_str, left_motor_dir);
  //right_motor_vel = Str_to_Long(right_motor_vel_str, right_motor_dir);
  
  left_motor_vel = rpm2MPS(left_motor_vel_str, left_motor_dir);
  right_motor_vel = rpm2MPS(right_motor_vel_str, right_motor_dir);
}

void get_position()
{    
    for(int i = 0; i < 7; i++)
    {
      left_motor_pos_str[i] = inPosData[i+4];
      right_motor_pos_str[i] = inPosData[i+13];
    }    
    left_motor_pos_str[7] = '\0';
    right_motor_pos_str[7] = '\0';
    
    
  //left_motor_vel = Str_to_Long(left_motor_vel_str, left_motor_dir);
  //right_motor_vel = Str_to_Long(right_motor_vel_str, right_motor_dir);
  
  left_motor_pos = rpm2MPS(left_motor_vel_str, left_motor_dir);
  right_motor_pos = rpm2MPS(right_motor_vel_str, right_motor_dir);  
}


void receive_motor_data()
{
  int str_length = 0;
  int i; 

  char inChar; // Where to store the character read
  byte index = 0; // Index into array; where to store the character

  str_length = DXL_PORT.available();

  if(goal_velocity[LINEAR] > 0 || goal_velocity[LINEAR] < 0 || goal_velocity[ANGULAR] > 0 || goal_velocity[ANGULAR] < 0)
  {       
    if(left_motor_pos == 0 && right_motor_pos == 0)
    {
      not_response_cnt++;   
    }    
    else
    {
      not_response_cnt = 0;
    }

    /// if Motor controller don't response from OpenCR
    if(not_response_cnt >= 5)
    {
      init_MotorController();
      not_response_cnt = 0;
    }
  }

  duty_msg.data = not_response_cnt;
  debug_pub.publish(&duty_msg);
    
  if(str_length > 0 )
  {
    for (i = 0; i < str_length; i++ ) // Don't read unless there you know there is data
    {
      //tx_vel_buffer[i] = DXL_PORT.read();  // Read a character
      inChar = DXL_PORT.read();  // Read a character
      inData[index] = inChar; // Store it
      index++; // Increment where to write next
    } 
    
    inData[index] = '\0'; // Null terminate the string
    
  }

  if(inData[1] == 'Q' && inData[2] == 'V')
  {
    strcpy(inVelData, inData);
  }
  else if(inData[1] == 'Q' && inData[2] == 'P')
  {
    strcpy(inPosData, inData);
  }   
}

void get_motor_position_data()
{
  drv_dxl_tx_enable(TRUE);
  DXL_PORT.write(0x01);
  DXL_PORT.print("QP?;");
  drv_dxl_tx_enable(FALSE);
}

void get_motor_velocity_data() 
{
  drv_dxl_tx_enable(TRUE);
  DXL_PORT.write(0x01);
  DXL_PORT.print("QV?;"); 
  drv_dxl_tx_enable(FALSE);
}

float rpm2MPS(const char* vel, char dir) 
{  
  // Conversion from RPM composed of bits to m/s
  float vel_;
  
  vel_= atof(vel);
  
  if(dir == '+')
    return vel_ * 0.00033f;
  else if(dir == '-')
    return -vel_ * 0.00033f;
  else 
    return 0;

}

float get_angle_vel(float left_vel, float right_vel) 
{
  float diff_vel = right_vel - left_vel;
  //float angle_w = diff_vel  * 5.2631578947368421052631578947368;
  float angle_w = diff_vel  * 2.631578947f;
  return angle_w;
}

long Str_to_Long(String vel_str, char dir) 
{
  unsigned long pos = 1;
  long ans = 0;

  int len = vel_str.length();

  for ( int i = len - 1 ; i >= 0 ; i--) 
  {
    int n = vel_str.charAt(i) - '0';
    ans += n * pos;
    pos *= 10;
  }

  if (dir == '-') 
  {
    return ans * (-1);
  }
  else
  {
    return ans;
  }
}

double trans_rpmtometerpersec(long vel_rpm) 
{
  // rpm to m/s
  return (double)(0.075 * 0.10472 * (vel_rpm));
}

/*******************************************************************************
  Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  char log_msg[100];
  const char * init_log_data = INIT_LOG_DATA;

  if (nh.connected())
  {
    if (log_flag == false)
    {
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
       
  }
  else
  {
    sprintf(log_msg, "Not connected");
    nh.loginfo(log_msg);
    //log_flag = false;
  }
}
