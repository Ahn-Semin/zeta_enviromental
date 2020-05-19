/*******************************************************************************
* Copyright 2018 SEOULTECH CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Seo, Jae Yong */

#ifndef ZETABANK_CORE_CONFIG_NEW_H_
#define ZETABANK_CORE_CONFIG_NEW_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
/////
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/UInt32.h>
/////

#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <zetabank_msgs/SensorState.h>
#include <zetabank_msgs/Sound.h>
#include <zetabank_msgs/VersionInfo.h>

#include <Zetabank.h>

#include <math.h>

#define INIT_LOG_DATA "This core(v1.2.0) is compatible with Zetabank"

#define HARDWARE_VER "1.0.0"
#define SOFTWARE_VER "1.0.0"
#define FIRMWARE_VER "1.2.0"

#define CONTROL_MOTOR_SPEED_PERIOD          100   //hz. 20ms
#define IMU_SONAR_PUBLISH_PERIOD            50  //hz. 20ms
#define CMD_VEL_PUBLISH_PERIOD              50   //hz. 20ms
#define DRIVE_INFORMATION_PUBLISH_PERIOD    10   //hz. 100ms
#define VERSION_INFORMATION_PUBLISH_PERIOD  1    //hz . 1s

#define WHEEL_NUM                        2
#define WHEEL_RADIUS                     0.0812           // meter
#define WHEEL_SEPARATION                 0.360           // meter (BURGER : 0.160, WAFFLE : 0.287, zetabank : 0.360)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define MAX_LINEAR_VELOCITY              2.0   // m/s   (BURGER : 0.22, WAFFLE : 0.25)
#define MAX_ANGULAR_VELOCITY             2.0   // rad/s (BURGER : 2.84, WAFFLE : 1.82)

#define TICK2RAD                         0.000255603f  // (0.02929[deg] * 3.14159265359 / 180) /2(pully_) = 0.000511207f(rad)

#define PULSE2RAD                        0.000058999f  // (360/1024/4/26 * 3.14159265359 / 180) 

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define VELOCITY_UNIT                    2

#define LINEAR_X_MAX_VELOCITY            2.0

#define PI 3.141592f
#define FORWARD_DIR true
#define BACKWARD_DIR false
#define STOP 2

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))  
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))  

/////
#define DXL_PORT              Serial3
#define DXL_BAUD              115200

#define DXL_TX_BUFFER_LENGTH  1024
/////

#define SET_RATE 104    //104us
/////

#define SONAR_SAFETY_CNT 25

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void resetCallback(const std_msgs::Empty& reset_msg);
void teleopCallback(const std_msgs::Bool& bool_msg);
void ledCallback(const std_msgs::String& str_msg);

// Function prototypes
void publishCmdVelFromRC100Msg(void);
void publishImuMsg(void);
void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
void publishBatteryStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros);

void updateVariable(void);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGyroCali(void);
void updateGoalVelocity(void);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);


/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);
ros::Subscriber<std_msgs::Bool> teleop_sub("teleop", teleopCallback);
ros::Subscriber<std_msgs::String> led_sub("led", ledCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Zetabank
zetabank_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// Version information of Zetabank
zetabank_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("version_info", &version_info_msg);

// IMU of Zetabank
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Odometry of Zetabank
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of Zetabank
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Battey state of Zetabank
sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Ultrasonic sensor of Zetabank
std_msgs::String sonar_msg;
ros::Publisher sonar_pub("sonar", &sonar_msg);



/*******************************************************************************
* For debugging
*******************************************************************************/ 
std_msgs::Float64 left_vel_msg;
ros::Publisher left_vel_pub("left_vel", &left_vel_msg);
std_msgs::Float64 right_vel_msg;
ros::Publisher right_vel_pub("right_vel", &right_vel_msg);
std_msgs::Float64 duty_msg;
ros::Publisher debug_pub("debug", &duty_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Zetabank
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* SoftwareTimer of Zetabank
*******************************************************************************/
static uint32_t tTime[5];


/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t g_last_diff_tick[WHEEL_NUM] = {0.0, 0.0};
double  g_last_rad[WHEEL_NUM]       = {0.0, 0.0};

void updateMotorInfo(int32_t left_tick, int32_t right_tick);


/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
ZetabankSensor sensors;


/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float goal_velocity[VELOCITY_UNIT] = {0.0, 0.0};
float goal_velocity_from_cmd[VELOCITY_UNIT] = {0.0, 0.0};


/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
ZetabankDiagnosis diagnosis;


/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];




/*******************************************************************************
* Declaration for reset
*******************************************************************************/
uint8_t reset_cnt=0;

/*******************************************************************************
* Declaration for Zetabank
*******************************************************************************/
/*******************************************************************************
* Encoder of Zetabank 
*******************************************************************************/
bool writeVelocity(float left_vel, float right_vel);


/*******************************************************************************
* Ultrasonic of Zetabank 
*******************************************************************************/
int setCount = 0;
bool ultra_set = false;
bool ultra_set_tick = false;

const char setPin = 7;
const char echoPin[7] = {0,1,2,3,4,5,6}; 
const char setting_50cm[8] = {1,0,0,0, 1,0,0,0};   //detect 0.5m
const char setting_1m[8] = {0,1,0,0, 1,0,0,0};   //detect 1m

HardwareTimer Timer(TIMER_CH1); 

void ultrasonic_pul_cmd();
void ultratimer_tick();
void ultrasonic_check();
void ultrasonic_setup();
void ultrasonic_dist_setting();



/*******************************************************************************
* control motors of Zetabank 
*******************************************************************************/
bool controlMotor(const float wheel_separation, float* value);
void init_MotorController();
/*******************************************************************************
*******************************************************************************/

void init_led();
void get_motor_velocity_data();
void receive_motor_data();
void sendLogMsg();
void get_position();
float relative_position(float current_pos, float previous_pos);
void get_velocity(); 
void   updateJointStates();
float get_angle_vel(float left_vel, float right_vel);
float rpm2MPS(const char* vel, char dir);


/*******************************************************************************
* For safety
*******************************************************************************/ 
unsigned int danger_cnt[7];
unsigned int sonar_cnt;
float pre_velocity[VELOCITY_UNIT] = {0.0, 0.0};
unsigned int vel_same_cnt = 0;
unsigned int vel_total_same_cnt = 0;

void ultrasonic_safe();
void network_disconnect();
void resetGoalVelocity();
void check_vel_safety(float, float);

bool teleop_flg = false;
bool log_flag = false;


/*******************************************************************************
*******************************************************************************/
///// Motor

bool vel_flg;
std_msgs::Char rx_msg;
ros::Publisher pub_rx("rx", &rx_msg);
char PE[8] = "PE0001;";
char SM[8] = "SM0505;";

char tx_buffer[DXL_TX_BUFFER_LENGTH];

static uint32_t rx_data_cnt = 0;
static uint32_t tx_data_cnt = 0;


/////
std_msgs::UInt32 left_motor_pos_msg;
std_msgs::UInt32 right_motor_pos_msg;


String left_vel_str;
String right_vel_str;

// Get data from motor controller
char left_motor_pos_str[10];
char right_motor_pos_str[10];
char left_motor_vel_str[10]; 
char right_motor_vel_str[10]; 

float left_motor_vel; 
float right_motor_vel; 

char left_motor_dir = '+'; 
char right_motor_dir = '-'; 

float left_motor_pos = 19264; 
float right_motor_pos = 19264; 

float left_pre_motor_pos; 
float right_pre_motor_pos; 

bool pre_relative_dir; 
bool relative_dir = true; 

float left_relative_pos_motor = 0; 
float right_relative_pos_motor = 0; 

char tx_pos_buffer[1024]; 
char tx_vel_buffer[1024]; 

float pre_relative_pos;
float relative_pos = 0; 

/////
float joint_states_pos[WHEEL_NUM] = 
{
  0.0,
  0.0
};
float joint_states_vel[WHEEL_NUM] = 
{
  0.0,
  0.0
};
float joint_states_eff[WHEEL_NUM] = 
{
  0.0,
  0.0
};

///// Receive data
char inData[30]; // Allocate some space for the string
char inPosData[30]; // Allocate some space for the string
char inVelData[30]; // Allocate some space for the string

int not_response_cnt = 0;

#endif // ZETABANK_CORE_CONFIG_H_
