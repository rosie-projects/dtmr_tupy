/*
  :Version 1.0
  :Author: Dragos Calin
  :Email: dragos@intorobotics.com
  :License: BSD
  :Date: 01/05/2020
  :Last update: dd/mm/YYYY
*/
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#include <NewPing.h>
#include <SimpleKalmanFilter.h>

#define SONAR_NUM 4          //The number of sensors. 
#define MAX_DISTANCE 200     //Mad distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the pings after 33 microseconds.

#define IN1 13
#define IN2 12
#define IN3 11 
#define IN4 10

char str[15];
int valor;

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

unsigned long _timerStart = 0;

int LOOPING = 40; //Loop for every 40 milliseconds.

uint8_t oldSensorReading[3];    //Store last valid value of the sensors.

uint8_t oneSensor;             //Store raw sensor's value.
uint8_t twoSensor;
uint8_t threeSensor;
uint8_t fourSensor;
uint8_t fiveSensor;
uint8_t sixSensor;
uint8_t sevenSensor;
uint8_t eightSensor;

uint8_t oneSensorKalman;       //Store filtered sensor's value.
uint8_t twoSensorKalman;
uint8_t threeSensorKalman;
uint8_t fourSensorKalman;       //Store filtered sensor's value.
uint8_t fiveSensorKalman;
uint8_t sixSensorKalman;
uint8_t sevenSensorKalman;       //Store filtered sensor's value.
uint8_t eightSensorKalman;


NewPing sonar[SONAR_NUM] = {
  NewPing(3, 2, MAX_DISTANCE), // Trigger pin, echo pin, and max distance to ping.
  NewPing(5, 4, MAX_DISTANCE),
  NewPing(7, 6, MAX_DISTANCE),
  NewPing(9, 8, MAX_DISTANCE)  
//  NewPing(15, 14, MAX_DISTANCE),
//  NewPing(17, 16, MAX_DISTANCE),
//  NewPing(19, 18, MAX_DISTANCE),
//  NewPing(21, 20, MAX_DISTANCE)
  
};

/*
  create Kalman filter objects for the sensors.
   SimpleKalmanFilter(e_mea, e_est, q);
   e_mea: Measurement Uncertainty
   e_est: Estimation Uncertainty
   q: Process Noise
*/
SimpleKalmanFilter KF_1(2, 2, 0.01);
SimpleKalmanFilter KF_2(2, 2, 0.01);
SimpleKalmanFilter KF_3(2, 2, 0.01);
SimpleKalmanFilter KF_4(2, 2, 0.01);
//SimpleKalmanFilter KF_5(2, 2, 0.01);
//SimpleKalmanFilter KF_6(2, 2, 0.01);
//SimpleKalmanFilter KF_7(2, 2, 0.01);
//SimpleKalmanFilter KF_8(2, 2, 0.01);

ros::NodeHandle nh; //create an object which represents the ROS node.

//looping the sensors
void sensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
}

// If ping received, set the sensor distance to array.
void echoCheck() {
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

//Return the last valid value from the sensor.
void oneSensorCycle() {
  oneSensor =   returnLastValidRead(0, cm[0]);
  twoSensor =   returnLastValidRead(1, cm[1]);
  threeSensor = returnLastValidRead(2, cm[2]);
  fourSensor =  returnLastValidRead(3, cm[3]);
//  fiveSensor =  returnLastValidRead(4, cm[4]);
//  sixSensor =   returnLastValidRead(5, cm[5]);
//  sevenSensor = returnLastValidRead(6, cm[6]);
//  eightSensor = returnLastValidRead(7, cm[7]);
}

//If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, uint8_t cm) {
  if (cm != 0) {
    return oldSensorReading[sensorArray] = cm;
  } else {
    return oldSensorReading[sensorArray];
  }
}

//Apply Kalman Filter to sensor reading.
void applyKF() {
  oneSensorKalman   = KF_1.updateEstimate(oneSensor);
  twoSensorKalman   = KF_2.updateEstimate(twoSensor);
  threeSensorKalman = KF_3.updateEstimate(threeSensor);
  fourSensorKalman  = KF_4.updateEstimate(fourSensor);
//  fiveSensorKalman  = KF_5.updateEstimate(fiveSensor);
//  sixSensorKalman   = KF_6.updateEstimate(sixSensor);
//  sevenSensorKalman = KF_7.updateEstimate(sevenSensor);
//  eightSensorKalman = KF_8.updateEstimate(eightSensor);
}

void startTimer() {
  _timerStart = millis();
}

bool isTimeForLoop(int _mSec) {
  return (millis() - _timerStart) > _mSec;
}

void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.0;
  range_name.max_range = 2.0;
}

//Create three instances for range messages.
sensor_msgs::Range range_1;
sensor_msgs::Range range_2;
sensor_msgs::Range range_3;
sensor_msgs::Range range_4;
//sensor_msgs::Range range_5;
//sensor_msgs::Range range_6;
//sensor_msgs::Range range_7;
//sensor_msgs::Range range_8;


//Create publisher onjects for all sensors
ros::Publisher pub_range_1("/sonar1", &range_1);
ros::Publisher pub_range_2("/sonar2", &range_2);
ros::Publisher pub_range_3("/sonar3", &range_3);
ros::Publisher pub_range_4("/sonar4", &range_4);
//ros::Publisher pub_range_5("/sonar5", &range_5);
//ros::Publisher pub_range_6("/sonar6", &range_6);
//ros::Publisher pub_range_7("/sonar7", &range_7);
//ros::Publisher pub_range_8("/sonar8", &range_8);

void onTwist(const geometry_msgs::Twist& msg)
{

  //forward
  if(msg.linear.x > 0)
  {
    //digitalWrite(ENA,HIGH); //habilita roda direita
    //digitalWrite(ENB,HIGH); //roda esquerda
    digitalWrite(IN1,HIGH); //roda direita para frente
    digitalWrite(IN2,LOW);  //roda direita para trás
    digitalWrite(IN3,HIGH); //roda esquerda para frente
    digitalWrite(IN4,LOW);  //roda esquerda para trás
  }
  //backward
  else if(msg.linear.x < 0)
  {
    //digitalWrite(ENA,HIGH);
    //digitalWrite(ENB,HIGH);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
  }
  //right
  else if(msg.angular.z < 0)
  {
    //digitalWrite(ENA,HIGH);
    //digitalWrite(ENB,LOW);
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    
  }
  //left
  else if(msg.angular.z > 0)
  {
    //digitalWrite(ENA,LOW);
    //digitalWrite(ENB,HIGH);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
  }

  //stop
  else 
  {
    //digitalWrite(ENA,LOW);
    //digitalWrite(ENB,LOW);    
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",onTwist); 
//ros::NodeHandle nh;

void setup() {
  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  nh.initNode();
  nh.advertise(pub_range_1);
  nh.advertise(pub_range_2);
  nh.advertise(pub_range_3);
  nh.advertise(pub_range_4);
//  nh.advertise(pub_range_5);
//  nh.advertise(pub_range_6);
//  nh.advertise(pub_range_7);
//  nh.advertise(pub_range_8);

  sensor_msg_init(range_1, "/sonar1");
  sensor_msg_init(range_2, "/sonar2");
  sensor_msg_init(range_3, "/sonar3");
  sensor_msg_init(range_4, "/sonar4");
//  sensor_msg_init(range_5, "/sonar5");
//  sensor_msg_init(range_6, "/sonar6");
//  sensor_msg_init(range_7, "/sonar7");
//  sensor_msg_init(range_8, "/sonar8");

  // put your setup code here, to run once:
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  //pinMode(ENA,OUTPUT);
  //pinMode(ENB,OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  if (isTimeForLoop(LOOPING)) {
    sensorCycle();
    oneSensorCycle();
    applyKF();
    range_1.range = oneSensorKalman;
    range_2.range = twoSensorKalman;
    range_3.range = threeSensorKalman;
    range_4.range = fourSensorKalman;
//    range_5.range = fiveSensorKalman;
//    range_6.range = sixSensorKalman;
//    range_7.range = sevenSensorKalman;
//    range_8.range = eightSensorKalman;

    range_1.header.stamp = nh.now();
    range_2.header.stamp = nh.now();
    range_3.header.stamp = nh.now();
    range_4.header.stamp = nh.now();
//    range_5.header.stamp = nh.now();
//    range_6.header.stamp = nh.now();
//    range_7.header.stamp = nh.now();
//    range_8.header.stamp = nh.now();

    pub_range_1.publish(&range_1);
    pub_range_2.publish(&range_2);
    pub_range_3.publish(&range_3);
    pub_range_4.publish(&range_4);
//    pub_range_5.publish(&range_5);
//    pub_range_6.publish(&range_6);
//    pub_range_7.publish(&range_7);
//    pub_range_8.publish(&range_8);

    startTimer();
  }
  nh.spinOnce();//Handle ROS events
}
