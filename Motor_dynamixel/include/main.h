
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <ArduinoHardware.h>

//Defino los pines de uso
const int sensor = 15;
const int rele = 4;

//Defino las variables
int volt = 0;
float Sensibilidad = 0.1;

ros::NodeHandle  nh;

std_msgs::Float32 analog_msg;
ros::Publisher analog_pub("sensor_vacio", &analog_msg);
ros::Subscriber<std_msgs::UInt8> sub;
ros::Subscriber<std_msgs::UInt8> sub("Rele", &messageCb);

// Declaración de la función callback
void messageCb(const std_msgs::UInt8& msg);