// Para que funcione ros.h
#undef ESP32
#undef ESP8266
#include "HX711.h"
#include <ros.h>
#include <std_msgs/Float32.h>

// Pines HX711
const int DTZ = 19;
const int SCKZ = 18;

// Objeto HX711
HX711 LCz;

// ROS
ros::NodeHandle nh;
std_msgs::Float32 force_z_msg;
ros::Publisher force_z_pub("Force_Z", &force_z_msg);

// Tiempo para 10 Hz (100 ms)
unsigned long previousMicros = 0;
const unsigned long intervalMicros = 100000;

// Coeficientes del polinomio (de Python)
const double a = 1.29093394e-10;
const double b = -2.04055523e-5;
const double c = -1.94389186;

void setup() {
  LCz.begin(DTZ, SCKZ);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(force_z_pub);
}

void loop() {
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= intervalMicros) {
    previousMicros = currentMicros;

    // Lectura en bits (promedio)
    long bits_z = LCz.read_average(5);

    // Calcular fuerza usando el polinomio ajustado
    float fuerza_z = a * bits_z * bits_z + b * bits_z + c;

    // Publicar en ROS
    force_z_msg.data = fuerza_z;
    force_z_pub.publish(&force_z_msg);

    nh.spinOnce();
  }
}
