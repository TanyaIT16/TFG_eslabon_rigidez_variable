// rostopic pub /Rele1 std_msgs/UInt8 "data: 1"

// rostopic pub /Rele2 std_msgs/UInt8 "data: 0"


#undef ESP32
#undef ESP8266
#include <ros.h>  // Defino pines de uso
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>


const int sensor = 34;    // Pin del sensor
const int rele1 = 4;       // Pin del primer relé
const int rele2 = 16;     // Pin del segundo relé


unsigned long previousMicros = 0;
const long interval = 100000;  // 10 Hz

// Defino las variables
int volt = 0;
ros::NodeHandle nh;


std_msgs::Float32 analog_msg;

ros::Publisher analog_pub("sensor_vacio", &analog_msg);


void messageCbRele1(const std_msgs::UInt8& msg) {
  
  uint8_t received_value = msg.data;

  if (received_value == 1) {
    
    digitalWrite(rele1, HIGH);
    
  } else if (received_value == 0) {
    
    digitalWrite(rele1, LOW);
    
  }
}



// Callback para controlar el segundo relé
void messageCbRele2(const std_msgs::UInt8& msg) {

  uint8_t received_value = msg.data;

  if (received_value == 0) {

    digitalWrite(rele2, LOW);

  }

  if (received_value == 1) {

    digitalWrite(rele2, HIGH);

  }

}


// Subscriptores de cada relé
ros::Subscriber<std_msgs::UInt8> sub_rele1("Rele1", &messageCbRele1);
ros::Subscriber<std_msgs::UInt8> sub_rele2("Rele2", &messageCbRele2);


void setup() {

  // Para asegurar que la velocidad en baudios en el código de Arduino y rosserial coinciden
  nh.getHardware()->setBaud(57600);

  // Inicializo los pines de los relés como salida
  pinMode(sensor,INPUT);
  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
  digitalWrite(rele1, LOW);
  digitalWrite(rele2, LOW);

  // Inicialización del nodo y del publisher
  nh.initNode();
  nh.advertise(analog_pub);

  // Subscríbete a los dos tópicos de control de los relés
  nh.subscribe(sub_rele1);
  nh.subscribe(sub_rele2);

}


void loop() {
unsigned long currentMicros = micros();

  // Ejecutar solo si ha pasado el intervalo de 
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    // Tu código aquí
    int adc = analogRead(sensor);
    float presion = 0.000657 * adc -1.5049;

    analog_msg.data = presion;  
    analog_pub.publish(&analog_msg);

    nh.spinOnce();
  }
}
