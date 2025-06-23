// Para que funcione ros.h
#undef ESP32
#undef ESP8266
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

#define MAX_MUESTRAS 4000  // Buffer grande para muestreo rápido

const int sensor = 34;    // GPIO34 (ADC1_CH6 en ESP32)
const int rele1 = 4;
const int rele2 = 16;

ros::NodeHandle nh;

// Mensajes
std_msgs::Float32 tiempo_vacio_msg;
std_msgs::Float32 tiempo_atmosfera_msg;

// Publishers
ros::Publisher tiempo_vacio_pub("tiempo_hasta_vacio", &tiempo_vacio_msg);
ros::Publisher tiempo_atmosfera_pub("tiempo_hasta_atmosfera", &tiempo_atmosfera_msg);

// Flags y buffers
bool medir = false;
bool modo_vacio = true;  // true: mide vacío, false: mide desvacío
bool vacio_detectado = false;
bool atm_detectado = false;

unsigned long tiempo_inicio = 0;
float presiones[MAX_MUESTRAS];
int indice = 0;

// Subscriptores
ros::Subscriber<std_msgs::UInt8> sub_rele1("Rele1", [](const std_msgs::UInt8& msg) {
  if (msg.data == 1) {
    digitalWrite(rele1, HIGH);
    modo_vacio = true;
    medir = true;
    vacio_detectado = false;
  } else {
    digitalWrite(rele1, LOW);
    modo_vacio = false;
    medir = true;
    atm_detectado = false;
  }
  tiempo_inicio = millis();
  indice = 0;
});

ros::Subscriber<std_msgs::UInt8> sub_rele2("Rele2", [](const std_msgs::UInt8& msg) {
  digitalWrite(rele2, msg.data == 1 ? HIGH : LOW);
});

void setup() {
  nh.getHardware()->setBaud(57600);

  pinMode(sensor, INPUT);     // GPIO34 como entrada analógica
  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
  digitalWrite(rele1, LOW);
  digitalWrite(rele2, LOW);

  nh.initNode();
  nh.advertise(tiempo_vacio_pub);
  nh.advertise(tiempo_atmosfera_pub);
  nh.subscribe(sub_rele1);
  nh.subscribe(sub_rele2);
}

void loop() {
  if (medir && indice < MAX_MUESTRAS) {
    int adc = analogRead(sensor);
    float presion = 0.000657 * adc - 1.5049;
    presiones[indice++] = presion;

    // Medir tiempo hasta vacío
    if (modo_vacio && !vacio_detectado && presion <= -0.85) {
      vacio_detectado = true;
      medir = false;
      float tiempo_vacio = (millis() - tiempo_inicio) / 1000.0;
      tiempo_vacio_msg.data = tiempo_vacio;
      tiempo_vacio_pub.publish(&tiempo_vacio_msg);
    }

    // Medir tiempo hasta atmósfera
    if (!modo_vacio && !atm_detectado && presion > -0.1) {
      atm_detectado = true;
      medir = false;
      float tiempo_atm = (millis() - tiempo_inicio) / 1000.0;
      tiempo_atmosfera_msg.data = tiempo_atm;
      tiempo_atmosfera_pub.publish(&tiempo_atmosfera_msg);
    }
  }

  nh.spinOnce();  // Procesar mensajes entrantes
}
