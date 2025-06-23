#include "dynamixel_ros_library/dynamixel_ros_library.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// Declaración del motor Dynamixel
dynamixelMotor J1("J1", 1);

// Constantes
const float LIMITE_CORRIENTE = 3000; // Límite de corriente en mA
const float VELOCIDAD_ROTACION = 0.5; // Velocidad constante
const float POSICION_LIMITE = 180.0; // Posición límite

int main(int argc, char *argv[]) {
    if (argc != 4) {
        printf("Uso: rosrun dynamixel_ros_library jamming <port_name> <protocol_version> <baud_rate>\n");
        return 0;
    }

    char *port_name = argv[1];
    float protocol_version = atof(argv[2]);
    int baud_rate = atoi(argv[3]);

    ros::init(argc, argv, "motor_control");
    ros::NodeHandle nh;

    if (!dynamixelMotor::iniComm(port_name, protocol_version, baud_rate)) {
        ROS_ERROR("Error al inicializar la comunicación con el motor.");
        return -1;
    }

    J1.setControlTable();

    ROS_INFO("Estado inicial del motor:");
    ROS_INFO("Torque: %s", J1.getTorqueState() ? "ON" : "OFF");
    ROS_INFO("Modo de operación: %s", J1.getOperatingMode().c_str());

    // Asegurar que el torque está desactivado antes de cambiar el modo
    J1.setTorqueState(false);
    ros::Duration(0.5).sleep(); // Esperar un poco para asegurar el cambio

    // Configurar el modo de operación en control de velocidad
    J1.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);
    ros::Duration(0.5).sleep(); // Esperar para aplicar el modo

    std::string modo_actual = J1.getOperatingMode();
    ROS_INFO("Modo actual tras cambio: [%s]", modo_actual.c_str());
    if (modo_actual != "Velocity Control") {
        ROS_ERROR("Error al configurar el modo de operación. Abortando.");
        return -1;
    } else {
        ROS_INFO("Modo de operación configurado correctamente en VELOCITY_CONTROL_MODE.");
    }

    // Activar el torque del motor
    J1.setTorqueState(true);
    if (J1.getTorqueState()) {
        ROS_INFO("Torque activado correctamente.");
    } else {
        ROS_ERROR("Error al activar el torque.");
        return -1;
    }

    // Limitar la corriente a 3000 mA
    J1.setGoalCurrent(LIMITE_CORRIENTE);

    // Esperar 3 segundos antes de comenzar
    ROS_INFO("Esperando 3 segundos antes de iniciar el movimiento...");
    ros::Duration(3.0).sleep();

    // Configurar la velocidad y activar el movimiento
    J1.setGoalVelocity(VELOCIDAD_ROTACION);
    ROS_INFO("Motor activado y en movimiento.");

    ros::Publisher motor_position_pub = nh.advertise<std_msgs::Float32>("motor_position", 10);
    ros::Publisher motor_current_pub = nh.advertise<std_msgs::Float32>("motor_current", 10);

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok()) {
        float posicion = J1.getPresentPosition();
        float corriente = J1.getPresentCurrent();

        if (corriente < 0 || corriente > 5000) {
            ROS_WARN("Corriente anómala detectada: %.2f mA. Ignorando lectura.", corriente);
            corriente = 0.0;
        }

        std_msgs::Float32 posicion_msg;
        posicion_msg.data = posicion;
        motor_position_pub.publish(posicion_msg);

        std_msgs::Float32 current_msg;
        current_msg.data = corriente;
        motor_current_pub.publish(current_msg);

        ROS_INFO("Posicion: %.2f°, Corriente: %.2f mA", posicion, corriente);

        if (corriente >= LIMITE_CORRIENTE) {
            ROS_WARN("Límite de corriente alcanzado. Deteniendo motor.");
            J1.setTorqueState(false);
            J1.setGoalVelocity(0.0);
        } else if (posicion >= POSICION_LIMITE) {
            ROS_INFO("Posición límite alcanzada. Deteniendo motor.");
            J1.setTorqueState(false);
            J1.setGoalVelocity(0.0);
        } else {
            J1.setTorqueState(true);
            J1.setGoalVelocity(VELOCIDAD_ROTACION);
        }

        loop_rate.sleep();
    }

    return 0;
}
