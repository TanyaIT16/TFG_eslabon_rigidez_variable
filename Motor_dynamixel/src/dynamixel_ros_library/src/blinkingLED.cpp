/**
 * This example is created to demonstrate how to use the library with ROS. 
 * In this case, a ROS node is responsible for switching the integrated LED state every 0.5s,
 * so the user can launch this example to identificate every motors ID without scanning entire system.
 * 
 * 1. Launch roscore:
 *      roscore
 * 
 * 2. Launch the example:
 *      rosrun dynamixel_ros_library blinkingLED  YOUR_PORT PROTOCOL_TYPE BAUDRATE DMXL_ID
*/

#include <ros/ros.h>
#include "dynamixel_ros_library/dynamixel_ros_library.h"

void switchLed(dynamixelMotor& motor)
{
   motor.setLedState(!motor.getLedState());
}

int main(int argc, char *argv[])
{
    char* port_name;
    int baud_rate, dmxl_id;
    float protocol_version;

    if (argc != 5)
    {
        printf("Please set '-port_name', '-protocol_version' '-baud_rate' '-dynamixel_id' arguments for connected Dynamixels\n");
        return 0;
    } else
    {
        port_name = argv[1];
        protocol_version = atoi(argv[2]);
        baud_rate = atoi(argv[3]);
        dmxl_id = atoi(argv[4]);
    }

    dynamixelMotor J1("J1",dmxl_id);
    dynamixelMotor::iniComm(port_name,protocol_version,baud_rate);
    
    J1.setControlTable();

    // ROS node init
    ros::init(argc, argv, "blinkingLED");
    ros::NodeHandle nh;

    // Callback creation
    auto callbackFunction = std::bind(switchLed,J1);
    ros::Timer timer = nh.createTimer(ros::Duration(0.5), callbackFunction);

    ros::spin();

    return 0;
}
