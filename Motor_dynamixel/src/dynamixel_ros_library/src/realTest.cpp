#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "dynamixel_ros_library/dynamixel_ros_library.h"
#include <iostream>

dynamixelMotor motorJ0,motorJ1,motorJ2,motorJ10,motorJ11,motorJ12;

// Callback when some data was published in 'pos_user_input'
void callBack(const std_msgs::Int32::ConstPtr& msg)
{
    int userValue = msg->data;
    
    motorJ0.setTorqueState(true);
    motorJ10.setTorqueState(true);
    motorJ1.setTorqueState(true);
    motorJ11.setTorqueState(true);
    motorJ2.setTorqueState(true);
    motorJ12.setTorqueState(true);

    switch (userValue)
    {
        case 0:
            motorJ0.setGoalPosition(60);
            motorJ10.setGoalPosition(345);
            motorJ1.setGoalPosition(12);
            motorJ11.setGoalPosition(358);
            motorJ2.setGoalPosition(120);
            motorJ12.setGoalPosition(240);
        break;

        case 1:
            motorJ0.setGoalPosition(180);
            motorJ10.setGoalPosition(240);
            motorJ1.setGoalPosition(120);
            motorJ11.setGoalPosition(280);
            motorJ2.setGoalPosition(240);
            motorJ12.setGoalPosition(120);
        break;

        case 2:
            motorJ0.setGoalPosition(180);
            motorJ10.setGoalPosition(180);
            motorJ1.setGoalPosition(120);
            motorJ11.setGoalPosition(315);
            motorJ2.setGoalPosition(240);
            motorJ12.setGoalPosition(200);
        break;
    
        default:

        break;
    }
}

int main(int argc, char *argv[])
{
    char* port_name;
    int baud_rate;
    float protocol_version;

    if (argc != 4)
    {
        printf("Please set '-port_name', '-protocol_version' '-baud_rate' arguments for connected Dynamixels\n");
        return 0;
    } else
    {
        port_name = argv[1];
        protocol_version = atoi(argv[2]);
        baud_rate = atoi(argv[3]);
    }

    dynamixelMotor::iniComm(port_name,protocol_version,baud_rate);
    motorJ0 = dynamixelMotor("J0",0);
    motorJ10 = dynamixelMotor("J10",10);
    motorJ1 = dynamixelMotor("J1",1);
    motorJ11 = dynamixelMotor("J11",11);
    motorJ2 = dynamixelMotor("J2",2);
    motorJ12 = dynamixelMotor("J12",12);

    motorJ0.setControlTable();
    motorJ10.setControlTable();
    motorJ1.setControlTable();
    motorJ11.setControlTable();
    motorJ2.setControlTable();
    motorJ12.setControlTable();
    

    // ROS node init
    ros::init(argc, argv, "testFingers");
    ros::NodeHandle nh;

    ros::Subscriber user_input_subscriber = nh.subscribe("pos_user_input",10,callBack);

    // ROS freq = 10 Hz
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        // publishMotorStatus(pos_publisher, vel_publisher, curr_publisher, temp_publisher);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
