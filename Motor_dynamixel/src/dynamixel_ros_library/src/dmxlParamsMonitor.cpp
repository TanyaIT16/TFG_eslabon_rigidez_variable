/**
 * This example is created to demonstrate how to use the library with ROS. 
 * In this case, a ROS node is responsible for publishing data on temperature, velocity, 
 * current, and position every 0.1 seconds. Additionally, it listens to a ROS topic named 
 * "pos_user_input," where the user must input the desired motor position. 
 * When a new value appears on this topic, the program executes the corresponding 
 * callback and then resumes its normal routine.
 * 
 * 1. Launch roscore:
 *      roscore
 * 
 * 2. Launch PlotJuggler (only if you want to visualize some data):
 *      rosrun plotjuggler plotjuggler
 *  
 * 3. Launch dmxlParamsMonitor:
 *      rosrun dynamixel_ros_library dmxlParamsMonitor  YOUR_PORT PROTOCOL_TYPE BAUDRATE DMXL_ID
 * 
 * 4. Publish any position between 0 and 360 degrees in the created topic:
 *      rostopic pub -1 /pos_user_input std_msgs/Int32 "data: DEGREES"
*/

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "dynamixel_ros_library/dynamixel_ros_library.h"
#include <iostream>


dynamixelMotor myMotor;

void publishMotorStatus(ros::Publisher& pos_pub, ros::Publisher& vel_pub, ros::Publisher& curr_pub, ros::Publisher& temp_pub)
{
    // Creating MSG objects
    std_msgs::Float32 pos_msg;
    std_msgs::Float32 vel_msg;
    std_msgs::Float32 curr_msg;
    std_msgs::Float32 temp_msg;

    // Getting params from dmxl
    float position = (float)(myMotor.getPresentPosition());
    float velocity = (float)(myMotor.getPresentVelocity());
    float current = (float)(myMotor.getPresentCurrent());
    float temperature = (float)(myMotor.getPresentTemperature());

    // data assignation
    pos_msg.data = position;
    vel_msg.data = velocity;
    curr_msg.data = current;
    temp_msg.data = temperature;

    // Publishing
    pos_pub.publish(pos_msg);
    vel_pub.publish(vel_msg);
    curr_pub.publish(curr_msg);
    temp_pub.publish(temp_msg);
}

// Callback when some data was published in 'pos_user_input'
void callBack(const std_msgs::Int32::ConstPtr& msg)
{
    int userValue = msg->data;
    if (userValue >= 0 && userValue <= 360)
    {
        if(myMotor.getTorqueState())
        {
            myMotor.setTorqueState(false);
        }

        if(myMotor.getOperatingMode() != "EXTENDED_POSITION_CONTROL_MODE")
        {
            myMotor.setOperatingMode(dynamixelMotor::EXTENDED_POSITION_CONTROL_MODE);
        }

        myMotor.setTorqueState(true);
        myMotor.setGoalPosition(userValue);
    }
    else
    {
        ROS_WARN("Invalid input: %d. Please enter a value between 0 and 360.", userValue);
    }
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

    myMotor = dynamixelMotor("M1",dmxl_id);
    dynamixelMotor::iniComm(port_name,protocol_version,baud_rate);
    
    myMotor.setControlTable();

    // ROS node init
    ros::init(argc, argv, "dmxlParamsMonitor");
    ros::NodeHandle nh;

    // Publishers and Subscribers creation
    ros::Publisher pos_publisher = nh.advertise<std_msgs::Float32>("motor_position",10);
    ros::Publisher vel_publisher = nh.advertise<std_msgs::Float32>("motor_velocity",10);
    ros::Publisher curr_publisher = nh.advertise<std_msgs::Float32>("motor_current",10);
    ros::Publisher temp_publisher = nh.advertise<std_msgs::Float32>("motor_temperature",10);

    ros::Subscriber user_input_subscriber = nh.subscribe("pos_user_input",10,callBack);

    // ROS freq = 10 Hz
    ros::Rate loop_rate(10);


    while(ros::ok())
    {
        publishMotorStatus(pos_publisher, vel_publisher, curr_publisher, temp_publisher);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
