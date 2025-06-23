/**
 * This example is created to demonstrate how to use the library with ROS. 
 * In this case, a ROS node is responsible for checking permanently a 'user_position_input' topic
 * and to change the DMXLs position if there is some information in the topic.
 * 
 * 1. Launch roscore:
 *      roscore
 * 
 * 2. Launch PlotJuggler (only if you want to visualize some data):
 *      rosrun plotjuggler plotjuggler 
 * 
 * 3. Launch testPositionControl:
 *      rosrun dynamixel_ros_library testPositionControl  YOUR_PORT PROTOCOL_TYPE BAUDRATE DMXL_ID
 * 
 * 4. Publish any position between 0 and 360 degrees in the created topic:
 *      rostopic pub -1 /user_position_input std_msgs/Int32 "data: DEGREES"
*/

#include "ros/ros.h"
#include "std_msgs/Int32.h" 
#include "dynamixel_ros_library/dynamixel_ros_library.h"

dynamixelMotor myDynamixel;

void userInputCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int userValue = msg->data;

    if(myDynamixel.getOperatingMode() != "Position Control")
    {
        if(myDynamixel.getTorqueState())
        {
            myDynamixel.setTorqueState(false);
        }

        myDynamixel.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    }

    myDynamixel.setTorqueState(true);
    myDynamixel.setGoalPosition(userValue);
}

int main(int argc, char **argv)
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

    myDynamixel = dynamixelMotor("example",dmxl_id);
    dynamixelMotor::iniComm(port_name,protocol_version,baud_rate);    
    myDynamixel.setControlTable();


    ros::init(argc, argv, "goal_position_reader");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("user_position_input", 10, userInputCallback);

    ros::spin();

    return 0;
}
