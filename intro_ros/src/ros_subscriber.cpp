#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

// Global Variables
// They need to be global, because need to be seen by the callback routine.

int counter=0;
// Declare topic to publisher
int queue_size=1;
ros::Publisher pub;
std_msgs::Int32 msg_counter;
    
// Subscriber routine
// It's called everytime that there is a new message
void fcnCallback(const std_msgs::StringConstPtr& msg) {
    
    counter++;
    
    //std::cout << "Received: " << msg->data << std::endl;
    ROS_INFO("Received: %s",msg->data.c_str());
    
    msg_counter.data = counter;
    pub.publish(msg_counter);
}


int main(int argc, char **argv) {
    // Start ROS within the context of this node.
    ros::init(argc, argv, "ros_sub");
    
    // Declare node.
    ros::NodeHandle node;
    
    pub = node.advertise<std_msgs::Int32>("ros_world/counter",queue_size);
    
    // Declare topic to subscriber
    ros::Subscriber sub = node.subscribe("ros_world", queue_size,fcnCallback);
    
    // ROS routine
    ros::spin();
}