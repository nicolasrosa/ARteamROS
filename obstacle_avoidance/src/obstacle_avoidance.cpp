#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// Global Variables
// They need to be global, because need to be seen by the callback routine.
int queue_size=1;
ros::Publisher pub;
geometry_msgs::Twist robotSpeeds;
    
// Subscriber routine
// It's called everytime that there is a new message
void sonarCallback(const std_msgs::Float32Ptr& msg) {
    ROS_INFO("Received Sonar Value: %f", msg->data);
    
    if(msg->data != 0){
        robotSpeeds.linear.x = 0;
        robotSpeeds.angular.z = 2;
    }else{
        robotSpeeds.linear.x = 0.25;
        robotSpeeds.angular.z = 0;
    }
    
    pub.publish(robotSpeeds);
}


int main(int argc, char **argv) {
    // Start ROS within the context of this node.
    ros::init(argc, argv, "object_avoidance");
    
    // Declare node.
    ros::NodeHandle node;
    
    // Declare topic to publisher
    pub = node.advertise<geometry_msgs::Twist>("/object_avoidance/robotSpeeds",queue_size);
    
    // Declare topic to subscriber
    ros::Subscriber sub = node.subscribe("/vrep/vehicle/frontSonar", queue_size, sonarCallback);
    
    // ROS routine
    ros::spin();
}