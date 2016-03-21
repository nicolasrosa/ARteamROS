#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// Global Variables - ROS
// They need to be global, because need to be seen by the callback routine.
int queue_size=1;
ros::Publisher pub;
geometry_msgs::Twist robotSpeeds;

// Global Variable - Sonar
float sonarFront;
float sonarLeft;
float sonarRight; 

// Subscriber routine
// It's called everytime that there is a new message
void sonarFrontCallback(const std_msgs::Float32Ptr& msg) {
    std::cout << "[Sonar Values] front: " << msg->data;
    
    float sonarFront=msg->data;
    
    if(sonarFront != 0){
        robotSpeeds.linear.x = 0;
        robotSpeeds.angular.z = 2;
    }else{
        robotSpeeds.linear.x = 0.25;
        robotSpeeds.angular.z = 0;
    }
    
    pub.publish(robotSpeeds);
}

void sonarLeftCallback(const std_msgs::Float32Ptr& msg) {
    std::cout << "\tleft: " << msg->data;
//    float sonarFront=msg->data;
//    
//    if(sonarFront != 0){
//        robotSpeeds.linear.x = 0;
//        robotSpeeds.angular.z = 2;
//    }else{
//        robotSpeeds.linear.x = 0.25;
//        robotSpeeds.angular.z = 0;
//    }
//    
//    pub.publish(robotSpeeds);
}

void sonarRightCallback(const std_msgs::Float32Ptr& msg) {
     std::cout << "\tright: " << msg->data << std::endl;
//    
//    float sonarFront=msg->data;
//    
//    if(sonarFront != 0){
//        robotSpeeds.linear.x = 0;
//        robotSpeeds.angular.z = 2;
//    }else{
//        robotSpeeds.linear.x = 0.25;
//        robotSpeeds.angular.z = 0;
//    }
//    
//    pub.publish(robotSpeeds);
}


int main(int argc, char **argv) {
    // Start ROS within the context of this node.
    ros::init(argc, argv, "object_avoidance");
    
    // Declare node.
    ros::NodeHandle node;
    
    // Declare topic to publisher
    pub = node.advertise<geometry_msgs::Twist>("/cmd_vel",queue_size);
    
    // Declare topic to subscriber
    ros::Subscriber sub_front = node.subscribe("/vrep/vehicle/frontSonar", queue_size, sonarFrontCallback);
    ros::Subscriber sub_left  = node.subscribe("/vrep/vehicle/leftSonar",  queue_size, sonarLeftCallback);
    ros::Subscriber sub_right = node.subscribe("/vrep/vehicle/rightSonar", queue_size, sonarRightCallback);
    
    // ROS routine
    ros::spin();
}