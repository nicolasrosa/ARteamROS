#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

// Global Variables
double v,w;
//double v=0.25,w=0;
double b=0.35,R=0.1; 

ros::Publisher pubL;
ros::Publisher pubR;

// Instantiate messages to be published
std_msgs::Float64 velAngularWheelL_msg;
std_msgs::Float64 velAngularWheelR_msg;

// Subscriber routine
// It's called everytime that there is a new message
void robotSpeedsCallback(const geometry_msgs::TwistPtr& msg) {
    ROS_INFO("Received: Robot Speeds!");
    
    v = msg->linear.x;
    w = msg->angular.z;
    
    // Check if ROS connection is fine each iteration
    
    // Build message according to a logic.
    velAngularWheelL_msg.data = (v-(b/2.0)*w)/R;
    velAngularWheelR_msg.data = (v+(b/2.0)*w)/R;
        
    // Publish message.
    //std::cout << "Publishing..." << std::endl;
    ROS_INFO("Publishing motors linear velocities...");
    pubL.publish(velAngularWheelL_msg);
    pubR.publish(velAngularWheelR_msg);
}

int main(int argc, char **argv){    
    //ROS_INFO("Hello World using ROS function!");
    
    // Start ROS within the context of this node.
    ros::init(argc, argv, "kine_pub");
    
    // Declare node.
    ros::NodeHandle node;
    
    // Declare topic to be published.
    int queue_size = 1; 
    pubL = node.advertise<std_msgs::Float64>("/vrep/vehicle/motorLeftSpeed",queue_size);
    pubR = node.advertise<std_msgs::Float64>("/vrep/vehicle/motorRightSpeed",queue_size);
    
    // Declare topic to subscriber
    //ros::Subscriber sub = node.subscribe("/object_avoidance/robotSpeeds", queue_size, robotSpeedsCallback);
    //ros::Subscriber sub = node.subscribe("/cmd_vel", queue_size, robotSpeedsCallback);
    ros::Subscriber sub = node.subscribe("/go2point/robotSpeeds", queue_size, robotSpeedsCallback);
    
    
    // Define ROS loop rate (Hertz!).
    //int frequency = 60;
    //ros::Rate loop_rate(frequency); 
    ros::spin();
    
    return 0;
}