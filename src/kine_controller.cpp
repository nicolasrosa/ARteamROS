#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv){    
    //ROS_INFO("Hello World using ROS function!");
    
    // Start ROS within the context of this node.
    ros::init(argc, argv, "kine_pub");
    
    // Declare node.
    ros::NodeHandle node;
    
    // Declare topic to be published.
    int queue_size = 1; 
    ros::Publisher pubL = node.advertise<std_msgs::Float64>("/vrep/vehicle/motorLeftSpeed",queue_size);
    ros::Publisher pubR = node.advertise<std_msgs::Float64>("/vrep/vehicle/motorRightSpeed",queue_size);
    
    // Define ROS loop rate (Hertz!).
    int frequency = 60;
    ros::Rate loop_rate(frequency);
    
    // Instantiate messages to be published
    std_msgs::Float64 velAngularWheelL_msg;
    std_msgs::Float64 velAngularWheelR_msg;
    
    double v=0.25,w=0;
    double b=0.35,R=0.1;
    
    double velWheelL;
    double velWheelR; 
    
    // Check if ROS connection is fine each iteration
    while ( ros::ok() ){
        // Build message according to a logic.
        velWheelL = v+(b/2.0)*w;
        velWheelR = v-(b/2.0)*w;
        
        velAngularWheelL_msg.data = velWheelL/R;
        velAngularWheelR_msg.data = velWheelR/R;
        
        // Publish message.
        //std::cout << "Publishing..." << std::endl;
        ROS_INFO("Publishing motors linear velocities...");
        pubL.publish(velAngularWheelL_msg);
        pubR.publish(velAngularWheelR_msg);

        // ROS routines
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}