#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

// Subscriber routine
// It's called everytime that there is a new message
void odometryCallback(const nav_msgs::OdometryPtr& msg) {
    
    geometry_msgs::Quaternion qt = msg->pose.pose.orientation;
    
    double yaw = tf::getYaw(qt);
    
    ROS_INFO("Received Odometry message: %f",(yaw*180)/M_PI);
    
}

int main (int argc,char **argv){
    // Start ROS within the context of this node.
    ros::init(argc, argv, "quat2yaw");
    
    // Declare node.
    ros::NodeHandle node;
    
    // Declare topic to subscriber
    int queue_size=1;
    ros::Subscriber sub = node.subscribe("/vrep/vehicle/odometry", queue_size, odometryCallback);
   
    // Declare topic to publisher
    //pub = node.advertise<geometry_msgs::Twist>("/object_avoidance/robotSpeeds",queue_size); 
     
    // ROS routine
    ros::spin();
}