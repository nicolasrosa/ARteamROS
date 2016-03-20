#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>

// K Constant
#define K 1

//Global Variables
ros::Publisher pub;
geometry_msgs::Twist robotSpeeds;

//Desired Position
double xf=4,yf=1.5;

// Subscriber routine
// It's called everytime that there is a new message
void odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf::getYaw(msg->pose.pose.orientation);
    //double yaw_degrees = (yaw*180)/M_PI;
    
    double theta = atan2(yf-y,xf-x);
    //double d = calculate_distance(x,y,xf,yf);
    double d = sqrt(pow((xf-x),2)+pow((yf-y),2));
    
    //double deltaTheta = ;
        
    if(d > 0.1){
        robotSpeeds.linear.x = 1;
        robotSpeeds.angular.z = K*(theta - yaw);
    }else{
        robotSpeeds.linear.x = 0;
        robotSpeeds.angular.z = 0;
    }
    
    ROS_INFO("d = %f , x = %f , y = %f,yaw: %f,theta: %f",d,x,y,yaw,theta);
    pub.publish(robotSpeeds);
}

int main (int argc,char **argv){
    // Start ROS within the context of this node.
    ros::init(argc, argv, "go2point");
    
    // Declare node.
    ros::NodeHandle node;
    
    // Declare topic to subscriber
    int queue_size=1;
    ros::Subscriber sub = node.subscribe("/vrep/vehicle/odometry", queue_size, odometryCallback);
   
    // Declare topic to publisher
    pub = node.advertise<geometry_msgs::Twist>("/go2point/robotSpeeds",queue_size); 
     
    // ROS routine
    ros::spin();
}