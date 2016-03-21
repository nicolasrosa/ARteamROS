#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>

// K Constant
#define K_v 1
#define K_w 1


//Global Variables
ros::Publisher pub;
geometry_msgs::Twist robotSpeeds;

//Desired Position
double xf=4,yf=1.5;
double w;

double calculate_distance(double x,double y,double xf,double yf){
    return (sqrt(pow((xf-x),2)+pow((yf-y),2)));
}

double normalizeAngle(double angle){
    if(angle > M_PI){
        return(angle - 2*M_PI);
    }else if(angle < -M_PI){
        return (angle + 2*M_PI);
    }else{
        return angle;
    }
}

// Subscriber routine
// It's called everytime that there is a new message
void odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    double x = msg->pose.pose.position.x;
    
    double y = msg->pose.pose.position.y;
    double yaw = tf::getYaw(msg->pose.pose.orientation);
   
    double theta = atan2(yf-y,xf-x);
    double d = calculate_distance(x,y,xf,yf);
    
    double dTheta = normalizeAngle(theta-yaw);
    
//    if((dTheta > 1*M_PI/180) || (dTheta < -1*M_PI/180)){
//        w = K_w*dTheta/std::abs(dTheta);
//    }else{
//        w = 0;
//    }
    
    if(d > 0.1){
        d > 1 ? robotSpeeds.linear.x = K_v : robotSpeeds.linear.x = K_v*d;
        robotSpeeds.angular.z = K_w*dTheta;
    }else{
        robotSpeeds.linear.x = 0;
        robotSpeeds.angular.z = 0;
    }
    
    ROS_INFO("x = %f\ty = %f\td = %f\tyaw: %f\ttheta: %f",d,x,y,yaw,theta);
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
    pub = node.advertise<geometry_msgs::Twist>("/cmd_vel",queue_size); 
     
    // ROS routine
    ros::spin();
}