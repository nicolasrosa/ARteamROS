#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <math.h>

// K Constant
#define K_v 1
#define K_w 0.75

//Global Variables - ROS
ros::Publisher pub;
geometry_msgs::Twist robotSpeeds;

//Global Variables - Carrot Chasing
geometry_msgs::Pose2D w1;   //wi
geometry_msgs::Pose2D w2;   //wi+1

double delta = 2;
double w;

double calculateDistance(double x,double y,double xf,double yf){
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

void initializeWayPoints(){
    w1.x = 4;
    w1.y = 1.5;

    w2.x = -3;
    w2.y = 1.5;
    
    //std::cout << "w1(" << w1.x << "," << w1.y << ")" << std::endl;
    //std::cout << "w2(" << w2.x << "," << w2.y << ")" << std::endl;
}

// Subscriber routine - It's called everytime that there is a new message
void odometryCallback(const nav_msgs::OdometryConstPtr &msg) {    
    geometry_msgs::Pose2D p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.theta = tf::getYaw(msg->pose.pose.orientation);
        
    double Ru = calculateDistance(w1.x,w1.y,p.x,p.y);
    double theta = atan2(w2.y-w1.y,w2.x-w1.x);
    double thetaU = atan2(p.y-w1.y,p.x - w1.x);
    double beta = theta - thetaU;
    double R = sqrt(pow(Ru,2)-pow(Ru*sin(beta),2));
    
    geometry_msgs::Pose2D pc;
    pc.x = (R+delta)*cos(theta)+w1.x;
    pc.y = (R+delta)*sin(theta)+w1.y;
    
    //double d = calculateDistance(p.x,p.y,xf,yf);
    
    double psi_d = atan2(pc.y-p.y,pc.x-p.x);
    
    double dTheta = normalizeAngle(psi_d-p.theta);
    
    w = K_w*dTheta;
    if(std::abs(w)>0.75){
        w = 0.75*dTheta/std::abs(dTheta);
    }
    
//    if(d > 0.1){
//        d > 1 ? robotSpeeds.linear.x = K_v : robotSpeeds.linear.x = K_v*d;
    robotSpeeds.linear.x = 1; // 0.5 -> Robot with 1 sonar sensor
    //robotSpeeds.angular.z = K_w*dTheta;
    robotSpeeds.angular.z = w;
        
       // robotSpeeds.angular.z = w;
        
//    }else{
//        robotSpeeds.linear.x = 0;
//        robotSpeeds.angular.z = 0;
//    }
//    
    ROS_INFO("x: %f\ty: %f\tyaw: %f\ttheta: %f\txc: %f\tyc: %f",p.x,p.y,p.theta,theta,pc.x,pc.y);
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
     
    
    initializeWayPoints();
    
    // ROS routine
    ros::spin();
}