#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv){
    //ROS_INFO("Hello World using ROS function!");
    
    // Start ROS within the context of this node.
    ros::init(argc, argv, "ros_pub");
    
    // Declare node.
    ros::NodeHandle node;
    
    // Declare topic to be published.
    int queue_size = 1; 
    ros::Publisher pub = node.advertise<std_msgs::String>("ros_world",queue_size);

    // Define ROS loop rate (Hertz!).
    int frequency = 10;
    ros::Rate loop_rate(frequency);
    
    // Instantiate messages to be published
    std_msgs::String msg;
    
    
    // Check if ROS connection is fine each iteration
    while ( ros::ok() ){
        // Build message according to a logic.
        msg.data = "Hello World!";
        
        // Publish message.
        //std::cout << "Publishing..." << std::endl;
        ROS_INFO("Publishing...");
        pub.publish(msg);

        // ROS routines
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}