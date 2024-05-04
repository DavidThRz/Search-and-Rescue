
#include "ros/ros.h"

// #include "sar_robot/missionTarget.h"
#include <std_msgs/Bool.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// sar_robot::missionTarget message;
std_msgs::Bool messageBool;
geometry_msgs::PoseWithCovarianceStamped initPose;

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Rate rate(10);

    // ros::Publisher msg_pub = n.advertise<sar_robot::missionTarget>("/targetIdentified", 5);
    ros::Publisher msg_pub = n.advertise<std_msgs::Bool>("/targetIdentified", 5);
    ros::Publisher initPose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_waypoints", 5);
    
    bool targetIdentified;
    int num = std::atoi(argv[1]);
    if (num == 1) 
        targetIdentified = true;
    else if (num == 2)
        targetIdentified = false;
    else if (num == 3) {
        initPose.header.seq = 0;
        initPose.header.frame_id = "map";
        initPose.pose.pose.position.x = 1.0;
        initPose.pose.pose.position.y = 0.0;
        initPose.pose.pose.position.z = 0.0;
        initPose.pose.pose.orientation.x = 0.0;
        initPose.pose.pose.orientation.y = 0.0; 
        initPose.pose.pose.orientation.z = 0.0;
        initPose.pose.pose.orientation.w = 1.0;
        ROS_INFO("Initial pose sent!");

    } else if (num == 4) {
        initPose.header.seq = 0;
        initPose.header.frame_id = "map";
        initPose.pose.pose.position.x = 0.0;
        initPose.pose.pose.position.y = 1.0;
        initPose.pose.pose.position.z = 0.0;
        initPose.pose.pose.orientation.x = 0.0;
        initPose.pose.pose.orientation.y = 0.0; 
        initPose.pose.pose.orientation.z = 0.0;
        initPose.pose.pose.orientation.w = 1.0;
        ROS_INFO("Initial pose sent!");

    
    } else if (num == 5) {
        initPose.header.seq = 0;
        initPose.header.frame_id = "map";
        initPose.pose.pose.position.x = 0.0;
        initPose.pose.pose.position.y = 3.0;
        initPose.pose.pose.position.z = 0.0;
        initPose.pose.pose.orientation.x = 0.0;
        initPose.pose.pose.orientation.y = 0.0; 
        initPose.pose.pose.orientation.z = 0.0;
        initPose.pose.pose.orientation.w = 1.0;
        ROS_INFO("Initial pose sent!");
        
    } else {
        ROS_INFO("Invalid argument!");
        return 0;
    }

    int count = 0;
    while(ros::ok()) {

        // message.identified = targetIdentified;
        messageBool.data = targetIdentified;

        if (num == 1 || num == 2) 
            msg_pub.publish(messageBool);
        else if (num == 3 || num == 4 || num == 5) 
            initPose_pub.publish(initPose);



        ros::spinOnce();
        rate.sleep();

        if (count++ == 3)  //Sends the message 10 times
            break;
    }

    return 1;
}


