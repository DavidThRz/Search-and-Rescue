
#include "ros/ros.h"

#include "sar_robot/missionTarget.h"

sar_robot::missionTarget message;

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Rate rate(10);

    ros::Publisher msg_pub = n.advertise<sar_robot::missionTarget>("/targetIdentified", 5);
    
    bool targetIdentified;
    int num = std::atoi(argv[1]);
    if (num == 1) 
        targetIdentified = true;
    else if (num == 0)
        targetIdentified = false;

    int count = 0;
    while(ros::ok()) {

        message.identified = targetIdentified;

        msg_pub.publish(message);

        ros::spinOnce();
        rate.sleep();

        if (count++ == 10)  //Sends the message 10 times
            break;
    }

    return 1;
}


