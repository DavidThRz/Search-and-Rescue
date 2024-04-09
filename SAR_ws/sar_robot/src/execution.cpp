
#include "ros/ros.h"

#include "sar_robot/missionTarget.h"
#include "geometry_msgs/PoseStamped.h"


class SAR_Robot {

public:

    bool targetIdentified;
    geometry_msgs::PoseStamped currentGoal;

    /* Class contructor */
    SAR_Robot() {  

        this->targetIdentified = false;

        this->currentGoal.header.seq = 0;
        this->currentGoal.header.frame_id = "map";
    }
};
//Global instance for class SAR_Robot
SAR_Robot omniRobot;


void targetIdentified_cb(const sar_robot::missionTarget::ConstPtr& msg) {

    omniRobot.targetIdentified = msg->identified;

}


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "execution");
    ros::NodeHandle n;

    ros::Rate rate(10);

    ros::Subscriber targetIdentified_sub = n.subscribe<sar_robot::missionTarget>("/targetIdentified", 1, targetIdentified_cb);

    ros::Publisher currentGoal_pub       = n.advertise<geometry_msgs::PoseStamped>("/move_base/currentGoal", 1);

    while(ros::ok()) {

        if (!omniRobot.targetIdentified) {

            //Si currentGoal not reached -> esperar
            //Si currentGoal reached 
                //Preparar siguiente waypoint
                //Comandarlo

        } else {

            ROS_INFO("Mission complete!");
            return 1;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return -1;
}