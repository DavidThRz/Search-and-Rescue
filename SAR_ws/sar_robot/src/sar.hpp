

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "move_base_msgs/MoveBaseActionGoal.h"

#include "ros/ros.h"


#define MISSION_STAT_PRECHECK 0
#define MISSION_STAT_EXPLORE 1
#define MISSION_STAT_WAYPOINTS 2
#define MISSION_STAT_RETURN 3
#define MISSION_STAT_COMPLETE 4


class SAR_Robot {

public:
    bool targetIdentified;

    // geometry_msgs::PoseWithCovarianceStamped currentGoal;
    geometry_msgs::PoseWithCovarianceStamped currentLocation;
    // geometry_msgs::PoseStamped currentGoal;
    move_base_msgs::MoveBaseActionGoal currentGoal;

    int mission_status;

    /* Class contructor */
    SAR_Robot() {  

        this->targetIdentified = true;

        //PoseStamped
        // this->currentGoal.header.seq = 0;
        // this->currentGoal.header.frame_id = "map";
        // this->currentGoal.pose.position.x = 0.0;
        // this->currentGoal.pose.position.y = 0.0;
        // this->currentGoal.pose.position.z = 0.0;
        // this->currentGoal.pose.orientation.x = 0.0;
        // this->currentGoal.pose.orientation.y = 0.0;
        // this->currentGoal.pose.orientation.z = 0.0;
        // this->currentGoal.pose.orientation.w = 1.0;

        //MoveBaseActionGoal
        this->currentGoal.header.seq = 0;
        this->currentGoal.header.frame_id = "map";
        this->currentGoal.goal.target_pose.header.frame_id = "map";
        this->currentGoal.goal.target_pose.header.seq = 0;
        this->currentGoal.goal.target_pose.pose.position.x = 0.0;
        this->currentGoal.goal.target_pose.pose.position.y = 0.0;
        this->currentGoal.goal.target_pose.pose.position.z = 0.0;
        this->currentGoal.goal.target_pose.pose.orientation.x = 0.0;
        this->currentGoal.goal.target_pose.pose.orientation.y = 0.0;
        this->currentGoal.goal.target_pose.pose.orientation.z = 0.0;
        this->currentGoal.goal.target_pose.pose.orientation.w = 1.0;

        this->currentLocation.header.seq = 0;
        this->currentLocation.header.frame_id = "map";
        this->currentLocation.pose.pose.position.x = 0.0;
        this->currentLocation.pose.pose.position.y = 0.0;
        this->currentLocation.pose.pose.position.z = 0.0;

        this->mission_status = MISSION_STAT_PRECHECK;
    }

    /* Function to check and change mission status */
    void changeMissionStatus();

};