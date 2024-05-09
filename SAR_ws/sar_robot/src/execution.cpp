
#include "ros/ros.h"

#include "sar.hpp"

// #include "sar_robot/missionTarget.h"
#include "geometry_msgs/PoseStamped.h"

#include <std_msgs/Bool.h>

#define MAP_MIN_X -5
#define MAP_MAX_X 4
#define MAP_MIN_Y -3
#define MAP_MAX_Y 5


struct Point2f {
    float x;
    float y;
};

Point2f initLocation;

bool allowExplore;

ros::Publisher initExplore_pub;


// Global instance for class SAR_Robot
SAR_Robot omniRobot;


// Funcion para calcular distancia entre dos puntos de PoseStamped
double distToGoal(geometry_msgs::PoseWithCovarianceStamped goal, geometry_msgs::PoseWithCovarianceStamped location) 
{
    double dist = sqrt(pow(goal.pose.pose.position.x - location.pose.pose.position.x, 2) + pow(goal.pose.pose.position.y - location.pose.pose.position.y, 2));

    return dist;
}

// double distToGoal(geometry_msgs::PoseStamped goal, geometry_msgs::PoseWithCovarianceStamped location) 
// {
//     double dist = sqrt(pow(goal.pose.position.x - location.pose.pose.position.x, 2) + pow(goal.pose.position.y - location.pose.pose.position.y, 2));

//     return dist;
// }

double distToGoal(move_base_msgs::MoveBaseActionGoal goal, geometry_msgs::PoseWithCovarianceStamped location) 
{
    double dist = sqrt(pow(goal.goal.target_pose.pose.position.x - location.pose.pose.position.x, 2) + pow(goal.goal.target_pose.pose.position.y - location.pose.pose.position.y, 2));

    return dist;
}

// Callback for targetIdentified topic - published by AI_vision node when target is identified
// void targetIdentified_cb(const sar_robot::missionTarget::ConstPtr& msg) {
void targetIdentified_cb(const std_msgs::Bool::ConstPtr& msg) {

    omniRobot.targetIdentified = msg->data;

}


// Callback for location topic - published by /amcl_pose
void location_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

    omniRobot.currentLocation = *msg;

}


Point2f getRandomCoordinate() {
    Point2f p;

    srand(time(0));
    p.x = rand() % 1000;
    p.y = rand() % 1000;

    p.x = MAP_MIN_X + p.x * (MAP_MAX_X - MAP_MIN_X) / 1000;
    p.y = MAP_MIN_Y + p.y * (MAP_MAX_Y - MAP_MIN_Y) / 1000;

    // ROS_INFO("Random point: %f, %f", p.x, p.y);

    return p;
}


// Main function
int main(int argc, char *argv[]) {

    ros::init(argc, argv, "execution");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    ros::Rate rate(1);     // 1Hz

    private_nh.param("allowExplore", allowExplore, false);    

    ros::Subscriber targetIdentified_sub = n.subscribe<std_msgs::Bool>("/targetIdentified", 1, targetIdentified_cb);
    ros::Subscriber location_sub         = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, location_cb);
    
    initExplore_pub                      = n.advertise<std_msgs::Bool>("/initExplore", 1);
    ros::Publisher currentGoal_pub       = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);


    ROS_INFO("Execution node ready -- %s", allowExplore ? "true" : "false");

    while(ros::ok()) {

        //Si currentGoal not reached -> esperar
        //Si currentGoal reached 
            //Preparar siguiente waypoint
            //Comandarlo

        /* Comportamiento en base a mission status */
        switch(omniRobot.mission_status) {
        case MISSION_STAT_PRECHECK:
            
            ROS_INFO("MISSION STATUS: PRECHECK");
            break;

        
        case MISSION_STAT_EXPLORE:
        {
            ROS_INFO("MISSION STATUS: EXPLORE");
            break;
    
        }   //Case MISSION_STAT_EXPLORE

        case MISSION_STAT_WAYPOINTS:
        {
            ROS_INFO("MISSION STATUS: WAYPOINTS");
        
            double dist = distToGoal(omniRobot.currentGoal, omniRobot.currentLocation);
            ROS_INFO("Current Goal: %f, %f", omniRobot.currentGoal.goal.target_pose.pose.position.x, omniRobot.currentGoal.goal.target_pose.pose.position.y);
            ROS_INFO("Current Location: %f, %f", omniRobot.currentLocation.pose.pose.position.x, omniRobot.currentLocation.pose.pose.position.y);
            ROS_INFO("DISSST: %f", dist);

            static ros::Time lastCheck = ros::Time::now();

            if (dist < 0.3) {   // Waypoint reached
                ROS_INFO("Waypoint reached... sending new Waypoint");

                // MoveBaseActionGoal
                omniRobot.currentGoal.header.seq++;
                omniRobot.currentGoal.header.stamp = ros::Time::now();
                omniRobot.currentGoal.goal.target_pose.header.seq++;
                omniRobot.currentGoal.goal.target_pose.header.stamp = ros::Time::now();

                Point2f p = getRandomCoordinate();
                omniRobot.currentGoal.goal.target_pose.pose.position.x = p.x;
                omniRobot.currentGoal.goal.target_pose.pose.position.y = p.y;

                // Publicar siguiente waypoint
                currentGoal_pub.publish(omniRobot.currentGoal);

                lastCheck = ros::Time::now();
            }
            else
            {
            
                double timeDiff = ros::Duration(ros::Time::now() - lastCheck).toSec();

                if (timeDiff > 30) 
                {
                    omniRobot.currentGoal.header.seq++;
                    omniRobot.currentGoal.header.stamp = ros::Time::now();
                    omniRobot.currentGoal.goal.target_pose.header.seq++;
                    omniRobot.currentGoal.goal.target_pose.header.stamp = ros::Time::now();

                    Point2f p = getRandomCoordinate();

                    omniRobot.currentGoal.goal.target_pose.pose.position.x = p.x;
                    omniRobot.currentGoal.goal.target_pose.pose.position.y = p.y;

                    currentGoal_pub.publish(omniRobot.currentGoal);

                    lastCheck = ros::Time::now();
                }

            }

            break;
        }

        case MISSION_STAT_RETURN:

            ROS_INFO("MISSION STATUS: RETURN");
            
            break;

        case MISSION_STAT_COMPLETE:

            ROS_INFO("MISSION STATUS: COMPLETE");

            omniRobot.currentGoal.header.seq++;
            omniRobot.currentGoal.header.stamp = ros::Time::now();
            omniRobot.currentGoal.goal.target_pose.header.seq++;
            omniRobot.currentGoal.goal.target_pose.header.stamp = ros::Time::now();

            Point2f p = getRandomCoordinate();

            omniRobot.currentGoal.goal.target_pose.pose.position.x = initLocation.x;
            omniRobot.currentGoal.goal.target_pose.pose.position.y = initLocation.y;

            currentGoal_pub.publish(omniRobot.currentGoal);

            break;

        }

        // Check and change mission status
        omniRobot.SAR_Robot::changeMissionStatus();

        ros::spinOnce();
        rate.sleep();
    }

    return -1;
}


/* Function to check and change mission status */
void SAR_Robot::changeMissionStatus()  
{
    if (this->mission_status == MISSION_STAT_PRECHECK) {

        // All precondititions have been met?
        if (this->targetIdentified == false)
        {
            if (allowExplore)
            {
                this->mission_status = MISSION_STAT_EXPLORE;

                std_msgs::Bool msg;
                msg.data = true;
                initExplore_pub.publish(msg);

                ROS_INFO("MISSION STATUS: changing to EXPLORE");
            }
            else
            {
                this->mission_status = MISSION_STAT_WAYPOINTS;

                initLocation.x = this->currentLocation.pose.pose.position.x;
                initLocation.y = this->currentLocation.pose.pose.position.y;

                ROS_INFO("MISSION STATUS: changing to WAYPOINTS");
            }
        }

    
    } else if (this->mission_status == MISSION_STAT_EXPLORE) {

        ROS_INFO("EXPLOREEEE");

    } else if (this->mission_status == MISSION_STAT_WAYPOINTS) {

        // Target has been identified while navigating waypoints?
        if (this->targetIdentified) 
        {
            this->mission_status = MISSION_STAT_RETURN;
            ROS_INFO("MISSION STATUS: changing to RETURN");
        }
        

    } else if (this->mission_status == MISSION_STAT_RETURN) {

        // Back to init location?
        this->mission_status = MISSION_STAT_COMPLETE;
        ROS_INFO("MISSION STATUS: MISSION COMPLETE");


    } 
}