
#include "sar.hpp"


// /* Function to check and change mission status */
// bool SAR_Robot::checkMissionStatus()  
// {
//     if (this->mission_status == MISSION_STAT_PRECHECK) {

//         // All precondititions have been met?
//         this->mission_status = MISSION_STAT_WAYPOINTS;
//         ROS_INFO("MISSION STATUS: changing to WAYPOINTS");

//         return 1;

//     } else if (this->mission_status == MISSION_STAT_WAYPOINTS) {

//         // Target has been identified while navigating waypoints?
//         if (this->targetIdentified) 
//         {
//             this->mission_status = MISSION_STAT_RETURN;
//             ROS_INFO("MISSION STATUS: changing to RETURN");
//         }
        
//         return 1;

//     } else if (this->mission_status == MISSION_STAT_RETURN) {

//         // Back to init location?
//         this->mission_status = MISSION_STAT_COMPLETE;
//         ROS_INFO("MISSION STATUS: MISSION COMPLETE");

//         return 0;

//     } 
// }