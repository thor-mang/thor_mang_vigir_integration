#ifndef THOR_MANG_CONTROL_MODE_SWITCHER_H
#define THOR_MANG_CONTROL_MODE_SWITCHER_H

#include <ros/ros.h>
#include <vigir_humanoid_control_msgs/ChangeControlModeAction.h>
#include <actionlib/server/simple_action_server.h>
#include <flor_control_msgs/FlorControlMode.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
namespace control_mode_switcher{

    class ControlModeSwitcher {

    public:
      ControlModeSwitcher(ros::NodeHandle& nh);
      virtual ~ControlModeSwitcher();

    protected:
     void executeSwitchControlModeCallback(const vigir_humanoid_control_msgs::ChangeControlModeGoalConstPtr& goal);
     void goToStandMode();

    private:
     ros::NodeHandle nh_;
     ros::Publisher mode_changed_pub_;
     actionlib::SimpleActionServer<vigir_humanoid_control_msgs::ChangeControlModeAction> control_mode_action_server;
     ros::ServiceClient execute_kinematic_path_client_;

    };
}

namespace thor_mang_control_mode{

    const unsigned char NONE = 0;
    const unsigned char FREEZE = 1;
    const unsigned char STAND_PREP = 2;
    const unsigned char STAND = 3;
    const unsigned char STAND_MANIPULATE = 3;
    const unsigned char WALK = 4;
    const unsigned char STEP = 5;
    const unsigned char MANIPULATE = 6;
    const unsigned char USER = 7;
    const unsigned char CALIBRATE = 8;
    const unsigned char SOFT_STOP = 9;
}


#endif // THOR_MANG_CONTROL_MODE_SWITCHER_H
