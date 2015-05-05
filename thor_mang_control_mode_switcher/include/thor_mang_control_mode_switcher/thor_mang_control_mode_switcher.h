#ifndef THOR_MANG_CONTROL_MODE_SWITCHER_H
#define THOR_MANG_CONTROL_MODE_SWITCHER_H

#include <ros/ros.h>
#include <vigir_humanoid_control_msgs/ChangeControlModeAction.h>
#include <actionlib/server/simple_action_server.h>
namespace control_mode_switcher{

    class ControlModeSwitcher {

    public:
      ControlModeSwitcher(ros::NodeHandle& nh);
      virtual ~ControlModeSwitcher();
    protected:
     void executeSwitchControlModeCallback(const vigir_humanoid_control_msgs::ChangeControlModeGoalConstPtr& goal);
    private:
     //actionlib::SimpleActionServer<vigir_humanoid_control_msgs::ChangeControlModeAction> control_mode_action_server;
     ros::NodeHandle nh_;
     actionlib::SimpleActionServer<vigir_humanoid_control_msgs::ChangeControlModeAction> control_mode_action_server;
    };
}


#endif // THOR_MANG_CONTROL_MODE_SWITCHER_H
