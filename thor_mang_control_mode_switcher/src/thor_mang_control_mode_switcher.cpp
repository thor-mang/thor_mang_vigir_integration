#include <ros/ros.h>
#include <thor_mang_control_mode_switcher/thor_mang_control_mode_switcher.h>

namespace control_mode_switcher{
    ControlModeSwitcher::ControlModeSwitcher(ros::NodeHandle &nh):
        control_mode_action_server(nh, "/mode_controllers/control_mode_controller/change_control_mode", boost::bind(&ControlModeSwitcher::executeSwitchControlModeCallback, this, _1), false)

    //Action
    {// /mode_controllers/control_mode_controller/change_control_mode
        // ChangeControlModeAction
        //vigir_humanoid_control_msgs.msg

        //Topic /flor/controller/mode publish changed mode
       nh_ = nh;
    }

     void ControlModeSwitcher::executeSwitchControlModeCallback(const vigir_humanoid_control_msgs::ChangeControlModeGoalConstPtr &goal) {
         vigir_humanoid_control_msgs::ChangeControlModeResult action_result;
         action_result.result.status = action_result.result.MODE_ACCEPTED;
         std::string mode_request = goal->mode_request;
         action_result.result.requested_control_mode = mode_request;
         action_result.result.current_control_mode = mode_request;
         control_mode_action_server.setSucceeded(action_result,"Fake succeeded from control mode switcher");
     }

    ControlModeSwitcher::~ControlModeSwitcher()
    {}

}
