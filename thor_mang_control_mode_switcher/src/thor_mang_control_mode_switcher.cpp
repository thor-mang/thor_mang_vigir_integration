#include <ros/ros.h>
#include <thor_mang_control_mode_switcher/thor_mang_control_mode_switcher.h>

namespace control_mode_switcher{
    ControlModeSwitcher::ControlModeSwitcher(ros::NodeHandle &nh):
        control_mode_action_server(nh, "/mode_controllers/control_mode_controller/change_control_mode", boost::bind(&ControlModeSwitcher::executeSwitchControlModeCallback, this, _1), false)
    {
       nh_ = nh;
       control_mode_action_server.start();
       mode_changed_pub_ = nh_.advertise<flor_control_msgs::FlorControlMode>("/flor/controller/mode", 10, false);

    }


     void ControlModeSwitcher::executeSwitchControlModeCallback(const vigir_humanoid_control_msgs::ChangeControlModeGoalConstPtr &goal) {

         flor_control_msgs::FlorControlMode changed_mode_msg;
         vigir_humanoid_control_msgs::ChangeControlModeResult action_result;
         std::string mode_request = goal->mode_request;

         // Publish changed mode       
         changed_mode_msg.header.stamp = ros::Time::now();

          if (mode_request == "calibrate")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::CALIBRATE;
             changed_mode_msg.control_mode = thor_mang_control_mode::CALIBRATE;
          }

          else if (mode_request == "freeze")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::FREEZE;
             changed_mode_msg.control_mode = thor_mang_control_mode::FREEZE;
          }

          else if (mode_request == "manipulate")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::MANIPULATE;
             changed_mode_msg.control_mode = thor_mang_control_mode::MANIPULATE;
          }

          else if (mode_request == "none")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::NONE;
             changed_mode_msg.control_mode = thor_mang_control_mode::NONE;
          }

          else if (mode_request == "soft_stop")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::SOFT_STOP;
             changed_mode_msg.control_mode = thor_mang_control_mode::SOFT_STOP;
          }

          else if (mode_request == "stand")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::STAND;
             changed_mode_msg.control_mode = thor_mang_control_mode::STAND;
          }

          else if (mode_request == "stand_manipulate")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::STAND_MANIPULATE;
             changed_mode_msg.control_mode = thor_mang_control_mode::STAND_MANIPULATE;
          }

          else if (mode_request == "stand_prep")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::STAND_PREP;
             changed_mode_msg.control_mode = thor_mang_control_mode::STAND_PREP;
          }

          else if (mode_request == "step")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::STEP;
             changed_mode_msg.control_mode = thor_mang_control_mode::STEP;
          }

          else if (mode_request == "user")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::USER;
             changed_mode_msg.control_mode = thor_mang_control_mode::USER;
          }

          else if (mode_request == "walk")  {
             changed_mode_msg.bdi_current_behavior = thor_mang_control_mode::WALK;
             changed_mode_msg.control_mode = thor_mang_control_mode::WALK;
          }

          else {


            action_result.result.status = action_result.result.MODE_REJECTED;
            action_result.result.requested_control_mode = mode_request;
            action_result.result.current_control_mode = mode_request;
            control_mode_action_server.setAborted(action_result,"Fake succeeded from control mode switcher");

            ROS_WARN("Not a known control mode for thor, returning NOT SUCEEDED");
            return;
          }

          // If requested mode in known publish changed mode

          mode_changed_pub_.publish(changed_mode_msg);

          // Set Action Goal as succeeded
          action_result.result.status = action_result.result.MODE_ACCEPTED;
          action_result.result.requested_control_mode = mode_request;
          action_result.result.current_control_mode = mode_request;
          control_mode_action_server.setSucceeded(action_result,"Fake succeeded from control mode switcher");

     }

    ControlModeSwitcher::~ControlModeSwitcher()
    {}

}


