#ifndef THOR_MANG_CONTROL_MODE_SWITCHER_H
#define THOR_MANG_CONTROL_MODE_SWITCHER_H

#include <ros/ros.h>
#include <vigir_humanoid_control_msgs/ChangeControlModeAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <flor_control_msgs/FlorControlMode.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <vigir_planning_msgs/MoveAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>
#include <vigir_footstep_planning_msgs/ExecuteStepPlanActionGoal.h>
#include <vigir_footstep_planning_msgs/ExecuteStepPlanActionResult.h>
#include <vigir_footstep_planning_msgs/ExecuteStepPlanAction.h>
#include <vigir_footstep_planning_msgs/ExecuteStepPlanGoal.h>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <thor_mang_control_mode_switcher/trajectory_control_helper.h>

namespace control_mode_switcher{
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryActionClient;
typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::ExecuteStepPlanAction> StepPlanActionClient;

    class ControlModeSwitcher {

    public:
      ControlModeSwitcher(ros::NodeHandle& nh);
      virtual ~ControlModeSwitcher();

    protected:
     void executeSwitchControlModeCallback(const vigir_humanoid_control_msgs::ChangeControlModeGoalConstPtr& goal);
     void goToStandMode();
     void goToSoftStop();
     void goToShutdownMode1();
     void goToShutdownMode2();
     void goToShutdownMode3();
     void goToShutdownMode4();
     void goToShutdownMode5();

     void getStartedAndStoppedControllers();
     void notifyNewControlMode(std::string new_mode, int new_idx, flor_control_msgs::FlorControlMode msg);
     void executeFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionGoalConstPtr& goal);
     void resultFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionResultConstPtr& result);
     void ocsModeChangeCb(const flor_control_msgs::FlorControlModeCommand& mode);
     void allowAllModeTransitionsCb(const std_msgs::Bool & allow);
     bool changeControlMode(std::string mode_request);
     bool switchControllers(std::vector<std::string> controllers_to_start);
     bool switchToTrajectoryControllers();
     bool switchToWalkingControllers();
     bool switchToWalkManipulateControllers();

     void trajectoryActiveCB();
     void trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
     void trajectoryLeftArmDoneCb(const actionlib::SimpleClientGoalState& state,
                                                       const control_msgs::FollowJointTrajectoryResultConstPtr& result);
     void trajectoryRightArmDoneCb(const actionlib::SimpleClientGoalState& state,
                                                       const control_msgs::FollowJointTrajectoryResultConstPtr& result);
     void stepPlanActiveCB();
     void stepPlanFeedbackCB(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback);
     void stepPlanDoneCb(const actionlib::SimpleClientGoalState& state,
                                                       const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result);


    private:
     ros::NodeHandle nh_;
     ros::Publisher mode_changed_pub_;
     ros::Publisher mode_name_pub_;
     ros::Publisher allow_all_mode_transitions_ack_pub_;
     ros::Publisher stand_prep_calibration_pub_;

     ros::Subscriber execute_footstep_sub_;
     ros::Subscriber result_footstep_sub_;
     ros::Subscriber ocs_mode_switch_sub_;
     ros::Subscriber allow_all_mode_transitions_sub_;

     actionlib::SimpleActionServer<vigir_humanoid_control_msgs::ChangeControlModeAction> control_mode_action_server;

     ros::ServiceClient switch_controllers_client_;
     ros::ServiceClient list_controllers_client_;
     TrajectoryActionClient* trajectory_client_left_;
     TrajectoryActionClient* trajectory_client_right_;
     StepPlanActionClient* stepplan_client_;

     std::vector<std::string> started_controllers;
     std::vector<std::string> stopped_controllers;

     bool run_on_real_robot;
     bool stand_complete_right;
     bool stand_complete_left;
     bool allow_all_mode_transitions;

     std::string current_mode_;
     int current_mode_int_;

     std::vector<std::string> allowed_control_modes;
     std::vector<int> bdi_control_modes;
     std::vector<int> flor_control_modes;
     std::vector < std::vector<std::string> > desired_controllers;
     std::vector <std::string> default_desired_controllers;
     std::vector < std::vector<std::string> > allowed_transitions;
     std::vector <std::string> default_allowed_transitions;

     TrajectoryControlHelper trajectory_control_helper;

     bool accept_new_mode_change;

    };
}

#endif // THOR_MANG_CONTROL_MODE_SWITCHER_H
