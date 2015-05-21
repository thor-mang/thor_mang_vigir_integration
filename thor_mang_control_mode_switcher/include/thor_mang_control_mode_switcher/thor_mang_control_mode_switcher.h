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
#include <flor_control_msgs/FlorControlModeCommand.h>

namespace control_mode_switcher{
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryActionClient;

    class ControlModeSwitcher {

    public:
      ControlModeSwitcher(ros::NodeHandle& nh);
      virtual ~ControlModeSwitcher();

    protected:
     void executeSwitchControlModeCallback(const vigir_humanoid_control_msgs::ChangeControlModeGoalConstPtr& goal);
     void goToStandMode();

     void trajectoryActiveCB();
     void getStartedAndStoppedControllers();
     void executeFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionGoalConstPtr& goal);
     void resultFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionResultConstPtr& result);
     void ocsModeChangeCb(const flor_control_msgs::FlorControlModeCommand& mode);
     void changeControlMode(std::string mode_request);
     bool switchControllers(std::vector<std::string> controllers_to_start);
     bool switchToTrajectoryControllers();
     bool switchToWalkingControllers();
     bool switchToWalkManipulateControllers();
     void trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
     void trajectoryDoneCb(const actionlib::SimpleClientGoalState& state,
                                                       const control_msgs::FollowJointTrajectoryResultConstPtr& result);


    private:
     ros::NodeHandle nh_;
     ros::Publisher mode_changed_pub_;
     ros::Publisher mode_name_pub_;

     ros::Subscriber execute_footstep_sub_;
     ros::Subscriber result_footstep_sub_;
     ros::Subscriber ocs_mode_switch_sub_;

     actionlib::SimpleActionServer<vigir_humanoid_control_msgs::ChangeControlModeAction> control_mode_action_server;

     ros::ServiceClient switch_controllers_client_;
     ros::ServiceClient list_controllers_client_;
     TrajectoryActionClient* trajectory_client_left_;
     TrajectoryActionClient* trajectory_client_right_;

     std::vector<std::string> started_controllers;
     std::vector<std::string> stopped_controllers;

     bool run_on_real_robot;
     bool stand_complete;

     std::string current_mode_;
     int current_mode_int_;

     std::vector<std::string> allowed_control_modes;
     std::vector<int> bdi_control_modes;
     std::vector<int> flor_control_modes;
     std::vector < std::vector<std::string> > desired_controllers;
     std::vector <std::string> default_desired_controllers;
     std::vector < std::vector<std::string> > allowed_transitions;
     std::vector <std::string> default_allowed_transitions;

    };
}

namespace thor_mang_bdi_control_mode{

    const unsigned char NONE = 0;
    const unsigned char FREEZE = 1;
    const unsigned char STAND_PREP = 2;
    const unsigned char STAND = 3;
    const unsigned char STAND_MANIPULATE = 3;
    const unsigned char IMPEDANCE_STIFF = 6;
    const unsigned char	IMPEDANCE_COMPLIANT = 6;
    const unsigned char	IMPEDANCE_OBSERVER = 6;
    const unsigned char WALK = 4;
    const unsigned char STEP = 5;
    const unsigned char MANIPULATE = 6;
    const unsigned char USER = 7;
    const unsigned char CALIBRATE = 8;
    const unsigned char SOFT_STOP = 9;
}

namespace thor_mang_control_mode{
const unsigned char NONE = 0;
const unsigned char STOP = 1;
const unsigned char FREEZE = 2;
const unsigned char STAND_PREP = 3;
const unsigned char STAND = 4;
const unsigned char WALK = 5;
const unsigned char STEP = 6;
const unsigned char MANIPULATE = 7;
const unsigned char WHOLE_BODY = 8;
const unsigned char DANCE = 9;
const unsigned char CALIBRATE = 10;
const unsigned char SOFT_STOP = 11;
const unsigned char STAND_MANIPULATE = 12;
const unsigned char WALK_MANIPULATE = 13;
const unsigned char STEP_MANIPULATE = 14;
//const unsigned char MANIPULATE_GRAVITY = #        - "manipulate_gravity"
//const unsigned char MANIPULATE_INVERSE_DYNAMICS =#        - "manipulate_inverse_dynamics"
const unsigned char MANIPULATE_LIMITS = 15 ; //- "manipulate_limits"
 //const unsigned char#        - "manipulate_limits_stabilized"
 //const unsigned char#        - "manipulate_friction_with_gravity"
const unsigned char MANIPULATE_COMPLIANT_IMPEDANCE = 16 ; //"manipulate_compliant_impedance"
const unsigned char MANIPULATE_STIFF_IMPEDANCE = 17 ; //"manipulate_stiff_impedance"
const unsigned char MANIPULATE_OBSERVER_IMPEDANCE = 18 ; // "manipulate_observer_impedance"
const unsigned char DANCE_IMPEDANCE = 19 ; //"dance_impedance"

}


#endif // THOR_MANG_CONTROL_MODE_SWITCHER_H
