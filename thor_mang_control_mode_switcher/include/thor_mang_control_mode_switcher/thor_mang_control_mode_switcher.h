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
namespace control_mode_switcher{
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryActionClient;

    class ControlModeSwitcher {

    public:
      ControlModeSwitcher(ros::NodeHandle& nh);
      virtual ~ControlModeSwitcher();

    protected:
     void executeSwitchControlModeCallback(const vigir_humanoid_control_msgs::ChangeControlModeGoalConstPtr& goal);
     void goToStandMode();
     bool stand_complete;
     void trajectoryActiveCB();

     void trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
     void trajectoryDoneCb(const actionlib::SimpleClientGoalState& state,
                                                       const control_msgs::FollowJointTrajectoryResultConstPtr& result);


    private:
     ros::NodeHandle nh_;
     ros::Publisher mode_changed_pub_;
     actionlib::SimpleActionServer<vigir_humanoid_control_msgs::ChangeControlModeAction> control_mode_action_server;
     //actionlib::SimpleActionClient<vigir_planning_msgs::MoveAction> trajectory_client_;

     ros::ServiceClient execute_kinematic_path_client_;
     TrajectoryActionClient* trajectory_client_left_;
     TrajectoryActionClient* trajectory_client_right_;

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
