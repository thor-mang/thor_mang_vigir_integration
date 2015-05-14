#include <ros/ros.h>
#include <thor_mang_control_mode_switcher/thor_mang_control_mode_switcher.h>

namespace control_mode_switcher{
    ControlModeSwitcher::ControlModeSwitcher(ros::NodeHandle &nh):
        control_mode_action_server(nh, "/mode_controllers/control_mode_controller/change_control_mode", boost::bind(&ControlModeSwitcher::executeSwitchControlModeCallback, this, _1), false)
    {
       nh_ = nh;
       control_mode_action_server.start();
       mode_changed_pub_ = nh_.advertise<flor_control_msgs::FlorControlMode>("/flor/controller/mode", 10, false);
       //execute_kinematic_path_client_ = nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");

       //trajectory_client_ = new  TrajectoryActionClient("/vigir_move_group", true);
       trajectory_client_left_ = new  TrajectoryActionClient("/thor_mang/left_leg_traj_controller/follow_joint_trajectory", true);
       trajectory_client_right_ = new  TrajectoryActionClient("/thor_mang/right_leg_traj_controller/follow_joint_trajectory", true);

    }


    ControlModeSwitcher::~ControlModeSwitcher()
    {}


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
             goToStandMode();
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

    void ControlModeSwitcher::goToStandMode(){

        std::vector<std::string> names_l, names_r;
        std::vector<double> positions_l, positions_r;

        names_l.push_back("l_ankle_pitch");   positions_l.push_back(0.5920838259292829);
        names_l.push_back("l_ankle_roll");    positions_l.push_back(-0.06228113556334662);
        names_l.push_back("l_hip_pitch");     positions_l.push_back(-0.736909995463745);
        names_l.push_back("l_hip_roll");      positions_l.push_back(-0.062043325761155385);
        names_l.push_back("l_hip_yaw");       positions_l.push_back(0.0);
        names_l.push_back("l_knee");          positions_l.push_back(1.1722896780543826);
        names_r.push_back("r_ankle_pitch");   positions_r.push_back(-0.5921088585400399);
        names_r.push_back("r_ankle_roll");    positions_r.push_back(0.06164280398904383);
        names_r.push_back("r_hip_pitch");     positions_r.push_back(0.7369350280745021);
        names_r.push_back("r_hip_roll");      positions_r.push_back(0.06183054856972112);
        names_r.push_back("r_hip_yaw");       positions_r.push_back(0.0);
        names_r.push_back("r_knee");          positions_r.push_back(-1.1723397432758964);




        control_msgs::FollowJointTrajectoryGoal trajectory_goal_r_;
        control_msgs::FollowJointTrajectoryGoal trajectory_goal_l_;
        if (!trajectory_client_left_->waitForServer(ros::Duration(5.0)))
            ROS_WARN("[control_mode_changer] Time out while waititing for left_leg_traj_controller");
        if (!trajectory_client_right_->waitForServer(ros::Duration(5.0)))
            ROS_WARN("[control_mode_changer] Time out while waititing for right_leg_traj_controller");
        if (trajectory_client_left_->isServerConnected() && trajectory_client_right_->isServerConnected() )
        {
            // Goal for left leg
            trajectory_msgs::JointTrajectory joint_trajectory_l;
            joint_trajectory_l.joint_names = names_l;

            trajectory_msgs::JointTrajectoryPoint point_l;
            point_l.positions = positions_l;
            point_l.time_from_start = ros::Duration(2.0);
            joint_trajectory_l.points.push_back(point_l);

            trajectory_goal_l_.trajectory = joint_trajectory_l;

            //Goal for right leg
            trajectory_msgs::JointTrajectory joint_trajectory_r;
            joint_trajectory_r.joint_names = names_r;

            trajectory_msgs::JointTrajectoryPoint point_r;
            point_r.positions = positions_r;
            point_r.time_from_start = ros::Duration(2.0);
            joint_trajectory_r.points.push_back(point_r);


            trajectory_goal_r_.trajectory = joint_trajectory_r;


            //Send goals to controllers
            trajectory_client_left_->sendGoal(trajectory_goal_l_, boost::bind(&ControlModeSwitcher::trajectoryDoneCb, this, _1, _2),
                                         boost::bind(&ControlModeSwitcher::trajectoryActiveCB, this),
                                         boost::bind(&ControlModeSwitcher::trajectoryFeedbackCB, this, _1));

            trajectory_client_right_->sendGoal(trajectory_goal_r_, boost::bind(&ControlModeSwitcher::trajectoryDoneCb, this, _1, _2),
                                         boost::bind(&ControlModeSwitcher::trajectoryActiveCB, this),
                                         boost::bind(&ControlModeSwitcher::trajectoryFeedbackCB, this, _1));
            stand_complete = false;

            ros::Rate rate(ros::Duration(0.1));
            while (!stand_complete){
                rate.sleep();
            }
            return;
        }
        else{
            ROS_WARN("[control_mode_changer] Skipping Leg Motion");
        }

//        for (int i=0; i < names.size(); i++) {
//          velocities.push_back(0.0);
//          accelerations.push_back(0.0);
//        }

//        trajectory_msgs::JointTrajectoryPoint point;
//        point.positions = positions;
//        point.velocities = velocities;
//        point.accelerations = accelerations;
//        point.time_from_start = ros::Duration(4.0);

//        vigir_planning_msgs::MoveGoal trajectory_action_;
//        while (!trajectory_client_->waitForServer(ros::Duration(5.0)))
//           ROS_INFO("[control_mode_changer] Waititing for lower Body TrajectoryActionServer");
//        if (trajectory_client_->isServerConnected())
//        {
//            trajectory_action_.request.group_name = "lower_body_group";
//            moveit_msgs::Constraints constraints;
//            moveit_msgs::JointConstraint joint_constraint;

//            for (int i=0; i<names.size(); i++) {
//                joint_constraint.joint_name = names[i];
//                joint_constraint.position = positions[i];
//                constraints.joint_constraints.push_back(joint_constraint);
//            }

//            trajectory_action_.request.goal_constraints.push_back(constraints);
//            trajectory_client_->sendGoal(trajectory_action_, boost::bind(&ControlModeSwitcher::trajectoryDoneCb, this, _1, _2),
//                                         boost::bind(&ControlModeSwitcher::trajectoryActiveCB, this),
//                                         boost::bind(&ControlModeSwitcher::trajectoryFeedbackCB, this, _1));
//            stand_complete = false;

//            ros::Rate rate(ros::Duration(0.1));
//            while (!stand_complete){
//                rate.sleep();
//            }
//            return;
//        }

    }

    void ControlModeSwitcher::trajectoryActiveCB()
    {

    }

    void ControlModeSwitcher::trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
    {
    }

    void ControlModeSwitcher::trajectoryDoneCb(const actionlib::SimpleClientGoalState& state,
                                               const control_msgs::FollowJointTrajectoryResultConstPtr& result)
    {
        stand_complete = true;
    }


}


