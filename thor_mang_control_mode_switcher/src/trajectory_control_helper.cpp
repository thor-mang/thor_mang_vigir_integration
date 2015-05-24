#include <ros/ros.h>
#include <thor_mang_control_mode_switcher/trajectory_control_helper.h>

namespace control_mode_switcher{
    TrajectoryControlHelper::TrajectoryControlHelper()
    {
       left_hand_trajectory_client_ = new  TrajectoryActionClient("/thor_mang/left_hand_traj_controller/follow_joint_trajectory", true);
       right_hand_trajectory_client_ = new  TrajectoryActionClient("/thor_mang/right_hand_traj_controller/follow_joint_trajectory", true);

       std::vector<std::string> left_arm_joints;
       left_arm_joints.push_back("l_shoulder_pitch"); left_arm_joints.push_back("l_shoulder_roll"); left_arm_joints.push_back("l_shoulder_yaw");
       left_arm_joints.push_back("l_elbow");
       left_arm_joints.push_back("l_wrist_yaw1"); left_arm_joints.push_back("l_wrist_roll"); left_arm_joints.push_back("l_wrist_yaw2");
       controller_joints[LEFT_ARM] = left_arm_joints;
       controller_clients[LEFT_ARM] = new  TrajectoryActionClient("/thor_mang/left_arm_traj_controller/follow_joint_trajectory", true);

       std::vector<std::string> right_arm_joints;
       right_arm_joints.push_back("r_shoulder_pitch"); right_arm_joints.push_back("r_shoulder_roll"); right_arm_joints.push_back("r_shoulder_yaw");
       right_arm_joints.push_back("r_elbow");
       right_arm_joints.push_back("r_wrist_yaw1"); right_arm_joints.push_back("r_wrist_roll"); right_arm_joints.push_back("r_wrist_yaw2");
       controller_joints[RIGHT_ARM] = right_arm_joints;
       controller_clients[RIGHT_ARM] = new  TrajectoryActionClient("/thor_mang/right_arm_traj_controller/follow_joint_trajectory", true);

       std::vector<std::string> left_leg_joints;
       left_leg_joints.push_back("l_ankle_pitch"); left_leg_joints.push_back("l_ankle_roll");
       left_leg_joints.push_back("l_knee");
       left_leg_joints.push_back("l_hip_pitch"); left_leg_joints.push_back("l_hip_roll"); left_leg_joints.push_back("l_hip_yaw");

       controller_joints[LEFT_LEG] = left_leg_joints;
       controller_clients[LEFT_LEG] = new  TrajectoryActionClient("/thor_mang/left_leg_traj_controller/follow_joint_trajectory", true);

       std::vector<std::string> right_leg_joints;
       right_leg_joints.push_back("r_ankle_pitch"); right_leg_joints.push_back("r_ankle_roll");
       right_leg_joints.push_back("r_knee");
       right_leg_joints.push_back("r_hip_pitch"); right_leg_joints.push_back("r_hip_roll"); right_leg_joints.push_back("r_hip_yaw");

       controller_joints[RIGHT_LEG] = right_leg_joints;
       controller_clients[RIGHT_LEG] = new  TrajectoryActionClient("/thor_mang/right_leg_traj_controller/follow_joint_trajectory", true);

       std::vector<std::string> head_joints;
       head_joints.push_back("head_pan"); head_joints.push_back("head_tilt");
       controller_joints[HEAD] = head_joints;
       controller_clients[HEAD] = new  TrajectoryActionClient("/thor_mang/head_traj_controller/follow_joint_trajectory", true);

       std::vector<std::string> torso_joints;
       torso_joints.push_back("waist_pan"); torso_joints.push_back("waist_tilt");
       controller_joints[TORSO] = torso_joints;
       controller_clients[TORSO] = new  TrajectoryActionClient("/thor_mang/torso_traj_controller/follow_joint_trajectory", true);





      }


    TrajectoryControlHelper::~TrajectoryControlHelper()
    {}




    void TrajectoryControlHelper::goToJointConfiguration(std::map<TrajectoryController , std::vector<double> > joint_config, float duration){

        completion_counter = joint_config.size();

        for (std::map<TrajectoryController, std::vector<double> >::iterator iter = joint_config.begin(); iter != joint_config.end(); iter++) {
            std::vector<std::string> names = controller_joints[iter->first];
            std::vector<double> positions = iter->second;

            control_msgs::FollowJointTrajectoryGoal trajectory_goal;

            if (!controller_clients[iter->first]->waitForServer(ros::Duration(5.0)))
                ROS_WARN("[control_mode_switcher] Time out while waititing for left_arm_traj_controller");
            if (controller_clients[iter->first]->isServerConnected() )
            {
                // Goal for left arm
                trajectory_msgs::JointTrajectory joint_trajectory;
                joint_trajectory.joint_names = names;

                trajectory_msgs::JointTrajectoryPoint point;
                point.positions = positions;
                point.time_from_start = ros::Duration(duration);
                joint_trajectory.points.push_back(point);

                trajectory_goal.trajectory = joint_trajectory;

                //Send goals to controllers
                controller_clients[iter->first]->sendGoal(trajectory_goal, boost::bind(&TrajectoryControlHelper::trajectoryDoneCb, this, _1, _2),
                                                          boost::bind(&TrajectoryControlHelper::trajectoryActiveCB, this),
                                                          boost::bind(&TrajectoryControlHelper::trajectoryFeedbackCB, this, _1));
            }

            else{
                //ROS_WARN("[control_mode_switcher] Can g");
            }




        }

        ROS_INFO("[control_mode_switcher] starting to wait for feedback of trajectory control action");

        //TODO testing
        ros::Duration max_wait_time(2*duration);
        ros::Time begin = ros::Time::now();
        ros::Rate rate(ros::Duration(0.1));
        while (completion_counter>0){
            if (ros::Time::now() - begin > max_wait_time) {
                ROS_WARN("[control_mode_switcher] Timeout in send trajectory");
                completion_counter = 0;
                break;
            }
            rate.sleep();
        }

    }



    void TrajectoryControlHelper::trajectoryActiveCB()
    {    }

    void TrajectoryControlHelper::trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
    {    }

    void TrajectoryControlHelper::trajectoryDoneCb(const actionlib::SimpleClientGoalState& state,
                                               const control_msgs::FollowJointTrajectoryResultConstPtr& result)
    {        completion_counter--; }


    //template<typename T>
    std::vector<double> TrajectoryControlHelper::makeVector(double values[], int length){

        std::vector<double> result(values,values+length);
        return result;
    }

}

