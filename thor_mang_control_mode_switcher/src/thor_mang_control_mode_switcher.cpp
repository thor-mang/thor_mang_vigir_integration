#include <ros/ros.h>
#include <thor_mang_control_mode_switcher/thor_mang_control_mode_switcher.h>

namespace control_mode_switcher{
    ControlModeSwitcher::ControlModeSwitcher(ros::NodeHandle &nh):
        control_mode_action_server(nh, "/mode_controllers/control_mode_controller/change_control_mode", boost::bind(&ControlModeSwitcher::executeSwitchControlModeCallback, this, _1), false)
    {
       nh_ = nh;
       nh_.getParam("/atlas_controller/allowed_control_modes",allowed_control_modes);

       for (int i= 0; i < allowed_control_modes.size(); i++) {
           std::string temp_mode_string = "/atlas_controller/control_mode_to_controllers/"+allowed_control_modes[i]+"/bdi_mode";
           int temp_bdi_mode;
           nh_.param(temp_mode_string,temp_bdi_mode,-2);
           bdi_control_modes.push_back(temp_bdi_mode);

           std::string temp_controller_string = "/atlas_controller/control_mode_to_controllers/"+allowed_control_modes[i]+"/desired_controllers_to_start";

           std::vector <std::string> temp_controllers;
           nh_.getParam(temp_controller_string,temp_controllers);
           desired_controllers.push_back(temp_controllers);

           std::string temp_transitions_string = "/atlas_controller/control_mode_to_controllers/"+allowed_control_modes[i]+"/transitions";

           std::vector <std::string> temp_transitions;
           nh_.getParam(temp_transitions_string,temp_transitions);
           allowed_transitions.push_back(temp_transitions);

       }


       nh_.getParam("/atlas_controller/control_mode_to_controllers/all/desired_controllers_to_start",default_desired_controllers);
       nh_.getParam("/atlas_controller/control_mode_to_controllers/all/transitions",default_allowed_transitions);

       allow_all_mode_transitions = false;
       nh_.param("run_on_real_robot", run_on_real_robot,true);
       control_mode_action_server.start();
       mode_changed_pub_ = nh_.advertise<flor_control_msgs::FlorControlMode>("/flor/controller/mode", 10, true);
       mode_name_pub_ = nh_.advertise<std_msgs::String>("/flor/controller/mode_name", 10, true);
       allow_all_mode_transitions_ack_pub_ = nh_.advertise<std_msgs::Bool>("/mode_controllers/control_mode_controller/allow_all_mode_transitions_acknowledgement", 10, false);
       stand_prep_calibration_pub_ = nh_.advertise<std_msgs::Empty>("/thor_mang/start_calibration", 10, false);

       execute_footstep_sub_ = nh_.subscribe("/vigir/footstep_manager/execute_step_plan/goal", 10, &ControlModeSwitcher::executeFootstepCb, this);
       result_footstep_sub_ = nh_.subscribe("/vigir/footstep_manager/execute_step_plan/result", 10, &ControlModeSwitcher::resultFootstepCb, this);
       ocs_mode_switch_sub_ = nh_.subscribe("/flor/controller/mode_command", 10, &ControlModeSwitcher::ocsModeChangeCb, this);
       allow_all_mode_transitions_sub_ = nh_.subscribe("/mode_controllers/control_mode_controller/allow_all_mode_transitions", 10, &ControlModeSwitcher::allowAllModeTransitionsCb, this);

       switch_controllers_client_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("/thor_mang/controller_manager/switch_controller");
       list_controllers_client_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("/thor_mang/controller_manager/list_controllers");

       //trajectory_client_ = new  TrajectoryActionClient("/vigir_move_group", true);
       trajectory_client_left_ = new  TrajectoryActionClient("/thor_mang/left_arm_traj_controller/follow_joint_trajectory", true);
       trajectory_client_right_ = new  TrajectoryActionClient("/thor_mang/right_arm_traj_controller/follow_joint_trajectory", true);

       getStartedAndStoppedControllers();
//       started_controllers.push_back("joint_state_controller");
//       started_controllers.push_back("joint_state_controller");
//       // /step_controller  -> + zweiter typ spater ohne haende /step_manipulate_controller

       flor_control_msgs::FlorControlMode changed_mode_msg;
       changed_mode_msg.header.stamp = ros::Time::now();
       changed_mode_msg.bdi_current_behavior = 1;
       changed_mode_msg.control_mode = 0;

       notifyNewControlMode("none", 0, changed_mode_msg);
    }


    ControlModeSwitcher::~ControlModeSwitcher()
    {}


     void ControlModeSwitcher::executeSwitchControlModeCallback(const vigir_humanoid_control_msgs::ChangeControlModeGoalConstPtr &goal) {
           std::string mode_request = goal->mode_request;
           bool switch_successfull = changeControlMode( mode_request);

           vigir_humanoid_control_msgs::ChangeControlModeResult action_result;
           // If requested mode in known publish changed mode
           if (switch_successfull){

               // Set Action Goal as succeeded
               action_result.result.status = action_result.result.MODE_ACCEPTED;
               action_result.result.requested_control_mode = mode_request;
               action_result.result.current_control_mode = mode_request;
               control_mode_action_server.setSucceeded(action_result,"Fake succeeded from control mode switcher");

           }

           else{
               action_result.result.status = action_result.result.MODE_REJECTED;
               action_result.result.requested_control_mode = mode_request;
               action_result.result.current_control_mode = current_mode_;
               control_mode_action_server.setAborted(action_result,"Fake succeeded from control mode switcher");

           }

     }

     void ControlModeSwitcher::executeFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionGoalConstPtr& goal){
         //if (!goal->goal.step_plan.steps.empty()){
         std::string new_mode = (current_mode_ == "stand_manipulate")? "walk_manipulate" : "walk";
         changeControlMode(new_mode);
         //}
         //TODO check for empty plans
     }         //}



     void ControlModeSwitcher::resultFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionResultConstPtr& result){
         if ( (current_mode_ == "walk") || (current_mode_ =="walk_manipulate") ){
         std::string new_mode = (current_mode_ == "walk_manipulate")? "stand_manipulate" : "stand";
         changeControlMode(new_mode);
         }

     }

     void ControlModeSwitcher::ocsModeChangeCb(const flor_control_msgs::FlorControlModeCommand& mode){
        int requested_mode = mode.requested_control_mode;
        std::string switch_mode = allowed_control_modes[requested_mode];
        changeControlMode(switch_mode);
     }

     void ControlModeSwitcher::allowAllModeTransitionsCb(const std_msgs::Bool & allow){

         allow_all_mode_transitions = allow.data;

         std_msgs::Bool ack;
         ack.data = allow_all_mode_transitions;
         allow_all_mode_transitions_ack_pub_.publish(ack);

         ROS_INFO("[control_mode_switcher] Allowing all transitions between modes");

     }

     bool ControlModeSwitcher::changeControlMode(std::string mode_request){

         // Ignore case
         std::transform(mode_request.begin(), mode_request.end(), mode_request.begin(), ::tolower);

         if (current_mode_ == mode_request){
             ROS_INFO("[control_mode_switcher] No switch necessary, robot already in %s !", mode_request.c_str());
             return true;
         }

         // test if the requested transition is allowed
         bool switch_successfull = true;
         flor_control_msgs::FlorControlMode changed_mode_msg;
         int mode_idx_int = current_mode_int_;
         bool transition_ok = false;


         std::vector <std::string> transitions_allowed;
         transitions_allowed= default_allowed_transitions;
         for (int i = 0; i< allowed_transitions[current_mode_int_].size();i++){
             transitions_allowed.push_back(allowed_transitions[current_mode_int_][i]);
         }

         transition_ok = (  find (transitions_allowed.begin (),transitions_allowed.end (), mode_request ) < transitions_allowed.end() );

         if (allow_all_mode_transitions) {
             if(!transition_ok){
                 ROS_WARN("[control_mode_switcher] Normally not allowed to switch from %s to %s - but currently manually enabled",current_mode_.c_str(),mode_request.c_str());
             }
             transition_ok = true;
         }


         if(! transition_ok){
             switch_successfull = false;
             ROS_WARN("[control_mode_switcher] Not allowed to switch from %s to %s - returning NOT SUCEEDED",current_mode_.c_str(),mode_request.c_str());
         }


         else{

             // get index of requested mode
             std::vector<std::string>::iterator mode_idx= find (allowed_control_modes.begin (),allowed_control_modes.end (), mode_request );

             if( mode_idx == allowed_control_modes.end() ) {
                 mode_idx_int = -1;
                 switch_successfull = false;
                 ROS_WARN("[control mode changer] %s is not a known control mode for thor, returning NOT SUCEEDED",mode_request.c_str());
             }
             else{
                 mode_idx_int = std::distance( allowed_control_modes.begin(), mode_idx );


                 // Publish changed mode
                 changed_mode_msg.header.stamp = ros::Time::now();

                 if (mode_request != "none"){
                     getStartedAndStoppedControllers();

                     //to do load right controllers
                     std::vector <std::string> controllers_to_start;
                     controllers_to_start=default_desired_controllers;

                     for (int i = 0; i< desired_controllers[mode_idx_int].size();i++){
                         controllers_to_start.push_back(desired_controllers[mode_idx_int][i]);
                     }

                     if (mode_request == "soft_stop") {

                         if (current_mode_ == "walk") {
                             controllers_to_start.push_back("step_controller");
                         }

                         else if (current_mode_ == "walk_manipulate") {
                             controllers_to_start.push_back("step_manipulate_controller");
                         }

                     }

                     switch_successfull = switchControllers(controllers_to_start);

                 }

                 changed_mode_msg.bdi_current_behavior = bdi_control_modes[mode_idx_int];
                 changed_mode_msg.control_mode = mode_idx_int;

             }

         }

         // If requested mode in known publish changed mode
         if (switch_successfull){
             if (mode_request == "stand"){
                 goToStandMode();
             }
             if (mode_request == "stand_prep"){
                 stand_prep_calibration_pub_.publish(std_msgs::Empty());
             }

             notifyNewControlMode(mode_request, mode_idx_int, changed_mode_msg);


         }

         else{
             ROS_WARN("[control_mode_switcher] Not possible to switch to requested mode %s", mode_request.c_str());
         }

         return switch_successfull;

     }

     void ControlModeSwitcher::notifyNewControlMode(std::string new_mode, int new_idx, flor_control_msgs::FlorControlMode msg){
         mode_changed_pub_.publish(msg);
         std_msgs::String mode_name;
         mode_name.data = new_mode;
         mode_name_pub_.publish(mode_name);
         current_mode_ = new_mode;
         current_mode_int_ = new_idx;
         ROS_INFO("[control_mode_switcher] Successfully switched to mode %s !", new_mode.c_str());
     }

    void ControlModeSwitcher::goToStandMode(){

        std::vector<std::string> names_l, names_r;
        std::vector<double> positions_l, positions_r;

        names_l.push_back("l_shoulder_pitch");   positions_l.push_back(0.79);
        names_l.push_back("l_shoulder_roll");    positions_l.push_back(-0.27);
        names_l.push_back("l_shoulder_yaw");    positions_l.push_back(0.0);
        names_l.push_back("l_elbow");     positions_l.push_back(-1.57);
        names_l.push_back("l_wrist_yaw1");      positions_l.push_back(1.55);
        names_l.push_back("l_wrist_roll");       positions_l.push_back(0.0);
        names_l.push_back("l_wrist_yaw2");          positions_l.push_back(0.0);

        names_r.push_back("r_shoulder_pitch");   positions_r.push_back(-0.79);
        names_r.push_back("r_shoulder_roll");    positions_r.push_back(0.27);
        names_r.push_back("r_shoulder_yaw");    positions_r.push_back(0.0);
        names_r.push_back("r_elbow");     positions_r.push_back(1.57);
        names_r.push_back("r_wrist_yaw1");      positions_r.push_back(-1.55);
        names_r.push_back("r_wrist_roll");       positions_r.push_back(0.0);
        names_r.push_back("r_wrist_yaw2");          positions_r.push_back(0.0);

        control_msgs::FollowJointTrajectoryGoal trajectory_goal_r_;
        control_msgs::FollowJointTrajectoryGoal trajectory_goal_l_;
        if (!trajectory_client_left_->waitForServer(ros::Duration(5.0)))
            ROS_WARN("[control_mode_switcher] Time out while waititing for left_arm_traj_controller");
        if (!trajectory_client_right_->waitForServer(ros::Duration(5.0)))
            ROS_WARN("[control_mode_switcher] Time out while waititing for right_arm_traj_controller");
        if (trajectory_client_left_->isServerConnected() && trajectory_client_right_->isServerConnected() )
        {
            // Goal for left arm
            trajectory_msgs::JointTrajectory joint_trajectory_l;
            joint_trajectory_l.joint_names = names_l;

            trajectory_msgs::JointTrajectoryPoint point_l;
            point_l.positions = positions_l;
            point_l.time_from_start = ros::Duration(2.0);
            joint_trajectory_l.points.push_back(point_l);

            trajectory_goal_l_.trajectory = joint_trajectory_l;

            //Goal for right arm
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
            ROS_WARN("[control_mode_switcher] Skipping Arm Motion while going to stand");
        }

    }

    void ControlModeSwitcher::trajectoryActiveCB()
    {    }

    void ControlModeSwitcher::trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
    {    }

    void ControlModeSwitcher::trajectoryDoneCb(const actionlib::SimpleClientGoalState& state,
                                               const control_msgs::FollowJointTrajectoryResultConstPtr& result)
    {        stand_complete = true; }


    void ControlModeSwitcher::getStartedAndStoppedControllers(){
        started_controllers.clear();
        stopped_controllers.clear();

        controller_manager_msgs::ListControllers srv;
        list_controllers_client_.call(srv);

        for (int i = 0; i < srv.response.controller.size(); i++){
            controller_manager_msgs::ControllerState controller = srv.response.controller[i];
            if (controller.state == "running"){
                started_controllers.push_back(controller.name);
            }
            else{
                stopped_controllers.push_back(controller.name);
            }
        }
    }

    bool ControlModeSwitcher::switchControllers(std::vector<std::string> desired_controllers_to_start){

        std::vector<std::string> controllers_to_stop;
        std::vector<std::string> controllers_to_start;

        // Add undesired running controllers to stoping list
        for (int i = 0; i < started_controllers.size(); i++){
            bool should_be_stopped = true;
            for (int j = 0; j < desired_controllers_to_start.size(); j++){
                if (started_controllers[i] == desired_controllers_to_start[j] ){
                    should_be_stopped = false;
                }
            }

            if (should_be_stopped){
                controllers_to_stop.push_back(started_controllers[i]);
            }
        }


        // Remove already running controllers from starting list
        for (int i = 0; i < desired_controllers_to_start.size(); i++){
            bool already_running = false;
            for (int j = 0; j < started_controllers.size(); j++){
                if (started_controllers[j] == desired_controllers_to_start[i] ){
                    already_running = true;
                }
            }
            if (!already_running){
                controllers_to_start.push_back(desired_controllers_to_start[i]);
            }
        }

        if ((controllers_to_start.size() > 0) || (controllers_to_stop.size() > 0)) {
        controller_manager_msgs::SwitchController srv;
        srv.request.start_controllers = controllers_to_start;
        srv.request.stop_controllers = controllers_to_stop;
        srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;


        switch_controllers_client_.call(srv);
        return srv.response.ok;
        }
        else{
            ROS_DEBUG("[control_mode_switcher] No changes made, all controllers already running");
            return true;
        }
    }





}

