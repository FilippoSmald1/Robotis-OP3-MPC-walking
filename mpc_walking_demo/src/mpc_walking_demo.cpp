#include <stdio.h>
#include "mpc_walking_demo.h"
//catkin_make -DCMAKE_BUILD_TYPE=Release


using namespace robotis_op;

op3LocomotionModule_walking_demo::op3LocomotionModule_walking_demo()
  : control_cycle_msec_(8)  //25
{

  enable_       = true;
  module_name_  = "mpc_walking_demo"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;

  iter = 0.0;

  imu_subscriber = nh_.subscribe("/robotis_op3/imu", 1, &op3LocomotionModule_walking_demo::callbackImuGyro, this); 


  // initialize KDL tree and chain
  std::string path_to_urdf = ros::package::getPath("op3_description");
  path_to_urdf += "/urdf/robotis_op3.urdf";
  if (!kdl_parser::treeFromFile(path_to_urdf, mRobot)){ 
     ROS_ERROR("Failed to construct kdl tree");
  }
  
  /*  FOR REAL ROBOT:
  imu_subscriber = nh_.subscribe("/robotis/open_cr/imu", 1, &op3LocomotionModule_walking_demo::callbackImuGyro, this); 

  // initialize KDL tree and chain

  if (!kdl_parser::treeFromFile("/home/robotis/catkin_ws/src/ROBOTIS-OP3-Common/op3_description/urdf/robotis_op3.urdf", mRobot)){ 
     ROS_ERROR("Failed to construct kdl tree");
  }
  /**/

  mRobot.getChain("body_link", "r_el_link", op3_right_arm_kinematic_chain_);
  mRobot.getChain("body_link", "l_el_link", op3_left_arm_kinematic_chain_);
  mRobot.getChain("r_ank_roll_link", "body_link", op3_right_leg_kinematic_chain_);
  mRobot.getChain("l_ank_roll_link", "body_link", op3_left_leg_kinematic_chain_);
  mRobot.getChain("r_ank_roll_link", "l_ank_roll_link", op3_right_foot_to_left_foot_kinematic_chain_);
  mRobot.getChain("l_ank_roll_link", "r_ank_roll_link", op3_left_foot_to_right_foot_kinematic_chain_);


  q_right_arm.resize(op3_right_arm_kinematic_chain_.getNrOfJoints());
  q0_right_arm.resize(op3_right_arm_kinematic_chain_.getNrOfJoints());
  qdot_right_arm.resize(op3_right_arm_kinematic_chain_.getNrOfJoints());
  J_right_arm.resize(op3_right_arm_kinematic_chain_.getNrOfJoints());
  q_right_arm(0) = 0.0;

  q_left_arm.resize(op3_left_arm_kinematic_chain_.getNrOfJoints());
  q0_left_arm.resize(op3_left_arm_kinematic_chain_.getNrOfJoints());
  qdot_left_arm.resize(op3_left_arm_kinematic_chain_.getNrOfJoints());
  q_left_arm(0) = 0.0;

  q_right_leg.resize(op3_right_leg_kinematic_chain_.getNrOfJoints());
  q0_right_leg.resize(op3_right_leg_kinematic_chain_.getNrOfJoints());
  qdot_right_leg.resize(op3_right_leg_kinematic_chain_.getNrOfJoints());
  J_right_leg.resize(op3_right_leg_kinematic_chain_.getNrOfJoints());
  J_rightf_to_leftf_leg.resize(op3_right_foot_to_left_foot_kinematic_chain_.getNrOfJoints());
        
  q_left_leg.resize(op3_left_leg_kinematic_chain_.getNrOfJoints());
  q0_left_leg.resize(op3_left_leg_kinematic_chain_.getNrOfJoints());
  qdot_left_leg.resize(op3_left_leg_kinematic_chain_.getNrOfJoints());

  J_left_leg.resize(op3_left_leg_kinematic_chain_.getNrOfJoints());
  J_leftf_to_rightf_leg.resize(op3_left_foot_to_right_foot_kinematic_chain_.getNrOfJoints());

  q0_sf_to_swg.resize(op3_left_foot_to_right_foot_kinematic_chain_.getNrOfJoints());
  q_sf_to_swg.resize(op3_left_foot_to_right_foot_kinematic_chain_.getNrOfJoints());


  right_arm_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(op3_right_arm_kinematic_chain_));

  left_arm_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(op3_left_arm_kinematic_chain_));

  right_leg_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(op3_right_leg_kinematic_chain_));
 
  left_leg_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(op3_left_leg_kinematic_chain_));
 
  left_foot_to_right_foot_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(op3_left_foot_to_right_foot_kinematic_chain_));

  right_foot_to_left_foot_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(op3_right_foot_to_left_foot_kinematic_chain_));

  right_arm_jacobian_solver.reset(new KDL::ChainJntToJacSolver(op3_right_arm_kinematic_chain_));
  right_leg_jacobian_solver.reset(new KDL::ChainJntToJacSolver(op3_right_leg_kinematic_chain_));
  left_leg_jacobian_solver.reset(new KDL::ChainJntToJacSolver(op3_left_leg_kinematic_chain_));

  right_foot_to_left_foot_jacobian_solver.reset(new KDL::ChainJntToJacSolver(op3_right_foot_to_left_foot_kinematic_chain_));
  left_foot_to_right_foot_jacobian_solver.reset(new KDL::ChainJntToJacSolver(op3_left_foot_to_right_foot_kinematic_chain_));


  x_right_arm_des = Eigen::VectorXd::Zero(6);
  x_right_arm_meas = Eigen::VectorXd::Zero(6);
  x_right_arm_vel_des = Eigen::VectorXd::Zero(6);
  x_right_leg_des = Eigen::VectorXd::Zero(6);
  x_right_leg_meas = Eigen::VectorXd::Zero(6);
  x_right_leg_vel_des = Eigen::VectorXd::Zero(6);
  x_left_leg_des = Eigen::VectorXd::Zero(6);
  x_left_leg_meas = Eigen::VectorXd::Zero(6);
  x_left_leg_vel_des = Eigen::VectorXd::Zero(6);
  sf_pose = Eigen::VectorXd::Zero(6);
  CoM_pose = Eigen::VectorXd::Zero(6);
  swg_pose = Eigen::VectorXd::Zero(6);
  CoM_pose_des = Eigen::VectorXd::Zero(6);
  swg_pose_des = Eigen::VectorXd::Zero(6);
  CoM_pose_meas = Eigen::VectorXd::Zero(6);
  swg_pose_meas = Eigen::VectorXd::Zero(6);

  J_stacked = Eigen::MatrixXd::Zero(12,12);
  Id = Eigen::MatrixXd::Identity(12,12);

  // initialize desired pose
  x_right_leg_des << 0.00, 0.035, 0.22, 0.0, 0.15, 0.0;
  x_left_leg_des << 0.00, -0.035, 0.22, 0.0, 0.15, 0.0;

  // initialize walk state
  walkState.mpcIter = 0;
  walkState.controlIter = 0;
  walkState.footstepCounter = 0;
  walkState.supportFoot = true;

  // initialize robot current and desired state    
  current.comPos = Eigen::Vector3d(0.0, 0.0, comTargetHeight);
  current.comVel = Eigen::Vector3d(0.0, 0.0, 0.0);
  current.comAcc = Eigen::Vector3d(0.0, 0.0, 0.0);
  desired.comPos = Eigen::Vector3d(current.comPos(0), current.comPos(1), comTargetHeight);
  desired.comVel = Eigen::Vector3d::Zero();
  desired.zmpPos = Eigen::Vector3d(current.comPos(0), current.comPos(1),0.0);
  desired.leftFootPos = Eigen::Vector3d(0.0, 0.039, 0.0);
  desired.rightFootPos = Eigen::Vector3d(0.0, -0.039, 0.0);
  desired.leftFootVel = Eigen::Vector3d(0.0, 0.0, 0.0); 
  desired.leftFootAcc = Eigen::Vector3d(0.0, 0.0, 0.0);
  desired.rightFootVel = Eigen::Vector3d(0.0, 0.0, 0.0);
  desired.rightFootAcc = Eigen::Vector3d(0.0, 0.0, 0.0);
               
  desired.torsoOrient = Eigen::Vector3d(0.0, 0.0, 0.0);  //(0.0, 0.15, 0.0)
  desired.leftFootOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
  desired.rightFootOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
  desired.comAcc = eta * eta * (desired.comPos - desired.zmpPos);
   
  current.comAngVel = Eigen::Vector3d(0.0, 0.0, 0.0);
  current.comAngAcc = Eigen::Vector3d(0.0, 0.0, 0.0);    
  current.leftFootVel = Eigen::Vector3d(0.0, 0.0, 0.0); 
  current.rightFootVel = Eigen::Vector3d(0.0, 0.0, 0.0); 
  current.leftFootAcc = Eigen::Vector3d(0.0, 0.0, 0.0); 
  current.rightFootAcc = Eigen::Vector3d(0.0, 0.0, 0.0);   
  current.leftFootAngVel = Eigen::Vector3d(0.0, 0.0, 0.0); 
  current.rightFootAngVel = Eigen::Vector3d(0.0, 0.0, 0.0); 
  current.leftFootAngAcc = Eigen::Vector3d(0.0, 0.0, 0.0); 
  current.rightFootAngAcc = Eigen::Vector3d(0.0, 0.0, 0.0);         

  desired.comAngVel = Eigen::Vector3d(0.0, 0.0, 0.0);
  desired.comAngAcc = Eigen::Vector3d(0.0, 0.0, 0.0);    
  desired.leftFootVel = Eigen::Vector3d(0.0, 0.0, 0.0); 
  desired.rightFootVel = Eigen::Vector3d(0.0, 0.0, 0.0); 
  desired.leftFootAcc = Eigen::Vector3d(0.0, 0.0, 0.0); 
  desired.rightFootAcc = Eigen::Vector3d(0.0, 0.0, 0.0);  
  desired.leftFootAngVel = Eigen::Vector3d(0.0, 0.0, 0.0); 
  desired.rightFootAngVel = Eigen::Vector3d(0.0, 0.0, 0.0); 
  desired.leftFootAngAcc = Eigen::Vector3d(0.0, 0.0, 0.0); 
  desired.rightFootAngAcc = Eigen::Vector3d(0.0, 0.0, 0.0);     


  //  desired footsteps and timings in world frame
  int N_footsteps = 80;
  ftsp_and_time = Eigen::MatrixXd::Zero(N_footsteps,4);

  for (int i = start; i < N_footsteps; i++) {
      ftsp_and_time(i,0) = (i-start)*0.00; //0.156
      if (i==start) ftsp_and_time(i,1) = pow(-(double)1,(i-start))*0.049;
      else ftsp_and_time(i,1) = pow(-(double)1,(i-start))*0.049;  //0.08  0.035
      ftsp_and_time(i,2) = 0.0;
      ftsp_and_time(i,3) = (double)(s_+f_)*i;
   }
  int start_ = 4;
  for (int i = start_; i < N_footsteps; i++) {
      ftsp_and_time(i,0) = (i-start_)*stride_length; 
  } 

  for (int i = n_steps+1; i < N_footsteps; i++) {
      ftsp_and_time(i,0) = ftsp_and_time(n_steps,0); 
      ftsp_and_time(i,1) = 0.5 * (ftsp_and_time(n_steps,1)+ftsp_and_time(n_steps-1,1)); 
  } 
     
  /* 
  ftsp_and_time(6,2) = 0.025;
  ftsp_and_time(8,2) = -0.015;
  ftsp_and_time(9,2) = -0.04;
  ftsp_and_time(10,2) = -0.04;
  /**/
  //ftsp_and_time(7,2) = 0.01;

  // initialize mpc solver    
  const Eigen::MatrixXd& ftsp_and_time_ref = ftsp_and_time;
  solver = new MPCSolver(ftsp_and_time_ref);

  rate = 1000.0/control_cycle_msec_;

  T = 1000.0*control_cycle_msec_/1000.0;

  t_start = 500;
  t_stand = 1500;
  t_walk = t_stand + (n_steps+1)*(S+F);
  t_sit = t_walk + (t_stand - t_start);
  T =  (double)(t_stand - t_start) * (double)control_cycle_msec_/1000.0;

  mode = "wait";

}

op3LocomotionModule_walking_demo::~op3LocomotionModule_walking_demo()
{
  queue_thread_.join();
}

void op3LocomotionModule_walking_demo::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;

  queue_thread_ = boost::thread(boost::bind(&op3LocomotionModule_walking_demo::queueThread, this));

  for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
       it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;

  }


}

void op3LocomotionModule_walking_demo::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void op3LocomotionModule_walking_demo::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                   std::map<std::string, double> sensors)
{

  auto start = std::chrono::high_resolution_clock::now();
  if (enable_ == false)
    return;

  //********************
  // OPERATION MODES:
  //********************
  
  // initial setup before stand up motion
  if (iter == t_start-1) {
    left_leg_fk_solver->JntToCart(q0_left_leg, x_left_leg_fk);
    z0 = x_left_leg_fk.p(2) + 0.008;
    zf = comTargetHeight;
    a3 = (zf - z0)/(pow(T,3)-pow(T,4)*3.0/(2.0*T));
    a2 = - 3.0*a3*pow(T,2)/(2.0*T);
    a0 = z0;
    a1 = 0.0;
  }

  // stand up motion
  if (iter >= t_start && iter < t_stand) {
    t = (iter-t_start)*control_cycle_msec_/1000.0;
    desired.comPos << desired.leftFootPos(0), ((double)iter-(double)t_start)*(desired.leftFootPos(1)-0.043)/1000.0, a3*pow(t,3) + a2*pow(t,2) + a1*t + a0;
    desired.comVel(2) = 3.0*a3*pow(t,2) + 2.0*a2*t + a1;
    desired.torsoOrient << 0.0, ((double)iter-(double)t_start)*0.12/1000.0,  0.0;
    desired.rightFootPos << 0.0, -0.045, 0.0; 
    desired.rightFootOrient << 0.0, 0.0, 0.0;
    mode = "stand up";
  }

  // walk motion
  if (iter >= t_stand && iter < t_walk) {
    if (walkState.simulationTime >= ftsp_and_time(walkState.footstepCounter,3) -1) {
      walkState.controlIter = 0;
      walkState.mpcIter = 0;
      walkState.footstepCounter++;
      walkState.supportFoot = !walkState.supportFoot;
      std::cout <<  iter << std::endl;
    }
    walkState.simulationTime = iter-t_stand;
    // compute MPC iteration and swing foot trajectory
    //const Eigen::MatrixXd& ftsp_and_time_ref = ftsp_and_time;
    if (walkState.footstepCounter < n_steps+2) desired = solver->solve(desired, walkState, ftsp_and_time); 
    ++walkState.controlIter;
    walkState.mpcIter = floor(walkState.controlIter*controlTimeStep/mpcTimeStep);
    // arm swing
    q_left_arm(0)  = - 0.1 + 0.15*sin(2.0*3.14*(iter-t_stand)/((double)s_*2.0+(double)f_*2.0));
    q_right_arm(0) = + 0.1 + 0.15*sin(2.0*3.14*(iter-t_stand)/((double)s_*2.0+(double)f_*2.0));
    mode = "walk";
  }

  // initial setup before sit down motion
  if (iter == t_walk-1) {
    left_leg_fk_solver->JntToCart(q0_left_leg, x_left_leg_fk);
    zf = z0;
    z0 = comTargetHeight;
    a3 = (zf - z0)/(pow(T,3)-pow(T,4)*3.0/(2.0*T));
    a2 = - 3.0*a3*pow(T,2)/(2.0*T);
    a0 = z0;
    a1 = 0.0;
  }

  // sit down motion
  if (iter >= t_walk && iter < t_sit) {
    t = (iter-t_walk)*control_cycle_msec_/1000.0;
    desired.comPos(2) = a3*pow(t,3) + a2*pow(t,2) + a1*t + a0;
    desired.comVel(2) = 3.0*a3*pow(t,2) + 2.0*a2*t + a1;
    desired.torsoOrient(1) = 0.12-((double)iter-(double)t_walk)*0.12/1000.0;
    mode = "sit down";
  }

  //***************
  // MEASUREMENTS:
  //***************

  storeEncoderReadings(dxls);
  computeDirectKinematics();


  //*************************
  // KINEMATIC CONTROLLER:
  //*************************
 
  if (iter >= t_start && iter < t_sit) computeInverseKinematicsWholeBody();

  
  //*******************
  // MOTOR COMMANDS:
  //*******************

  if (iter >= t_start && iter < t_sit){
    dxls["l_hip_yaw"]->dxl_state_->goal_position_ = q_left_leg(5);
    dxls["l_hip_roll"]->dxl_state_->goal_position_ = q_left_leg(4);
    dxls["l_hip_pitch"]->dxl_state_->goal_position_ = q_left_leg(3);
    dxls["l_knee"]->dxl_state_->goal_position_ = q_left_leg(2);
    dxls["l_ank_pitch"]->dxl_state_->goal_position_ = q_left_leg(1);
    dxls["l_ank_roll"]->dxl_state_->goal_position_ = q_left_leg(0);

    dxls["r_hip_yaw"]->dxl_state_->goal_position_ = q_right_leg(5);
    dxls["r_hip_roll"]->dxl_state_->goal_position_ = q_right_leg(4);
    dxls["r_hip_pitch"]->dxl_state_->goal_position_ = q_right_leg(3);
    dxls["r_knee"]->dxl_state_->goal_position_ = q_right_leg(2);
    dxls["r_ank_pitch"]->dxl_state_->goal_position_ = q_right_leg(1);
    dxls["r_ank_roll"]->dxl_state_->goal_position_ = q_right_leg(0);

    dxls["l_sho_pitch"]->dxl_state_->goal_position_ = q_left_arm(0);
    dxls["r_sho_pitch"]->dxl_state_->goal_position_ = q_right_arm(0);
  }


  //*************************
  // CONCLUDE CONTROL CYCLE:
  //*************************

  iter++;
  auto finish = std::chrono::high_resolution_clock::now();
  auto interval = std::chrono::time_point_cast<std::chrono::microseconds>(finish) - std::chrono::time_point_cast<std::chrono::microseconds>(start);
  std::cout << "time to execute one step of the locomotion controller is " << interval.count()/1000.0 << " ms" << std::endl;        
  std::cout << "ITER " << iter 
            << " fsCount " << walkState.footstepCounter 
            << " walkState.controlIter " << walkState.controlIter 
            << " operational mode: " << mode << std::endl;


}

void op3LocomotionModule_walking_demo::storeEncoderReadings(std::map<std::string, robotis_framework::Dynamixel *>& dxls){

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin(); state_iter != result_.end();
      state_iter++)
  {
    //double p_pos = dxls[state_iter->first]->dxl_state_->present_position_;
    if(state_iter->first == "r_sho_pitch") q0_right_arm(0) = dxls[state_iter->first]->dxl_state_->present_position_;
    if(state_iter->first == "r_sho_roll")  q0_right_arm(1) = dxls[state_iter->first]->dxl_state_->present_position_;
    if(state_iter->first == "r_el") q0_right_arm(2) = dxls[state_iter->first]->dxl_state_->present_position_;

    if(state_iter->first == "l_sho_pitch") q0_left_arm(0) = dxls[state_iter->first]->dxl_state_->present_position_;
    if(state_iter->first == "l_sho_roll") q0_left_arm(1) = dxls[state_iter->first]->dxl_state_->present_position_;
    if(state_iter->first == "l_el") q0_left_arm(2) = dxls[state_iter->first]->dxl_state_->present_position_;

    if(state_iter->first == "r_hip_yaw"){ 
      q0_right_leg(5) = dxls[state_iter->first]->dxl_state_->present_position_;
    }           
    if(state_iter->first == "r_hip_roll"){
       q0_right_leg(4) = dxls[state_iter->first]->dxl_state_->present_position_;
    }
    if(state_iter->first == "r_hip_pitch"){
      q0_right_leg(3) =dxls[state_iter->first]->dxl_state_->present_position_;
    }
    if(state_iter->first == "r_knee"){
      q0_right_leg(2) = dxls[state_iter->first]->dxl_state_->present_position_;
    }
    if(state_iter->first == "r_ank_pitch"){
      q0_right_leg(1) = dxls[state_iter->first]->dxl_state_->present_position_;
    }
    if(state_iter->first == "r_ank_roll"){
       q0_right_leg(0) =dxls[state_iter->first]->dxl_state_->present_position_;
    }


    if(state_iter->first == "l_hip_yaw"){ 
       q0_left_leg(5) = dxls[state_iter->first]->dxl_state_->present_position_;
    }           
    if(state_iter->first == "l_hip_roll"){
       q0_left_leg(4) = dxls[state_iter->first]->dxl_state_->present_position_;
    }
    if(state_iter->first == "l_hip_pitch"){
      q0_left_leg(3) = dxls[state_iter->first]->dxl_state_->present_position_;
    }
    if(state_iter->first == "l_knee"){
      q0_left_leg(2) = dxls[state_iter->first]->dxl_state_->present_position_;
    }
    if(state_iter->first == "l_ank_pitch"){
       q0_left_leg(1) = dxls[state_iter->first]->dxl_state_->present_position_;
    }
    if(state_iter->first == "l_ank_roll"){
      q0_left_leg(0) = dxls[state_iter->first]->dxl_state_->present_position_;
    }
  }

}

void op3LocomotionModule_walking_demo::computeDirectKinematics(){

  if (walkState.supportFoot == true) {

    left_leg_fk_solver->JntToCart(q0_left_leg, x_left_leg_fk);
    for (int i = 0; i < 12; i++) {
       if (i<6) q0_sf_to_swg(i) = q0_left_leg(i);
       else q0_sf_to_swg(i) = q0_right_leg(11-i); 
    }
    left_foot_to_right_foot_fk_solver->JntToCart(q0_sf_to_swg, x_sf_to_swg);
    CoM_pose_meas.segment(0,3) = Eigen::Vector3d(x_left_leg_fk.p(0),x_left_leg_fk.p(1),x_left_leg_fk.p(2));  
    swg_pose_meas.segment(0,3) = Eigen::Vector3d(x_sf_to_swg.p(0),x_sf_to_swg.p(1),x_sf_to_swg.p(2));  
    x_left_leg_fk.M.GetRPY(CoM_pose_meas(3),CoM_pose_meas(4),CoM_pose_meas(5));  
    x_sf_to_swg.M.GetRPY(swg_pose_meas(3),swg_pose_meas(4),swg_pose_meas(5));  

   } else {

    right_leg_fk_solver->JntToCart(q0_right_leg, x_right_leg_fk);
    for (int i = 0; i < 12; i++) {
       if (i<6) q0_sf_to_swg(i) = q0_right_leg(i);
       else q0_sf_to_swg(i) = q0_left_leg(11-i); 
    }
    right_foot_to_left_foot_fk_solver->JntToCart(q0_sf_to_swg, x_sf_to_swg);
    CoM_pose_meas.segment(0,3) = Eigen::Vector3d(x_right_leg_fk.p(0),x_right_leg_fk.p(1),x_right_leg_fk.p(2));  
    swg_pose_meas.segment(0,3) = Eigen::Vector3d(x_sf_to_swg.p(0),x_sf_to_swg.p(1),x_sf_to_swg.p(2));  
    x_right_leg_fk.M.GetRPY(CoM_pose_meas(3),CoM_pose_meas(4),CoM_pose_meas(5));  
    x_sf_to_swg.M.GetRPY(swg_pose_meas(3),swg_pose_meas(4),swg_pose_meas(5));  

   }

}

void op3LocomotionModule_walking_demo::computeInverseKinematicsWholeBody() {

  Eigen::MatrixXd weights = Eigen::MatrixXd::Identity(12,12);
 
  // com pos
  weights(0,0) = 100.0;
  weights(1,1) = 100.0;
  weights(2,2) = 88.0;

  // com orient
  weights(3,3) = 150.0;
  weights(4,4) = 150.0;
  weights(5,5) = 100.0;

  // foot pos
  weights(6,6) = 100.0;
  weights(7,7) = 100.0;
  weights(8,8) = 110.0;

  // foot orient
  weights(9,9) = 115.0;
  weights(10,10) = 115.0;
  weights(11,11) = 110.0;

  // just useful for better performance
  if (iter < 750) weights *= 0.5;
  else  weights *= 0.7;

  //weights *= 1.1;  in real robot

  if (walkState.supportFoot == true) {
    // left support foot

    sf_pose << desired.leftFootPos, desired.leftFootOrient;
    CoM_pose_des << desired.comPos, desired.torsoOrient; 
    CoM_pose_des(2) = CoM_pose_des(2);
    swg_pose_des << desired.rightFootPos, desired.rightFootOrient;
    CoM_pose_des = vvRel(CoM_pose_des, sf_pose); 
    swg_pose_des = vvRel(swg_pose_des, sf_pose); 
    CoM_pose_des(0) = CoM_pose_des(0) + sagittal_com_offset;
  
    Eigen::VectorXd v_des, pos_des, pos_meas;
    v_des = Eigen::VectorXd::Zero(12);
    v_des.segment(0,3) = desired.comVel;
    pos_des = Eigen::VectorXd::Zero(12);
    pos_meas = Eigen::VectorXd::Zero(12);
    pos_des << CoM_pose_des, swg_pose_des;
    pos_meas << CoM_pose_meas, swg_pose_meas; 

    if (left_foot_to_right_foot_jacobian_solver->JntToJac(q0_sf_to_swg, J_leftf_to_rightf_leg) < 0) {
       ROS_ERROR( "jacobian error");
    }
    J_left_leg_to_right = J_leftf_to_rightf_leg.data;
    if (left_leg_jacobian_solver->JntToJac(q0_left_leg, J_left_leg) < 0) {
       ROS_ERROR( "jacobian error");
    }
    J_left_leg_ = J_left_leg.data;
    J_stacked << J_left_leg_, Eigen::MatrixXd::Zero(6,6), J_left_leg_to_right;

    Eigen::VectorXd q_dot = J_stacked.transpose() * (J_stacked*J_stacked.transpose() + Id*sigma).inverse() * (v_des + weights*(pos_des-pos_meas));
    for (int i = 0; i < 6; i++) q_left_leg(i) = q0_left_leg(i) + (1.0/rate)*q_dot(i);
    for (int i = 0; i < 6; i++) q_right_leg(5-i) = q0_right_leg(5-i) + (1.0/rate)*q_dot(i+6);

  } else {
    // right support foot 

    sf_pose << desired.rightFootPos, desired.rightFootOrient;
    CoM_pose_des << desired.comPos, desired.torsoOrient; 
    CoM_pose_des(2) = CoM_pose_des(2);
    swg_pose_des << desired.leftFootPos, desired.leftFootOrient;
    CoM_pose_des = vvRel(CoM_pose_des, sf_pose); 
    swg_pose_des = vvRel(swg_pose_des, sf_pose);  
    CoM_pose_des(0) = CoM_pose_des(0) + sagittal_com_offset;

    Eigen::VectorXd v_des, pos_des, pos_meas;
    v_des = Eigen::VectorXd::Zero(12);
    v_des.segment(0,3) = desired.comVel;
    pos_des = Eigen::VectorXd::Zero(12);
    pos_meas = Eigen::VectorXd::Zero(12);
    pos_des << CoM_pose_des, swg_pose_des;
    pos_meas << CoM_pose_meas, swg_pose_meas; 

    if (right_foot_to_left_foot_jacobian_solver->JntToJac(q0_sf_to_swg, J_rightf_to_leftf_leg) < 0) {
      ROS_ERROR( "jacobian error");
    }
    J_right_leg_to_left = J_rightf_to_leftf_leg.data;
    if (right_leg_jacobian_solver->JntToJac(q0_right_leg, J_right_leg) < 0) {
      ROS_ERROR( "jacobian error");
    }
    J_right_leg_ = J_right_leg.data;
    J_stacked << J_right_leg_, Eigen::MatrixXd::Zero(6,6), J_right_leg_to_left;

    Eigen::VectorXd q_dot = J_stacked.transpose() * (J_stacked*J_stacked.transpose() + Id*sigma).inverse() * (v_des + weights*(pos_des-pos_meas));
    for (int i = 0; i < 6; i++) q_right_leg(i) = q0_right_leg(i) + (1.0/rate)*q_dot(i);
    for (int i = 0; i < 6; i++) q_left_leg(5-i) = q0_left_leg(5-i) + (1.0/rate)*q_dot(i+6);

  } 

}

void op3LocomotionModule_walking_demo::callbackImuGyro(const sensor_msgs::Imu & msg){

  angular_rates << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z; 
  linear_acc << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
  KDL::Rotation::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w).GetRPY(rpy(0),rpy(1),rpy(2)); 

}

void op3LocomotionModule_walking_demo::storeData(){
  /*
  myfile.open ("/home/robotis/catkin_ws/src/ROBOTIS-OP3/mpc_walking_module/data/z.txt",ios::app);
  myfile << desired.comPos(2) <<endl; 
  myfile.close();
  /**/
}

void op3LocomotionModule_walking_demo::stop()
{
  return;
}

bool op3LocomotionModule_walking_demo::isRunning()
{
  return false;
}




