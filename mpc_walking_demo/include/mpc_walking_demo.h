#ifndef MOTION_MODULE_MPC_WALKING_DEMO_H_
#define MOTION_MODULE_MPC_WALKING_DEMO_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include "robotis_framework_common/motion_module.h"
#include "op3_ball_detector/CircleSetStamped.h"
//#include "op3_ball_detector/CircleSetStamped.h"

#include <chrono>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <boost/scoped_ptr.hpp>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <fstream>

#include "QPSolverInterface.cpp"
#include "MPCSolver.hpp"


namespace robotis_op
{
 
class op3LocomotionModule_walking_demo
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<op3LocomotionModule_walking_demo>
{
private:

  // methods
  void queueThread();
  void storeEncoderReadings(std::map<std::string, robotis_framework::Dynamixel *>&);
  void computeInverseKinematicsWholeBody();
  void computeDirectKinematics();
  void callbackImuGyro(const sensor_msgs::Imu & msg);
  void storeData();

  // useful attributes
  int control_cycle_msec_;
  boost::thread queue_thread_;
  std::string mode;
  double iter;
  double rate;
  double ball_error;
  double T;
  double z0;
  double zf;
  double a0, a1, a2, a3;
  double t;
  int t_start, t_stand, t_walk, t_sit;
  std::ofstream myfile;

  /* sample subscriber & publisher*/
  ros::NodeHandle nh_;
  ros::Subscriber sub1_;
  ros::Subscriber imu_subscriber;

  // matrices
  Eigen::MatrixXd J_right_arm_, J_right_leg_, J_left_leg_;  
  Eigen::MatrixXd J_right_leg_to_left, J_left_leg_to_right;    
  Eigen::MatrixXd J_right_arm_resized, J_right_leg_resized, J_left_leg_resized;
  Eigen::MatrixXd J_stacked, Id;          
  
  Eigen::VectorXd x_right_arm_des, x_right_arm_meas, x_right_arm_vel_des;    
  Eigen::VectorXd x_right_leg_des, x_right_leg_meas, x_right_leg_vel_des;     
  Eigen::VectorXd x_left_leg_des, x_left_leg_meas, x_left_leg_vel_des;     
  Eigen::Vector3d rpy, angular_rates, linear_acc;  
  Eigen::VectorXd sf_pose, CoM_pose, swg_pose;   
  Eigen::VectorXd sf_pose_meas, CoM_pose_meas, swg_pose_meas;       
  Eigen::VectorXd CoM_pose_des, swg_pose_des;       

  Eigen::MatrixXd ftsp_and_time; 

  //// KDL ////
  // robot
  KDL::Tree mRobot;

  // kinematic chains
  KDL::Chain op3_right_arm_kinematic_chain_;
  KDL::Chain op3_left_arm_kinematic_chain_;
  KDL::Chain op3_right_leg_kinematic_chain_; // from right support foot to CoM
  KDL::Chain op3_left_leg_kinematic_chain_; // from left support foot to CoM

  KDL::Chain op3_right_foot_to_left_foot_kinematic_chain_; // from right support foot to left foot
  KDL::Chain op3_left_foot_to_right_foot_kinematic_chain_; // from left support foot to right foot

  // ik solvers
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> right_arm_fk_solver;
  boost::scoped_ptr<KDL::ChainFkSolverPos> left_arm_fk_solver;
  boost::scoped_ptr<KDL::ChainFkSolverPos> right_leg_fk_solver;
  boost::scoped_ptr<KDL::ChainFkSolverPos> left_leg_fk_solver;
  boost::scoped_ptr<KDL::ChainFkSolverPos> left_foot_to_right_foot_fk_solver;
  boost::scoped_ptr<KDL::ChainFkSolverPos> right_foot_to_left_foot_fk_solver;

  boost::scoped_ptr<KDL::ChainJntToJacSolver> right_arm_jacobian_solver;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> right_leg_jacobian_solver;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> left_leg_jacobian_solver;

  boost::scoped_ptr<KDL::ChainJntToJacSolver> right_foot_to_left_foot_jacobian_solver;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> left_foot_to_right_foot_jacobian_solver;

  // joints and frames
  KDL::JntArray  q_right_arm, q0_right_arm;                                                                                                       
  KDL::JntArrayVel  qdot_right_arm; 
  KDL::Frame     x_right_arm_fk, x_right_arm_d; 
  KDL::Frame     x_right_leg_fk, x_right_leg_d; 
  KDL::Frame     x_left_leg_fk, x_left_leg_d; 
  KDL::JntArray  q_left_arm;                                                                                                       
  KDL::JntArray  q0_left_arm;             
  KDL::JntArrayVel  qdot_left_arm; 
  KDL::Frame     x_left_arm; 
  KDL::JntArray  q_right_leg;                                                                                                       
  KDL::JntArray  q0_right_leg;             
  KDL::JntArrayVel  qdot_right_leg; 
  KDL::Frame     x_right_leg; 
  KDL::JntArray  q_left_leg;                                                                                                       
  KDL::JntArray  q0_left_leg;             
  KDL::JntArrayVel  qdot_left_leg; 
  KDL::Frame     x_left_leg; 
  KDL::Frame     x_sf_to_swg; 
  KDL::JntArray  q_sf_to_swg, q0_sf_to_swg;                                                                                                       

  // other kdl stuff      
  KDL::JntArray  tau_; 
  KDL::Frame     x_; 
  KDL::Frame     xd_;   
  KDL::Frame     x0_;  
  KDL::Twist     xerr_;   
  KDL::Twist     xdot_; 
  KDL::Wrench    F_;  
  KDL::Jacobian  J_right_arm, J_right_leg, J_left_leg, J_rightf_to_leftf_leg, J_leftf_to_rightf_leg;  
  KDL::Twist     Kp_; //gains      
  KDL::Twist     Kd_; 

  // mpc solver and data structures
  MPCSolver* solver;
  State desired;
  State current;
  WalkState walkState;

public:
  op3LocomotionModule_walking_demo();
  virtual ~op3LocomotionModule_walking_demo();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
};

}

#endif /* MOTION_MODULE_TUTORIAL_MOTION_MODULE_TUTORIAL_H_*/
