#pragma once

#include <Eigen/Core>
#include "types.hpp"
#include "parameters.cpp"
#include "QPSolverInterface.cpp"
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>       /* isnan, sqrt */
//#include "FootstepPlan.hpp"
#include <chrono>

class MPCSolver{
    public:
    MPCSolver(const Eigen::MatrixXd& ftsp_and_timings);
    ~MPCSolver();

    // Compute the next desired state starting from the current state
    State solve(State current, WalkState walkState, const Eigen::MatrixXd& ftsp_and_timings);
    State WalkingSwingFoot(State current, State next, WalkState walkState, Eigen::MatrixXd ftsp_and_timings);
     
    // some stuff
    int itr;
    int fsCount;

    private:
    // Matrices for prediction
    Eigen::VectorXd p;
    Eigen::MatrixXd P;

    // Matrices for cost function
    Eigen::MatrixXd costFunctionH;
    Eigen::VectorXd costFunctionF;

    // Matrices for stability constraint
    Eigen::MatrixXd Aeq;
    Eigen::VectorXd beq;
    Eigen::VectorXd beq_x, beq_y; 
    Eigen::MatrixXd Aeq_x, Aeq_y;

    //Matrices for balance constraint
    Eigen::MatrixXd AZmp;
    Eigen::VectorXd bZmpMax;
    Eigen::VectorXd bZmpMin;
    Eigen::VectorXd Zmin_x, Zmax_x, Zmin_y, Zmax_y;
    Eigen::MatrixXd A_ineq_xy;

    // Matrices for kinematic constraints
    Eigen::MatrixXd AFootsteps;
    Eigen::VectorXd bFootstepsMax;
    Eigen::VectorXd bFootstepsMin;

    // Matrices for the stacked constraints
    Eigen::MatrixXd AConstraint;
    Eigen::VectorXd bConstraintMax;
    Eigen::VectorXd bConstraintMin;

    // Matrices for QP and constraints
    Eigen::MatrixXd costFunctionH_xy;
    Eigen::VectorXd costFunctionF_x, costFunctionF_y;
    Eigen::VectorXd lambda, etas;
    Eigen::MatrixXd deltas;
    Eigen::MatrixXd u_diff, u_diff_0;
    Eigen::VectorXd z_trajectory;
    Eigen::VectorXd f_trajectory, f_u_diff_0;
    Eigen::MatrixXd A_virtual_ZMP;
    Eigen::VectorXd b_virtual_ZMP_x, b_virtual_ZMP_y;
    Eigen::MatrixXd phi_input, phi_state, C_sc;
    Eigen::VectorXd decisionVariables_x, decisionVariables_y;

    //Matrices State update
    Eigen::Matrix2d A_xy;    
    Eigen::MatrixXd B_xy;    
    
    // Midpoint of ZMP constraint
    Eigen::MatrixXd ftsp_midpoint; 

    // useful parameters
    double zmp_smoothness = 0.99;

};
