#pragma once

#include <math.h>

// Definable parameters
// ********************

// Times
const double mpcTimeStep = 0.008; //0.025;
const double controlTimeStep = 0.008;
const double singleSupportDuration = 0.4; //0.6
const double doubleSupportDuration = 0.24; //0.4
const double predictionTime = 0.8;

// Walk parameters
const double stepHeight = 0.032; //0.032 on real robot is better
const double comTargetHeight = 0.231; //0.24  very good
const double kSwingFoot = 0.082; //0.285   0.08
const double stride_length = 0.06; //0.06

// Constraints
const double footConstraintSquareWidth = 0.05;  //0.05

// Cost function weights for horizontal QP
const double qZ = 1;
const double qF = 1000000;
// Cost function weights for vertical QP
const double q_force = 1.0;
const double q_position = 1000000000000.0;

// Kinematic control
const double IKerrorGain = 1.0; 
const double sagittal_com_offset = 0.03; // 0.0328 on real robot
const double vertical_com_offset = 0.0; //0.08
const double sigma = 0.001;

// Used in the code
// ****************
const double op3_mass = 3.8; 
const double g = 9.81; 
const double eta = sqrt(g/comTargetHeight);
const int N = round(predictionTime/mpcTimeStep);
const int S = round(singleSupportDuration/mpcTimeStep);
const int F = round(doubleSupportDuration/mpcTimeStep);
const int n_ = round(predictionTime/controlTimeStep);
const int s_ = round(singleSupportDuration/controlTimeStep);
const int f_ = round(doubleSupportDuration/controlTimeStep);
const int M = 2; 
const int n_steps = 10; // 20
const int start = 1;

// Flags
//**************
const int calibration = 0;
const int walking = 1;
const int vh_walk = 0;
const int stair = 0;
const int run = 0;

