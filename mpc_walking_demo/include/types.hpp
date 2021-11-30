#pragma once

#include <Eigen/Core>
#include "utils.cpp"
#include <string>

// Contains the state of the LIP robot
struct State {
    Eigen::Vector3d comPos;
    Eigen::Vector3d comVel;
    Eigen::Vector3d comAcc;
    Eigen::Vector3d zmpPos;
    
    Eigen::Vector3d comAngVel;
    Eigen::Vector3d comAngAcc;    

    Eigen::Vector3d leftFootPos;
    Eigen::Vector3d leftFootVel;
    Eigen::Vector3d leftFootAcc;

    Eigen::Vector3d rightFootPos;
    Eigen::Vector3d rightFootVel;
    Eigen::Vector3d rightFootAcc;
    
    Eigen::Vector3d leftFootAngVel;
    Eigen::Vector3d leftFootAngAcc;

    Eigen::Vector3d rightFootAngVel;
    Eigen::Vector3d rightFootAngAcc;   

    Eigen::Vector3d torsoOrient;
    Eigen::Vector3d leftFootOrient;
    Eigen::Vector3d rightFootOrient;

    inline Eigen::VectorXd getComPose() {
	Eigen::VectorXd comPose(6);
        comPose << torsoOrient, comPos;
        return comPose;
    }

    inline Eigen::VectorXd getSupportFootPose(bool supportFoot) {
	Eigen::VectorXd sfPose(6);
        if (supportFoot == 0) sfPose << leftFootOrient, leftFootPos;
        else sfPose << rightFootOrient, rightFootPos;
        return sfPose;
    }

    inline Eigen::VectorXd getSwingFootPose(bool supportFoot) {
	Eigen::VectorXd sfPose(6);
        if (supportFoot == 1) sfPose << leftFootOrient, leftFootPos;
        else sfPose << rightFootOrient, rightFootPos;
        return sfPose;
    }

    inline Eigen::VectorXd getFootPose(std::string foot) {
	Eigen::VectorXd sfPose(6);
        if (foot == "left_foot") sfPose << leftFootOrient, leftFootPos;
        else sfPose << rightFootOrient, rightFootPos;
        return sfPose;
    }
    
    inline Eigen::VectorXd getFootVel(std::string foot) {
	Eigen::VectorXd sfVel(6);
        if (foot == "left_foot") sfVel << leftFootAngVel, leftFootVel;
        else sfVel << rightFootAngVel, rightFootVel;
        return sfVel;
    }
    
    inline Eigen::VectorXd getFootAcc(std::string foot) {
	Eigen::VectorXd sfAcc(6);
        if (foot == "left_foot") sfAcc << leftFootAngAcc, leftFootAcc;
        else sfAcc << rightFootAngAcc, rightFootAcc;
        return sfAcc;
    }        
    
    inline Eigen::VectorXd getRelComPose(bool supportFoot) {
	return vvRel(getComPose(), getSupportFootPose(supportFoot));
    }

    inline Eigen::VectorXd getRelSwingFootPose(bool supportFoot) {
	return vvRel(getSwingFootPose(supportFoot), getSupportFootPose(supportFoot));
    }

    inline Eigen::VectorXd getRelFootEEPose(std::string foot) {
	return vvRel(getFootPose(foot), getComPose());
    }

    inline Eigen::VectorXd getRelFootEEVel(std::string foot) {
        Eigen::VectorXd sfVel(6);
        Eigen::VectorXd CoMVel(6); 
        CoMVel << comAngVel, comVel;    
        sfVel = getFootVel(foot) - CoMVel;
	return sfVel;
    }
    
    inline Eigen::VectorXd getRelFootEEAcc(std::string foot) {
        Eigen::VectorXd sfAcc(6);
        Eigen::VectorXd CoMAcc(6);  
        CoMAcc << comAngAcc, comAcc;          
        sfAcc = getFootAcc(foot) - CoMAcc;
	return sfAcc;
    }        
        
};

struct WalkState {
    bool supportFoot;
    double simulationTime;
    int mpcIter, controlIter, footstepCounter, indInitial;
};

struct Vref {
    Vref(double _x, double _y, double _omega) : x(_x), y(_y), omega(_omega) {}

    double x = 0;
    double y = 0;
    double omega = 0;
}; 
