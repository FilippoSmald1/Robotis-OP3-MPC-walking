#include "MPCSolver.hpp"
 
static std::ofstream foutDebug(realpath("../data/debug.txt", NULL), std::ofstream::trunc);

MPCSolver::MPCSolver(const Eigen::MatrixXd& ftsp_and_timings) {



    // HORIZONTAL QP

    // Matrices for cost function (unique QP for XY)
    costFunctionH = Eigen::MatrixXd::Zero(2*(N+M),2*(N+M));
    costFunctionF = Eigen::VectorXd::Zero(2*(N+M));

    // Matrices for stability constraint
    Aeq_x = Eigen::MatrixXd::Zero(1,N);
    Aeq_y = Eigen::MatrixXd::Zero(1,N);
    beq_x = Eigen::VectorXd::Zero(1,1);
    beq_y = Eigen::VectorXd::Zero(1,1); 

    // Matrices for ZMP constraint
    AZmp = Eigen::MatrixXd::Zero(2*N,2*(N+M));
    bZmpMax = Eigen::VectorXd::Zero(2*N);
    bZmpMin = Eigen::VectorXd::Zero(2*N);
    A_ineq_xy = Eigen::MatrixXd::Identity(N,N);

    // Matrices for kinematic constraint
    AFootsteps = Eigen::MatrixXd::Zero(2*M,2*(M+N));
    bFootstepsMax = Eigen::VectorXd::Zero(2*M);
    bFootstepsMin = Eigen::VectorXd::Zero(2*M);

    // Matrices for ZMP prediction (might be useful)
    p = Eigen::VectorXd::Ones(N);
    P = Eigen::MatrixXd::Ones(N,N)*mpcTimeStep;
    for(int i = 0; i < N; ++i) {
    	for(int j = 0; j < N; ++j) {
            if (j > i) P(i, j)=0;
        }
    }

    // Matrices for cost function smoothness
    u_diff = Eigen::MatrixXd::Identity(N,N);
    for (int i = 1; i < N; i++) u_diff(i-1,i) = -1;
    u_diff = u_diff.transpose()*u_diff;
    u_diff_0 = Eigen::MatrixXd::Zero(N,N);
    u_diff_0(0,0) = 1.0;

    // initialize lambda vector
    lambda = p*eta*eta;
    etas = p*eta;

    // build midpoint sequence
    ftsp_midpoint = Eigen::MatrixXd::Zero(ftsp_and_timings.rows()*(S+F)*2,4);
    Eigen::MatrixXd double_support_transition = Eigen::MatrixXd::Zero(F,1);
    Eigen::MatrixXd p_dst = Eigen::MatrixXd::Ones(F,1);
    for (int i = 0; i < F; i++) double_support_transition(i,0) = (double)i/(double)F;
    for (int i = 0; i < ftsp_and_timings.rows() - 1 ; i++) {

        double offset_y  = (ftsp_and_timings(i,1)>0 && i>1 && i < n_steps)? 0.0 : -0.0;
        // single support phases
        ftsp_midpoint.block(i*(S+F),0,S,1) = ftsp_and_timings(i,0)*Eigen::MatrixXd::Ones(S,1);
        ftsp_midpoint.block(i*(S+F),1,S,1) = (ftsp_and_timings(i,1)+offset_y)*Eigen::MatrixXd::Ones(S,1);
        ftsp_midpoint.block(i*(S+F),2,S,1) = ftsp_and_timings(i,2)*Eigen::MatrixXd::Ones(S,1);
        // double support phases
        ftsp_midpoint.block(i*(S+F)+S,0,F,1) = ftsp_and_timings(i,0)*p_dst + (ftsp_and_timings(i+1,0)-ftsp_and_timings(i,0))*double_support_transition;
        ftsp_midpoint.block(i*(S+F)+S,1,F,1) =  (ftsp_and_timings(i,1)+offset_y)*p_dst + (ftsp_and_timings(i+1,1)-ftsp_and_timings(i,1)+2.0*offset_y)*double_support_transition;
        ftsp_midpoint.block(i*(S+F)+S,2,F,1) =  ftsp_and_timings(i,2)*p_dst + (ftsp_and_timings(i+1,2)-ftsp_and_timings(i,2))*double_support_transition;
        
    }
    
    // useful multiplier
    deltas = Eigen::MatrixXd::Zero(1,N);
    for (int i = 0; i<N; i++) deltas(0,i) = exp(-mpcTimeStep*eta*i);

    // cost function xy
    costFunctionH_xy = (Eigen::MatrixXd::Identity(N,N));
    costFunctionH_xy(0,0) = costFunctionH_xy(0,0) + zmp_smoothness;

    // matrices and parameters fo horizontal QP
    A_xy = Eigen::MatrixXd::Zero(2,2);
    B_xy = Eigen::MatrixXd::Zero(2,1); 
    A_virtual_ZMP = Eigen::MatrixXd::Zero(F,N);
    b_virtual_ZMP_x = Eigen::VectorXd::Zero(F);
    b_virtual_ZMP_y = Eigen::VectorXd::Zero(F);
    C_sc = Eigen::MatrixXd::Zero(1,2);
    phi_state = Eigen::MatrixXd::Identity(2,2);
    phi_input = Eigen::MatrixXd::Zero(2,N);
    decisionVariables_x = Eigen::VectorXd::Zero(N);
    decisionVariables_y = Eigen::VectorXd::Zero(N);

}

MPCSolver::~MPCSolver() {}

State MPCSolver::solve(State current, WalkState walkState, const Eigen::MatrixXd& ftsp_and_timings) {


    itr = walkState.mpcIter;
    fsCount = walkState.footstepCounter;

    // Output struct
    State next = current;

    Eigen::VectorXd z_acceleration = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd z_position = comTargetHeight * Eigen::VectorXd::Ones(N);;

    for (int j = 0; j< N; j++){ 
        lambda(j) = (g + z_acceleration(j))/z_position(j);
        etas(j) = sqrt(lambda(j));
        if (lambda(j) <= 0.0) etas(j) = eta;
    }


    Eigen::MatrixXd state_x = Eigen::MatrixXd::Zero(2,1);
    state_x << current.comPos(0), current.comVel(0);
    Eigen::MatrixXd state_y = Eigen::MatrixXd::Zero(2,1);
    state_y << current.comPos(1), current.comVel(1);

    // ZMP constraints
    Eigen::VectorXd Zmin_x, Zmax_x, Zmin_y, Zmax_y;
    Eigen::MatrixXd A_ineq_xy = Eigen::MatrixXd::Identity(N,N);

    Zmin_x = ftsp_midpoint.block((int)(walkState.simulationTime/(mpcTimeStep/controlTimeStep)),0,N,1) - p*footConstraintSquareWidth/2.0;
    Zmax_x = ftsp_midpoint.block((int)(walkState.simulationTime/(mpcTimeStep/controlTimeStep)),0,N,1) + p*footConstraintSquareWidth/2.0;
    Zmin_y = ftsp_midpoint.block((int)(walkState.simulationTime/(mpcTimeStep/controlTimeStep)),1,N,1) - p*footConstraintSquareWidth/2.0;
    Zmax_y = ftsp_midpoint.block((int)(walkState.simulationTime/(mpcTimeStep/controlTimeStep)),1,N,1) + p*footConstraintSquareWidth/2.0;   

    // Stability constraints  
    phi_state = Eigen::MatrixXd::Identity(2,2);
    
    for (int i = 0; i < N; i++) {

        double ch = cosh(sqrt(lambda(i))*mpcTimeStep);
        double sh = sinh(sqrt(lambda(i))*mpcTimeStep);
        A_xy << ch, sh/sqrt(lambda(i)), sqrt(lambda(i))*sh, ch;
        B_xy << 1-ch, -sqrt(lambda(i))*sh;
        phi_state = A_xy*phi_state;
        phi_input.block(0,i,2,1) = B_xy;

        for (int j = i+1; j < N; j++) {
          double ch = cosh(sqrt(lambda(j))*mpcTimeStep);
          double sh = sinh(sqrt(lambda(j))*mpcTimeStep);
          A_xy << ch, sh/sqrt(lambda(j)), sqrt(lambda(j))*sh, ch;
          phi_input.block(0,i,2,1) = A_xy*phi_input.block(0,i,2,1);
        }
        
     }

    double eta_sc = sqrt(lambda(N-1));
    eta_sc = eta;
    C_sc << 1.0, 1.0/eta_sc;
    Aeq_x = C_sc*phi_input;
    Aeq_y = C_sc*phi_input;

    Eigen::VectorXd buffer_eq_constr = -C_sc*phi_state*state_x + eta_sc*mpcTimeStep*deltas*ftsp_midpoint.block((int)(walkState.simulationTime/(mpcTimeStep/controlTimeStep))+N,0,N,1);
    beq_x(0) = buffer_eq_constr(0);
    buffer_eq_constr = -C_sc*phi_state*state_y + eta_sc*mpcTimeStep*deltas*ftsp_midpoint.block((int)(walkState.simulationTime/(mpcTimeStep/controlTimeStep))+N,1,N,1);
    beq_y(0) = buffer_eq_constr(0);

    // cost function xy
    costFunctionF_x = - ftsp_midpoint.block((int)(walkState.simulationTime/(mpcTimeStep/controlTimeStep)),0,N,1);
    costFunctionF_y = - ftsp_midpoint.block((int)(walkState.simulationTime/(mpcTimeStep/controlTimeStep)),1,N,1);

    costFunctionF_x(0) = costFunctionF_x(0) - current.zmpPos(0);
    costFunctionF_y(0) = costFunctionF_y(0) - current.zmpPos(1);

    // solve QP
    decisionVariables_x = solveQP_hpipm(costFunctionH_xy, costFunctionF_x, A_ineq_xy, Zmin_x, Zmax_x, Aeq_x, beq_x);
    decisionVariables_y = solveQP_hpipm(costFunctionH_xy, costFunctionF_y, A_ineq_xy, Zmin_y, Zmax_y, Aeq_y, beq_y);

    // State integration
    double ch = cosh(sqrt(lambda(0))*controlTimeStep);
    double sh = sinh(sqrt(lambda(0))*controlTimeStep);
    A_xy << ch, sh/sqrt(lambda(0)), sqrt(lambda(0))*sh, ch;
    B_xy << 1.0-ch, -sqrt(lambda(0))*sh;
 
    state_x = A_xy*state_x + B_xy*decisionVariables_x(0);
    state_y = A_xy*state_y + B_xy*decisionVariables_y(0);

    next.comPos << state_x(0), state_y(0), comTargetHeight;
    next.comVel << state_x(1), state_y(1), 0.0;
    next.comAcc << lambda(0) * (state_x(0) - decisionVariables_x(0)), lambda(0) * (state_y(0) - decisionVariables_y(0)), 0.0;
    next.zmpPos << decisionVariables_x(0), decisionVariables_y(0), 0.0;

    if (walkState.footstepCounter > 1 && walkState.footstepCounter <= n_steps+1) next = WalkingSwingFoot(current, next, walkState, ftsp_and_timings);

    return next;
}

State MPCSolver::WalkingSwingFoot(State current, State next, WalkState walkState, Eigen::MatrixXd ftsp_and_timings){

    Eigen::Vector4d footstepPredicted;
    footstepPredicted << ftsp_and_timings(fsCount,0), ftsp_and_timings(fsCount,1), 0.0, 0.0;

    if (walkState.footstepCounter == n_steps+1) footstepPredicted(1) = ftsp_and_timings(fsCount-2,1);

    double stepHeight_ = stepHeight;

    // If it's the first step, or we are in double support, keep the foot on the ground
    double timeSinceFootstepStart = (walkState.controlIter) * controlTimeStep;
    double singleSupportEnd = (double)(s_) * controlTimeStep;
    double swingFootHeight = -(4*stepHeight_/pow(singleSupportEnd,2)) * timeSinceFootstepStart * (timeSinceFootstepStart - singleSupportEnd);
    int samplesTillNextFootstep = s_+f_ - walkState.controlIter;

    if (walkState.footstepCounter <= 1) swingFootHeight = 0.0;
    if (walkState.controlIter >= s_) swingFootHeight = 0.0;

    int wait = 8;

    // If support foot is left, move right foot
    if (walkState.supportFoot == 1) {


	if (walkState.controlIter > wait && walkState.controlIter < s_-wait) next.rightFootPos += kSwingFoot * (footstepPredicted.head(3) - current.rightFootPos);
        

        next.rightFootPos(2) = swingFootHeight;
        next.leftFootPos(2) = 0.0;
        next.rightFootVel = Eigen::Vector3d::Zero();
        next.rightFootAcc = Eigen::Vector3d::Zero();
        next.rightFootOrient(2) += 5 * timeSinceFootstepStart * kSwingFoot * wrapToPi(footstepPredicted(3) - current.rightFootOrient(2));
        if (samplesTillNextFootstep > F+10 && samplesTillNextFootstep < F+S-10 && walkState.footstepCounter > 2) next.rightFootOrient(1) = 0.0;


    } else {


	if (walkState.controlIter > wait && walkState.controlIter < s_-wait) next.leftFootPos += kSwingFoot * (footstepPredicted.head(3) - current.leftFootPos);
       

        next.leftFootPos(2) = swingFootHeight;
        next.rightFootPos(2) = 0.0;
        next.leftFootVel = Eigen::Vector3d::Zero();
        next.leftFootAcc = Eigen::Vector3d::Zero();
        next.leftFootOrient(2) += 5 * timeSinceFootstepStart * kSwingFoot * wrapToPi(footstepPredicted(3) - current.leftFootOrient(2));
        if (samplesTillNextFootstep > F+10 && samplesTillNextFootstep < F+S-10 && walkState.footstepCounter > 2) next.leftFootOrient(1) = 0.0;

    }

    return next;
}

