// MPC control with moving masses for the MORUS project
// MPC algorithm file
// master thesis
// author: Luka Pevec

#include <morus_control/attitude_mpc_ctl_ceres.h>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <stdio.h>
#include <thread>
using namespace std; 
using namespace Eigen;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::CostFunction;
using ceres::SizedCostFunction;
namespace mav_control_attitude {
    MPCAttitudeController::MPCAttitudeController(const ros::NodeHandle& nh,
                                                 const ros::NodeHandle& private_nh)
            : nh_(nh),
              private_nh_(private_nh),
              initialized_parameters_(false),  // after call of the "initializedSystem" it gets to "true"
              initialized_observer_(false),
              enable_integrator_(true),
              enable_offset_free_(true),
              angle_error_integration_(0.0),
              disturbance_observer_(nh, private_nh),
              steady_state_calculation_(nh, private_nh),
              verbose_(false),
              sampling_time_(0.078684),
              prediction_sampling_time_(0.078684)
    {
     initializeParameters(); // init the system and its parameters

     // debugging publishers
     target_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/target_states/", 1);
     target_input_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/target_input/",  1);
     disturbances_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/disturbances/",  1);
     MPC_solver_status_pub_ = nh_.advertise<std_msgs::Int64>("mpc/solver_status/", 1);
     rotor_velocities_linearizes_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("motor_speed_lin/", 1);
     last_control_signal  << 0.0, 0.0, 0.0, 0.0;
    }

    MPCAttitudeController::~MPCAttitudeController() { }
    
    bool MPCAttitudeController::setSolverParameterSettings(){
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        options.num_threads = std::thread::hardware_concurrency();
        options.initial_trust_region_radius = 1e-2;//smanjuj ako su losi -10
        options.max_solver_time_in_seconds = 0.050;
        options.gradient_tolerance=1e-4;//-4 ili -5
        options.parameter_tolerance=1e-4;
        options.function_tolerance=1e-3;
    
    }

    bool MPCAttitudeController::SolveMyOptimizationProblem(ceres::Problem& problem) {
      //CHECK(&problem == NULL) return false;
      
      //Setting bounderies
      
      problem_upper_bounderies = last_control_signal;
      problem_lower_bounderies = last_control_signal;
      for(int j = 0; j< kControlHorizonSteps; j++){
            for(int i = 0; i< 2; i++){
                problem_upper_bounderies(i, 0) += du_max(i,0);
                problem_lower_bounderies(i, 0) += du_min(i,0);
                
                if(problem_upper_bounderies(i, 0) > u_max(i,0)) problem_upper_bounderies(i, 0) = u_max(i,0);
                if(problem_lower_bounderies(i, 0) < u_min(i,0)) problem_lower_bounderies(i, 0) = u_min(i,0);
                //ne bi smjelo bit tu
                problem_upper_bounderies(i, 0) = u_max(i,0);
                problem_lower_bounderies(i, 0) = u_min(i,0);
                
                problem.SetParameterLowerBound(x,i*kControlHorizonSteps + j, problem_lower_bounderies(i, 0));
                problem.SetParameterUpperBound(x,i*kControlHorizonSteps + j, problem_upper_bounderies(i,0));
            }
      
      
      }
      
      // Run the solver!

      ceres::Solve(options, &problem, &summary);

      //std::cout << summary.FullReport() << '\n';
      //std::cout << summary.BriefReport() << '\n';
    //std::cout << summary.IsSolutionUsable() << '\n';
      return summary.IsSolutionUsable();
    }
/**
    Set the matrices of the system dynamics model_A, model_B and model_Bd
    TODO Cound be done with Yaml file !!!
 */
    void MPCAttitudeController::initializeParameters()
    {
        // TODO throw in .yaml file !!!
        mass_ = 1.0;
        mass_quad_ = 30.8;
        M_ = mass_quad_ + 4 * mass_;
        mi_ = mass_ / mass_quad_;
        cd_ = 1.5;
        zr_ = 0.2;
        beta_ = 0;
        beta_gm_ = 0;
        zm_ = 0.05;
        Km_ = 1;

        lm_ = 0.6;
        arm_offset_ = 0.1;
        l_ = lm_ + arm_offset_;
        Tr_ = 100;
        double Iq_xx = 5.5268 + 0.2;
        double Iq_yy = 5.5268 + 0.2;
        double Iq_zz = 6.8854 + 0.4;
        Iq_ = Eigen::Matrix3d::Zero(3,3);
        Iq_(0,0) = Iq_xx;
        Iq_(1,1) = Iq_yy;
        Iq_(2,2) = Iq_zz;
        Iyy_b_ = 5.5268;
        Iyy_ = Iyy_b_ + 2*mass_*pow(lm_/2, 2);

        Tgm_ = 0.25;
        w_gm_n_ = 7000 / 60 * 2*M_PI;
        F_n_ = 25 * kGravity;
        b_gm_f_ = F_n_ / (pow(w_gm_n_,2));
        b_gm_m_ = 0.01;

        w_gm_0_ = sqrt(M_ * kGravity / 4.0 / b_gm_f_);
        F0_ = b_gm_f_ * pow(w_gm_0_,2);

        // true parameters experimental
        /*
        zeta_mm_ = 0.6551;
        w_mm_ = 14.8508;
        */

        // close true parameters proven with simulation
        // tm = 0.21 s, sigma_m = 0.065944
        zeta_mm_ = 0.6544;
        w_mm_ =   19.7845;

        // construct model matrices
        A_continous_time = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
        B_continous_time = Eigen::MatrixXd::Zero(kStateSize, kInputSize);
        Eigen::MatrixXd Bd_continous_time(kStateSize, kDisturbanceSize);
        Bd_continous_time = Eigen::MatrixXd::Zero(kStateSize, kDisturbanceSize);

        if (combined_control_mpc_use_){
          // CC_MPC //
          // states: (x)
          // [x1, dx1, x3, dx3, omega1, omega3, theta, dtheta] -> A is [8,8]
          // input signals: (u)
          // ['x1_ref (m)'; 'x3_ref (m)'; 'd_omega1 (rad/s)'; 'd_omega3 (rad/s)'] -> B is [8,4]
          A_continous_time(0,1) = 1.0;
          A_continous_time(1,0) = -pow(w_mm_,2);
          A_continous_time(1,1) = -2.0*zeta_mm_*w_mm_;
          A_continous_time(2,3) = 1.0;
          A_continous_time(3,2) = -pow(w_mm_,2);
          A_continous_time(3,3) = -2.0*zeta_mm_*w_mm_;
          A_continous_time(4,4) = -1.0/Tgm_;
          A_continous_time(5,5) = -1.0/Tgm_;
          A_continous_time(6,7) = 1.0;
          A_continous_time(7,0) = 1.0*mass_ / Iyy_ * (kGravity + ((1.0-4.0*mi_)*zm_*pow(w_mm_,2)));
          A_continous_time(7,1) = 2.0 * mass_*(1.0-4.0*mi_)*zm_*zeta_mm_*w_mm_/Iyy_;
          A_continous_time(7,2) = mass_ / Iyy_ * (kGravity + ((1.0-4.0*mi_)*zm_*pow(w_mm_,2)));
          A_continous_time(7,3) = 2.0 * mass_*(1.0-4.0*mi_)*zm_*zeta_mm_*w_mm_/Iyy_;
          A_continous_time(7,4) =  2 * b_gm_f_ * w_gm_0_ * lm_ / Iyy_;
          A_continous_time(7,5) = -2 * b_gm_f_ * w_gm_0_ * lm_ / Iyy_;

          B_continous_time(1,0) = pow(w_mm_,2);
          B_continous_time(3,1) = pow(w_mm_,2);
          B_continous_time(4,2) = 1.0 / Tgm_;
          B_continous_time(5,3) = 1.0 / Tgm_;
          B_continous_time(7,0) = -mass_ * (1.0-4.0*mi_)*zm_*pow(w_mm_,2) / Iyy_;
          B_continous_time(7,1) = -mass_ * (1.0-4.0*mi_)*zm_*pow(w_mm_,2) / Iyy_;

          // disturbance on every state -> B_d is [8,8]
          Bd_continous_time.setIdentity();

        } else{
          // MM_MPC //
          // states: (x)
          // [x1, dx1, x3, dx3, theta, dtheta] -> A is [6,6]
          // input signals: (u)
          // [x1_ref (m), x3_ref (m)] -> B is [6,2]
          A_continous_time(0,1) = 1.0;
          A_continous_time(1,0) = -pow(w_mm_,2);
          A_continous_time(1,1) = -2.0*zeta_mm_*w_mm_;
          A_continous_time(2,3) = 1.0;
          A_continous_time(3,2) = -pow(w_mm_,2);
          A_continous_time(3,3) = -2.0*zeta_mm_*w_mm_;
          A_continous_time(4,5) = 1.0;
          A_continous_time(5,0) = mass_ / Iyy_ * (kGravity + ((1.0-4.0*mi_)*zm_*pow(w_mm_,2)));      // dtheta =f(x1)
          A_continous_time(5,1) = 2.0 * mass_*(1.0-4.0*mi_)*zm_*zeta_mm_*w_mm_/Iyy_;                 // dtheta =f(v1)
          A_continous_time(5,2) = mass_ / Iyy_ * (kGravity + ((1.0-4.0*mi_)*zm_*pow(w_mm_,2)));      // dtheta =f(x1)
          A_continous_time(5,3) = 2.0 * mass_*(1.0-4.0*mi_)*zm_*zeta_mm_*w_mm_/Iyy_;                 // dtheta =f(v1)

          B_continous_time(1,0) = pow(w_mm_,2);
          B_continous_time(3,1) = pow(w_mm_,2);
          B_continous_time(5,0) = -mass_ * (1.0-4.0*mi_)*zm_*pow(w_mm_,2) / Iyy_;
          B_continous_time(5,1) = -mass_ * (1.0-4.0*mi_)*zm_*pow(w_mm_,2) / Iyy_;

          // disturbance on every state -> B_d is [6,6]
          Bd_continous_time.setIdentity();
        }
        Eigen::MatrixXd obnovljivost(8, 7*4);
        obnovljivost = Eigen::MatrixXd::Zero(8, 7*4);
        for(int i = 0; i< 7; i++){
            obnovljivost.block(0,4*i,8,4) = (A_continous_time.pow((double)i))*B_continous_time;
        }
        FullPivLU<Eigen::MatrixXd> lu_decomp(obnovljivost);

        ROS_INFO_STREAM("YOLOYOLOYOLO   obnovljivost   : "<< lu_decomp.rank());
        ROS_INFO_STREAM("A_contin   : "<< A_continous_time);
        ROS_INFO_STREAM("B_contin   : "<< B_continous_time);
        // discretization of matrix A
        model_A_ = ((prediction_sampling_time_) * A_continous_time).exp();
        //model_A_ = ((prediction_sampling_time_) * A_continous_time) + Eigen::MatrixXd::Identity(kStateSize, kStateSize);

        // calculate equation (42) from
        // "Model predictive control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System"
        Eigen::MatrixXd integral_exp_A;
        integral_exp_A = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
        const int count_integral_A = 100; // TODO see why that size, maybe aproximation of integration

        // discrete integration
        for (int i = 0; i < count_integral_A; i++) {
            integral_exp_A += (A_continous_time * prediction_sampling_time_ * i / count_integral_A).exp()
                              * prediction_sampling_time_ / count_integral_A;
        }

        // making the discrete matrices B and C
        //model_B_ = (model_A_ - Eigen::MatrixXd::Identity(kStateSize, kStateSize))*((A_continous_time).inverse())*B_continous_time;
        //model_B_ = (model_A_ - Eigen::MatrixXd::Identity(kStateSize, kStateSize))*((A_continous_time.transpose() * A_continous_time +4*Eigen::MatrixXd::Identity(kStateSize, kStateSize)).inverse()* A_continous_time.transpose())*B_continous_time;
        //model_B_ = (prediction_sampling_time_) *B_continous_time;
        model_B_ = integral_exp_A * B_continous_time;
          //integral_exp_A * B_continous_time;
        model_Bd_ = integral_exp_A * Bd_continous_time;
        
        obnovljivost = Eigen::MatrixXd::Zero(8, 7*4);
        for(int i = 0; i< 7; i++){
            obnovljivost.block(0,4*i,8,4) = (model_A_.pow((double)i))*model_B_;
        }
        FullPivLU<Eigen::MatrixXd> lu_decomp1(obnovljivost);

        ROS_INFO_STREAM("YOLOYOLOYOLO   obnovljivost disk  : "<< lu_decomp1.rank());
        
        Eigen::MatrixXd upravljivost(8, 7*4+8);
        upravljivost = Eigen::MatrixXd::Zero(8, 7*4 + 8);
        upravljivost.block(0,0,8, 7*4 )= obnovljivost;
        upravljivost.block(0, 7*4,8,8  ) = model_A_.pow(8.0);
        FullPivLU<Eigen::MatrixXd> lu_decomp2(upravljivost);

        ROS_INFO_STREAM("YOLOYOLOYOLO   upravljivost  disk : "<< lu_decomp2.rank());
        ROS_INFO_STREAM("model_A_   : "<< model_A_);
        ROS_INFO_STREAM("model_B_   : "<< model_B_);
        angle_error_integration_.setZero();

        if (verbose_) {
            ROS_INFO_STREAM("A:   \n" << model_A_);
            ROS_INFO_STREAM("B:   \n" << model_B_);
            ROS_INFO_STREAM("B_d: \n" << model_Bd_);
        }

        initialized_parameters_ = true;
        ROS_INFO("Linear MPC attitude controller: initialized correctly");
    }

    void MPCAttitudeController::applyParameters()
    {
      



      // init the cost matrices
      Q.setZero();
      Q_final.setZero();
      R.setZero();
      R_delta.setZero();

      if (combined_control_mpc_use_){
        // CC_MPC
        // fill the cost matrices - Q
        Q.block(0, 0, 4, 4) = q_moving_masses_.asDiagonal();
        Q.block(4, 4, 2, 2) = q_rotors_.asDiagonal();
        Q.block(6, 6, 2, 2) = q_attitude_.asDiagonal();

        // fill the cost matrices - R
        R = r_command_.asDiagonal();

        // fill the cost matrices - R_delta
        R_delta = r_delta_command_.asDiagonal();

      } else {
        // MM_MPC
        // fill the cost matrices - Q
        Q.block(0, 0, 4, 4) = q_moving_masses_.asDiagonal();
        Q.block(4, 4, 2, 2) = q_attitude_.asDiagonal();

        Eigen::Matrix<double, 2, 1> temp_r;
        // take only the first two for the moving masses
        temp_r << r_command_(0), r_command_(1);
        // fill the cost matrices - R
        R = temp_r.asDiagonal();

        Eigen::Matrix<double, 2, 1> temp_delta_r;
        // take only the first two for the moving masses
        temp_delta_r << r_delta_command_(0), r_delta_command_(1);
        // fill the cost matrices - R_delta
        R_delta = temp_delta_r.asDiagonal();
      }

      steady_state_calculation_.setRCommand(r_command_);
      steady_state_calculation_.initialize(A_continous_time, B_continous_time, model_Bd_);

      //Compute terminal cost - Riccaty equation
      //Q_final(k+1) = Q + A'*Q_final(k)*A - (A'*Q_final(k)*B)*inv(B'*Q_final(k)*B+R)*(B'*Q_final(k)*A);
      Q_final = Q;
      for (int i = 0; i < 1000; i++) {
        Eigen::MatrixXd temp = (model_B_.transpose() * Q_final * model_B_ + R);
        Q_final = model_A_.transpose() * Q_final * model_A_
            - (model_A_.transpose() * Q_final * model_B_) * temp.inverse()
                * (model_B_.transpose() * Q_final * model_A_) + Q;
      }

      // init the backup regulator - LQR
      Eigen::MatrixXd temporary_matrix = model_B_.transpose() * Q_final * model_B_ + R;
      LQR_K_ = temporary_matrix.inverse() * (model_B_.transpose() * Q_final * model_A_);


        
      u_max(0,0) =  lm_/2 - 0.01; 
      //u_max(1,0) =  lm_/2 - 0.01;
      u_min(0,0) =  -u_max(0,0); 
      //u_min(1,0) =  -u_max(1,0); 


      du_max(0,0) =  sampling_time_ * 2; 
      //du_max(1,0) =  sampling_time_ * 2;
      du_min(0,0) =  -du_max(0,0); 
      //du_min(1,0) =  -du_max(1,0); 

      if (combined_control_mpc_use_){
        u_max(1,0) =  50;
        //u_max(2,0) =  50; 
        //u_max(3,0) =  50;
        u_min(1,0) =  -u_max(1,0); 
        //u_min(2,0) =  -u_max(2,0); 
        //u_min(3,0) =  -u_max(3,0);

        du_max(1,0) =  1;
        //du_max(2,0) =  1; 
        //du_max(3,0) =  1;
        du_min(1,0) =  -du_max(1,0);
        //du_min(2,0) =  -du_max(2,0); 
        //du_min(3,0) =  -du_max(3,0);
      }


      ROS_INFO("Linear MPC: Tuning parameters updated...");
      if (verbose_) {
        ROS_INFO_STREAM("diag(Q) = \n" << Q.diagonal().transpose());
        ROS_INFO_STREAM("diag(R) = \n" << R.diagonal().transpose());
        ROS_INFO_STREAM("diag(R_delta) = \n " << R_delta.diagonal());

        ROS_INFO_STREAM("Q_final (terminal cost) = \n" << Q_final);
        ROS_INFO_STREAM("LQR_K_ = \n" << LQR_K_);
        Eigen::Matrix<double, kStateSize, kStateSize> closed_loop_dynamics = model_A_ - model_B_ * LQR_K_;
        ROS_INFO_STREAM("Closed loop dynamics = \n" << closed_loop_dynamics);
        ROS_INFO_STREAM("Closed loop eigenvalues absolute value (needed <1) = \n" << closed_loop_dynamics.eigenvalues().cwiseAbs());
      }
      if( isnan( Q_final.maxCoeff() ))  Q_final = 10*MatrixXd::Identity(kStateSize, kStateSize);
      
      
      
      
      R(0,0) =0.67670;
    R(1,1) = 0.67670;
    R(2,2) = 0.13534000;
    R(3,3) = 0.13534000;
       
    R_delta(0,0) = 0.738879858135067 ;
    R_delta(1,1) = 0.738879858135067 ;
    R_delta(2,2) = 0.007388798581351;
    R_delta(3,3) = 0.007388798581351;



    Q(0,0) = 0.135340000000000;
    Q(1,1) = 0.002706800000000;
    Q(2,2) =  0.1353400;
    Q(3,3) = 0.002706800;
    Q(4,4) = 0.002706800;
    Q(5,5) = 0.002706800;
    Q(6,6) = 10.7068000;
    Q(7,7) =9.676700;
      
      MV_scale = MatrixXd::Zero(kInputSize, kInputSize);
        OV_scale = MatrixXd::Zero(kStateSize, kStateSize);
        OV_scale(0,0) = 0.58;
        OV_scale(1,1) = 4.0;
        OV_scale(2,2) = 0.58;
        OV_scale(3,3) = 4.0;
        OV_scale(4,4) = 800;
        OV_scale(5,5) = 800;
        OV_scale(6,6) = 0.5236;
        OV_scale(7,7) = 0.5236;

        MV_scale(0,0) = 0.58;
        MV_scale(1,1) = 0.58;
        MV_scale(2,2) = 100;
        MV_scale(3,3) = 100;
      
      cost1 =  new MPC_cost(  model_A_,  model_B_,  model_Bd_,  Q, Q_final,
                              R,  R_delta, estimated_disturbances_, kStateSize, 14,kControlHorizonSteps , MV_scale,OV_scale);
      problem.AddResidualBlock(cost1, NULL, x);
      setSolverParameterSettings();
      
      
    }

    void MPCAttitudeController::calculateControlCommand(
        Eigen::Matrix<double, kInputSize, 1> *control_commands)
    {
      assert(control_commands != nullptr);
      assert(initialized_parameters_);
      
      //Declare variables
      Eigen::VectorXd KF_estimated_state;

      // Kalman filter and disturbance observer
      if (!initialized_observer_){
        disturbance_observer_.setInitialState(movable_mass_0_position_, movable_mass_0_speed_,
                                              movable_mass_1_position_, movable_mass_1_speed_,
                                              motor_0_speed_, motor_1_speed_,
                                              angle_, angular_velocity_);
        disturbance_observer_.setSystemMatrices(model_A_, model_B_, model_Bd_);
        initialized_observer_ = true;
      }

      disturbance_observer_.setMeasuredStates(movable_mass_0_position_, movable_mass_0_speed_,
                                              movable_mass_1_position_, movable_mass_1_speed_,
                                              motor_0_speed_, motor_1_speed_,
                                              angle_, angular_velocity_);
      disturbance_observer_.setMovingMassCommand(control_commands_temp_);

      bool observer_update_successful = disturbance_observer_.updateEstimator();

      if (!observer_update_successful){
        // reset observer and its states
        disturbance_observer_.setInitialState(movable_mass_0_position_, movable_mass_0_speed_,
                                              movable_mass_1_position_, movable_mass_1_speed_,
                                              motor_0_speed_, motor_1_speed_,
                                              angle_, angular_velocity_);
      }

      disturbance_observer_.getEstimatedState(&KF_estimated_state);

      if (enable_offset_free_) {
        estimated_disturbances_ = KF_estimated_state.segment(kStateSize, kDisturbanceSize);
        if (!getControllerName().compare("Roll controller") && verbose_){
          ROS_INFO_STREAM("estimated disturbances: \n" << estimated_disturbances_);
        }
      } else {
        estimated_disturbances_.setZero();
      }
        
      // feedback integration
      if(enable_integrator_){
        Eigen::Matrix<double, kMeasurementSize, 1> angle_error;

        angle_error(0) = angle_sp_ - angle_;

        double antiwindup_ball = 0.4; // TODO magic numbers - if current number too big
        // discrete integrator
        if (angle_error.norm() < antiwindup_ball) {
          angle_error_integration_ += angle_error * sampling_time_;
        } else {
          angle_error_integration_.setZero();
        }

        Eigen::Matrix<double, kMeasurementSize, 1> integration_limits;
        integration_limits(0) = 20.0; // TODO magic numbers - if integration too big
        angle_error_integration_ = angle_error_integration_.cwiseMax(-integration_limits);
        angle_error_integration_ = angle_error_integration_.cwiseMin(integration_limits);

        Eigen::Matrix<double, kDisturbanceSize, kMeasurementSize> K_I_MPC;
        K_I_MPC.Zero();
        K_I_MPC(4 + 2*combined_control_mpc_use_) = K_I_MPC_angle_; // set by dynamic reconfigure
        estimated_disturbances_ -= K_I_MPC * angle_error_integration_;
      };

      

      // creating the "target_state" and "current_state" variables
      target_state.setZero();
      target_state(4 + 2*combined_control_mpc_use_,0) = angle_sp_;

      current_state(0) = movable_mass_0_position_;
      current_state(1) = movable_mass_0_speed_;
      current_state(2) = movable_mass_1_position_;
      current_state(3) = movable_mass_1_speed_;
      if (combined_control_mpc_use_) {
        current_state(4) = motor_0_speed_;
        current_state(5) = motor_1_speed_;
      }
      current_state(4 + 2*combined_control_mpc_use_) = angle_;
      current_state(5 + 2*combined_control_mpc_use_) = angular_velocity_;

      Eigen::VectorXd ref(kMeasurementSize);
      ref << angle_sp_;
      
      if (enable_offset_free_){
        steady_state_calculation_.computeSteadyState(estimated_disturbances_, ref,
                                                    &target_state, &target_input);
        // Debugging variables
        if (!getControllerName().compare("Pitch controller")){
          // publish target_state
          std_msgs::Float64MultiArray target_state_msg;
          target_state_msg.data.clear();
          for (int index = 0; index < kStateSize; ++index) {
            target_state_msg.data.push_back(target_state(index));
          }
          target_state_pub_.publish(target_state_msg);

          // publish target_input
          std_msgs::Float64MultiArray target_input_msg;
          target_input_msg.data.clear();
          for (int index = 0; index < kInputSize; ++index) {
            target_input_msg.data.push_back(target_input(index));
          }
          target_input_pub_.publish(target_input_msg);

          // publish disturbances
          std_msgs::Float64MultiArray disturbances_msg;
          disturbances_msg.data.clear();
          for (int index = 0; index < kDisturbanceSize; ++index) {
            disturbances_msg.data.push_back(estimated_disturbances_(index));
          }
          disturbances_pub_.publish(disturbances_msg);
        }
      }

      if (!getControllerName().compare("Pitch controller") && verbose_){
        ROS_INFO_STREAM("target_states = \n" << target_state);
      }
       
      target_input.setZero();
      target_state.setZero();
      target_state(6,0) = ref(0);
      cost1->set_disturbance(estimated_disturbances_);
      cost1->set_x_ss(target_state);
      cost1->set_u_ss(target_input);
      cost1->set_u_current(last_control_signal);
      cost1->set_x0_(current_state);


      std_msgs::Int64 solver_status_msg;
      bool solution_found = SolveMyOptimizationProblem(problem);
      solver_status_msg.data = 0; 
      if(solution_found) solver_status_msg.data = 1;
      MPC_solver_status_pub_.publish(solver_status_msg);

      control_commands_temp_.setZero(); // reset the msg for input signals
      //ROS_INFO_STREAM( "oooooooooo target_state "  << target_state<< "\n");
      //ROS_INFO_STREAM( "++++ !!!!!!! residuum "  << cost1->get_residuum() << "  " <<solution_found << "\n");
      if (solution_found){ // solution found

        if (combined_control_mpc_use_) {
          // CC_MPC has 4 input variables
          
          control_commands_temp_ << x[0],x[0],  x[kControlHorizonSteps], -x[kControlHorizonSteps];
          last_control_signal = control_commands_temp_;

          for(int i = 0; i< kControlHorizonSteps-1; i++){
            x[0 + i] =  x[ i+1];
             x[kControlHorizonSteps + i] = x[kControlHorizonSteps +i +1];
              
          }
        } else {

        }
      }
      /*else { // solution not found -> LQR working
        ROS_WARN("Linear MPC: Solver failed, use LQR backup");
        Eigen::Matrix<double, kInputSize, 1> K_I;
        K_I.setOnes();
        K_I *= 1.5; // TODO magic number to look at, integrator constant

        // CALCULATING FEEDBACK WITH LQR !!!!!!
        error_states = target_state - current_state;
        control_commands_temp_ = LQR_K_ * error_states; // + K_I * angle_error_integration_;
      }*/

      // command min limits
      Eigen::Matrix<double, kInputSize, 1> lower_limits_roll;
      if (combined_control_mpc_use_) {
        lower_limits_roll << -(lm_/2.0 - 0.01), -(lm_/2.0 - 0.01), -50, -50;
      } else {
        lower_limits_roll << -(lm_/2.0 - 0.01), -(lm_/2.0 - 0.01);
      }
      control_commands_temp_ = control_commands_temp_.cwiseMax(lower_limits_roll);

      // command max limits
      Eigen::Matrix<double, kInputSize, 1> upper_limits_roll;
      if (combined_control_mpc_use_) {
        upper_limits_roll << (lm_/2.0 - 0.01), (lm_/2.0 - 0.01), 50, 50;
      } else {
        upper_limits_roll << (lm_/2.0 - 0.01), (lm_/2.0 - 0.01);
      }
      control_commands_temp_ = control_commands_temp_.cwiseMin(upper_limits_roll);
      *control_commands = control_commands_temp_;
    }
}
