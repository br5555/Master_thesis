// MPC control with moving masses for the MORUS project
// MPC algorithm file
// master thesis
// author: Luka Pevec

#include <morus_control/attitude_mpc_ctl_adam.h>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <stdio.h>
#include <thread>

using namespace std;
using namespace Eigen;


namespace mav_control_attitude {
    MPCAttitudeController::MPCAttitudeController(const ros::NodeHandle &nh,
                                                 const ros::NodeHandle &private_nh)
            : nh_(nh),
              private_nh_(private_nh),
              initialized_parameters_(false),  // after call of the "initializedSystem" it gets to "true"
              //initialized_observer_(false),
              //enable_integrator_(true),
              //enable_offset_free_(true),
              //angle_error_integration_(0.0),
              //disturbance_observer_(nh, private_nh),
              //steady_state_calculation_(nh, private_nh),
              //verbose_(false),
              //sampling_time_(0.078684),
              //prediction_sampling_time_(0.078684)
              cost_minimizer(std::unique_ptr<Adam_MPC>(new Adam_MPC()))
              {
        initializeParameters(); // init the system and its parameters
        new_ref = true;
        // debugging publishers
        target_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/target_states/", 1);
        target_input_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/target_input/", 1);
        disturbances_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/disturbances/", 1);
        MPC_solver_status_pub_ = nh_.advertise<std_msgs::Int64>("mpc/solver_status/", 1);
        rotor_velocities_linearizes_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("motor_speed_lin/", 1);
        current_state_space_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/current_state_space/", 1);

         
        //last_control_signal << 0.0, 0.0, 0.0, 0.0;
    }

    MPCAttitudeController::~MPCAttitudeController() {}


/**
    Set the matrices of the system dynamics model_A, model_B and model_Bd
    TODO Cound be done with Yaml file !!!
 */
    void MPCAttitudeController::initializeParameters() {
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
        Iq_ = Eigen::Matrix3d::Zero(3, 3);
        Iq_(0, 0) = Iq_xx;
        Iq_(1, 1) = Iq_yy;
        Iq_(2, 2) = Iq_zz;
        Iyy_b_ = 5.5268;
        Iyy_ = Iyy_b_ + 2 * mass_ * pow(lm_ / 2, 2);

        Tgm_ = 0.25;
        w_gm_n_ = 7000 / 60 * 2 * M_PI;
        F_n_ = 25 * kGravity;
        b_gm_f_ = F_n_ / (pow(w_gm_n_, 2));
        b_gm_m_ = 0.01;

        w_gm_0_ = sqrt(M_ * kGravity / 4.0 / b_gm_f_);
        F0_ = b_gm_f_ * pow(w_gm_0_, 2);

        // true parameters experimental
        /*
        zeta_mm_ = 0.6551;
        w_mm_ = 14.8508;
        */

        // close true parameters proven with simulation
        // tm = 0.21 s, sigma_m = 0.065944
        zeta_mm_ = 0.6544;
        w_mm_ = 19.7845;

        // construct model matrices
        A_continous_time = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
        B_continous_time = Eigen::MatrixXd::Zero(kStateSize, kInputSize);
        Eigen::MatrixXd Bd_continous_time(kStateSize, kDisturbanceSize);
        Bd_continous_time = Eigen::MatrixXd::Zero(kStateSize, kDisturbanceSize);

        
            // CC_MPC //
            // states: (x)
            // [x1, dx1, x3, dx3, omega1, omega3, theta, dtheta] . A is [8,8]
            // input signals: (u)
            // ['x1_ref (m)'; 'x3_ref (m)'; 'd_omega1 (rad/s)'; 'd_omega3 (rad/s)'] . B is [8,4]
            A_continous_time(0, 1) = 1.0;
            A_continous_time(1, 0) = -pow(w_mm_, 2);
            A_continous_time(1, 1) = -2.0 * zeta_mm_ * w_mm_;
            A_continous_time(2, 3) = 1.0;
            A_continous_time(3, 2) = -pow(w_mm_, 2);
            A_continous_time(3, 3) = -2.0 * zeta_mm_ * w_mm_;
            A_continous_time(4, 4) = -1.0 / Tgm_;
            A_continous_time(5, 5) = -1.0 / Tgm_;
            A_continous_time(6, 7) = 1.0;
            A_continous_time(7, 0) = 1.0 * mass_ / Iyy_ * (kGravity + ((1.0 - 4.0 * mi_) * zm_ * pow(w_mm_, 2)));
            A_continous_time(7, 1) = 2.0 * mass_ * (1.0 - 4.0 * mi_) * zm_ * zeta_mm_ * w_mm_ / Iyy_;
            A_continous_time(7, 2) = mass_ / Iyy_ * (kGravity + ((1.0 - 4.0 * mi_) * zm_ * pow(w_mm_, 2)));
            A_continous_time(7, 3) = 2.0 * mass_ * (1.0 - 4.0 * mi_) * zm_ * zeta_mm_ * w_mm_ / Iyy_;
            A_continous_time(7, 4) = 2 * b_gm_f_ * w_gm_0_ * lm_ / Iyy_;
            A_continous_time(7, 5) = -2 * b_gm_f_ * w_gm_0_ * lm_ / Iyy_;

            B_continous_time(1, 0) = pow(w_mm_, 2);
            B_continous_time(3, 1) = pow(w_mm_, 2);
            B_continous_time(4, 2) = 1.0 / Tgm_;
            B_continous_time(5, 3) = 1.0 / Tgm_;
            B_continous_time(7, 0) = -mass_ * (1.0 - 4.0 * mi_) * zm_ * pow(w_mm_, 2) / Iyy_;
            B_continous_time(7, 1) = -mass_ * (1.0 - 4.0 * mi_) * zm_ * pow(w_mm_, 2) / Iyy_;

            // disturbance on every state . B_d is [8,8]
            Bd_continous_time.setIdentity();

        


        std::cout << "B cont "<< std::endl <<  B_continous_time << std::endl << std::endl;
        std::cout << "A cont "<< std::endl <<  A_continous_time << std::endl << std::endl;


        // discretization of matrix A
        model_A_ = ((kSamplingTime) * A_continous_time).exp();


        // calculate equation (42) from
        // "Model predictive control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System"
        Eigen::MatrixXd integral_exp_A;
        integral_exp_A = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
        const int count_integral_A = 10000; // TODO see why that size, maybe aproximation of integration

        // discrete integration
        for (int i = 0; i < count_integral_A; i++) {
            integral_exp_A += (A_continous_time * kSamplingTime * i / count_integral_A).exp()
                              * kSamplingTime / count_integral_A;
        }


        model_B_ = integral_exp_A * B_continous_time;
        //integral_exp_A * B_continous_time;
        model_Bd_ = integral_exp_A * Bd_continous_time;


        //angle_error_integration_.setZero();


        initialized_parameters_ = true;
    }

    void MPCAttitudeController::applyParameters() {

        // init the cost matrices
        Q.setZero();
        Q_final.setZero();
        R.setZero();
        R_delta.setZero();


        u_max(0, 0) = lm_ / 2 - 0.01;
        u_min(0, 0) = -u_max(0, 0);

        du_max(0, 0) = kSamplingTime * 2;
        du_min(0, 0) = -du_max(0, 0);

        u_max(1, 0) = 50;
        u_min(1, 0) = -u_max(1, 0);

        du_max(1, 0) = 2;
        du_min(1, 0) = -du_max(1, 0);


        MV_scale.setZero();
        OV_scale.setZero();
        OV_scale(0, 0) = 0.58;
        OV_scale(1, 1) = 4.0;
        OV_scale(2, 2) = 0.58;
        OV_scale(3, 3) = 4.0;
        OV_scale(4, 4) = 800;
        OV_scale(5, 5) = 800;
        OV_scale(6, 6) = 0.5236;
        OV_scale(7, 7) = 0.5236;

        MV_scale(0, 0) = 0.58;
        MV_scale(1, 1) = 0.58;
        MV_scale(2, 2) = 100;
        MV_scale(3, 3) = 100;


        Q_final.setIdentity();

        Q = Q_final;
        R.setIdentity();

        R_delta = R;


        R(0, 0) = 0.67670;
        R(1, 1) = 0.67670;
        R(2, 2) = 0.13534000;
        R(3, 3) = 0.13534000;

        R_delta(0, 0) = 0.738879858135067 ;
        R_delta(1, 1) = 0.738879858135067 ;
        R_delta(2, 2) = 0.007388798581351;
        R_delta(3, 3) = 0.007388798581351;


        Q(0, 0) = 0.135340000000000;
        Q(1, 1) = 0.002706800000000;
        Q(2, 2) = 0.1353400;
        Q(3, 3) = 0.002706800;
        Q(4, 4) = 0.002706800;
        Q(5, 5) = 0.002706800;
        Q(6, 6) = 10.7068000;
        Q(7, 7) = 9.676700;




        //Q(0, 0) = 0.135340000000000*1;
        //Q(2, 2) = 0.1353400*1;
        Q(6, 6) = 1089.4068000;
        //Q(7, 7) = 20.676700;


        
        //PSO valja only angle
       R(0, 0) = 0.011112323538018 *1e7;
        R(1, 1) = 0.011112323538018 *1e7;
        R(2, 2) = 7.918260215207514 *1e7;
        R(3, 3) = 7.918260215207514 *1e7;

        R_delta(0, 0) = 0 *1e7;
        R_delta(1, 1) = 0 *1e7;
        R_delta(2, 2) = 3.431888811914739 *1e7;
        R_delta(3, 3) = 3.431888811914739 *1e7;


        Q(0, 0) = 0.002917586383233 *1e7;
        Q(1, 1) = 0*1e7;
        Q(2, 2) = 0.002917586383233 *1e7;
        Q(3, 3) = 0 *1e7;
        Q(4, 4) = 0 *1e7;
        Q(5, 5) = 0 *1e7;
        Q(6, 6) = 10.009244324116079 *1e7;
        Q(7, 7) = 0*1e7;






        x.setZero();
        
        std::cout << "A model " << std::endl << std::endl << model_A_ << std::endl;
         std::cout << "B model " << std::endl << std::endl << model_B_ << std::endl;
        
        estimated_disturbances_.setZero();
        cost_minimizer = std::unique_ptr<Adam_MPC>(new Adam_MPC(model_A_, model_B_, model_Bd_, Q, Q_final, R, R_delta, estimated_disturbances_,
                                  kStateSize, 14, 5, MV_scale, OV_scale, kSamplingTime));

//        cost_minimizer = std::unique_ptr<RMS_Prop_with_Nester_momentum_MPC>(new RMS_Prop_with_Nester_momentum_MPC(model_A_, model_B_, model_Bd_, Q, Q_final, R, R_delta, estimated_disturbances_,
//                                  kStateSize, 14, kControlHorizonSteps, MV_scale, OV_scale, kSamplingTime));

        /*cost_minimizer = std::unique_ptr<RMS_Prop_MPC>(new RMS_Prop_MPC(model_A_, model_B_, model_Bd_, Q, Q_final, R, R_delta, estimated_disturbances_,
                                                                        kStateSize, 14, kControlHorizonSteps, MV_scale, OV_scale));*/
//        cost_minimizer = std::unique_ptr<AdaGrad>(new AdaGrad(model_A_, model_B_, model_Bd_, Q, Q_final, R, R_delta, estimated_disturbances_,
//                                  kStateSize, 14, kControlHorizonSteps, MV_scale, OV_scale, kSamplingTime));
                                  
        /* cost_minimizer =  Adam_MPC(model_A_, model_B_, model_Bd_, Q, Q_final, R, R_delta, estimated_disturbances_,
                                  kStateSize, 14, kControlHorizonSteps, MV_scale, OV_scale, kSamplingTime);   */
        //don't care
        cost_minimizer->set_disturbance(estimated_disturbances_);

        target_state.setZero();
        cost_minimizer->set_x_ss(target_state);
        //always the same
        target_input.setZero();
        cost_minimizer->set_u_ss(target_input);
        current_state.setZero();


    }

    void MPCAttitudeController::calculateControlCommand(
            Eigen::Matrix<double, kInputSize, 1> &control_commands) {
        if(new_ref){
            target_state(6,0) = angle_sp_;
            cost_minimizer->set_x_ss(target_state);
            //std::cout << "========================== NEW TARGET ==============" << std::endl << std::endl;
            new_ref = false;
        }





        //std::cout << "Current state " << controller_name_<< std::endl << current_state << std::endl << std::endl;


        cost_minimizer->set_x0_(current_state);

        cost_minimizer->Evaluate(x);


        
        control_commands << x(0 * kControlHorizonSteps, 0), x(0 * kControlHorizonSteps, 0),
                x(1 * kControlHorizonSteps, 0), -x(1 * kControlHorizonSteps, 0);

//        for(int x_iter = 0; x_iter < kControlHorizonSteps-1; x_iter++)
//        {
//            x(x_iter,0) = x(x_iter+1,0);
//            x(x_iter + kControlHorizonSteps,0) = x(x_iter+1 + kControlHorizonSteps,0);
//        }

        cost_minimizer->set_u_current(control_commands);

        if (!getControllerName().compare("Roll controller")) {
            std_msgs::Float64MultiArray target_state_msg;
            target_state_msg.data.clear();
            for (int index = 0; index < kStateSize; ++index) {
                target_state_msg.data.push_back(current_state(index));
            }
            current_state_space_pub_.publish(target_state_msg);
        }


       // std::cout << control_commands << std::endl << std::endl;


    }
}
