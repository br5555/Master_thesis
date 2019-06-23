/// <summary>
/// Class representing MPC controller
/// 
/// author: Branko Rado�
/// </summary>

#ifndef PROJECT_MASS_CTL_ATTITUDE_MPC_H
#define PROJECT_MASS_CTL_ATTITUDE_MPC_H

#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <rosgraph_msgs/Clock.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64.h>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>
#include <memory>
#include <morus_control/Adam_MPC.h>
#include <morus_control/RMS_Prop_MPC.h>
#include <morus_control/RMS_Prop_with_Nester_momentum_MPC.h>
#include <morus_control/AdaGrad.h>
#include <iostream>




namespace mav_control_attitude {

    constexpr int combined_control_mpc_use_ = 1;  // still working with moving masses

    // MM_MPC + added variables for CC_MPC
    constexpr int kStateSize = 6 + 2 * combined_control_mpc_use_; // [x1, dx1, x3, dx3, theta, dtheta] -> A is [6,6]
    constexpr int kInputSize = 2 + 2 * combined_control_mpc_use_; // [x1_ref (m), x3_ref (m)]          -> B is [6,2]
    constexpr int kMeasurementSize = 1;                        // [theta] -> C is [1,6]
    constexpr int kDisturbanceSize = kStateSize;               // disturbances are looked on all states -> B_d is [6,6]

	/// <summary>
	/// Prediction horizon
	/// </summary>
    constexpr int kPredictionHorizonSteps = 14;

	/// <summary>
	/// Control horizon
	/// </summary>
    constexpr int kControlHorizonSteps = 5;

	/// <summary>
	/// Gravity constant
	/// </summary>
    constexpr double kGravity = 9.80665;

	/// <summary>
	/// Simulation rate
	/// </summary>
    constexpr double ksimRate = 0.97;

	/// <summary>
	/// Sampling time
	/// </summary>
    constexpr double kSamplingTime =0.04;//0.078684;//0.04


    class MPCAttitudeController {
    public:
		/// <summary>
		/// Constructor which construct instance of MPC controller
		/// </summary>
		/// <param name="nh">public ros node handle</param>
		/// <param name="private_nh">private ros node handle</param>
        MPCAttitudeController(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);

		/// <summary>
		/// Default constructor
		/// </summary>
        ~MPCAttitudeController();


        /// <summary>
        /// Applying desire parameters 
        /// </summary>
        void applyParameters();

        /// <summary>
        /// compute constrol inpute
        /// </summary>
        /// <param name="control_commands">calculated control input</param>
        void calculateControlCommand(Eigen::Matrix<double, kInputSize, 1> &control_commands);


        /// <summary>
        /// Setters which set angle reference
        /// </summary>
        /// <param name="angle_sp">desire angle reference</param>
        inline void setAngleRef(double angle_sp) {
            new_ref = false;
            if(abs(angle_sp_ - angle_sp)>1e-6){
                angle_sp_ = angle_sp;
                new_ref = true;
            }
                
                
        }

		/// <summary>
		/// Setters which sets clock
		/// </summary>
		/// <param name="clock">desire clock</param>
        inline void setClock(rosgraph_msgs::Clock clock) {
            clock_read_ = clock;
        }

		/// <summary>
		/// Setters which sets information from sensors about moving masses
		/// </summary>
		/// <param name="msg">sensor information</param>
		/// <param name="number_moving_mass">ordinal number of moving mass</param>
		/// <param name="gain_reading">direction</param>
        inline void setMovingMassState(control_msgs::JointControllerState msg,
                                int number_moving_mass,
                                double gain_reading) {
            switch (number_moving_mass) {
                case 0 :
                    movable_mass_0_position_ = gain_reading * msg.process_value;
                    movable_mass_0_speed_ = gain_reading * msg.process_value_dot;
                    current_state(0) = movable_mass_0_position_;
                    current_state(1) = movable_mass_0_speed_;
                    break;
                case 1 :
                    movable_mass_1_position_ = gain_reading * msg.process_value;
                    movable_mass_1_speed_ = gain_reading * msg.process_value_dot;
                    current_state(2) = movable_mass_1_position_;
                    current_state(3) = movable_mass_1_speed_;
                    break;
            }
        }

		/// <summary>
		/// Setters which sets information from sensors about ICE states
		/// </summary>
		/// <param name="motor_0_speed">speed of motor 0 </param>
		/// <param name="motor_1_speed">speed of motor 1</param>
        inline void setMotorState(double motor_0_speed, double motor_1_speed) {
            //double motor_diff = (motor_0_speed - motor_1_speed)/2.0;
            //std::cout << "****************Hover je " << w_gm_0_ << std::endl << std::endl;

            motor_0_speed_ =  motor_0_speed - w_gm_0_; // linearization around hovering speed
            motor_1_speed_ = motor_1_speed - w_gm_0_;

            current_state(4) = motor_0_speed_;
            current_state(5) = motor_1_speed_;

            /*if (!getControllerName().compare("Pitch controller")) {
                std_msgs::Float64MultiArray motor_msg;
                motor_msg.data.clear();
                motor_msg.data.push_back(motor_0_speed_);
                motor_msg.data.push_back(motor_1_speed_);
                // debugging to see the rotor behaviour
                rotor_velocities_linearizes_pub_.publish(motor_msg);
                current_state(2) = movable_mass_1_position_;
                current_state(3) = movable_mass_1_speed_;
            }*/
        }

		/// <summary>
		/// Setters which sets information from sensors about angle
		/// </summary>
		/// <param name="angle">read angle </param>
        inline void setAngleState(double angle) {
            angle_ = angle;
            current_state(6) = angle_;

        }

		/// <summary>
		/// Setters which sets information from sensor about angular velocity
		/// </summary>
		/// <param name="angular_velocity"></param>
        inline void setAngularVelocityState(double angular_velocity) {
            angular_velocity_ = angular_velocity;
            current_state(7) = angular_velocity_;
        }

       /* inline void setIntegratorConstantMPC(double K_I_MPC_angle) {
            K_I_MPC_angle_ = K_I_MPC_angle;
        }*/

       /* inline void setPenaltyMovingMasses(double q_p0, double q_v0, double q_p1, double q_v1) {
            q_moving_masses_ << q_p0, q_v0, q_p1, q_v1;
        }

        inline void setPenaltyRotors(double q_omega_0, double q_omega_1) {
            q_rotors_ << q_omega_0, q_omega_1;
        }

        inline void setPenaltyAttitude(double q_theta, double q_omega) {
            q_attitude_ << q_theta, q_omega;
        }

        inline void setPenaltyCommand(double r_0, double r_1, double r_2, double r_3) {
            r_command_ << r_0, r_1, r_2, r_3;
        }

        inline void setPenaltyChangeCommand(double r_delta_0, double r_delta_1, double r_delta_2, double r_delta_3) {
            r_delta_command_ << r_delta_0, r_delta_1, r_delta_2, r_delta_3;
        }*/

		/// <summary>
		/// Setters which sets controller name
		/// </summary>
		/// <param name="controller_name">controller name</param>
        inline void setControllerName(std::string controller_name) {
            controller_name_ = controller_name;
        }


        /// <summary>
        /// Getters which gets controller name
        /// </summary>
        /// <returns>controller name</returns>
        inline std::string getControllerName() {
            return controller_name_;
        }

//If you define a structure having members of fixed-size vectorizable Eigen types, you must overload its "operator new" so that it generates 16-bytes-aligned pointers. Fortunately, Eigen provides you with a macro EIGEN_MAKE_ALIGNED_OPERATOR_NEW that does that for you.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    private:
        bool new_ref;
        //Adam_MPC cost_minimizer;
        std::unique_ptr<MPC_SOLVER> cost_minimizer;
        Eigen::Matrix<double, kStateSize, kStateSize> A_continous_time;

        Eigen::Matrix<double, kStateSize, kInputSize> B_continous_time;

        Matrix<double, 2 * mpc_control_horizon, 1> x;

        Matrix<double, num_state_variables, 1> estimated_disturbances_;

        // ros node handles
        ros::NodeHandle nh_, private_nh_;

        // publishers for debugging
        ros::Publisher target_state_pub_,current_state_space_pub_;
        ros::Publisher target_input_pub_;
        ros::Publisher disturbances_pub_;
        ros::Publisher MPC_solver_status_pub_;
        ros::Publisher rotor_velocities_linearizes_pub_;

        //initialize system
        void initializeParameters();

        bool initialized_parameters_;

        // controller variables
        double angle_sp_;
        rosgraph_msgs::Clock clock_read_;
        // states of the system
        double movable_mass_0_position_;
        double movable_mass_0_speed_;
        double movable_mass_1_position_;
        double movable_mass_1_speed_;
        double motor_0_speed_; // speed of the rotor which increases the angle regulated
        double motor_1_speed_; // speed of the rotor which decreases the angle regulated
        double angle_;
        double angular_velocity_;

        // name
        std::string controller_name_;

        // system model
        // Model: A, B, Bd
        // x(k+1) = A*x(k) + B*u(k) + Bd*d(k)
        Eigen::Matrix<double, kStateSize, kStateSize> model_A_;   //dynamics matrix
        Eigen::Matrix<double, kStateSize, kInputSize> model_B_;   //transfer matrix
        Eigen::Matrix<double, kStateSize, kDisturbanceSize> model_Bd_;  //Disturbance transfer  gas motor paramsmatrix

        /// dynamic init of controller parameters
        Eigen::Matrix<double, kStateSize, kStateSize> Q;
        Eigen::Matrix<double, kStateSize, kStateSize> Q_final;
        Eigen::Matrix<double,kInputSize,kInputSize>  R;
        Eigen::Matrix<double,kInputSize,kInputSize> R_delta;
        Eigen::Matrix<double, kStateSize, kStateSize> OV_scale;
        Eigen::Matrix<double, kInputSize, kInputSize> MV_scale;
        Eigen::Matrix<double, kStateSize, 1> target_state, current_state, error_states;
        Eigen::Matrix<double, kInputSize, 1> target_input;

        //upper and lower bounderies for input signals
        Eigen::Matrix<double, kInputSize, 1> u_max;
        Eigen::Matrix<double, kInputSize, 1> du_max;
        Eigen::Matrix<double, kInputSize, 1> u_min;
        Eigen::Matrix<double, kInputSize, 1> du_min;

        Eigen::Matrix<double, kInputSize, 1> problem_upper_bounderies;
        Eigen::Matrix<double, kInputSize, 1> problem_lower_bounderies;


        // quadrotor params with moving masses
        double mass_;        // mass of a movable mass
        double mass_quad_;   // mass of the quadrotor body (including gas motors)
        double M_;           // total mass
        double mi_;          // additional constant for equations
        double cd_;          // drag constant (translational)
        double zr_;          // added coefficient for flapping
        double beta_;        // inclination angle of the motor arms
        double beta_gm_;     // this is additional angle of the gas motor prop w.r.t. the motor arm
        double zm_;          // mass displacement iz z-axis
        double Km_;          // voltage to acceleration
        double lm_;          // mass path maximal length
        double arm_offset_;  // motor arm offset from the origin
        double l_;           // motor arm length
        double Tr_;          // transmission rate
        Eigen::Matrix3d Iq_; // moment of inertia of the quadrotor body (without masses)
        double Iyy_b_;
        double Iyy_;

        double Tgm_;  // time constant
        double w_gm_n_; // rpm to rad/s
        double F_n_;
        double b_gm_f_;
        double b_gm_m_; // lucky guess
        double w_gm_0_;
        double F0_;

        // moving mass dynamics parameters (w_mm and zeta_mm)
        double zeta_mm_;
        double tm_;
        double w_mm_;


        // TODO prebacit u YAML FILE pa da se dobije sa get_param
        // sampling time parameters
        //double sampling_time_;
        //double prediction_sampling_time_;
    };
}
#endif //PROJECT_MASS_CTL_ATTITUDE_MPC_H
