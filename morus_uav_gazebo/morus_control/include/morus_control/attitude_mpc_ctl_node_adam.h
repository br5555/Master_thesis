/// <summary>
/// Class which receives data from sensors and send calculated signal to actuators.
/// </summary>
#ifndef PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H
#define PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H

#include <math.h>

#include <morus_control/PID.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <rosgraph_msgs/Clock.h>
#include <control_msgs/JointControllerState.h> // for moving masses states
#include <mav_msgs/Actuators.h> // for motor speed states
#include <morus_msgs/CommandMovingMasses.h>
#include <morus_msgs/AngleAndAngularVelocity.h>

#include <morus_control/attitude_mpc_ctl_adam.h>
#include <morus_control/attitude_teleop_joy.h>
#include <morus_control/ctpl.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <functional>
#include <iostream>
#include <string>
#include <fstream>

// dynamic reconfigure files
//#include <dynamic_reconfigure/server.h>
//#include <morus_control/MPCAttitudeControllerConfig.h>

namespace mav_control_attitude {

	/// <summary>
	/// Structur whoch represent #D point
	/// </summary>
    struct Vector3D{
        Vector3D()
        :m_x(0.0),m_y(0.0), m_z(0.0)
        {}
        Vector3D(double x,double y,double z)
        :m_x(x),m_y(y), m_z(z)
        {}

        double m_x;
        double m_y;
        double m_z;
    };


    class MPCAttitudeControllerNode{

    public:
		/// <summary>
		/// Constructor 
		/// </summary>
		/// <param name="nh">public ros node handle</param>
		/// <param name="private_nh">private ros node handle</param>
        MPCAttitudeControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

		/// <summary>
		/// Defaukt destructor
		/// </summary>
        ~MPCAttitudeControllerNode();

		/// <summary>
		/// Execute control loop
		/// </summary>
        void run();


    private:



        double a;
        ros::NodeHandle nh_, private_nh_;

        // classes and structures used

        AttitudeJoy attitude_joy_;

        //thread safe 
        ctpl::thread_pool p;// = ctpl::thread_pool(2 /* two threads in the pool */);
        
        double pid_on_off , MPC_on_off;
        
	    std::vector<std::future<void>> results;
        double roll_ref, pitch_ref;
        rosgraph_msgs::Clock roll_clock, pitch_clock;
        control_msgs::JointControllerState roll_moving_mass_state_0, roll_moving_mass_state_1, pitch_moving_mass_state_0, pitch_moving_mass_state_1;
        double roll_motor_0, roll_motor_1, pitch_motor_0, pitch_motor_1;
        double roll_angle_state, pitch_angle_state;
        double roll_angular_vel_state, pitch_angular_vel_state;

        // dynamic reconfigure server init
        //dynamic_reconfigure::Server<morus_control::MPCAttitudeControllerConfig> dyn_config_server_;
        //void DynConfigCallback(morus_control::MPCAttitudeControllerConfig &config, uint32_t level);

        // calculation of the future input signals
        virtual bool calculateControlCommand(Eigen::Matrix<double, kInputSize, 1>& control_commands,
                                             MPCAttitudeController& linear_mpc_commanded_angle);
        bool calculateCommands();

        // publishers
        ros::Publisher pub_mass0_;
        ros::Publisher pub_mass1_;
        ros::Publisher pub_mass2_;
        ros::Publisher pub_mass3_;
        ros::Publisher pub_rotors_;
        ros::Publisher pub_rotors_attitude_;
        ros::Publisher pub_turnOnOffMPC;
        // debugging publisher
        ros::Publisher pub_angle_state_;
        ros::Publisher euler_ref_pub;

        void publishCommands();

        // variables to hold the control variable

        bool start_flag_;

        // subscribers
        ros::Subscriber imu_subscriber_;
			/// <summary>
			/// Accepts reading from IMU sensor
			/// </summary>
			/// <param name="msg"></param>
            void AhrsCallback(const sensor_msgs::Imu& msg);
            struct euler_mv {
                double x;
                double y;
                double z;
            } euler_mv_;
            struct euler_rate_mv {
                double x;
                double y;
                double z;
            } euler_rate_mv_;
            bool imu_received_; // received msg from imu


        ros::Subscriber mot_vel_ref_subscriber_;
			/// <summary>
			/// Accepts reference for motor velocity
			/// </summary>
			/// <param name="msg">desire refrence</param>
            void MotVelRefCallback(const std_msgs::Float32& msg);
            float w_sp_;
        
        ros::Subscriber pid_mass_0_subscriber_;
			/// <summary>
			/// Accepts PID signal for mass 0
			/// </summary>
			/// <param name="msg">PID signal</param>
            void PIDMass0Callback(const std_msgs::Float64& msg);
            double mass_0_pid;
        
        ros::Subscriber pid_mass_1_subscriber_;
		/// <summary>
			/// Accepts PID signal for mass 1
			/// </summary>
			/// <param name="msg">sesnor readings</param>
            void PIDMass1Callback(const std_msgs::Float64& msg);
            double mass_1_pid;
            
            
        ros::Subscriber pid_mass_2_subscriber_;
		/// <summary>
			/// Accepts PID signal for mass 2
			/// </summary>
			/// <param name="msg">sesnor readings</param>
            void PIDMass2Callback(const std_msgs::Float64& msg);
            double mass_2_pid;
        
        ros::Subscriber pid_mass_3_subscriber_;
		/// <summary>
			/// Accepts PID signal for mass 3
			/// </summary>
			/// <param name="msg">sesnor readings</param>
            void PIDMass3Callback(const std_msgs::Float64& msg);
            double mass_3_pid;
        
        
        ros::Subscriber euler_ref_subscriber_;
			/// <summary>
			/// Accepts desire angle reference
			/// </summary>
			/// <param name="msg">desire reference</param>
            void EulerRefCallback(const geometry_msgs::Vector3& msg);
            geometry_msgs::Vector3 euler_sp_, nova_referenca;

        ros::Subscriber clock_subscriber_;
			/// <summary>
			/// Accepts Clock message
			/// </summary>
			/// <param name="msg">clock information</param>
            void ClockCallback(const rosgraph_msgs::Clock& msg);
            rosgraph_msgs::Clock clock_read_;

        ros::Subscriber movable_mass_0_state_subscriber_;
			/// <summary>
			/// Accepts sensor information about moving mass 0
			/// </summary>
			/// <param name="msg">sensor information</param>
            void MovingMass0Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_0_position_;
            double movable_mass_0_speed_;
            bool movable_mass_0_state_received_;

        ros::Subscriber movable_mass_1_state_subscriber_;
		/// <summary>
		/// Accepts sensor information about moving mass 1
		/// </summary>
		/// <param name="msg">sensor information</param>
            void MovingMass1Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_1_position_;
            double movable_mass_1_speed_;
            bool movable_mass_1_state_received_;

        ros::Subscriber turn_on_off_MPC_sub;

		/// <summary>
		/// Accepts turn on/off MPC controller
		/// </summary>
		/// <param name="msg"></param>
        void TurnOnOffCallback(const std_msgs::Bool& msg);
        bool turn_on_MPC;

        ros::Subscriber movable_mass_2_state_subscriber_;
			/// <summary>
			/// Accepts sensor information about moving mass 2
			/// </summary>
			/// <param name="msg">sensor information</param>
            void MovingMass2Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_2_position_;
            double movable_mass_2_speed_;
            bool movable_mass_2_state_received_;

        ros::Subscriber movable_mass_3_state_subscriber_;
			/// <summary>
			/// Accepts sensor information about moving mass 3
			/// </summary>
			/// <param name="msg">sensor information</param>
            void MovingMass3Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_3_position_;
            double movable_mass_3_speed_;
            bool movable_mass_3_state_received_;

        ros::Subscriber motor_speed_subscriber_;
			/// <summary>
			/// Accepts sensor information about rotors 
			/// </summary>
			/// <param name="msg">sensor information</param>
            void MotorSpeedCallback(const mav_msgs::Actuators& msg);
            double motor_0_speed_;
            double motor_1_speed_;
            double motor_2_speed_;
            double motor_3_speed_;
            bool motor_speed_received_;

        ros::Subscriber motor_speed_height_subscriber_;
			/// <summary>
			/// Accepts information from PID controller for z axis
			/// </summary>
			/// <param name="msg">controller signal</param>
            void MotorSpeedHeightCallback(const mav_msgs::Actuators& msg);

        // debug info
        bool verbose_;

        // variable for reference setting
        bool automatic_reference_;
		/// <summary>
		/// Sets default references
		/// </summary>
        void setAutomaticReference();

        
        PID m_pid_z, m_pid_vz, m_pid_yaw_rate, m_pid_roll , m_pid_roll_rate,
        m_pid_pitch, m_pid_pitch_rate, m_pid_yaw;


        bool m_start_flag, m_config_start;
        double m_z_sp, m_z_ref_filt, m_z_mv, m_vz_sp, m_vz_mv, m_dwz, m_mot_speed, m_gm_attitude_ctl, m_w_sp, m_mot_speed_hover;
        geometry_msgs::Vector3 m_euler_mv, m_euler_sp, m_euler_rate_mv;
        rosgraph_msgs::Clock m_clock;
        ros::Time m_t_old;

        ros::Subscriber pose_with_covariance_subscriber_;
		/// <summary>
		/// Accepts information from sensors about pose
		/// </summary>
		/// <param name="msg">sensor information</param>
        void PoseWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        geometry_msgs::PoseWithCovariance pose_with_covariance_msgs;

        ros::Subscriber velocity_subscriber_;
		/// <summary>
		/// Accepts information from sensors about velocity
		/// </summary>
		/// <param name="msg">sensor information</param>
        void VelocityCallback(const geometry_msgs::TwistStamped& msg);
        geometry_msgs::TwistStamped velocity_msgs;

        ros::Subscriber velocity_ref_subscriber_;
		/// <summary>
		/// Accepts desire velocity reference
		/// </summary>
		/// <param name="msg">vlocity reference</param>
        void VelocityRefCallback(const geometry_msgs::Vector3& msg);
        geometry_msgs::Vector3 velocity_ref_msgs;

        ros::Subscriber pose_ref_subscriber_;
		/// <summary>
		/// Accepts desire pose reference
		/// </summary>
		/// <param name="msg">pose refrence</param>
        void PoseRefCallback(const geometry_msgs::Vector3& msg);
        geometry_msgs::Vector3 Pose_ref_msgs;

    };
}
#endif //PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H
