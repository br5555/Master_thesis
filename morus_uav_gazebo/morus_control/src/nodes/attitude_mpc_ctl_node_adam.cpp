#include <morus_control/attitude_mpc_ctl_node_adam.h>
#include <morus_control/PID.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <rosgraph_msgs/Clock.h>


typedef std::chrono::high_resolution_clock Clock;

namespace mav_control_attitude {
    MPCAttitudeControllerNode::MPCAttitudeControllerNode(const ros::NodeHandle &nh,
                                                         const ros::NodeHandle &private_nh)
            : nh_(nh),
              private_nh_(private_nh),
              attitude_joy_(nh_, private_nh_),
            //dyn_config_server_(private_nh_),
              start_flag_(false),  // flag for the first measurement
              automatic_reference_(false),
              verbose_(false) {

        p.resize(2);
        results = std::vector<std::future<void>>(2);
        // init the readings od moving mass and rotor sensors
        movable_mass_0_position_ = 0.0;
        movable_mass_1_position_ = 0.0;
        movable_mass_2_position_ = 0.0;
        movable_mass_3_position_ = 0.0;
        movable_mass_0_speed_ = 0.0;
        movable_mass_1_speed_ = 0.0;
        movable_mass_2_speed_ = 0.0;
        movable_mass_3_speed_ = 0.0;
        motor_0_speed_ = 0.0;
        motor_1_speed_ = 0.0;
        motor_2_speed_ = 0.0;
        motor_3_speed_ = 0.0;

        euler_sp_.x = 0.0;
        euler_sp_.y = 0.0;
        euler_sp_.z = 0.0;

        nova_referenca.x = 0.0;
        nova_referenca.y = 0.0;
        nova_referenca.z = 0.0;

        m_mot_speed_hover = 427.0;

        // dynamic reconfigure server
        //dynamic_reconfigure::Server<morus_control::MPCAttitudeControllerConfig>::CallbackType f;
        //f = boost::bind(&MPCAttitudeControllerNode::DynConfigCallback, this, _1, _2);
        //dyn_config_server_.setCallback(f);

        // Publishers  ( nh -> )
        pub_mass0_ = nh_.advertise<std_msgs::Float64>("movable_mass_0_position_controller/command", 1);
        pub_mass1_ = nh_.advertise<std_msgs::Float64>("movable_mass_1_position_controller/command", 1);
        pub_mass2_ = nh_.advertise<std_msgs::Float64>("movable_mass_2_position_controller/command", 1);
        pub_mass3_ = nh_.advertise<std_msgs::Float64>("movable_mass_3_position_controller/command", 1);
        pub_angle_state_ = nh_.advertise<morus_msgs::AngleAndAngularVelocity>("angles", 1);
        pub_rotors_ = nh_.advertise<mav_msgs::Actuators>("/gazebo/command/motor_speed", 1);
        pub_rotors_attitude_ = nh_.advertise<mav_msgs::Actuators>("command/attitude/motor_speed", 1);
        pub_turnOnOffMPC = nh_.advertise<std_msgs::Bool>("turnOnOFFMPC", 1);
        euler_ref_pub = nh_.advertise<geometry_msgs::Vector3>("euler_ref", 1);

        // Subscribers ( nh <- )
        imu_subscriber_ = nh_.subscribe("imu", 1, &MPCAttitudeControllerNode::AhrsCallback,
                                        this); // measured values info
        imu_received_ = false;
        mot_vel_ref_subscriber_ = nh_.subscribe("mot_vel_ref", 1, &MPCAttitudeControllerNode::MotVelRefCallback, this);
        euler_ref_subscriber_ = nh_.subscribe("euler_ref", 1, &MPCAttitudeControllerNode::EulerRefCallback,
                                              this); // reference for the angles
        clock_subscriber_ = nh_.subscribe("/clock", 1, &MPCAttitudeControllerNode::ClockCallback,
                                          this);  // internal clock variable
        // position of mass 0
        movable_mass_0_state_subscriber_ = nh_.subscribe("movable_mass_0_position_controller/state", 1,
                                                         &MPCAttitudeControllerNode::MovingMass0Callback, this);
        movable_mass_0_state_received_ = false;
        // position of mass 1
        movable_mass_1_state_subscriber_ = nh_.subscribe("movable_mass_1_position_controller/state", 1,
                                                         &MPCAttitudeControllerNode::MovingMass1Callback, this);
        movable_mass_1_state_received_ = false;
        // position of mass 2
        movable_mass_2_state_subscriber_ = nh_.subscribe("movable_mass_2_position_controller/state", 1,
                                                         &MPCAttitudeControllerNode::MovingMass2Callback, this);
        movable_mass_2_state_received_ = false;
        // position of mass 3
        movable_mass_3_state_subscriber_ = nh_.subscribe("movable_mass_3_position_controller/state", 1,
                                                         &MPCAttitudeControllerNode::MovingMass3Callback, this);
        movable_mass_3_state_received_ = false;
        // rotors angular velocities
        motor_speed_subscriber_ = nh_.subscribe("motor_speed", 1, &MPCAttitudeControllerNode::MotorSpeedCallback, this);
        motor_speed_received_ = false;
        // rotors angular velocities sent from height controller
        motor_speed_height_subscriber_ = nh_.subscribe("command/height/motor_speed", 1,
                                                       &MPCAttitudeControllerNode::MotorSpeedHeightCallback, this);

        turn_on_off_MPC_sub = nh_.subscribe("turnOnOFFMPC", 1, &MPCAttitudeControllerNode::TurnOnOffCallback, this);
        turn_on_MPC = false;

        pose_with_covariance_subscriber_ = nh_.subscribe("pose_with_covariance", 1, &MPCAttitudeControllerNode::PoseWithCovarianceCallback, this);
        velocity_subscriber_ = nh_.subscribe("velocity", 1, &MPCAttitudeControllerNode::VelocityCallback, this);
        velocity_ref_subscriber_ = nh_.subscribe("vel_ref", 1, &MPCAttitudeControllerNode::VelocityRefCallback, this);
        pose_ref_subscriber_ = nh_.subscribe("pos_ref", 1, &MPCAttitudeControllerNode::PoseRefCallback, this);


        pid_mass_0_subscriber_ = nh_.subscribe("command/mass_0/motor_speed", 1, &MPCAttitudeControllerNode::PIDMass0Callback, this);
        pid_mass_1_subscriber_ = nh_.subscribe("command/mass_1/motor_speed", 1, &MPCAttitudeControllerNode::PIDMass1Callback, this);
        pid_mass_2_subscriber_ = nh_.subscribe("command/mass_2/motor_speed", 1, &MPCAttitudeControllerNode::PIDMass2Callback, this);
        pid_mass_3_subscriber_ = nh_.subscribe("command/mass_3/motor_speed", 1, &MPCAttitudeControllerNode::PIDMass3Callback, this);

        a = 0.1;








       // m_start_flag = False;  // flag indicates if the first measurement is received
       // m_config_start = False;  // flag indicates if the config callback is called for the first time
       // m_euler_mv = Vector3D();  // measured euler angles
       // m_euler_sp = Vector3D(0, 0, 0);  // euler angles referent values

        m_w_sp = 0;  // referent value for motor velocity - it should be the output of height controller

        //m_euler_rate_mv = Vector3D();  // measured angular velocities


        m_pid_roll = PID();  // roll controller
        m_pid_roll_rate = PID();  // roll rate (wx) controller

        m_pid_pitch = PID();  // pitch controller
        m_pid_pitch_rate = PID();  // pitch rate (wy) controller

        m_pid_yaw = PID();  // yaw controller
        m_pid_yaw_rate = PID();  // yaw rate (wz) controller

       

        m_pid_roll.set_kp(3.0);
        m_pid_roll.set_ki(1.0);
        m_pid_roll.set_kd(0);

        m_pid_roll_rate.set_kp(2.5);
        m_pid_roll_rate.set_ki(0.0);
        m_pid_roll_rate.set_kd(0);
        m_pid_roll_rate.set_lim_high(0.3);
        m_pid_roll_rate.set_lim_low(-0.3);

        m_pid_pitch.set_kp(3.0);
        m_pid_pitch.set_ki(1.0);
        m_pid_pitch.set_kd(0);

        m_pid_pitch_rate.set_kp(2.5);
        m_pid_pitch_rate.set_ki(0.0);
        m_pid_pitch_rate.set_kd(0);
        m_pid_pitch_rate.set_lim_high(0.3);
        m_pid_pitch_rate.set_lim_low(-0.3);

        

        
        
        
        
        
        
        
        

        m_start_flag = false;         // indicates if we received the first measurement
        m_config_start = false;       // flag indicates if the config callback is called for the first time

        m_z_sp = 0;                   // z-position set point
        m_z_ref_filt = 0;             // z ref filtered
        m_z_mv = 0 ;                  // z-position measured value
        m_pid_z = PID();              // pid instance for z control

        m_vz_sp = 0 ;                 // vz velocity set_point
        m_vz_mv = 0  ;                 // vz velocity measured value
        m_pid_vz = PID() ;            // pid instance for z-velocity control

        m_pid_yaw_rate = PID();
        //m_euler_mv = Vector3(0, 0, 0);           // measured euler angles
        //m_euler_sp = Vector3(0, 0, 0);    // euler angles referent values
        //m_euler_rate_mv = Vector3(0, 0, 0);      // measured angular velocities
        m_dwz = 0;


// Add parameters for z controller
        m_pid_z.set_kp(1.150); // 0.5
        m_pid_z.set_ki(0.1); // 0.01
        m_pid_z.set_kd(0.00);

// Add parameters for vz controller
        m_pid_vz.set_kp(75.0); // 20, 87.2)
        m_pid_vz.set_ki(1.0); // 0.1
        m_pid_vz.set_kd(0.0);// 10, 10.89)


        m_pid_yaw.set_kp(0.3);
        m_pid_yaw.set_ki(0);
        m_pid_yaw.set_kd(0);
// Yaw rate params
        m_pid_yaw_rate.set_kp(75);
        m_pid_yaw_rate.set_ki(5.0);
        m_pid_yaw_rate.set_kd(15.0);



        m_pid_z.set_lim_high(5) ;     // max vertical ascent speed
        m_pid_z.set_lim_low(-5);      // max vertical descent speed

        m_pid_vz.set_lim_high(350) ;  // max velocity of a motor
        m_pid_vz.set_lim_low(-350) ;  // min velocity of a motor

        m_mot_speed = 0 ;             // referent motors velocity, computed by PID cascade

        m_gm_attitude_ctl = 0 ;       // flag indicates if attitude control is turned on
       /* if(nh_.hasParam("~gm_attitude_ctl")){
            nh_.getParam("~gm_attitude_ctl", m_gm_attitude_ctl);
        }else{
            std::cout << "-----------------> nepostoji PARAM ~gm_attitude_ctl" << std::endl << std::endl;
        }*/



        m_t_old = ros::Time::now();
    }

    MPCAttitudeControllerNode::~MPCAttitudeControllerNode() {
        p.stop(false);

    }

/*
    void MPCAttitudeControllerNode::DynConfigCallback(morus_control::MPCAttitudeControllerConfig &config,
                                                      uint32_t level)
    {
        automatic_reference_ = config.aut_ref;

        // integral component init
        linear_mpc_roll_.setIntegratorConstantMPC( config.K_I_MPC);
        linear_mpc_pitch_.setIntegratorConstantMPC(config.K_I_MPC);

        // q_moving_masses setup
        linear_mpc_roll_.setPenaltyMovingMasses( config.q_p0, config.q_v0, config.q_p1, config.q_v1);
        linear_mpc_pitch_.setPenaltyMovingMasses(config.q_p0, config.q_v0, config.q_p1, config.q_v1);

        linear_mpc_roll_.setPenaltyRotors( config.q_rotor_speed_0, config.q_rotor_speed_1);
        linear_mpc_pitch_.setPenaltyRotors(config.q_rotor_speed_0, config.q_rotor_speed_1);

        // q_attitude setup
        linear_mpc_roll_.setPenaltyAttitude( config.q_angle, config.q_angular_velocity);
        linear_mpc_pitch_.setPenaltyAttitude(config.q_angle, config.q_angular_velocity);

        // r_command setup
        linear_mpc_roll_.setPenaltyCommand( config.r_mass_0, config.r_mass_1,config.r_rotor_0, config.r_rotor_1);
        linear_mpc_pitch_.setPenaltyCommand(config.r_mass_0, config.r_mass_1,config.r_rotor_0, config.r_rotor_1);

        // r_delta_command setup
        linear_mpc_roll_.setPenaltyChangeCommand( config.r_delta_mass_0, config.r_delta_mass_1,
                                                  config.r_delta_rotor_0, config.r_delta_rotor_1);
        linear_mpc_pitch_.setPenaltyChangeCommand(config.r_delta_mass_0, config.r_delta_mass_1,
                                                  config.r_delta_rotor_0, config.r_delta_rotor_1);

        // change the adequate matrices
        linear_mpc_roll_.applyParameters();
        linear_mpc_pitch_.applyParameters();
    }
*/
    void MPCAttitudeControllerNode::AhrsCallback(const sensor_msgs::Imu &msg) {
        /// @details AHRS callback. Used to extract roll, pitch, yaw and their rates.
        /// We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        /// @param msg: Type sensor_msgs::Imu

        ROS_INFO_ONCE("MPCAttitudeController got first odometry message.");

        // read the msg
        double qx = msg.orientation.x;
        double qy = msg.orientation.y;
        double qz = msg.orientation.z;
        double qw = msg.orientation.w;

        // conversion quaternion to euler (yaw - pitch - roll)
        euler_mv_.x = atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
        euler_mv_.y = asin(2 * (qw * qy - qx * qz));
        euler_mv_.z = atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);

        // gyro measurements (p,q,r)
        double p = msg.angular_velocity.x;
        double q = msg.angular_velocity.y;
        double r = msg.angular_velocity.z;

        double sx = sin(euler_mv_.x); // sin(roll)
        double cx = cos(euler_mv_.x); // cos(roll)
        double cy = cos(euler_mv_.y); // cos(pitch)
        double ty = tan(euler_mv_.y); // tan(pitch)

        // conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        euler_rate_mv_.x = p + sx * ty * q + cx * ty * r;
        euler_rate_mv_.y = cx * q - sx * r;
        euler_rate_mv_.z = sx / cy * q + cx / cy * r;

        // publish states in roll,pitch,jaw format
        morus_msgs::AngleAndAngularVelocity angles_velocities;
        angles_velocities.roll = (float) euler_mv_.x;
        angles_velocities.pitch = (float) euler_mv_.y;
        angles_velocities.jaw = (float) euler_mv_.z;

        angles_velocities.roll_dot = (float) euler_rate_mv_.x;
        angles_velocities.pitch_dot = (float) euler_rate_mv_.y;
        angles_velocities.jaw_dot = (float) euler_rate_mv_.z;
        angles_velocities.header.stamp = ros::Time::now();

        //std::cout << "********************* " << euler_rate_mv_.x << "  " << euler_rate_mv_.x << std::endl;
        //std::cout << "********************* " << euler_rate_mv_.y << "  " << euler_rate_mv_.y << std::endl;
        pub_angle_state_.publish(angles_velocities);

        if (!start_flag_) {
            start_flag_ = true;
            // first execution, not to have big jump at the beginning
            calculateCommands();
            publishCommands();
        }

        imu_received_ = true;


    }

    void MPCAttitudeControllerNode::PoseWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
        m_z_mv = msg.pose.pose.position.z;
    }

    void MPCAttitudeControllerNode::VelocityCallback(const geometry_msgs::TwistStamped& msg){
        m_vz_mv = msg.twist.linear.z;
    }

    void MPCAttitudeControllerNode::VelocityRefCallback(const geometry_msgs::Vector3& msg){
        m_vz_sp = msg.z;
    }

    void MPCAttitudeControllerNode::PoseRefCallback(const geometry_msgs::Vector3& msg){
        m_z_sp = msg.z;
    }


    void MPCAttitudeControllerNode::PIDMass0Callback(const std_msgs::Float64 &msg) {
        mass_0_pid = msg.data;
    }


    void MPCAttitudeControllerNode::PIDMass1Callback(const std_msgs::Float64 &msg) {

        mass_1_pid = msg.data;
    }

    void MPCAttitudeControllerNode::PIDMass2Callback(const std_msgs::Float64 &msg) {

        mass_2_pid = msg.data;
    }

    void MPCAttitudeControllerNode::PIDMass3Callback(const std_msgs::Float64 &msg) {

        mass_3_pid = msg.data;
    }

    void MPCAttitudeControllerNode::MotVelRefCallback(const std_msgs::Float32 &msg) {
        /// @details Referent motor velocity callback. (This should be published by height controller).
        /// @param msg: Type std_msgs::Float32
        w_sp_ = msg.data;
    }

    void MPCAttitudeControllerNode::EulerRefCallback(const geometry_msgs::Vector3 &msg) {
        /// @details Euler ref values callback.
        /// @param msg: Type geometry_msgs::Vector3 (x-roll, y-pitch, z-yaw)
        euler_sp_ = msg;

        roll_ref = msg.x;
        //pitch_ref = msg.y;

        //linear_mpc_roll_.setAngleRef(roll_ref);
        //linear_mpc_pitch_.setAngleRef(pitch_ref);

        /* if (automatic_reference_){
             setAutomaticReference();
         }*/
    }

    void MPCAttitudeControllerNode::setAutomaticReference() {
        static ros::Time t_previous = ros::Time::now();
        static double angle_sp_pitch = 0.1;

        ros::Time t0 = ros::Time::now();
        double dt = (t0 - t_previous).toSec();

        if (dt > 7.0) {
            angle_sp_pitch *= -1; // change the reference polarity (+, -, +, -, ...)
            t_previous = t0;
        }
        //linear_mpc_pitch_.setAngleRef(angle_sp_pitch);
    }

    void MPCAttitudeControllerNode::ClockCallback(const rosgraph_msgs::Clock &msg) {
        /// @param msg
        clock_read_ = msg;

        roll_clock = msg;
        pitch_clock = msg;

        //linear_mpc_roll_.setClock(roll_clock);
        //linear_mpc_pitch_.setClock(pitch_clock);
    }

    void MPCAttitudeControllerNode::MovingMass0Callback(const control_msgs::JointControllerState &msg) {
        movable_mass_0_position_ = msg.process_value;
        movable_mass_0_speed_ = msg.process_value_dot;

        pitch_moving_mass_state_0 = msg;

        //linear_mpc_pitch_.setMovingMassState(pitch_moving_mass_state_0, 0, +1.0);
        movable_mass_0_state_received_ = true;
    }

    void MPCAttitudeControllerNode::MovingMass1Callback(const control_msgs::JointControllerState &msg) {
        movable_mass_1_position_ = msg.process_value;
        movable_mass_1_speed_ = msg.process_value_dot;

        roll_moving_mass_state_0 = msg;

        //std::cout << "=========== Primio poruku 1"<< movable_mass_3_position_ << std::endl << std::endl;

        //linear_mpc_roll_.setMovingMassState(roll_moving_mass_state_0, 0, -1.0);
        movable_mass_1_state_received_ = true;
    }

    void MPCAttitudeControllerNode::MovingMass2Callback(const control_msgs::JointControllerState &msg) {
        movable_mass_2_position_ = msg.process_value;
        movable_mass_2_speed_ = msg.process_value_dot;

        pitch_moving_mass_state_1 = msg;

        //linear_mpc_pitch_.setMovingMassState(pitch_moving_mass_state_1, 1, -1.0);
        movable_mass_2_state_received_ = true;
    }

    void MPCAttitudeControllerNode::MovingMass3Callback(const control_msgs::JointControllerState &msg) {
        movable_mass_3_position_ = msg.process_value;
        movable_mass_3_speed_ = msg.process_value_dot;

        //std::cout << "=========== Primio poruku 3"<< movable_mass_3_position_ << std::endl << std::endl;

        roll_moving_mass_state_1 = msg;

        //linear_mpc_roll_.setMovingMassState(roll_moving_mass_state_1, 1, +1.0);
        movable_mass_3_state_received_ = true;
    }

    void MPCAttitudeControllerNode::MotorSpeedCallback(const mav_msgs::Actuators &msg) {
        motor_0_speed_ = -msg.angular_velocities.data()[0]; // +
        motor_1_speed_ = msg.angular_velocities.data()[1]; // +
        motor_2_speed_ = -msg.angular_velocities.data()[2]; // +
        motor_3_speed_ = msg.angular_velocities.data()[3]; // +
        // all the speeds are now positive
        // 1. state - rotor which increases regulated angle, 2. state - decreases

        pitch_motor_0 = motor_2_speed_;
        pitch_motor_1 = motor_0_speed_;
        roll_motor_0 = motor_1_speed_;
        roll_motor_1 = motor_3_speed_;


        //linear_mpc_pitch_.setMotorState(pitch_motor_0, pitch_motor_1);
        //linear_mpc_roll_.setMotorState( roll_motor_0, roll_motor_1);


        motor_speed_received_ = true; // acknowledgement of receiving message
    }

    void MPCAttitudeControllerNode::MotorSpeedHeightCallback(const mav_msgs::Actuators &msg) {
        if (turn_on_MPC) {
            pid_on_off = 1.0;
            MPC_on_off = 0.0;
        } else {
            pid_on_off = 0.0;
            MPC_on_off = 1.0;
        }

        // read the command sent from height controller
        // forward the msg from the hight controller and to the UAV
        mav_msgs::Actuators combined_rotor_command_msg;
        combined_rotor_command_msg.header.stamp = ros::Time::now();
        combined_rotor_command_msg.angular_velocities.clear();
        /*combined_rotor_command_msg.angular_velocities.push_back(msg.angular_velocities[0] + 0*pitch_commands_[2]);
        combined_rotor_command_msg.angular_velocities.push_back(msg.angular_velocities[1] + 0*roll_commands_[3]);
        combined_rotor_command_msg.angular_velocities.push_back(msg.angular_velocities[2] + 0*pitch_commands_[3]);
        combined_rotor_command_msg.angular_velocities.push_back(msg.angular_velocities[3] + 0*roll_commands_[2]);*/
       /* combined_rotor_command_msg.angular_velocities.push_back(
                1 * msg.angular_velocities[0] + MPC_on_off * pitch_commands_[2]);
        combined_rotor_command_msg.angular_velocities.push_back(
                1 * msg.angular_velocities[0] + MPC_on_off * roll_commands_[3]);
        combined_rotor_command_msg.angular_velocities.push_back(
                1 * msg.angular_velocities[0] + MPC_on_off * pitch_commands_[3]);
        combined_rotor_command_msg.angular_velocities.push_back(
                1 * msg.angular_velocities[0] + MPC_on_off * roll_commands_[2]);
        pub_rotors_.publish(combined_rotor_command_msg);
        // ROS_INFO_STREAM("++++ !!!!!!! pitch " << pitch_commands_[2] << "  " << pitch_commands_[3]);
        // ROS_INFO_STREAM("++++ !!!!!!! roll " << roll_commands_[2] << "  " << roll_commands_[3]);*/
    }

    bool MPCAttitudeControllerNode::calculateControlCommand(Eigen::Matrix<double, kInputSize, 1> &control_commands,
                                                            MPCAttitudeController &linear_mpc_commanded_angle) {
        Eigen::Matrix<double, kInputSize, 1> calculated_control_commands;
        linear_mpc_commanded_angle.calculateControlCommand(calculated_control_commands);
        control_commands = calculated_control_commands;
        return true;
    }

    bool MPCAttitudeControllerNode::calculateCommands() {

       /* // set the data to the controllers - angles and angular velocities

        roll_angle_state = euler_mv_.x;
        roll_angular_vel_state = euler_rate_mv_.x;

        //linear_mpc_roll_.setAngleState(roll_angle_state);
        //linear_mpc_roll_.setAngularVelocityState(roll_angular_vel_state);

        pitch_angle_state = euler_mv_.y;
        pitch_angular_vel_state = euler_rate_mv_.y;

        //linear_mpc_pitch_.setAngleState(pitch_angle_state);
        //linear_mpc_pitch_.setAngularVelocityState(pitch_angular_vel_state);

        // calculate the control signals - MAIN ALGORITHM !!!!!
        calculateControlCommand(roll_commands_, linear_mpc_roll_);
        calculateControlCommand(pitch_commands_, linear_mpc_pitch_);*/
    }

    void MPCAttitudeControllerNode::publishCommands() {
        //assert(pitch_commands_.data());
        //assert(roll_commands_.data());


    /*

        if (turn_on_MPC) {
            pid_on_off = 1.0;
            MPC_on_off = 0.0;
        } else {
            pid_on_off = 0.0;
            MPC_on_off = 1.0;
        }

        std_msgs::Float64 mass0_command_msg, mass1_command_msg, mass2_command_msg, mass3_command_msg;
        mass0_command_msg.data = pitch_commands_[0] * MPC_on_off + pid_on_off * mass_0_pid;
        mass1_command_msg.data = -roll_commands_[0] * MPC_on_off + pid_on_off * mass_1_pid;
        mass2_command_msg.data = -pitch_commands_[1] * MPC_on_off + pid_on_off * mass_2_pid;
        mass3_command_msg.data = roll_commands_[1] * MPC_on_off + pid_on_off * mass_3_pid;

        // publish the new references for the masses
        pub_mass0_.publish(mass0_command_msg);
        pub_mass1_.publish(mass1_command_msg);
        pub_mass2_.publish(mass2_command_msg);
        pub_mass3_.publish(mass3_command_msg);

        mav_msgs::Actuators rotor_attitude_command_msg;
        rotor_attitude_command_msg.header.stamp = ros::Time::now();
        rotor_attitude_command_msg.angular_velocities.clear();
        rotor_attitude_command_msg.angular_velocities.push_back(+pitch_commands_[2]);
        rotor_attitude_command_msg.angular_velocities.push_back(+roll_commands_[3]);
        rotor_attitude_command_msg.angular_velocities.push_back(+pitch_commands_[3]);
        rotor_attitude_command_msg.angular_velocities.push_back(+roll_commands_[2]);
        pub_rotors_attitude_.publish(rotor_attitude_command_msg);*/
    }


    void MPCAttitudeControllerNode::TurnOnOffCallback(const std_msgs::Bool &msg) {
        turn_on_MPC = msg.data;
    }





    void MPCAttitudeControllerNode::run() {
        /*std::ofstream myfile_gazebo_states_roll, myfile_inputs_roll, myfile_gazebo_states_pitch, myfile_inputs_pitch;
        myfile_gazebo_states_roll.open("/home/osboxes/roll_state.csv");
        myfile_inputs_roll.open("/home/osboxes/roll_input.csv");
        myfile_gazebo_states_pitch.open("/home/osboxes/pitch_state.csv");
        myfile_inputs_pitch.open("/home/osboxes/pitch_input.csv");*/

        // define sampling time
        // needs to be set in the controller as well, change to be unique !!! TODO
        //double sampling_time = 0.04;
        ros::Rate loop_rate(1.0 / kSamplingTime); // 25 Hz -> Ts = 0.04 s
        std::cout << "Sampling time " << kSamplingTime  << std::endl;

        auto t1 = Clock::now();
        auto t2 = Clock::now();
        auto t3 = Clock::now();
        auto t4 = Clock::now();


        roll_ref = 0.0;
        pitch_ref = 0.0;
        roll_moving_mass_state_0.process_value = 0.0;
        roll_moving_mass_state_1.process_value = 0.0;
        pitch_moving_mass_state_0.process_value = 0.0;
        pitch_moving_mass_state_1.process_value = 0.0;
        roll_moving_mass_state_0.process_value_dot = 0.0;
        roll_moving_mass_state_1.process_value_dot = 0.0;
        pitch_moving_mass_state_0.process_value_dot = 0.0;
        pitch_moving_mass_state_1.process_value_dot = 0.0;
        roll_motor_0 = 0.0;
        roll_motor_1 = 0.0;
        pitch_motor_0 = 0.0;
        pitch_motor_1 = 0.0;
        euler_mv_.x = 0.0;
        euler_mv_.y = 0.0;
        euler_mv_.z = 0.0;
        euler_rate_mv_.x = 0.0;
        euler_rate_mv_.y = 0.0;
        euler_rate_mv_.z = 0.0;
        double motor_diff;

        auto sampling_time_long = static_cast<long>(kSamplingTime * 1E3);
        double dy_roll ,dx_pitch;

        MPCAttitudeController linear_mpc_roll_(nh_, private_nh_);
        MPCAttitudeController linear_mpc_pitch_(nh_, private_nh_);


        Eigen::Matrix<double, kInputSize, 1> roll_commands_;
        Eigen::Matrix<double, kInputSize, 1> pitch_commands_;
        roll_commands_.setZero();
        pitch_commands_.setZero();

        linear_mpc_roll_.applyParameters();
        linear_mpc_roll_.setControllerName("Roll controller");

        linear_mpc_pitch_.applyParameters();
        linear_mpc_pitch_.setControllerName("Pitch controller");

        pitch_commands_.setZero();
        roll_commands_.setZero();

        std::ifstream file("/home/osboxes/roll.csv", std::ifstream::in);
        std::string ID, nome, idade, genero;
        std::string::size_type sz;

        int counter_YAW = 3;
        int counter_pitch_ref = 0;

        while (ros::ok()) {
            t1 = Clock::now();

            if (turn_on_MPC) {
               // std::cout << "************************************ MPC" << std::endl << std::endl;
                pid_on_off = 0.0;
                MPC_on_off = 1.0;

            } else {


                //std::cout << "==================================== PID" << std::endl << std::endl;
                pid_on_off = 1.0;
                MPC_on_off = 0.0;
            }



            m_z_ref_filt = (1-a) * m_z_ref_filt  + a * m_z_sp;
            double vz_ref = m_pid_z.compute(m_z_ref_filt, m_z_mv, kSamplingTime);
            m_mot_speed = m_mot_speed_hover + m_pid_vz.compute(vz_ref, m_vz_mv, kSamplingTime);
            if(counter_YAW == 0){
                double yaw_rate_sv = m_pid_yaw.compute(euler_sp_.z, euler_mv_.z, kSamplingTime);

                m_dwz = m_pid_yaw_rate.compute(yaw_rate_sv, euler_rate_mv_.z, kSamplingTime);
                counter_YAW = 4;
            }

            counter_YAW--;

            /*std::cout << "==================== MASE1 su " <<  roll_moving_mass_state_0.process_value <<std::endl << std::endl;
            std::cout << "==================== MASE2 su " <<  roll_moving_mass_state_1.process_value <<std::endl << std::endl;
            std::cout << "==================== MASE3 su " <<  pitch_moving_mass_state_0.process_value <<std::endl << std::endl;
            std::cout << "==================== MASE4 su " <<  pitch_moving_mass_state_1.process_value <<std::endl << std::endl;*/
            if (turn_on_MPC) {
//                double adagrad = 1.38;
                if(counter_pitch_ref < 1000)
                {
                    pitch_ref = 0.0;
                }else if(counter_pitch_ref < 1088)
                {
                    pitch_ref = 0.087266*1.05;
                }else if(counter_pitch_ref < 1129) {
                    pitch_ref = 0.00;
                }else if (counter_pitch_ref < 1217) {
                    pitch_ref = -0.087266*1.05;
                }else{
                    pitch_ref = 0.0;
                }
                counter_pitch_ref ++;

                nova_referenca.y = pitch_ref/1.05;

                //std::cout << "Pitch counter " << counter_pitch_ref << std::endl;

                /*
                std::cout << "########### Delta t2-t1: "
                          << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count()
                          << " nanoseconds" << std::endl;
                std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " milliseconds" << std::endl;*/


                euler_ref_pub.publish(nova_referenca);

                //ros::spinOnce();

                //std::cout << "Motori " << roll_motor_1 << "  "<<roll_motor_0 << "  "<<pitch_motor_0 << "  "<<pitch_motor_1 << "  " <<std::endl << std::endl;

                linear_mpc_roll_.setAngleRef(roll_ref);
                linear_mpc_roll_.setClock(roll_clock);
                linear_mpc_roll_.setMovingMassState(roll_moving_mass_state_0, 1, -1.0);
                linear_mpc_roll_.setMovingMassState(roll_moving_mass_state_1, 0, +1.0);
                linear_mpc_roll_.setMotorState(roll_motor_0, roll_motor_1);
                linear_mpc_roll_.setAngleState(euler_mv_.x);
                linear_mpc_roll_.setAngularVelocityState(euler_rate_mv_.x);
                motor_diff = (roll_motor_0 - roll_motor_1) / 2.0;

                /*myfile_gazebo_states_roll << roll_moving_mass_state_0.process_value << ","
                                          << roll_moving_mass_state_0.process_value_dot << ","
                                          << roll_moving_mass_state_1.process_value << ","
                                          << roll_moving_mass_state_1.process_value_dot << ","
                                          << motor_diff << "," << (-motor_diff) << ","
                                          << euler_mv_.x << "," <<  euler_rate_mv_.x;


                myfile_gazebo_states_roll << "\n";*/

                linear_mpc_pitch_.setAngleRef(pitch_ref);
                linear_mpc_pitch_.setClock(pitch_clock);
                linear_mpc_pitch_.setMovingMassState(pitch_moving_mass_state_0, 0, +1.0);
                linear_mpc_pitch_.setMovingMassState(pitch_moving_mass_state_1, 1, -1.0);
                linear_mpc_pitch_.setMotorState(pitch_motor_1, pitch_motor_0);
                linear_mpc_pitch_.setAngleState(euler_mv_.y);
                linear_mpc_pitch_.setAngularVelocityState(euler_rate_mv_.y);

                motor_diff = (pitch_motor_0 - pitch_motor_1) / 2.0;

                /* myfile_gazebo_states_pitch << pitch_moving_mass_state_0.process_value << ","
                                           << pitch_moving_mass_state_0.process_value_dot << ","
                                           << pitch_moving_mass_state_1.process_value << ","
                                           << pitch_moving_mass_state_1.process_value_dot << ","
                                           << motor_diff << "," << (-motor_diff) << ","
                                           << euler_mv_.y << "," <<  euler_rate_mv_.y;


                 myfile_gazebo_states_pitch << "\n";*/




                results[0] = p.push([&linear_mpc_roll_, &roll_commands_](int id) {

                    linear_mpc_roll_.calculateControlCommand(roll_commands_);

                });

                results[1] = p.push([&linear_mpc_pitch_, &pitch_commands_](int id) {

                    linear_mpc_pitch_.calculateControlCommand(pitch_commands_);

                });

                //results[0]= p.push(wrapMyTaskRun, *this);

                for (int j = 0; j < 2; ++j) {
                    results[j].get();
                }

















                /*linear_mpc_roll_.calculateControlCommand(roll_commands_);
                linear_mpc_pitch_.calculateControlCommand(pitch_commands_);

                std::cout << "ROLL commands " << std::endl <<   roll_commands_(0,0) << "\n"
                                                                << roll_commands_(1,0) << "\n"
                                                                << roll_commands_(2,0) << "\n"
                                                                << roll_commands_(3,0) << "\n";


                std::cout << "PITCH commands " << std::endl <<    pitch_commands_(0,0) << "\n"
                          << pitch_commands_(1,0) << "\n"
                          << pitch_commands_(2,0) << "\n"
                          << pitch_commands_(3,0) << "\n";*/

                // t2 = Clock::now();
                /* myfile_inputs_roll << roll_commands_(0,0) << ","
                                    << roll_commands_(1,0) << ","
                                    << roll_commands_(2,0) << ","
                                    << roll_commands_(3,0) << "\n";

                 myfile_inputs_pitch << pitch_commands_(0,0) << ","
                                    << pitch_commands_(1,0) << ","
                                    << pitch_commands_(2,0) << ","
                                    << pitch_commands_(3,0) << "\n";*/



                // loop_rate.sleep();


                /*     // if all the measurements are received
                     if (imu_received_ &&
                         movable_mass_0_state_received_ && movable_mass_1_state_received_ &&
                         movable_mass_2_state_received_ && movable_mass_3_state_received_ &&
                         motor_speed_received_) {

                         calculateCommands(); // calculate the output
                         publishCommands(); // send the received commands to output

                         // reset the flags for massages
                         imu_received_                  = false;
                         movable_mass_0_state_received_ = false;
                         movable_mass_1_state_received_ = false;
                         movable_mass_2_state_received_ = false;
                         movable_mass_3_state_received_ = false;
                         motor_speed_received_          = false;
                     }*/

                // go to another anotation and keep the sampling time
               /* auto razlika =
                        sampling_time_long - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
                // std::cout << "Razlika je " << razlika << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(razlika));*/

            } else {


                

                double roll_rate_sv = m_pid_roll.compute(euler_sp_.x, euler_mv_.x, kSamplingTime);
                // roll rate pid compute
                dy_roll = m_pid_roll_rate.compute(roll_rate_sv, euler_rate_mv_.x, kSamplingTime);

                double pitch_rate_sv = m_pid_pitch.compute(euler_sp_.y, euler_mv_.y, kSamplingTime);
                // pitch rate pid compute
                dx_pitch = m_pid_pitch_rate.compute(pitch_rate_sv, euler_rate_mv_.y, kSamplingTime);


               // std::cout << std::endl << "==========Signali " << std::endl << m_mot_speed << m_dwz <<dy_roll << dx_pitch <<std::endl << std::endl;

                // read the command sent from height controller
                // forward the msg from the hight controller and to the UAV

            }


            mav_msgs::Actuators combined_rotor_command_msg;
            combined_rotor_command_msg.header.stamp = ros::Time::now();
            combined_rotor_command_msg.angular_velocities.clear();





            combined_rotor_command_msg.angular_velocities.push_back(m_mot_speed + m_dwz  + MPC_on_off * pitch_commands_(3,0));
            combined_rotor_command_msg.angular_velocities.push_back(m_mot_speed - m_dwz  - MPC_on_off * roll_commands_(2,0));
            combined_rotor_command_msg.angular_velocities.push_back(m_mot_speed + m_dwz + MPC_on_off * pitch_commands_(2,0));
            combined_rotor_command_msg.angular_velocities.push_back(m_mot_speed - m_dwz  - MPC_on_off * roll_commands_(3,0)) ;
            pub_rotors_.publish(combined_rotor_command_msg);

           // std::cout << "Provjera * " <<( roll_commands_(2,0)) << "   " <<m_mot_speed << std::endl;
           // std::cout << "Provjera = " <<(roll_commands_(3,0)) <<"   " <<m_mot_speed<< std::endl;

            std_msgs::Float64 mass0_command_msg, mass1_command_msg, mass2_command_msg, mass3_command_msg;
            mass0_command_msg.data = dx_pitch * pid_on_off + MPC_on_off * pitch_commands_(0,0);
            mass1_command_msg.data = -dy_roll * pid_on_off + MPC_on_off * (-roll_commands_(1,0));
            mass2_command_msg.data = -dx_pitch * pid_on_off + MPC_on_off * (-pitch_commands_(1,0));
            mass3_command_msg.data = dy_roll * pid_on_off + MPC_on_off * (roll_commands_(0,0));

            // publish the new references for the masses
            pub_mass0_.publish(mass0_command_msg);
            pub_mass1_.publish(mass1_command_msg);
            pub_mass2_.publish(mass2_command_msg);
            pub_mass3_.publish(mass3_command_msg);
            ros::spinOnce();
            t2 = Clock::now();
            auto razlika =
                    sampling_time_long - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            //std::cout << "Razlika je " << razlika << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(razlika));

            //publishCommands();
            //t3 = Clock::now();

            /*std::cout << "########### Delta t3-t1: " << std::endl;

            std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t1).count()
                      << " milliseconds" << std::endl;*/

            /*myfile_gazebo_states_roll.close();
            myfile_inputs_roll.close();
            myfile_gazebo_states_pitch.close();
            myfile_inputs_pitch.close();*/

        }

    }
}
    int main(int argc, char **argv) {

        ros::init(argc, argv, "attitude_mpc_ctl");

        // fully initialize the node
        ros::NodeHandle nh, private_nh("~");

        mav_control_attitude::MPCAttitudeControllerNode MPC_attitude_controller_node(nh, private_nh);

        // run the regulation
        MPC_attitude_controller_node.run();

        return 0;
    }
