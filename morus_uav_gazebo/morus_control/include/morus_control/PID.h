#pragma once


#include <morus_msgs/PIDController.h>
#include <ros/ros.h>
#include <vector>


class PID{

public:
    PID();
    //~PID();
    void reset();

    inline void set_kp(double kp){
        m_kp= kp;
    }


    inline double get_kp(){
        return m_kp;
    }

    inline void set_ki(double ki){
        m_ki = ki;
    }


    inline double get_ki(){
        return m_ki;
    }


    inline void set_kd(double kd){
        m_kd = kd;
    }


    inline double get_kd(){
        return m_kd;
    }


    inline void set_lim_high(double lim_high){
        m_lim_high = lim_high;
    }


    inline double get_lim_high(){
        return m_lim_high;
    }

    inline void set_lim_low(double lim_low){
        m_lim_low = lim_low;
    }


    inline double get_lim_low(){
        return m_lim_low;
    }

    double compute(double ref,double meas,double dt);

    std::vector<double> get_pid_values();
    morus_msgs::PIDController create_msg();




private:
    double m_kp;     // proportional gain
    double m_ki;     // integral gain
    double m_kd;     // derivative gain

// initialize control values
    double m_up ;                     // P part
    double m_ui ;                    // I part
    double m_ui_old ;                 // I part from previous step
    double m_ud ;                     // D part
    double m_u ;                     // total control value
    double m_lim_high ;     // control value upper limit
    double m_lim_low ;    //control value lower limit

// init referent control value (set-value)
    double m_ref ;

//init measure control value
    double m_meas ;

// init time stamp of the previous algorithm step
    ros::Time m_t_old;

    double m_dt ;

// init error from the previous algorithm step
    double m_error_old ;

// flag indicates first step of the algorithm
    bool m_firstPass;

// ros message formed when create_msg method is called
    morus_msgs::PIDController m_pid_msg;



};


