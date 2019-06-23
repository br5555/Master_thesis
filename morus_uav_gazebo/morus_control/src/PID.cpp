#include <morus_control/PID.h>


PID::PID()
:m_kp(0.0), m_ki(0.0), m_kd(0.0), m_up(0.0), m_ui(0.0), m_ui_old(0.0),
m_ud(0.0), m_u(0.0), m_lim_high(1E8), m_lim_low(-1E8), m_ref(0.0),
m_meas(0.0),

m_t_old (ros::Time::now()), m_dt(0.0), m_error_old(0.0), m_firstPass(true),
m_pid_msg(morus_msgs::PIDController()) {
}

void PID::reset() {
    m_up = 0;
    m_ui = 0;
    m_ui_old = 0;
    m_ud = 0;
    m_u = 0;
    m_t_old = ros::Time::now();
}


double PID::compute(double ref, double meas, double dt) {
    m_ref = ref;
    m_meas = meas;

    if (m_firstPass) {
        m_error_old = m_ref - m_meas;
        m_firstPass = false;
        return m_u;
    } else {
        m_ref = ref;
        m_meas = meas;
        double error = ref - meas;

        double de = error - m_error_old;                         // diff error
        m_up = m_kp * error;                          // proportional term

        if (abs(m_ki) < 1e-8) {
            m_ui = 0;
        } else {
            m_ui = m_ui_old + m_ki * error * dt;
        }


        // integral term

        m_ud = m_kd * de / dt;                        // derivative term

        m_u = m_up + m_ui + m_ud;

        if (m_u > m_lim_high) {
            m_u = m_lim_high;
            m_ui = m_ui_old;  // antiwind up
        } else if (m_u < m_lim_low) {
            m_u = m_lim_low;
            m_ui = m_ui_old; // antiwind up
        }


        m_ui_old = m_ui;                           // save ui for next step
        m_error_old = error;


        return m_u;
    }


}

std::vector<double> PID::get_pid_values() {
    return {m_up, m_ui, m_ud, m_u};
}

morus_msgs::PIDController PID::create_msg() {
    m_pid_msg.ref = static_cast<float>(m_ref);
    m_pid_msg.meas = static_cast<float>(m_meas);
    m_pid_msg.P = static_cast<float>(m_up);
    m_pid_msg.I = static_cast<float>(m_ui);
    m_pid_msg.D = static_cast<float>(m_ud);
    m_pid_msg.U = static_cast<float>(m_u);
    m_pid_msg.header.stamp = ros::Time::now();
    return m_pid_msg;
}