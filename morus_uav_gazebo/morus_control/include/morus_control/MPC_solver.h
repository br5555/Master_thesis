#ifndef INTERFACE_MPC_SOLVER
#define INTERFACE_MPC_SOLVER

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>


constexpr int num_state_variables = 8;
constexpr int num_manipulated_variables = 4;
constexpr int num_heuristic_variables = 2; //using simetric u1 = u2 and u3 = -u4 
constexpr int mpc_control_horizon = 5;
constexpr int prediction_horizon = 14;
constexpr int num_cost_functions = 1;

class MPC_SOLVER
{

public:

MPC_SOLVER()=default;
virtual ~MPC_SOLVER()=default;
virtual Eigen::Matrix<double, num_heuristic_variables * mpc_control_horizon, 1>&  Evaluate(Eigen::Matrix<double, num_heuristic_variables * mpc_control_horizon, 1>&   x)=0;


virtual void set_u_past(Eigen::Matrix<double, num_manipulated_variables, 1> u_past_) =0;
virtual 	void set_u_current(Eigen::Matrix<double, num_manipulated_variables, 1> u_current_) =0;
virtual 	void set_u_ss(Eigen::Matrix<double, num_manipulated_variables, 1> u_ss) =0;
virtual 	void set_x_ss(Eigen::Matrix<double, num_state_variables, 1>  x_ss)=0;
virtual 	void set_x0_(Eigen::Matrix<double, num_state_variables, 1>  x0) =0;
virtual 	void set_A(Eigen::Matrix<double, num_state_variables, num_state_variables> A) =0;
virtual 	void set_B(Eigen::Matrix<double, num_state_variables, num_manipulated_variables> B) =0;
virtual 	void set_Bd(Eigen::Matrix<double, num_state_variables, num_state_variables> Bd) =0;
virtual 	void set_Q(Eigen::Matrix<double, num_state_variables, num_state_variables>  Q) =0;
virtual 	void set_Q_final(Eigen::Matrix<double, num_state_variables, num_state_variables>  Q_final)=0;
virtual 	void set_R(Eigen::Matrix<double, num_manipulated_variables, num_manipulated_variables>  R) =0;
virtual 	void set_R_delta(Eigen::Matrix<double, num_manipulated_variables, num_manipulated_variables>  R_delta) =0;
virtual 	void set_insecure(Eigen::Matrix<double, num_state_variables, 1> insecure) =0;
virtual 	void set_disturbance(Eigen::Matrix<double, num_state_variables, 1> disturbance) =0;
virtual 	void set_num_params(int num_params) =0;
virtual 	double get_residuum() =0;
virtual 	double get_residuum_signal() =0;
virtual 	double get_residuum_state() =0;
virtual 	Eigen::MatrixXd get_x_states()=0;

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


#endif
