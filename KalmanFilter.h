#ifndef KalmanFilter_H
#define KalmanFilter_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>

class KalmanFilter
{
public:
	KalmanFilter(){};
	virtual ~KalmanFilter(){};
	bool InitializeParameters(Eigen::MatrixXd *transition_matrices, Eigen::MatrixXd *transition_covariance, Eigen::MatrixXd *observation_matrices, Eigen::MatrixXd *observation_covariance, unsigned int n_dim_state, unsigned int n_dim_obs);
	bool FilterUpdate(Eigen::VectorXd &corrected_state_mean, Eigen::MatrixXd &corrected_state_covariance, const Eigen::VectorXd &filtered_state_mean, const Eigen::MatrixXd &filtered_state_covariance, const Eigen::VectorXd &observation);
	bool SimpleSetup(double Q, double R);
	bool SimpleSetup(unsigned int n_dim_state, double cycle, Eigen::MatrixXd Q, double R);

private:
	Eigen::MatrixXd transition_matrices		 = Eigen::MatrixXd::Zero(0, 0);
	Eigen::MatrixXd transition_covariance	 = Eigen::MatrixXd::Zero(0, 0);
	Eigen::MatrixXd observation_matrices	 = Eigen::MatrixXd::Zero(0, 0);
	Eigen::MatrixXd observation_covariance	 = Eigen::MatrixXd::Zero(0, 0);
	void _filter_predict(Eigen::VectorXd &predicted_state_mean, Eigen::MatrixXd &predicted_state_covariance, const Eigen::MatrixXd &transition_matrix, const Eigen::MatrixXd &transition_covariance, const Eigen::VectorXd &current_state_mean, const Eigen::MatrixXd &current_state_covariance);
	void _filter_correct(Eigen::MatrixXd &kalman_gain, Eigen::VectorXd &corrected_state_mean, Eigen::MatrixXd &corrected_state_covariance, const Eigen::MatrixXd &observation_matrix, const Eigen::VectorXd &predicted_state_mean, const Eigen::MatrixXd &predicted_state_covariance, const Eigen::VectorXd &observation);
};

#endif