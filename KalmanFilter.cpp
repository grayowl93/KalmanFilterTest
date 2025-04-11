#include "KalmanFilter.h"

/**
 * @fn InitializeParameters		
 * @brief kalmanfilter用パラメータ設定
 * @param unsigned int n_dim_state					：状態の次元数		
 * @param unsigned int n_dim_obs					：観測値の次元数		
 * @param Eigen::MatrixXd *transition_matrices		：F：推移行列（k×k次元）		
 * @param Eigen::MatrixXd *transition_covariance	：Q：システムノイズvの分散共分散行列（m×m次元）	
 * @param Eigen::MatrixXd *observation_matrices		：H：観測行列（l×k次元）
 * @param Eigen::MatrixXd *observation_covariance	：R：観測ノイズwの分散共分散行列
 * @return bool
 */
bool KalmanFilter::InitializeParameters(Eigen::MatrixXd *transition_matrices, Eigen::MatrixXd *transition_covariance, Eigen::MatrixXd *observation_matrices, Eigen::MatrixXd *observation_covariance, unsigned int n_dim_state, unsigned int n_dim_obs)
{
	bool ret = true;
	
	if(n_dim_state == 0){ret = false;}
	if(n_dim_obs == 0)	{ret = false;}
	this->transition_matrices   = Eigen::MatrixXd::Ones(1,1);
	this->transition_covariance = Eigen::MatrixXd::Ones(1,1);
	this->observation_matrices  = Eigen::MatrixXd::Ones(1,1);
	this->observation_covariance= Eigen::MatrixXd::Ones(1,1);
	if(transition_matrices		!= nullptr ){this->transition_matrices	 = *transition_matrices;	}
	if(transition_covariance	!= nullptr ){this->transition_covariance = *transition_covariance;	}
	if(observation_matrices		!= nullptr ){this->observation_matrices	 = *observation_matrices;	}
	if(observation_covariance	!= nullptr ){this->observation_covariance= *observation_covariance;	}
	if((this->transition_matrices.cols()	!= n_dim_state) || (this->transition_matrices.rows()	!= n_dim_state)){this->transition_matrices	 = Eigen::MatrixXd::Zero(0, 0);ret = false;}
	if((this->transition_covariance.cols()	!= n_dim_state) || (this->transition_covariance.rows()	!= n_dim_state)){this->transition_covariance = Eigen::MatrixXd::Zero(0, 0);ret = false;}
	if((this->observation_matrices.cols()	!= n_dim_state)	|| (this->observation_matrices.rows() 	!= n_dim_obs))	{this->observation_matrices	 = Eigen::MatrixXd::Zero(0, 0);ret = false;}
	if((this->observation_covariance.cols() != n_dim_obs)	|| (this->observation_covariance.rows() != n_dim_obs))	{this->observation_covariance= Eigen::MatrixXd::Zero(0, 0);ret = false;}
	
	return ret;	
}

/**
 * @fn SimpleSetup		
 * @brief 簡易パラメータ設定（観測値・状態共に1次元）
 * @param double Q		：Q：システムノイズvの分散共分散行列（m×m次元）	
 * @param double R		：R：観測ノイズwの分散共分散行列
 * @return bool
 */
bool KalmanFilter::SimpleSetup(double Q = 1.0, double R = 1.0 )
{
	bool ret = true;
	this->transition_matrices	 = Eigen::MatrixXd::Zero(1, 1);
	this->transition_covariance	 = Eigen::MatrixXd::Zero(1, 1);
	this->observation_matrices	 = Eigen::MatrixXd::Zero(1, 1);
	this->observation_covariance = Eigen::MatrixXd::Zero(1, 1);
	this->transition_matrices	<< 1.0;
	this->transition_covariance	<< Q;
	this->observation_matrices	<< 1.0;
	this->observation_covariance<< R;
	return ret;
}

/**
 * @fn SimpleSetup		
 * @brief 簡易パラメータ設定（観測値・状態が1～3次元まで）
 * @param unsigned int n_dim_state	：状態の次元数		
 * @param double cycle				：フィルタ更新周期		
 * @param Eigen::MatrixXd Q			：Q：システムノイズvの分散共分散行列（m×m次元）	
 * @param double R					：R：観測ノイズwの分散共分散行列
 * @return bool
 */
bool KalmanFilter::SimpleSetup(unsigned int n_dim_state, double cycle, Eigen::MatrixXd Q, double R)
{
	bool ret = false;

	if( (Q.cols() == n_dim_state) && (Q.rows() == n_dim_state) && (n_dim_state != 0) && (n_dim_state <= 3))
	{
		this->transition_matrices	 = Eigen::MatrixXd::Zero(n_dim_state, n_dim_state);
		this->transition_covariance	 = Eigen::MatrixXd::Zero(n_dim_state, n_dim_state);
		this->observation_matrices	 = Eigen::MatrixXd::Zero(1, 1);
		this->observation_covariance = Eigen::MatrixXd::Zero(1, 1);
		this->transition_covariance	 = Q;
		this->observation_matrices	<< 1.0;
		this->observation_covariance<< R;
		if(n_dim_state == 1)
		{
			this->transition_matrices << 1.0;
		}
		else if(n_dim_state == 2)
		{
			this->transition_matrices << 1.0, cycle, 0.0, 1.0;
		}
		else if(n_dim_state == 3)
		{
			this->transition_matrices << 1.0, cycle, cycle*cycle, 0.0, 1.0, cycle, 0.0, 0.0, 1.0;
		}
		ret = true;
	}

	return ret;
}

void KalmanFilter::_filter_predict(Eigen::VectorXd &predicted_state_mean, Eigen::MatrixXd &predicted_state_covariance, const Eigen::MatrixXd &transition_matrix, const Eigen::MatrixXd &transition_covariance, const Eigen::VectorXd &current_state_mean, const Eigen::MatrixXd &current_state_covariance)
{
	predicted_state_mean		= transition_matrix * current_state_mean;
	predicted_state_covariance	= ( transition_matrix * (current_state_covariance * transition_matrix.transpose()) + transition_covariance);
}

void KalmanFilter::_filter_correct(Eigen::MatrixXd &kalman_gain, Eigen::VectorXd &corrected_state_mean, Eigen::MatrixXd &corrected_state_covariance, const Eigen::MatrixXd &observation_matrix, const Eigen::VectorXd &predicted_state_mean, const Eigen::MatrixXd &predicted_state_covariance, const Eigen::VectorXd &observation)
{
	Eigen::VectorXd predicted_observation_mean = observation_matrix * predicted_state_mean;
	Eigen::MatrixXd predicted_observation_covariance = observation_matrix * (predicted_state_covariance * observation_matrix.transpose()) + this->observation_covariance;
	kalman_gain = predicted_state_covariance * (observation_matrix.transpose() * predicted_observation_covariance.inverse());
	corrected_state_mean = predicted_state_mean + (kalman_gain * (observation - predicted_observation_mean));
	corrected_state_covariance = predicted_state_covariance  - (kalman_gain * (observation_matrix * predicted_state_covariance));
}

/**
 * @fn FilterUpdate		
 * @brief フィルタ更新
 * @param const Eigen::VectorXd &filtered_state_mean		：現在の状態		
 * @param const Eigen::MatrixXd &filtered_state_covariance	：現在の分散共分散行列		
 * @param const Eigen::VectorXd &observation				：観測値	
 * @return Eigen::VectorXd &corrected_state_mean			：１世代進化した状態
 * @return Eigen::MatrixXd &corrected_state_covariance		：１世代進化した分散共分散行列
 * @return bool
 */
bool KalmanFilter::FilterUpdate(Eigen::VectorXd &corrected_state_mean, Eigen::MatrixXd &corrected_state_covariance, const Eigen::VectorXd &filtered_state_mean, const Eigen::MatrixXd &filtered_state_covariance, const Eigen::VectorXd &observation)
{
	bool ret = true;
	if(this->observation_matrices.rows() != observation.rows()					){ret = false;}
	if(this->transition_matrices.size()  != filtered_state_covariance.size()	){ret = false;}
	if(this->transition_matrices.rows()  != filtered_state_mean.rows()			){ret = false;}
	if(this->transition_matrices.size()  != corrected_state_covariance.size()	){ret = false;}
	if(this->transition_matrices.rows()  != corrected_state_mean.size()			){ret = false;}

	if(ret)
	{
		Eigen::VectorXd predicted_state_mean;
		Eigen::MatrixXd predicted_state_covariance;
		_filter_predict(predicted_state_mean, predicted_state_covariance, this->transition_matrices, this->transition_covariance, filtered_state_mean, filtered_state_covariance);
		Eigen::MatrixXd kalman_gain;
		_filter_correct(kalman_gain, corrected_state_mean, corrected_state_covariance, this->observation_matrices, predicted_state_mean, predicted_state_covariance, observation);
	}
	return ret;
}