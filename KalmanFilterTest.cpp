// KalmanFilterTest.cpp : アプリケーションのエントリ ポイントを定義します。
//

#include "KalmanFilterTest.h"
#include <fstream>
#include "KalmanFilter.h"

using namespace std;

int main()
{
	cout << "test Start." << endl;

	/*観測値*/
	std::vector<double> BinX = { 0.000195364,0.000380152,0.000233063,0.000314214,0.000312755,0.000274741,0.000220934,0.000120671,7.47E-05,5.40E-05,3.38E-05,3.76E-05,3.96E-05,-8.28E-05,-6.35E-05,-0.000132314,-0.000132314,-0.000139843,-0.000182932,-0.000211496,-0.000291974,-0.00023246,-0.000206397,-0.000352202,-0.000352757,-0.000349503,-0.000406733,-0.000367716,-0.000333695,-0.000344938,-0.000351308,-0.000459959,-0.000473329,-0.00058812,-0.000454408,-0.000511866,-0.000519845,-0.000550978,-0.000449941,-0.000449557,-0.000446741,-0.000593687,-0.000551388,-0.000554489,-0.000491248,-0.000439901,-0.000488231,-0.000368759,-0.000413185,-0.000500371,-0.000405095,-0.00050699,-0.000390195,-0.000494622,-0.000422059,-0.000469255,-0.000411177,-0.000487506,-0.000438427,-0.000498092,-0.000514028,-0.000428312,-0.000301973,-0.000301973,-0.000259925,-0.000292525,-0.000273108,-0.000285569,-0.000469857,-0.000367433,-0.000317571,-0.000409182,-0.000256064,-0.000285535,-0.000349635,-0.000396377,-0.000238284,-0.000329342,-0.000259631,-0.000424931,-0.000310203,-0.000306846,-0.000246082,-0.000299528,-0.000401512,-0.000265947,-0.000231835,-0.000418941,-0.000451342,-0.0003523,-0.000253478,-0.000474633,-0.000247198,-0.000228378,-0.000398542,-0.000248546,-0.000313077,-0.000213159,-0.000434366,-0.000291813,-0.000206218,-0.00017777,-0.000220003,-0.000203375,-0.000185779,-0.000180306,-0.000210789,-0.000198781,-0.000411088,-0.000271242,-0.000192766,-0.000192766,-0.000245446,-0.0001951,-0.00035753,-0.000201314,-0.000209192,-0.000229277,-0.000303716,-0.000251832,-0.000248601,-0.00024478,-0.000414704,-0.000271271,-0.000315723,-0.000218052,-0.000327159,-0.000453959,-0.000246007,-0.000262855,-0.000420683,-0.000315955,-0.000239827,-0.000260031,-0.000234769,-0.000413231,-0.000305526,-0.000318814,-0.000372878,-0.000290084,-0.000251233,-0.000299142,-0.000441537,-0.000380757,-0.000409761,-0.000419029,-0.000410691,-0.000284065,-0.000404196,-0.000286896,-0.000277011,-0.000287081,-0.000285385,-0.000310453,-0.000253875,-0.000346083,-0.000218225,-0.000415392,-0.000222772,-0.000235498,-0.000209345,-0.000389287,-0.000176729,-0.000323199,-0.000189153,-0.000175565,-0.000197796,-0.000332974,-0.000182255,-0.000128286,-0.000171231,-0.000144171,-0.000104135,-0.000150953,-0.000168548,-0.000275939,-0.00016088,-0.000120928,-0.000155168,-0.000239568,-0.000237216,-0.000286641,-0.000201258,-0.000157353,-0.000157274,-0.00010423 };

	/******詳細設定 状態一次  観測値一次********************************************************************************************************/

	/*インスタンス作成*/
	KalmanFilter kf;

	/*観測値 状態次数*/
	unsigned int n_dim_state = 1;
	unsigned int n_dim_obs = 1;

	/*その他いろいろ設定*/
	Eigen::VectorXd G = Eigen::VectorXd::Zero(n_dim_state);
	Eigen::MatrixXd transition_matrices = Eigen::MatrixXd::Identity(n_dim_state, n_dim_state);
	Eigen::MatrixXd transition_covariance = Eigen::MatrixXd::Identity(n_dim_state, n_dim_state);
	Eigen::MatrixXd observation_matrices = Eigen::MatrixXd::Identity(n_dim_obs, n_dim_state);
	Eigen::MatrixXd observation_covariance = Eigen::MatrixXd::Identity(n_dim_obs, n_dim_obs);
	transition_matrices << 1;
	//	transition_matrices  << 2,-1,0,1;
	//	transition_matrices  << 3,-3,1,1,0,0,0,1,0;
	observation_matrices(0, 0) = 1;
	G(0, 0) = 1;
	observation_covariance << 1;
	transition_covariance = G * G.transpose();

	/*パラメータ詳細設定*/
	kf.InitializeParameters(&transition_matrices, &transition_covariance, &observation_matrices, &observation_covariance, n_dim_state, n_dim_obs);

	std::vector<Eigen::VectorXd> filtered_state_mean = { Eigen::VectorXd::Zero(n_dim_state) };
	std::vector<Eigen::MatrixXd> filtered_state_covariance = { Eigen::MatrixXd::Ones(n_dim_state,n_dim_state) };

	std::ofstream file;
	file.open("trajectory_D1.txt");
	file << BinX[0] << std::endl;;
	for (size_t i = 0; i < BinX.size() - 1; i++)
	{
		Eigen::VectorXd corrected_state_mean = Eigen::VectorXd::Zero(n_dim_state);
		Eigen::MatrixXd corrected_state_covariance = Eigen::MatrixXd::Identity(n_dim_obs, n_dim_obs);
		Eigen::VectorXd observation = Eigen::VectorXd::Zero(n_dim_obs);
		observation(0) = BinX[i + 1];
		kf.FilterUpdate(corrected_state_mean, corrected_state_covariance, filtered_state_mean[i], filtered_state_covariance[i], observation);
		filtered_state_mean.push_back(corrected_state_mean);
		filtered_state_covariance.push_back(corrected_state_covariance);
		file << corrected_state_mean(0) << std::endl;;
	}

	file.close();

	/*****************************************************************************************************************************************/


	/******詳細設定 状態二次  観測値一次********************************************************************************************************/
	/*インスタンス作成*/
	KalmanFilter kf1;

	/*観測値 状態次数*/
	n_dim_state = 2;
	n_dim_obs = 1;

	/*その他いろいろ設定*/
	G = Eigen::VectorXd::Zero(n_dim_state);
	transition_matrices = Eigen::MatrixXd::Identity(n_dim_state, n_dim_state);
	transition_covariance = Eigen::MatrixXd::Identity(n_dim_state, n_dim_state);
	observation_matrices = Eigen::MatrixXd::Identity(n_dim_obs, n_dim_state);
	observation_covariance = Eigen::MatrixXd::Identity(n_dim_obs, n_dim_obs);
	transition_matrices << 2, -1, 0, 1;
	//	transition_matrices  << 3,-3,1,1,0,0,0,1,0;
	observation_matrices(0, 0) = 1;
	G(0, 0) = 1;
	observation_covariance << 1;
	transition_covariance = G * G.transpose();

	/*パラメータ詳細設定*/
	kf1.InitializeParameters(&transition_matrices, &transition_covariance, &observation_matrices, &observation_covariance, n_dim_state, n_dim_obs);

	filtered_state_mean = { Eigen::VectorXd::Zero(n_dim_state) };
	filtered_state_covariance = { Eigen::MatrixXd::Ones(n_dim_state,n_dim_state) };

	file.open("trajectory_D2.txt");
	file << BinX[0] << "," << BinX[0] << std::endl;;
	for (size_t i = 0; i < BinX.size() - 1; i++)
	{
		Eigen::VectorXd corrected_state_mean = Eigen::VectorXd::Zero(n_dim_state);
		Eigen::MatrixXd corrected_state_covariance = Eigen::MatrixXd::Identity(n_dim_obs, n_dim_obs);
		Eigen::VectorXd observation = Eigen::VectorXd::Zero(n_dim_obs);
		observation(0) = BinX[i + 1];
		kf1.FilterUpdate(corrected_state_mean, corrected_state_covariance, filtered_state_mean[i], filtered_state_covariance[i], observation);
		filtered_state_mean.push_back(corrected_state_mean);
		filtered_state_covariance.push_back(corrected_state_covariance);
		file << corrected_state_mean(0) << "," << corrected_state_mean(1) << std::endl;;
	}

	file.close();

	/*****************************************************************************************************************************************/


	/******詳細設定 状態三次  観測値一次********************************************************************************************************/
	/*インスタンス作成*/
	KalmanFilter kf2;

	/*観測値 状態次数*/
	n_dim_state = 3;
	n_dim_obs = 1;

	/*その他いろいろ設定*/
	G = Eigen::VectorXd::Zero(n_dim_state);
	transition_matrices = Eigen::MatrixXd::Identity(n_dim_state, n_dim_state);
	transition_covariance = Eigen::MatrixXd::Identity(n_dim_state, n_dim_state);
	observation_matrices = Eigen::MatrixXd::Identity(n_dim_obs, n_dim_state);
	observation_covariance = Eigen::MatrixXd::Identity(n_dim_obs, n_dim_obs);
	transition_matrices << 3, -3, 1, 1, 0, 0, 0, 1, 0;
	observation_matrices(0, 0) = 1;
	G(0, 0) = 1;
	observation_covariance << 1;
	transition_covariance = G * G.transpose();

	/*パラメータ詳細設定*/
	kf2.InitializeParameters(&transition_matrices, &transition_covariance, &observation_matrices, &observation_covariance, n_dim_state, n_dim_obs);

	filtered_state_mean = { Eigen::VectorXd::Zero(n_dim_state) };
	filtered_state_covariance = { Eigen::MatrixXd::Ones(n_dim_state,n_dim_state) };

	file.open("trajectory_D3.txt");
	file << BinX[0] << "," << BinX[0] << "," << BinX[0] << std::endl;;
	for (size_t i = 0; i < BinX.size() - 1; i++)
	{
		Eigen::VectorXd corrected_state_mean = Eigen::VectorXd::Zero(n_dim_state);
		Eigen::MatrixXd corrected_state_covariance = Eigen::MatrixXd::Identity(n_dim_obs, n_dim_obs);
		Eigen::VectorXd observation = Eigen::VectorXd::Zero(n_dim_obs);
		observation(0) = BinX[i + 1];
		kf2.FilterUpdate(corrected_state_mean, corrected_state_covariance, filtered_state_mean[i], filtered_state_covariance[i], observation);
		filtered_state_mean.push_back(corrected_state_mean);
		filtered_state_covariance.push_back(corrected_state_covariance);
		file << corrected_state_mean(0) << "," << corrected_state_mean(1) << "," << corrected_state_mean(2) << std::endl;;
	}

	file.close();


	/*****************************************************************************************************************************************/


	/******簡易設定 一次********************************************************************************************************/
	/*インスタンス作成*/
	KalmanFilter kf3;
	kf3.SimpleSetup(1.0, 1.0);
	filtered_state_mean = { Eigen::VectorXd::Zero(1) };
	filtered_state_covariance = { Eigen::MatrixXd::Ones(1,1) };

	file.open("trajectory_D4.txt");
	file << BinX[0] << std::endl;;
	for (size_t i = 0; i < BinX.size() - 1; i++)
	{
		Eigen::VectorXd corrected_state_mean = Eigen::VectorXd::Zero(1);
		Eigen::MatrixXd corrected_state_covariance = Eigen::MatrixXd::Identity(1, 1);
		Eigen::VectorXd observation = Eigen::VectorXd::Zero(1);
		observation(0) = BinX[i + 1];
		kf3.FilterUpdate(corrected_state_mean, corrected_state_covariance, filtered_state_mean[i], filtered_state_covariance[i], observation);
		filtered_state_mean.push_back(corrected_state_mean);
		filtered_state_covariance.push_back(corrected_state_covariance);
		file << corrected_state_mean(0) << std::endl;;
	}

	file.close();



	/*****************************************************************************************************************************************/

	return 0;
}
