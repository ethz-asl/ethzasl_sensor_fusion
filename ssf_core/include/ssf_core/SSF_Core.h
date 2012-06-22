/*
 * SSF_Core.h
 *
 *  Created on: Sep 30, 2010
 *      Author: Weiss
 */

#ifndef SSF_CORE_H_
#define SSF_CORE_H_


#include <Eigen/Eigen>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

//#include <ssf_core/DoubleArrayStamped.h>

#include <ssf_core/SSF_CoreConfig.h>

//#include <ssf_core/ext_imu.h>
//#include <ssf_core/ext_state.h>

// message includes
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/ExtEkf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <vector>

#include <ssf_core/state.h>

#define N_STATE_BUFFER 256	/// size of unsigned char, do not change!
#define HLI_EKF_STATE_SIZE 16 	/// number of states exchanged with external propagation. Here: p,v,q,bw,bw=16

namespace ssf_core{

typedef dynamic_reconfigure::Server<ssf_core::SSF_CoreConfig> ReconfigureServer;

//struct State
//{										//state	//error state	//Descr
//	Eigen::Matrix<double, 3, 1> p_;		/// 0- 2	//  0- 2		MAV position (IMU centered)
//	Eigen::Matrix<double, 3, 1> v_;		/// 3- 5	//  3- 5		MAV velocity
//	Eigen::Quaternion<double> q_;		/// 6- 9	//  6- 8		MAV attitude
//	Eigen::Matrix<double, 3, 1> b_w_;	///10-12	//  9-11		gyro biases
//	Eigen::Matrix<double, 3, 1> b_a_;	///13-15	// 12-14		acc biases
//	double L_;							///16	// 15			visual scale
//	Eigen::Quaternion<double> q_wv_;	///17-20	// 16-18		vision-world attitude drift
//	Eigen::Quaternion<double> q_ci_;	///21-24 // 19-21		camera-imu attitude calibration
//	Eigen::Matrix<double, 3, 1> p_ic_;	///25-27	// 22-24		camera-imu position calibration
//
//	Eigen::Matrix<double, N_STATE, N_STATE> P_;					/// error state covariance
//	Eigen::Matrix<double,3,1> w_m_;								/// angular velocity from IMU
//	Eigen::Quaternion<double> q_int_;	/// this is the integrated ang. vel. no corrections applied, to use for delta rot in external algos...
//	Eigen::Matrix<double,3,1> a_m_;								/// acc from IMU
//	double time_;												/// timestamp
//};

class SSF_Core {

public:
//        typedef Eigen::Matrix<double, Eigen::Dynamic,N_STATE> MatrixXSd;
        typedef Eigen::Matrix<double, N_STATE, 1> ErrorState;
        typedef Eigen::Matrix<double, N_STATE, N_STATE> ErrorStateCov;
	///dynamic reconfigure
	template<class T>
	void registerCallback(void(T::*cb_func)(ssf_core::SSF_CoreConfig& config, uint32_t level), T* p_obj){
		callbacks_.push_back(boost::bind(cb_func, p_obj, _1, _2));};

	/// big init routine
	void initialize(const Eigen::Matrix<double, 3, 1> & p, const Eigen::Matrix<double, 3, 1> & v, const Eigen::Quaternion<double> & q,
			const Eigen::Matrix<double, 3, 1> & b_w, const Eigen::Matrix<double, 3, 1> & b_a, const double & L, const Eigen::Quaternion<double> & q_wv,
			const Eigen::Matrix<double, N_STATE, N_STATE> & P,
			const Eigen::Matrix<double, 3, 1> & w_m, const Eigen::Matrix<double, 3, 1> & a_m,
			const Eigen::Matrix<double, 3, 1> & g, const Eigen::Quaternion<double> & q_ci, const Eigen::Matrix<double, 3, 1> & p_ic);


	// dynamic reconfigure callbacks
	int stateSize(){return N_STATE;};
	void setFixedScale(bool fixedScale) {fixedScale_ = fixedScale;};
	void setFixedBias(bool fixedBias) {fixedBias_ = fixedBias;};
	void setFixedCalib(bool fixedCalib) {fixedCalib_ = fixedCalib;};
	void setNoiseAcc(double val) {n_a_ = val;};
	void setNoiseAccBias(double val) {n_ba_ = val;};
	void setNoiseGyr(double val) {n_w_ = val;};
	void setNoiseGyrBias(double val) {n_bw_ = val;};
	void setNoiseScale(double val) {n_L_ = val;};
	void setNoiseWV(double val) {n_qwv_ = val;};
	void setNoiseQCI(double val) {n_qci_ = val;};
	void setNoisePIC(double val) {n_pic_ = val;};
	void setDELAY(double val) {DELAY_ = val;};

	SSF_Core();
	~SSF_Core();

private:
	const static int nFullState_ = 28;	/// complete state
	const static int nBuff_ = 30;		/// buffer size for median q_vw
	const static int nMaxCorr_ = 50;	/// number of IMU measurements buffered for time correction actions
	const static int QualityThres_ = 1e3;

	Eigen::Matrix<double, N_STATE, N_STATE> Fd_;	/// discrete state propagation matrix
	Eigen::Matrix<double, N_STATE, N_STATE> Qd_;	/// discrete propagation noise matrix
//	MatrixXSd H_;	/// measurement matrix
//	Eigen::MatrixXd S_;	/// innovation matrix
//	Eigen::MatrixXd K_;	/// Kalman Gain


	/// state variables
	State StateBuffer_[N_STATE_BUFFER];	/// EKF ringbuffer containing pretty much all info needed at time t
	unsigned char idx_state_;	/// pointer to state buffer at most recent state
	unsigned char idx_P_;		/// pointer to state buffer at P latest propagated
	unsigned char idx_time_;	/// pointer to state buffer at a specific time

	Eigen::Matrix<double, 3, 1> g_;	/// gravity vector

	/// vision-world drift watch dog to determine fuzzy tracking
	int qvw_inittimer_;
	Eigen::Matrix<double,nBuff_,4> qbuff_;

	Eigen::Matrix<double,N_STATE,1> correction_;	/// correction from EKF update

	double n_a_;	/// acc noise
	double n_ba_;	/// bias acc noise
	double n_w_;	/// gyro noise
	double n_bw_;	/// bias gyro noise
	double n_L_;	/// scale drift noise
	double n_qwv_;	/// vision world attitude drift noise
	double n_qci_;	/// imu-cam attitude drift noise
	double n_pic_;	/// imu-cam position drift noise
	double DELAY_;	/// const time delay of measurements

	Eigen::Matrix<double, 3, 3> R_IW_; 		/// Rot IMU->World
	Eigen::Matrix<double, 3, 3> R_CI_;  	/// Rot Camera->IMU
	Eigen::Matrix<double, 3, 3> R_WV_;  	/// Rot World->Vision

	/// dynamic reconfigure variables
	bool fixedScale_;
	bool fixedBias_;
	bool fixedCalib_;
	bool predictionMade_;
	bool initialized_;
	bool data_playback_ ;//< used to determine if internal states get overwritten by the external state prediction (online) or internal state prediction is performed (offline / logs)

	enum{NO_UP,GOOD_UP, FUZZY_UP};

	ros::Publisher pubState_;	/// This contains all states of the filter
	sensor_fusion_comm::DoubleArrayStamped msgState_;

	ros::Publisher pubPose_;	/// 6DoF pose output
	geometry_msgs::PoseWithCovarianceStamped msgPose_;

	ros::Publisher pubPoseCrtl_;	 /// 6DoF pose including velocity output
	sensor_fusion_comm::ExtState msgPoseCtrl_;

	ros::Publisher pubCorrect_;	/// Topic containng corrections for external state propagation
	sensor_fusion_comm::ExtEkf msgCorrect_;

	ros::Subscriber subState_;	/// input from external state propagation
	ros::Subscriber subImu_;	/// IMU readings

	sensor_fusion_comm::ExtEkf hl_state_buf_;	/// buffer to store external propagation data

	/// dynamic reconfigure
	ReconfigureServer *reconfServer_;
	typedef boost::function<void(ssf_core::SSF_CoreConfig& config, uint32_t level)> CallbackType;
	std::vector<CallbackType> callbacks_;

	void propagateState(const double dt);	/// state propagation
	void predictProcessCovariance(const double dt);	/// covariance propagation
        bool applyCorrection(unsigned char idx_delaystate, const ErrorState & res_delayed, double fuzzythres=0.1);    /// applies the correction

	void propPToIdx(unsigned char idx);	/// propagate covariance to a given index in the ringbuffer
	void imuCallback(const sensor_msgs::ImuConstPtr & msg);	/// internal state propagation, only read IMU data
	void stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg);	/// external state propagation
	void Config(ssf_core::SSF_CoreConfig &config, uint32_t level);	/// dynamic reconfigure callbacks
	void DynConfig(ssf_core::SSF_CoreConfig &config, uint32_t level);	/// dynamic reconfigure callbacks



	double getMedian(const Eigen::Matrix<double,nBuff_,1> & data);


public:
  // Interface for update sensors

  /// main update routine called by a given sensor
  template<class H_type, class Res_type, class R_type>
    bool applyMeasurement(unsigned char idx_delaystate, const Eigen::MatrixBase<H_type>& H_delayed,
                          const Eigen::MatrixBase<Res_type> & res_delayed, const Eigen::MatrixBase<R_type>& R_delayed,
                          double fuzzythres = 0.1)
    {
      EIGEN_STATIC_ASSERT_FIXED_SIZE(H_type);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(R_type);

      // get measurements
      if (!predictionMade_)
        return false;

      // make sure we have correctly propagated cov until idx_delaystate
      propPToIdx(idx_delaystate);

      R_type S;
      Eigen::Matrix<double, N_STATE, R_type::RowsAtCompileTime> K;
      ErrorStateCov & P = StateBuffer_[idx_delaystate].P_;

      S = H_delayed * StateBuffer_[idx_delaystate].P_ * H_delayed.transpose() + R_delayed;
      K = P * H_delayed.transpose() * S.inverse();

      correction_ = K * res_delayed;
      const ErrorStateCov KH = (ErrorStateCov::Identity() - K * H_delayed);
      P = KH * P * KH.transpose() + K * R_delayed * K.transpose();

      // make sure P stays symmetric
      P = 0.5 * (P + P.transpose());

      return applyCorrection(idx_delaystate, correction_, fuzzythres);
    }

  /// retreive all state information at time t. Used to build H, residual and noise matrix by update sensors
  unsigned char getClosestState(State* timestate, ros::Time tstamp, double delay = 0.00);

  /// get all state information at a given index in the ringbuffer
  bool getStateAtIdx(State* timestate, unsigned char idx);

};

};// end namespace

#endif /* SSF_CORE_H_ */
