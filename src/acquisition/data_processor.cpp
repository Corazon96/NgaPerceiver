#include "acquisition/data_processor.h"
#include "core/logger.h"
#include <chrono>
#include <cmath>
#include <algorithm>

DataProcessor::DataProcessor()
{
}

DataProcessor::~DataProcessor()
{
}

uint64_t DataProcessor::syncTimestamp(uint64_t timestamp_ns)
{
	// 1. 获取当前系统时间
	auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
											std::chrono::system_clock::now().time_since_epoch())
											.count());

	// 2. 检查是否需要同步（未初始化或偏差过大）
	bool initialized = time_sync_initialized_.load(std::memory_order_acquire);
	int64_t current_offset = time_offset_ns_.load(std::memory_order_acquire);
	
	// 计算当前使用的 offset 下的系统时间
	int64_t estimated_sys_ts = static_cast<int64_t>(timestamp_ns) + current_offset;
	int64_t drift = static_cast<int64_t>(now_ns) - estimated_sys_ts;

	// 阈值设为 10 秒
	if (!initialized || std::abs(drift) > LingerConfig::TIME_SYNC_DRIFT_THRESHOLD_NS)
	{
		std::lock_guard<std::mutex> lk(time_sync_mutex_);
		// 双重检查 - 使用 acquire 确保看到其他线程的最新修改
		int64_t locked_offset = time_offset_ns_.load(std::memory_order_acquire);
		bool locked_initialized = time_sync_initialized_.load(std::memory_order_acquire);
		int64_t locked_drift = static_cast<int64_t>(now_ns) - (static_cast<int64_t>(timestamp_ns) + locked_offset);
		
		if (!locked_initialized || std::abs(locked_drift) > LingerConfig::TIME_SYNC_DRIFT_THRESHOLD_NS)
		{
			// 重新获取精确时间
			now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
													std::chrono::system_clock::now().time_since_epoch())
													.count());
			int64_t diff = static_cast<int64_t>(now_ns) - static_cast<int64_t>(timestamp_ns);

			if (timestamp_ns < LingerConfig::MIN_VALID_EPOCH_NS || std::abs(diff) > LingerConfig::TIME_SYNC_DRIFT_THRESHOLD_NS)
			{
				time_offset_ns_.store(diff, std::memory_order_release);
				LOG_INFO("[DataProcessor] Timestamp sync. Offset: {} ns. (Lidar: {}, System: {}, Drift: {})", diff, timestamp_ns, now_ns, drift);
			}
			else
			{
				time_offset_ns_.store(0, std::memory_order_release);
				LOG_INFO("[DataProcessor] Absolute timestamp detected. Offset reset to 0.");
			}
			time_sync_initialized_.store(true, std::memory_order_release);
		}
	}
	return static_cast<uint64_t>(static_cast<int64_t>(timestamp_ns) + time_offset_ns_.load(std::memory_order_acquire));
}

void DataProcessor::updateLastPose(const Pose &p)
{
	// 确保 last_pose_ 的时间戳也与系统时间同步
	Pose synced_p = p;
	synced_p.timestamp_ns = syncTimestamp(p.timestamp_ns);

	std::lock_guard<std::mutex> lk(last_pose_mutex_);
	last_pose_ = synced_p;
}

Pose DataProcessor::getLastPose()
{
	std::unique_lock<std::mutex> lk(imu_mutex_);
	if (!imu_samples_.empty())
	{
		return imu_samples_.back();
	}
	lk.unlock();

	std::lock_guard<std::mutex> lk2(last_pose_mutex_);
	return last_pose_;
}

void DataProcessor::integratePose_(const Pose& prev, Pose& cur)
{
	if (cur.timestamp_ns <= prev.timestamp_ns)
	{
		cur.qx = prev.qx;
		cur.qy = prev.qy;
		cur.qz = prev.qz;
		cur.qw = prev.qw;
		return;
	}

	double dt = static_cast<double>(cur.timestamp_ns - prev.timestamp_ns) * 1e-9; // seconds
	
	if (dt <= 0.0)
	{
		cur.qx = prev.qx;
		cur.qy = prev.qy;
		cur.qz = prev.qz;
		cur.qw = prev.qw;
		return;
	}

	// 扣除零偏
	Eigen::Vector3f raw_gyro(cur.imu.gyro_x, cur.imu.gyro_y, cur.imu.gyro_z);
	Eigen::Vector3f omega = raw_gyro;
	if (is_calibrated_) {
		omega -= gyro_bias_;
	} else {
		if (calibration_count_ < LingerConfig::IMU_CALIBRATION_SAMPLES) {
			omega.setZero(); 
		}
	}

	// 死区过滤
	if (omega.norm() < LingerConfig::IMU_DEADZONE_THRESHOLD) {
		omega.setZero();
	}

	double ang = omega.norm();

	if (ang > LingerConfig::IMU_SMALL_ANGLE_THRESHOLD)
	{
		Eigen::Vector3f axis = omega / ang;
		float angle = static_cast<float>(ang * dt); // 小角度旋转
		
		Eigen::Quaternionf prev_q(prev.qw, prev.qx, prev.qy, prev.qz);
		Eigen::Quaternionf dq(Eigen::AngleAxisf(angle, axis));
		Eigen::Quaternionf cur_q = prev_q * dq;
		cur_q.normalize();

		// 重力校正
		Eigen::Vector3f acc(cur.imu.acc_x, cur.imu.acc_y, cur.imu.acc_z);
		float acc_norm = acc.norm();
		if (acc_norm > LingerConfig::IMU_GRAVITY_CORRECTION_MIN_ACC && acc_norm < LingerConfig::IMU_GRAVITY_CORRECTION_MAX_ACC)
		{
			acc.normalize();
			Eigen::Vector3f v_up_sensor = cur_q.conjugate()._transformVector(Eigen::Vector3f::UnitZ());

			if (acc.dot(v_up_sensor) > -0.99f) {
				Eigen::Quaternionf q_corr = Eigen::Quaternionf::FromTwoVectors(acc, v_up_sensor);
				float alpha = LingerConfig::IMU_GRAVITY_CORRECTION_ALPHA;
				Eigen::Quaternionf q_step = Eigen::Quaternionf::Identity().slerp(alpha, q_corr);
				
				cur_q = cur_q * q_step;
				cur_q.normalize();
			}
		}

		cur.qx = cur_q.x();
		cur.qy = cur_q.y();
		cur.qz = cur_q.z();
		cur.qw = cur_q.w();
	}
	else
	{
		cur.qx = prev.qx;
		cur.qy = prev.qy;
		cur.qz = prev.qz;
		cur.qw = prev.qw;
	}
}

void DataProcessor::addImuSample(uint64_t timestamp_ns, const IMUData &imu)
{
	// 1. 时间同步
	timestamp_ns = syncTimestamp(timestamp_ns);
	std::lock_guard<std::mutex> lk(imu_mutex_);

	// 2. 陀螺仪零偏校准
	if (!is_calibrated_) {
		Eigen::Vector3f gyro(imu.gyro_x, imu.gyro_y, imu.gyro_z);
		Eigen::Vector3f acc(imu.acc_x, imu.acc_y, imu.acc_z);

		bool is_static = (gyro.norm() < LingerConfig::IMU_STATIC_GYRO_THRESHOLD) && (std::abs(acc.norm() - 1.0f) < LingerConfig::IMU_STATIC_ACC_DEVIATION);

		if (!is_static) {
			if (calibration_count_ > 0) {
				LOG_WARN("Motion detected during calibration (gyro: {:.2f}, acc: {:.2f}). Resetting calibration...", gyro.norm(), acc.norm());
				gyro_bias_.setZero();
				calibration_count_ = 0;
			}
		} else {
			if (calibration_count_ < LingerConfig::IMU_CALIBRATION_SAMPLES) {
				gyro_bias_ += gyro;
				calibration_count_++;
			} 
			
			if (calibration_count_ >= LingerConfig::IMU_CALIBRATION_SAMPLES) {
				gyro_bias_ /= static_cast<float>(LingerConfig::IMU_CALIBRATION_SAMPLES);
				is_calibrated_ = true;
				LOG_INFO("IMU Calibration Done. Gyro Bias: {}, {}, {}", gyro_bias_.x(), gyro_bias_.y(), gyro_bias_.z());
			}
		}
	}

	// 3. 初始化姿态
	if (imu_samples_.empty())
	{
		Pose s{};
		s.timestamp_ns = timestamp_ns;
		s.imu = imu;
		
		Eigen::Vector3f acc(imu.acc_x, imu.acc_y, imu.acc_z);
		float acc_norm = acc.norm();
		if (acc_norm > LingerConfig::IMU_GRAVITY_CORRECTION_MIN_ACC && acc_norm < LingerConfig::IMU_GRAVITY_CORRECTION_MAX_ACC)
		{
			acc.normalize();
			Eigen::Quaternionf q_init = Eigen::Quaternionf::FromTwoVectors(acc, Eigen::Vector3f::UnitZ());
			s.qx = q_init.x();
			s.qy = q_init.y();
			s.qz = q_init.z();
			s.qw = q_init.w();
		}
		else
		{
			s.qx = 0.0f;
			s.qy = 0.0f;
			s.qz = 0.0f;
			s.qw = 1.0f;
		}
		
		imu_samples_.push_back(s);
		return;
	}

	auto it = std::upper_bound(imu_samples_.begin(), imu_samples_.end(), timestamp_ns,
							   [](const uint64_t val, const Pose &s)
							   { return val < s.timestamp_ns; });
	size_t idx = static_cast<size_t>(std::distance(imu_samples_.begin(), it));
	
	Pose s{};
	s.timestamp_ns = timestamp_ns;
	s.imu = imu;

	// 4. 从前一项积分方向
	if (idx > 0)
	{
		const Pose &prev = imu_samples_[idx - 1];
		integratePose_(prev, s);
	}
	else
	{
		s.qx = 0.0f;
		s.qy = 0.0f;
		s.qz = 0.0f;
		s.qw = 1.0f;
	}

	// 5. 插入并重算后续
	imu_samples_.insert(imu_samples_.begin() + idx, s);
	for (size_t j = idx + 1; j < imu_samples_.size(); ++j)
	{
		integratePose_(imu_samples_[j - 1], imu_samples_[j]);
	}
	
	// 6. 保持容量限制
	while (imu_samples_.size() > imu_capacity_)
	{
		imu_samples_.pop_front();
	}
}

bool DataProcessor::getPoseAt(uint64_t timestamp_ns, Pose &out_pose, uint64_t max_delta_ns)
{
	std::lock_guard<std::mutex> lk(imu_mutex_);
	if (imu_samples_.empty())
		return false;

	if (timestamp_ns <= imu_samples_.front().timestamp_ns)
	{
		uint64_t diff = (imu_samples_.front().timestamp_ns > timestamp_ns) ? (imu_samples_.front().timestamp_ns - timestamp_ns) : (timestamp_ns - imu_samples_.front().timestamp_ns);
		if (diff <= max_delta_ns)
		{
			const Pose &s = imu_samples_.front();
			out_pose.timestamp_ns = timestamp_ns;
			out_pose.qx = s.qx;
			out_pose.qy = s.qy;
			out_pose.qz = s.qz;
			out_pose.qw = s.qw;
			out_pose.imu = s.imu;
			return true;
		}
		return false;
	}
	if (timestamp_ns >= imu_samples_.back().timestamp_ns)
	{
		uint64_t diff = (imu_samples_.back().timestamp_ns > timestamp_ns) ? (imu_samples_.back().timestamp_ns - timestamp_ns) : (timestamp_ns - imu_samples_.back().timestamp_ns);
		if (diff <= max_delta_ns)
		{
			const Pose &s = imu_samples_.back();
			out_pose.timestamp_ns = timestamp_ns;
			out_pose.qx = s.qx;
			out_pose.qy = s.qy;
			out_pose.qz = s.qz;
			out_pose.qw = s.qw;
			out_pose.imu = s.imu;
			return true;
		}
		return false;
	}

	size_t lo = 0;
	size_t hi = imu_samples_.size();
	while (lo + 1 < hi)
	{
		size_t mid = lo + (hi - lo) / 2;
		if (imu_samples_[mid].timestamp_ns == timestamp_ns)
		{
			const Pose &s = imu_samples_[mid];
			out_pose.timestamp_ns = timestamp_ns;
			out_pose.qx = s.qx;
			out_pose.qy = s.qy;
			out_pose.qz = s.qz;
			out_pose.qw = s.qw;
			out_pose.imu = s.imu;
			return true;
		}
		if (imu_samples_[mid].timestamp_ns < timestamp_ns)
			lo = mid;
		else
			hi = mid;
	}

	const Pose &s_lo = imu_samples_[lo];
	const Pose &s_hi = imu_samples_[hi];
	if (hi >= imu_samples_.size())
	{
		return false;
	}

	uint64_t t_lo = s_lo.timestamp_ns;
	uint64_t t_hi = s_hi.timestamp_ns;
	uint64_t diff_lo = (t_lo > timestamp_ns) ? (t_lo - timestamp_ns) : (timestamp_ns - t_lo);
	uint64_t diff_hi = (t_hi > timestamp_ns) ? (t_hi - timestamp_ns) : (timestamp_ns - t_hi);
	uint64_t best_diff = (diff_lo < diff_hi) ? diff_lo : diff_hi;
	if (best_diff > max_delta_ns)
	{
		return false;
	}
	double alpha = 0.0;
	if (t_hi > t_lo)
		alpha = static_cast<double>(timestamp_ns - t_lo) / static_cast<double>(t_hi - t_lo);
	
	Eigen::Quaternionf q_lo(s_lo.qw, s_lo.qx, s_lo.qy, s_lo.qz);
	Eigen::Quaternionf q_hi(s_hi.qw, s_hi.qx, s_hi.qy, s_hi.qz);
	Eigen::Quaternionf q_out = q_lo.slerp(static_cast<float>(alpha), q_hi);

	IMUData imu_out;
	imu_out.gyro_x = static_cast<float>((1.0 - alpha) * s_lo.imu.gyro_x + alpha * s_hi.imu.gyro_x);
	imu_out.gyro_y = static_cast<float>((1.0 - alpha) * s_lo.imu.gyro_y + alpha * s_hi.imu.gyro_y);
	imu_out.gyro_z = static_cast<float>((1.0 - alpha) * s_lo.imu.gyro_z + alpha * s_hi.imu.gyro_z);
	imu_out.acc_x = static_cast<float>((1.0 - alpha) * s_lo.imu.acc_x + alpha * s_hi.imu.acc_x);
	imu_out.acc_y = static_cast<float>((1.0 - alpha) * s_lo.imu.acc_y + alpha * s_hi.imu.acc_y);
	imu_out.acc_z = static_cast<float>((1.0 - alpha) * s_lo.imu.acc_z + alpha * s_hi.imu.acc_z);

	out_pose.timestamp_ns = timestamp_ns;
	out_pose.qx = q_out.x();
	out_pose.qy = q_out.y();
	out_pose.qz = q_out.z();
	out_pose.qw = q_out.w();
	out_pose.imu = imu_out;
	return true;
}
