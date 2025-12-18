#include "acquisition/device_manager.h"
#include "core/logger.h"

#include <thread>
#include <chrono>
#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <filesystem>
#include <algorithm>
#include <cstring>
#include <cassert>
#include <atomic>
#include <mutex>

#include "core/common.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "livox_lidar_api.h"

DeviceManager::DeviceManager()
	: dm_running_(false)
{
	replayer_ = std::make_unique<Replayer>();
	data_processor_ = std::make_unique<DataProcessor>();
	
	// ת�� Replayer �Ļص�
	replayer_->onFrame = [this](PointCloudPtr cloud, const Pose& pose) {
		try {
			if (onFrameWithPose) {
				onFrameWithPose(cloud, pose);
			}
		} catch (const std::exception& e) {
			LOG_ERROR("[DeviceManager] Exception in replayer onFrame: {}", e.what());
		} catch (...) {
			LOG_ERROR("[DeviceManager] Unknown exception in replayer onFrame");
		}
	};

	replayer_->onProgress = [this](uint64_t current, uint64_t total) {
		try {
			if (onReplayProgress) {
				onReplayProgress(current, total);
			}
		} catch (...) {}
	};

	replayer_->onFinished = [this]() {
		try {
			if (onReplayFinished) {
				onReplayFinished();
			}
		} catch (...) {}
	};

	replayer_->onError = [this](const std::string& msg) {
		try {
			if (onError) {
				onError(msg);
			}
		} catch (...) {}
	};

	replayer_->onRecordingFinished = [this]() {
		try {
			if (onRecordingFinished) {
				onRecordingFinished();
			}
		} catch (...) {}
	};

	replayer_->onRecordingDrop = [this]() {
		try {
			if (onRecordingDrop) {
				onRecordingDrop();
			}
		} catch (...) {}
	};
}

DeviceManager::~DeviceManager()
{
	stop();
	stopReplay(); 
	stopRecording();
}

static bool IsPointCloudDataType(uint8_t type) {
	return type == kLivoxLidarCartesianCoordinateHighData ||
		   type == kLivoxLidarCartesianCoordinateLowData ||
		   type == kLivoxLidarSphericalCoordinateData;
}

// 将 LivoxLidarEthernetPacket 转换为 PointCloudPtr
static PointCloudPtr ParsePointCloudFromPacket(const LivoxLidarEthernetPacket *pkt)
{
	if (!pkt)
		return nullptr;

	// Ԥ�ȼ���������ͣ����⴦���ǵ������ݣ��� IMU��
	if (!IsPointCloudDataType(pkt->data_type)) {
		return nullptr;
	}

	// ʹ�� offsetof ȷ�� header ��С������ pkt->length ����ȫ�ж�
	size_t total_len = static_cast<size_t>(pkt->length);
	size_t header_size = offsetof(LivoxLidarEthernetPacket, data);
	if (total_len < header_size)
		return nullptr;
		
	size_t payload_len = total_len - header_size;
	const uint8_t *base = reinterpret_cast<const uint8_t *>(pkt);
	const uint8_t *payload_base = base + header_size;

	uint8_t data_type = pkt->data_type;

	auto cap_estimated = [](size_t est)
	{
		const size_t MAX_EST = 10 * 1000 * 1000; // ����������ֵ��10M
		return (est > MAX_EST) ? MAX_EST : est;
	};

	auto pc = std::make_shared<PointCloud>();

	auto has_enough = [&](size_t offset, size_t structSize) -> bool
	{
		return structSize <= payload_len && offset <= payload_len - structSize;
	};

	if (data_type == kLivoxLidarCartesianCoordinateHighData)
	{
		const size_t structSize = sizeof(LivoxLidarCartesianHighRawPoint);
		if (structSize == 0)
			return nullptr;
		size_t offset = 0;
		size_t estimated = payload_len / structSize;
		estimated = cap_estimated(estimated);
		if (estimated)
			pc->points.reserve(estimated);
		while (has_enough(offset, structSize))
		{
			LivoxLidarCartesianHighRawPoint raw_local;
			std::memcpy(&raw_local, payload_base + offset, structSize);

			Point p;
			p.x = static_cast<float>(raw_local.x) / 1000.0f; // ���� -> ��
			p.y = static_cast<float>(raw_local.y) / 1000.0f;
			p.z = static_cast<float>(raw_local.z) / 1000.0f;
			p.intensity = static_cast<float>(raw_local.reflectivity) / 255.0f;
			pc->points.push_back(p);
			offset += structSize;
		}
	}
	else if (data_type == kLivoxLidarCartesianCoordinateLowData)
	{
		const size_t structSize = sizeof(LivoxLidarCartesianLowRawPoint);
		if (structSize == 0)
			return nullptr;
		size_t offset = 0;
		size_t estimated = payload_len / structSize;
		estimated = cap_estimated(estimated);
		if (estimated)
			pc->points.reserve(estimated);
		while (has_enough(offset, structSize))
		{
			LivoxLidarCartesianLowRawPoint raw_local;
			std::memcpy(&raw_local, payload_base + offset, structSize);

			Point p;
			p.x = static_cast<float>(raw_local.x) / 100.0f; // 厘米 -> 米
			p.y = static_cast<float>(raw_local.y) / 100.0f;
			p.z = static_cast<float>(raw_local.z) / 100.0f;
			p.intensity = static_cast<float>(raw_local.reflectivity) / 255.0f;
			pc->points.push_back(p);
			offset += structSize;
		}
	}
	else if (data_type == kLivoxLidarSphericalCoordinateData)
	{
		const size_t structSize = sizeof(LivoxLidarSpherPoint);
		if (structSize == 0)
			return nullptr;
		size_t offset = 0;
		size_t estimated = payload_len / structSize;
		estimated = cap_estimated(estimated);
		if (estimated)
			pc->points.reserve(estimated);
		const float deg2rad = 3.14159265358979323846f / 180.0f;
		while (has_enough(offset, structSize))
		{
			LivoxLidarSpherPoint raw_local;
			std::memcpy(&raw_local, payload_base + offset, structSize);
			float depth_m = static_cast<float>(raw_local.depth) / 1000.0f;
			float theta_deg = static_cast<float>(raw_local.theta) / 100.0f;
			float phi_deg = static_cast<float>(raw_local.phi) / 100.0f;
			float theta_r = theta_deg * deg2rad;
			float phi_r = phi_deg * deg2rad;

			Point p;
			p.x = depth_m * std::cos(phi_r) * std::sin(theta_r);
			p.y = depth_m * std::sin(phi_r) * std::sin(theta_r);
			p.z = depth_m * std::cos(theta_r);
			p.intensity = static_cast<float>(raw_local.reflectivity) / 255.0f;
			pc->points.push_back(p);

			offset += structSize;
		}
	}

	pc->width = static_cast<uint32_t>(pc->points.size());
	pc->height = 1;
	pc->is_dense = false;

	return pc;
}

// �� packet �� timestamp �ֶν�� uint64_t��С�ˣ�
static uint64_t ParseTimestampNsFromPacket(const LivoxLidarEthernetPacket *pkt)
{
    // 1. ��ָ����
    if (pkt == nullptr)
    {
        return 0ULL;
    }

    // 2. ���Ȱ�ȫ��� (Bounds Checking)
    // ��ȡ���ݰ��������ܳ���
    size_t total_len = static_cast<size_t>(pkt->length);
    // ���� timestamp �ֶ��ڽṹ���е��ֽ�ƫ����
    const size_t ts_offset = offsetof(LivoxLidarEthernetPacket, timestamp);
    
    // �ؼ���飺ȷ�����յ������ݰ����� ���� ������ʱ����ֶ�
    // timestamp �� 8 �ֽ� (uint64_t)��������Ҫ offset + 8
    if (total_len < ts_offset + 8)
    {
        return 0ULL; // ���ݰ���ȱ���޷���ȡʱ���
    }

    // 3. ��ȡ�ֽ�ָ��
    const uint8_t *base = reinterpret_cast<const uint8_t *>(pkt);
    const uint8_t *ts_ptr = base + ts_offset;

    // 4. �ֶ�����С���� (Little-Endian Parsing)
    uint64_t t = 0ULL;
    for (int i = 0; i < 8; ++i)
    {
        // ��ÿ���ֽ��ƶ�����Ӧ��λ�� (0, 8, 16 ... 56 λ)
        t |= (static_cast<uint64_t>(ts_ptr[i]) << (8 * i));
    }
    return t;
}

// ��� data_type ==0��IMU�������� IMU ԭʼ�ṹ������ȫ��
static bool ParseImuFromPacket(const LivoxLidarEthernetPacket *pkt, IMUData &imu_out)
{
	if (!pkt)
		return false;
	if (pkt->data_type != kLivoxLidarImuData)
		return false;

	size_t total_len = static_cast<size_t>(pkt->length);
	size_t header_size = offsetof(LivoxLidarEthernetPacket, data);
	if (total_len < header_size)
		return false;
	size_t payload_len = total_len - header_size;
	const uint8_t *base = reinterpret_cast<const uint8_t *>(pkt);
	const uint8_t *payload_base = base + header_size;
	if (payload_len < sizeof(LivoxLidarImuRawPoint))
		return false;

	LivoxLidarImuRawPoint raw_local;
	std::memcpy(&raw_local, payload_base, sizeof(LivoxLidarImuRawPoint));
	imu_out.gyro_x = raw_local.gyro_x;
	imu_out.gyro_y = raw_local.gyro_y;
	imu_out.gyro_z = raw_local.gyro_z;
	imu_out.acc_x = raw_local.acc_x;
	imu_out.acc_y = raw_local.acc_y;
	imu_out.acc_z = raw_local.acc_z;
	return true;
}

// Livox �ص���װ
static void OnLivoxPointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data)
{
	if (!data)
		return;

	DeviceManager *mgr = reinterpret_cast<DeviceManager *>(client_data);
	if (!mgr)
		return;

	// ���ˣ����������Ի�豸�Ҵ��ڲɼ�״̬������
	if (!mgr->isActiveDevice(handle) || !mgr->isAcquiring()) {
		return;
	}

	mgr->handlePointCloud(data);
}

// ר�� IMU �ص���ע�ᵽ SDK �� IMU �ص��ӿڣ�
static void OnLivoxImuCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data)
{
	if (!data)
		return;

	DeviceManager *mgr = reinterpret_cast<DeviceManager *>(client_data);
	if (!mgr)
		return;

	// ���ˣ����������Ի�豸�Ҵ��ڲɼ�״̬������
	if (!mgr->isActiveDevice(handle) || !mgr->isAcquiring()) {
		return;
	}

	mgr->handleImu(data);
}

static void OnLivoxInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data)
{
	DeviceManager* mgr = reinterpret_cast<DeviceManager*>(client_data);
	if (mgr) {
		mgr->handleDeviceInfoChange(handle, info);
	}
}

void DeviceManager::handlePointCloud(const void* data_ptr)
{
	const LivoxLidarEthernetPacket* data = reinterpret_cast<const LivoxLidarEthernetPacket*>(data_ptr);
	try
	{
		// �ȿ��ټ��㲢����ʱ����������ظ�����
		uint64_t ts = ParseTimestampNsFromPacket(data);

		// ������־��ÿ���ӡһ�������ͳ��
		static std::atomic<uint64_t> s_pkt_count{0};
		s_pkt_count.fetch_add(1, std::memory_order_relaxed);
		// ʹ�ú����������ӡ
		LOG_DEBUG_THROTTLED(1000, "[Livox Input] pkts/s={} ts={}", s_pkt_count.exchange(0, std::memory_order_relaxed), ts);

		enqueuePacket(reinterpret_cast<const uint8_t*>(data), data->length, ts);
	}
	catch (const std::exception& e)
	{
		LOG_ERROR("[DeviceManager] Exception in handlePointCloud: {}", e.what());
	}
	catch (...)
	{
		LOG_ERROR("[DeviceManager] Unknown exception in handlePointCloud");
	}
}

void DeviceManager::handleImu(const void* data_ptr)
{
	const LivoxLidarEthernetPacket* data = reinterpret_cast<const LivoxLidarEthernetPacket*>(data_ptr);
	try
	{
        // �ȿ��ټ��㲢����ʱ����������ظ�����
		uint64_t ts = ParseTimestampNsFromPacket(data);

		// ��������������� IMU
		IMUData imu;
		if (ParseImuFromPacket(data, imu))
		{
			if (getDataProcessor()) {
				// 1. ��������λ������ʵʱ��ʾ
				Pose p;
				p.imu = imu;
				p.timestamp_ns = ts;
				getDataProcessor()->updateLastPose(p);

				// 2. �� IMU ���ݲ���������������̬����
				getDataProcessor()->addImuSample(ts, imu);
			}
		}
	}
	catch (const std::exception& e)
	{
		LOG_ERROR("[DeviceManager] Exception in handleImu: {}", e.what());
	}
	catch (...)
	{
		LOG_ERROR("[DeviceManager] Unknown exception in handleImu");
	}
}

void DeviceManager::handleDeviceInfoChange(uint32_t handle, const void* info_ptr)
{
	const LivoxLidarInfo* info = reinterpret_cast<const LivoxLidarInfo*>(info_ptr);
	try
	{
		std::vector<DeviceInfo> current_devices;
		
		{
			std::lock_guard<std::mutex> lk(devices_mutex_);
			
			// 1. �������������豸��������Ϣ
			bool found = false;
			for (auto& dev : devices_) {
				if (dev.handle == handle) {
					if (info) {
						dev.ip = info->lidar_ip;
						dev.sn = info->sn;
						dev.type = info->dev_type;
					}
					found = true;
					break;
				}
			}

			// 2. ��������豸�����ӵ��б�
			if (!found && info) {
				DeviceInfo dev;
				dev.handle = handle;
				dev.ip = info->lidar_ip;
				dev.sn = info->sn;
				dev.type = info->dev_type;
				dev.is_connected = false;
				dev.is_acquiring = false;
				devices_.push_back(dev);
			}

			// 3. �Զ����Ӳ��ԣ������ǰû�л�豸���Զ����ӵ�һ�����ֵ��豸
			if (active_device_handle_ == 0 && !devices_.empty()) {
				active_device_handle_ = devices_[0].handle;
				devices_[0].is_connected = true;
				devices_[0].is_acquiring = true; 
				is_acquiring_ = true;
				
				// �������ø��豸�ĵ��Ʒ���
				EnableLivoxLidarPointSend(active_device_handle_, nullptr, nullptr);
				
				LOG_INFO("[DeviceManager] Auto-connected to device: {}", devices_[0].ip);
			}
			
			// �����б��Թ��ص�ʹ�ã������ڻص��г�����
			current_devices = devices_;
		}

		// 4. ֪ͨ�ϲ�Ӧ�ã��� UI ���£�
		if (onDeviceListUpdated) {
			onDeviceListUpdated(current_devices);
		}
	}
	catch (const std::exception& e)
	{
		LOG_ERROR("[DeviceManager] Exception in handleDeviceInfoChange: {}", e.what());
	}
	catch (...)
	{
		LOG_ERROR("[DeviceManager] Unknown exception in handleDeviceInfoChange");
	}
}

std::vector<DeviceInfo> DeviceManager::getDevices() const
{
	std::lock_guard<std::mutex> lk(devices_mutex_);
	return devices_;
}

void DeviceManager::connectDevice(uint32_t handle)
{
	std::lock_guard<std::mutex> lk(devices_mutex_);
	
	if (active_device_handle_ == handle) return;

	if (active_device_handle_ != 0) {
		DisableLivoxLidarPointSend(active_device_handle_, nullptr, nullptr);
		for (auto& d : devices_) {
			if (d.handle == active_device_handle_) {
				d.is_connected = false;
				d.is_acquiring = false;
				break;
			}
		}
	}

	active_device_handle_ = handle;
	for (auto& d : devices_) {
		if (d.handle == handle) {
			d.is_connected = true;
			d.is_acquiring = false; 
			break;
		}
	}
	is_acquiring_ = false;

	if (onDeviceListUpdated) {
		onDeviceListUpdated(devices_);
	}
}

void DeviceManager::disconnectDevice()
{
	std::lock_guard<std::mutex> lk(devices_mutex_);
	if (active_device_handle_ != 0) {
		DisableLivoxLidarPointSend(active_device_handle_, nullptr, nullptr);
		for (auto& d : devices_) {
			if (d.handle == active_device_handle_) {
				d.is_connected = false;
				d.is_acquiring = false;
				break;
			}
		}
		active_device_handle_ = 0;
		is_acquiring_ = false;
	}
	if (onDeviceListUpdated) {
		onDeviceListUpdated(devices_);
	}
}

void DeviceManager::startAcquisition()
{
	// ������ڻطţ���ֹ�����ɼ�
	if (isReplaying()) {
		std::string msg = "Please stop replay and connect device first.";
		LOG_WARN("[DeviceManager] {}", msg);
		if (onInfo) onInfo(msg);
		return;
	}

	std::lock_guard<std::mutex> lk(devices_mutex_);
	if (active_device_handle_ != 0) {
		EnableLivoxLidarPointSend(active_device_handle_, nullptr, nullptr);
		is_acquiring_ = true;
		for (auto& d : devices_) {
			if (d.handle == active_device_handle_) {
				d.is_acquiring = true;
				break;
			}
		}
	}
	if (onDeviceListUpdated) {
		onDeviceListUpdated(devices_);
	}
}

void DeviceManager::stopAcquisition()
{
	std::lock_guard<std::mutex> lk(devices_mutex_);
	if (active_device_handle_ != 0) {
		DisableLivoxLidarPointSend(active_device_handle_, nullptr, nullptr);
		is_acquiring_ = false;
		for (auto& d : devices_) {
			if (d.handle == active_device_handle_) {
				d.is_acquiring = false;
				break;
			}
		}
	}
	if (onDeviceListUpdated) {
		onDeviceListUpdated(devices_);
	}
}

bool DeviceManager::startRecording(const std::string& filepath)
{
	return replayer_->startRecording(filepath);
}

void DeviceManager::stopRecording()
{
	replayer_->stopRecording();
}

bool DeviceManager::isRecording() const
{
	return replayer_->isRecording();
}

bool DeviceManager::startReplay(const std::string& filepath)
{
	if (isRecording()) {
		if (onError) onError("Cannot start replay while recording.");
		return false; 
	}

	// ֹͣ��ǰ���豸�ɼ���������ڽ��У����������ݳ�ͻ
	stopAcquisition();

	return replayer_->start(filepath);
}

void DeviceManager::stopReplay()
{
	replayer_->stop();
}

bool DeviceManager::isReplaying() const
{
	return replayer_->isPlaying();
}

void DeviceManager::setReplaySpeed(float speed)
{
	replayer_->setSpeed(speed);
}

void DeviceManager::pauseReplay()
{
	replayer_->pause();
}

void DeviceManager::resumeReplay()
{
	replayer_->resume();
}

bool DeviceManager::isReplayPaused() const
{
	return replayer_->isPaused();
}

void DeviceManager::seekReplay(double progress)
{
	replayer_->seek(progress);
}

void DeviceManager::setRetentionTime(int ms)
{
	replayer_->setRetentionTime(ms);
}

bool DeviceManager::start(const std::string& config_path)
{
	if (dm_running_.load())
	{
		return true;
	}
	
	// ʹ�� u8path ȷ���� Windows ����ȷ���� UTF-8 ·��
	if (!std::filesystem::exists(std::filesystem::u8path(config_path)))
	{
		LOG_ERROR("Livox config not found at: {}. Cannot start.", config_path);
		return false;
	}

	if (!LivoxLidarSdkInit(config_path.c_str(), ""))
	{
		LOG_ERROR("LivoxLidarSdkInit failed with config: {}. Cannot start.", config_path);
		return false;
	}

	// ע����ƻص�
	SetLivoxLidarPointCloudCallBack(OnLivoxPointCloudCallback, this);
	// ע����� IMU �ص���ʹ����ͬ�� client_data(this)������ʱ��ͬ��/����
	SetLivoxLidarImuDataCallback(OnLivoxImuCallback, this);
	// ע���豸״̬����ص�
	SetLivoxLidarInfoChangeCallback(OnLivoxInfoChangeCallback, this);

	if (!LivoxLidarSdkStart())
	{
		LOG_ERROR("LivoxLidarSdkStart failed. Cannot start.");
		LivoxLidarSdkUninit();
		return false;
	}

	dm_running_.store(true);
	// ����֡�ַ��߳�
	frame_worker_ = std::thread(&DeviceManager::frameDispatcher_, this);
	return true;
}

void DeviceManager::stop()
{
	if (!dm_running_.load())
	{
		return;
	}

	dm_running_.store(false);

	// ע�� IMU �ص����� SDK ֧�ִ� nullptr��
	SetLivoxLidarImuDataCallback(nullptr, nullptr);
	// ע�����ƻص�
	SetLivoxLidarPointCloudCallBack(nullptr, nullptr);
	// ע���豸��Ϣ����ص�
	SetLivoxLidarInfoChangeCallback(nullptr, nullptr);

	// ����֡�ַ��߳�
	{
		std::lock_guard<std::mutex> lk(frame_cv_mutex_);
	}
	frame_cv_.notify_all();

	// �ȴ�֡�߳��˳�
	if (frame_worker_.joinable())
		frame_worker_.join();

	LivoxLidarSdkUninit();
}

// ������ SDK ��֡���У��������ٷ��أ�SPSC ���λ�������
void DeviceManager::enqueuePacket(const uint8_t* data, size_t length, uint64_t timestamp_ns)
{
	uint64_t corrected_ts = data_processor_->syncTimestamp(timestamp_ns);

	RawPacket pkt;
	pkt.timestamp_ns = corrected_ts;
	// Ԥ���䲢��������
	pkt.data.assign(data, data + length);

	// ������������������֡����֤�����߲����������̰߳�ȫ��
	if (frame_queue_.push(pkt))
	{
		// ֪ͨ֡�ַ��߳���������
		{
			std::lock_guard<std::mutex> lk(frame_cv_mutex_);
		}
		frame_cv_.notify_one();
	}
	else
	{
		// ��������������֡
		LOG_WARN_THROTTLED(1000, "[DeviceManager] Frame queue full! Dropping packet. (ts={})", corrected_ts);
	}
}

// ֡�ַ��̣߳��Ӷ�����ȡ��֡�����Ҷ�Ӧ Pose �����ûص����ڷ� SDK�߳���ִ�У�
void DeviceManager::frameDispatcher_()
{
	while (dm_running_.load())
	{
		RawPacket raw_pkt;
		if (!frame_queue_.pop(raw_pkt))
		{
			// ����Ϊ�գ��ȴ�֪ͨ��ʱ����
			std::unique_lock<std::mutex> lk(frame_cv_mutex_);
			frame_cv_.wait_for(lk, std::chrono::milliseconds(100), [this]()
							   { return !dm_running_.load() || frame_queue_.approx_size() > 0; });
			
			// �����Դ�ӡ��/����ͳ��
			uint64_t cur_queue = frame_queue_.approx_size();
			LOG_DEBUG_THROTTLED(1000, "[FrameQueue] approx_size={}", cur_queue);
			
			continue;
		}

		// �ɹ� pop����¼ͳ��
		static std::atomic<uint64_t> s_pop_count{0};
		s_pop_count.fetch_add(1, std::memory_order_relaxed);

		// �������
		LivoxLidarEthernetPacket* pkt_ptr = reinterpret_cast<LivoxLidarEthernetPacket*>(raw_pkt.data.data());
		
		// ����Ƿ�Ϊ�������ݣ�������ǣ����� IMU ���ݻ��룩��������
		if (!IsPointCloudDataType(pkt_ptr->data_type)) {
			continue;
		}

		PointCloudPtr cloud = ParsePointCloudFromPacket(pkt_ptr);
		
		if (!cloud) continue;

		StampedCloud qf;
		qf.cloud = cloud;
		qf.timestamp_ns = raw_pkt.timestamp_ns;

		Pose pose;
		bool has_pose = data_processor_->getPoseAt(qf.timestamp_ns, pose);
		
		if (!has_pose)
		{
			// �������ԣ�ʹ�����һ����֪�� Pose
			pose = data_processor_->getLastPose();
			// ǿ��ͬ��ʱ�����ȷ��֡�ܱ��������̴���
			pose.timestamp_ns = qf.timestamp_ns;

			// ���ԣ���ӡΪʲôû���ҵ� Pose (ʹ��������)
			LOG_WARN_THROTTLED(2000, "[DeviceManager] Warning: Failed to find matching Pose for timestamp {}. Fallback to last known pose.", qf.timestamp_ns);
		}
		else 
		{
			pose.timestamp_ns = qf.timestamp_ns;
		}

		// �����Ƿ��ֵ�ɹ�������������
		try
		{
			// �������¼�ƣ���֡д���ļ�
			replayer_->writeFrame(qf.cloud, pose);

			// ���ظ��ƻص��Ա��⾺̬�������쳣
			auto onFP = onFrameWithPose;
			if (onFP)
				onFP(qf.cloud, pose);
		}
		catch (const std::exception& e)
		{
			LOG_ERROR("[DeviceManager] Exception in frame processing/callback: {}", e.what());
		}
		catch (...)
		{
			LOG_ERROR("[DeviceManager] Unknown exception in frame processing/callback");
		}

		// �����Դ�ӡͳ��
		uint64_t pop = s_pop_count.exchange(0, std::memory_order_relaxed);
		uint64_t qsz = frame_queue_.approx_size();
		LOG_DEBUG_THROTTLED(1000, "[FrameQueue Stats] pop/s={} approx_size={}", pop, qsz);
	}
}
