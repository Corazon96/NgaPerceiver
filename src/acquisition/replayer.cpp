#include "acquisition/replayer.h"
#include "core/logger.h"
#include <chrono>
#include <algorithm>

Replayer::Replayer()
{
}

Replayer::~Replayer()
{
	stop();
	// 确保录制停止线程已退出
	if (recording_stop_thread_.joinable()) {
		recording_stop_thread_.join();
	}
}

bool Replayer::startRecording(const std::string& filepath)
{
	if (running_.load()) {
		if (onError) onError("Cannot start recording during replay mode.");
		return false;
	}
	
	if (is_stopping_recording_.load()) {
		if (onError) onError("Cannot start recording while stopping previous recording.");
		return false;
	}

	if (streamer_.isWriting()) {
		if (onError) onError("Recording is already in progress.");
		return false;
	}

	streamer_.onWriteQueueFull = [this]() {
		if (onRecordingDrop) {
			onRecordingDrop();
		}
	};

	if (!streamer_.openWrite(filepath)) {
		std::string msg = "Failed to open file for recording: " + filepath;
		LOG_ERROR("[Replayer] {}", msg);
		if (onError) onError(msg);
		return false;
	}
	return true;
}

void Replayer::stopRecording()
{
	if (is_stopping_recording_.load()) return;
	if (!streamer_.isWriting()) return;

	// 回收之前的线程（如果存在）
	if (recording_stop_thread_.joinable()) {
		recording_stop_thread_.join();
	}

	is_stopping_recording_.store(true);
	
	// 异步关闭，避免阻塞 UI
	recording_stop_thread_ = std::thread([this]() {
		streamer_.closeWrite();
		is_stopping_recording_.store(false);
		if (onRecordingFinished) {
			onRecordingFinished();
		}
	});
}

bool Replayer::isRecording() const
{
	return streamer_.isWriting();
}

void Replayer::writeFrame(const PointCloudPtr& cloud, const Pose& pose)
{
	if (streamer_.isWriting()) {
		streamer_.writeFrame(cloud, pose);
	}
}

bool Replayer::start(const std::string& filepath)
{
	if (running_.load()) {
		if (onError) onError("Replay is already running.");
		return false;
	}

	if (replay_thread_.joinable()) {
		replay_thread_.join();
	}

	running_.store(true);
	paused_.store(false);
	replay_thread_ = std::thread(&Replayer::replayWorker_, this, filepath);
	return true;
}

void Replayer::stop()
{
	if (!running_.load()) return;

	running_.store(false);
	paused_.store(false); // 确保不卡在暂停状态

	if (replay_thread_.joinable()) {
		replay_thread_.join();
	}
	streamer_.closeRead();
}

bool Replayer::isPlaying() const
{
	return running_.load();
}

bool Replayer::isPaused() const
{
	return paused_.load();
}

void Replayer::setSpeed(float speed)
{
	if (speed > 0.0f) {
		speed_.store(speed);
	}
}

void Replayer::pause()
{
	paused_.store(true);
}

void Replayer::resume()
{
	paused_.store(false);
}

void Replayer::seek(double progress)
{
	std::lock_guard<std::mutex> lk(seek_mutex_);
	seek_request_.target = progress;
	seek_request_.requested = true;
}

void Replayer::setRetentionTime(int ms)
{
	if (ms > 0) {
		retention_ms_.store(ms);
	}
}

void Replayer::replayWorker_(std::string filepath)
{
	LOG_INFO("[Replayer] Worker started.");

	// 打开文件
	if (!streamer_.openRead(filepath, &running_)) {
		std::string msg = "Failed to open file for replay: " + filepath;
		LOG_ERROR("[Replayer] {}", msg);
		running_.store(false);
		if (onError) onError(msg);
		if (onFinished) onFinished();
		return;
	}

	LOG_INFO("[Replayer] File opened.");

	PointCloudPtr cloud;
	Pose pose;
	
	// 读取第一帧作为基准
	if (!streamer_.readFrame(cloud, pose)) {
		LOG_INFO("[Replayer] File is empty or invalid.");
		running_.store(false);
		streamer_.closeRead();
		if (onFinished) onFinished();
		return;
	}

	uint64_t start_ts_file = streamer_.getStartTime();
	uint64_t end_ts_file = streamer_.getEndTime();
	uint64_t duration_ns = end_ts_file - start_ts_file;

	auto start_time_sys = std::chrono::steady_clock::now();
	
	// 发送第一帧
	uint64_t now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count());
	
	pose.timestamp_ns = now_ns;
	
	if (onFrame) onFrame(cloud, pose);
	if (onProgress) onProgress(0, duration_ns);

	auto last_loop_time = std::chrono::steady_clock::now();
	double current_speed = speed_.load();
	bool eof_reached = false;

	while (running_.load())
	{
		// 处理 Seek
		bool has_seek_request = false;
		{
			std::lock_guard<std::mutex> lk(seek_mutex_);
			if (seek_request_.requested) {
				has_seek_request = true;
			}
		}
		
		if (has_seek_request) {
			handleSeek_(start_ts_file, duration_ns, start_time_sys, current_speed, eof_reached);
			last_loop_time = std::chrono::steady_clock::now();
			continue;
		}

		// 处理暂停 和 EOF
		if (paused_.load() || eof_reached) {
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			auto now = std::chrono::steady_clock::now();
			start_time_sys += (now - last_loop_time);
			last_loop_time = now;
			continue;
		}
		last_loop_time = std::chrono::steady_clock::now();

		if (!streamer_.readFrame(cloud, pose)) {
			if (!eof_reached) {
				LOG_INFO("[Replayer] Replay finished (EOF).");
				if (onFinished) onFinished();
				eof_reached = true;
			}
			continue;
		}

		uint64_t current_ts_file = pose.timestamp_ns;
		double speed = speed_.load();
		
		if (std::abs(speed - current_speed) > SPEED_EPSILON) {
			auto now = std::chrono::steady_clock::now();
			uint64_t offset_ns = static_cast<uint64_t>((current_ts_file - start_ts_file) / speed);
			start_time_sys = now - std::chrono::nanoseconds(offset_ns);
			current_speed = speed;
			LOG_INFO("[Replayer] Speed changed to {:.2f}.", speed);
		}
		
		uint64_t delay_ns = static_cast<uint64_t>((current_ts_file - start_ts_file) / speed);
		auto target_time = start_time_sys + std::chrono::nanoseconds(delay_ns);
		
		auto now_for_sleep = std::chrono::steady_clock::now();
		if (target_time > now_for_sleep) {
			auto remaining = target_time - now_for_sleep;
			auto spin_threshold = std::chrono::microseconds(SPIN_WAIT_THRESHOLD_US);
			if (remaining > spin_threshold) {
				std::this_thread::sleep_for(remaining - spin_threshold);
			}
			while (std::chrono::steady_clock::now() < target_time) {
			}
		}

		auto now_sys = std::chrono::system_clock::now();
		pose.timestamp_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
			now_sys.time_since_epoch()).count());

		if (onFrame) onFrame(cloud, pose);
		
		static int frame_count = 0;
		if (++frame_count % PROGRESS_UPDATE_INTERVAL == 0) {
			if (onProgress) onProgress(current_ts_file - start_ts_file, duration_ns);
		}
	}

	running_.store(false);
	streamer_.closeRead();
	LOG_INFO("[Replayer] Stopped.");
}

void Replayer::handleSeek_(uint64_t start_ts_file, uint64_t duration_ns, 
                           std::chrono::steady_clock::time_point& start_time_sys, 
                           double& current_speed, bool& eof_reached)
{
	double progress = 0.0;
	{
		std::lock_guard<std::mutex> lk(seek_mutex_);
		progress = seek_request_.target;
		seek_request_.requested = false;
	}

	progress = std::max(0.0, std::min(1.0, progress));
	
	uint64_t target_ts = start_ts_file + static_cast<uint64_t>(progress * duration_ns);
	size_t index = streamer_.getFrameIndexAtTime(target_ts);
	
	if (streamer_.seekToFrame(index)) {
		eof_reached = false;
		
		// 预读取逻辑
		uint64_t preload_duration_ns = static_cast<uint64_t>(retention_ms_.load()) * 1000000ULL; 
		uint64_t first_ts = 0;

		uint64_t last_ts_file = 0;
		PointCloudPtr cloud;
		Pose pose;

		for (int i = 0; i < MAX_PRELOAD_FRAMES; ++i) {
			if (streamer_.readFrame(cloud, pose)) {
				uint64_t current_ts_file = pose.timestamp_ns;
				last_ts_file = current_ts_file;
				if (i == 0) first_ts = current_ts_file;

				double speed = speed_.load();
				
				auto now = std::chrono::steady_clock::now();
				uint64_t offset_ns = static_cast<uint64_t>((current_ts_file - start_ts_file) / speed);
				start_time_sys = now - std::chrono::nanoseconds(offset_ns);
				current_speed = speed;
				
				uint64_t now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
					std::chrono::system_clock::now().time_since_epoch()).count());
				
				// 保留相对时间偏移以实现拖尾效果
				if (first_ts > 0 && preload_duration_ns > 0) {
					// Map [first_ts, first_ts + preload] to [now - preload, now]
					// Note: current_ts_file >= first_ts
					uint64_t offset = current_ts_file - first_ts;
					if (offset > preload_duration_ns) offset = preload_duration_ns;
					pose.timestamp_ns = now_ns - (preload_duration_ns - offset);
				} else {
					pose.timestamp_ns = now_ns;
				}
				
				if (onFrame) onFrame(cloud, pose);

				if (current_ts_file > first_ts && (current_ts_file - first_ts >= preload_duration_ns)) {
					break;
				}
			} else {
				break;
			}
		}
		
		if (paused_.load()) {
			streamer_.seekToFrame(index);
			if (onProgress) onProgress(target_ts - start_ts_file, duration_ns);
			
			auto now = std::chrono::steady_clock::now();
			double speed = speed_.load();
			uint64_t offset_ns = static_cast<uint64_t>((target_ts - start_ts_file) / speed);
			start_time_sys = now - std::chrono::nanoseconds(offset_ns);
			current_speed = speed;
		} else {
			if (onProgress && last_ts_file > 0) {
				onProgress(last_ts_file - start_ts_file, duration_ns);
			}
		}
	}
}
