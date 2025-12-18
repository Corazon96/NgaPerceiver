#pragma once

/**
 * @file queue.h
 * @brief 无锁环形缓冲区 (SPSC Ring Buffer)
 * 用于单生产者单消费者场景下的高性能数据传输
 */

#include <atomic>
#include <vector>
#include <memory>
#include <new>

#ifdef __cpp_lib_hardware_interference_size
    #include <new> // for hardware_destructive_interference_size
#endif

/**
 * @brief SPSC 单生产单消费环形缓冲
 * 存储 T（例如 PointCloudPtr），尽量无锁操作
 * 注意：可用槽位为 capacity_ - 1
 * 
 * @tparam T 存储的元素类型
 */
template <typename T>
class SpscRingBuffer
{
public:
    /**
     * @brief 构造函数
     * @param capacity 缓冲区容量（实际可用容量为 capacity - 1）
     */
    explicit SpscRingBuffer(size_t capacity = 8)
        : capacity_(capacity), buffer_(capacity), head_(0), tail_(0)
    {
    }

    /**
     * @brief 入队操作 (仅生产者调用)
     * @param item 要入队的元素
     * @return true 成功入队, false 队列已满
     */
    bool push(const T &item)
    {
        // 1. 加载 head (Relaxed)
        // 只有生产者自己修改 head，所以用 Relaxed 即可
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next = (head + 1) % capacity_;
        
        // 2. 加载 tail (Acquire)
        // 需要看到消费者线程对 tail 的最新修改，以判断队列是否已满
        if (next == tail_.load(std::memory_order_acquire))
            return false; // full
        
        // 3. 写入数据
        buffer_[head] = item;
        
        // 4. 更新 head (Release)
        // 保证在此之前的写入操作对消费者可见
        head_.store(next, std::memory_order_release);
        return true;
    }

    /**
     * @brief 出队操作 (仅消费者调用)
     * @param item 输出元素
     * @return true 成功出队, false 队列为空
     */
    bool pop(T &item)
    {
        // 1. 加载 tail (Relaxed)
        // 只有消费者自己修改 tail，所以 Relaxed
        size_t tail = tail_.load(std::memory_order_relaxed);
        
        // 2. 加载 head (Acquire)
        // 需要看到生产者对 head 的修改，以判断队列是否为空
        if (tail == head_.load(std::memory_order_acquire))
            return false; // empty
        
        // 3. 读取数据
        item = buffer_[tail];
        
        // 4. 更新 tail (Release)
        // 保证在此之前的读取操作完成，生产者可以安全覆盖
        tail_.store((tail + 1) % capacity_, std::memory_order_release);
        return true;
    }

    /** @brief 清空队列 */
    void clear()
    {
        head_.store(0);
        tail_.store(0);
    }

    /**
     * @brief 获取近似大小
     * 仅用于监控 metrics，不保证精确原子一致性
     */
    size_t approx_size() const
    {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        if (head >= tail)
            return head - tail;
        return capacity_ - (tail - head);
    }

    /** @brief 返回环形缓冲的总容量 */
    size_t capacity() const { return capacity_; }

private:
#ifdef __cpp_lib_hardware_interference_size
    static constexpr size_t kCacheLineSize = std::hardware_destructive_interference_size;
#else
    static constexpr size_t kCacheLineSize = 64;
#endif

    // 避免伪共享 (False Sharing)，将 head 与 tail 分隔在不同的缓存行
    alignas(kCacheLineSize) std::atomic<size_t> head_;
    alignas(kCacheLineSize) std::atomic<size_t> tail_;
    
    size_t capacity_;
    std::vector<T> buffer_;
};
