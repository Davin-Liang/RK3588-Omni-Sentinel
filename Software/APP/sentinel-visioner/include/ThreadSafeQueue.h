#pragma once
#include <chrono>
#include <mutex>
#include <queue>

template<typename T>
class ThreadSafeQueue {
public:
    // 阻塞式弹出，直到队列非空
    T pop() 
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]{ return !queue_.empty(); });
        T val = queue_.front();
        queue_.pop();
        return val;
    }

    // 非阻塞尝试弹出
    bool try_pop(T& val)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;
        val = queue_.front();
        queue_.pop();
        return true;
    }

    // 带超时的阻塞弹出：timeoutMs 毫秒内无数据则返回 false
    bool try_pop(T& val, int timeoutMs)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cond_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                            [this] { return !queue_.empty(); })) {
            return false;
        }
        val = queue_.front();
        queue_.pop();
        return true;
    }

    // 压入元素
    void push(T val) 
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(std::move(val));
        }
        cond_.notify_one(); // 通知等待的消费者
    }

    bool empty() const 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    std::queue<T> queue_;
};