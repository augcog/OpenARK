#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

/// This is a simple modification of concurrency.h taken from librealsense

///
/// This class is used as a queue for multiple asyncronous producers and a single consumer
/// 
template<class T, class Container = std::vector<T>, class Compare = std::less<typename Container::value_type> >
class SingleConsumerPriorityQueue
{
    std::priority_queue<T,Container,Compare> q;
    std::mutex mutex;
    std::condition_variable cv;

public:
    SingleConsumerPriorityQueue() : q(), mutex(), cv() {}

    void enqueue(T item)
    {
        std::unique_lock<std::mutex> lock(mutex);
        q.push(std::move(item));
        lock.unlock();
        cv.notify_one(); 
    }

    T dequeue()
    {
        std::unique_lock<std::mutex> lock(mutex);
        const auto ready = [this]() { return !q.empty(); };
        if (!ready() && !cv.wait_for(lock, std::chrono::seconds(5), ready)) throw std::runtime_error("Timeout waiting for queued items!");
        auto item = std::move(q.top());
        q.pop();
        return std::move(item);
    }

    bool try_dequeue(T* item)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if(q.size()>0)
        {
            auto val = std::move(q.top());
            q.pop();
            *item = std::move(val);
            return true;
        }
        return false;
    }

    void clear()
    {
        std::unique_lock<std::mutex> lock(mutex);
        while (q.size() > 0)
        {
            const auto ready = [this]() { return !q.empty(); };
            if (!ready() && !cv.wait_for(lock, std::chrono::seconds(5), ready)) throw std::runtime_error("Timeout waiting for queued items!");
            auto item = std::move(q.top());
            q.pop();
        }
        
        
    }
    size_t size()
    {
        std::unique_lock<std::mutex> lock(mutex); 
        return q.size();
    }
};

