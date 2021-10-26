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
    SingleConsumerPriorityQueue();

    void enqueue(T item);

    T dequeue();

    bool try_dequeue(T* item);

    void clear();
    size_t size();
};
