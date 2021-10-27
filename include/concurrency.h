// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

// This file is directly copied from librealsense
#ifndef _RS_CONCURRENCY_HPP_
#define _RS_CONCURRENCY_HPP_
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

// Simplest implementation of a blocking concurrent queue for thread messaging
template<class T>
class single_consumer_queue
{
    std::queue<T> q;
    std::mutex mutex;
    std::condition_variable cv;

public:
    single_consumer_queue<T>() : q(), mutex(), cv() {}

    void enqueue(T item);

    T dequeue();

    bool try_dequeue(T* item);

    bool try_get_front(T* item);

    void clear();
    size_t size();
};

bool any_costumers_alive(const std::vector<bool>& running);

#endif