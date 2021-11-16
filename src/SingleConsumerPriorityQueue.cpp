# include "slam/SingleConsumerPriorityQueue.h"

namespace ark {


    template<class T, class Container /*= std::vector<T>*/, class Compare /*= std::less<typename Container::value_type> */>
    SingleConsumerPriorityQueue<T, Container, Container, Compare>::SingleConsumerPriorityQueue() : q(), mutex(), cv()
    {

    }


    template<class T, class Container /*= std::vector<T>*/, class Compare /*= std::less<typename Container::value_type> */>
    void SingleConsumerPriorityQueue<T, Container, Container, Compare>::enqueue(T item)
    {
        std::unique_lock<std::mutex> lock(mutex);
        q.push(std::move(item));
        lock.unlock();
        cv.notify_one();
    }


    template<class T, class Container /*= std::vector<T>*/, class Compare /*= std::less<typename Container::value_type> */>
    T SingleConsumerPriorityQueue<T, Container, Container, Compare>::dequeue()
    {
        std::unique_lock<std::mutex> lock(mutex);
        const auto ready = [this]() { return !q.empty(); };
        if (!ready() && !cv.wait_for(lock, std::chrono::seconds(5), ready)) throw std::runtime_error("Timeout waiting for queued items!");
        auto item = std::move(q.top());
        q.pop();
        return std::move(item);
    }

    template<class T, class Container /*= std::vector<T>*/, class Compare /*= std::less<typename Container::value_type> */>
    bool SingleConsumerPriorityQueue<T, Container, Container, Compare>::try_dequeue(T* item)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (q.size() > 0)
        {
            auto val = std::move(q.top());
            q.pop();
            *item = std::move(val);
            return true;
        }
        return false;
    }

    template<class T, class Container /*= std::vector<T>*/, class Compare /*= std::less<typename Container::value_type> */>
    void SingleConsumerPriorityQueue<T, Container, Container, Compare>::clear()
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

    template<class T, class Container /*= std::vector<T>*/, class Compare /*= std::less<typename Container::value_type> */>
    size_t SingleConsumerPriorityQueue<T, Container, Container, Compare>::size()
    {
        std::unique_lock<std::mutex> lock(mutex);
        return q.size();
    }

}