# include "openark/util/concurrency.h"

template<class T>
void single_consumer_queue<T>::enqueue(T item)
{
    std::unique_lock<std::mutex> lock(mutex);
    q.push(std::move(item));
    lock.unlock();
    cv.notify_one();
}


template<class T>
T single_consumer_queue<T>::dequeue()
{
    std::unique_lock<std::mutex> lock(mutex);
    const auto ready = [this]() { return !q.empty(); };
    if (!ready() && !cv.wait_for(lock, std::chrono::seconds(5), ready)) throw std::runtime_error("Timeout waiting for queued items!");
    auto item = std::move(q.front());
    q.pop();
    return std::move(item);
}


template<class T>
bool single_consumer_queue<T>::try_dequeue(T* item)
{
    std::unique_lock<std::mutex> lock(mutex);
    if (q.size() > 0)
    {
        auto val = std::move(q.front());
        q.pop();
        *item = std::move(val);
        return true;
    }
    return false;
}


template<class T>
bool single_consumer_queue<T>::try_get_front(T* item)
{
    std::unique_lock<std::mutex> lock(mutex);
    if (q.size() > 0)
        *item = q.front();
    return true;
    return false;
}


template<class T>
void single_consumer_queue<T>::clear()
{
    std::unique_lock<std::mutex> lock(mutex);
    while (q.size() > 0)
    {
        const auto ready = [this]() { return !q.empty(); };
        if (!ready() && !cv.wait_for(lock, std::chrono::seconds(5), ready)) throw std::runtime_error("Timeout waiting for queued items!");
        auto item = std::move(q.front());
        q.pop();
    }
}

template<class T>
size_t single_consumer_queue<T>::size()
{
    std::unique_lock<std::mutex> lock(mutex);
    return q.size();
}


bool any_costumers_alive(const std::vector<bool>& running)
{
    for (auto is_running : running)
    {
        if (is_running)
        {
            return true;
        }
    }
    return false;
}
