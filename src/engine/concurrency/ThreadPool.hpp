#pragma once

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <atomic>
#include <memory>
#include <type_traits>

namespace Tessera {

class ThreadPool {
public:
    explicit ThreadPool(size_t threadCount = std::max(2u, std::thread::hardware_concurrency()))
        : m_stopRequested(false) {
        m_workers.reserve(threadCount);
        for (size_t i = 0; i < threadCount; ++i) {
            m_workers.emplace_back([this](std::stop_token stopToken) {
                WorkerLoop(stopToken);
            });
        }
    }

    ~ThreadPool() {
        Shutdown();
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    /// Enqueue a callable task and return a std::future for the result
    template <typename F, typename... Args>
    auto Enqueue(F&& f, Args&&... args) 
        -> std::future<std::invoke_result_t<F, Args...>> {
        using ReturnType = std::invoke_result_t<F, Args...>;

        auto task = std::make_shared<std::packaged_task<ReturnType()>>(
            [func = std::forward<F>(f), ...capturedArgs = std::forward<Args>(args)]() mutable {
                return std::invoke(func, capturedArgs...);
            }
        );

        std::future<ReturnType> futureResult = task->get_future();
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            if (m_stopRequested.load(std::memory_order_relaxed)) {
                throw std::runtime_error("ThreadPool is shutting down; cannot enqueue new tasks.");
            }
            m_tasks.emplace([task]() { (*task)(); });
        }
        m_cv.notify_one();
        return futureResult;
    }

    /// Explicitly trigger shutdown and drain
    void Shutdown() {
        bool expected = false;
        if (m_stopRequested.compare_exchange_strong(expected, true)) {
            m_cv.notify_all();
            for (auto& worker : m_workers) {
                worker.request_stop();
            }
            // Workers join automatically upon std::jthread destruction
        }
    }

    [[nodiscard]] size_t GetThreadCount() const noexcept {
        return m_workers.size();
    }

private:
    void WorkerLoop(std::stop_token stopToken) {
        while (!stopToken.stop_requested()) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(m_queueMutex);
                m_cv.wait(lock, [this, &stopToken]() {
                    return stopToken.stop_requested() || 
                           m_stopRequested.load(std::memory_order_relaxed) || 
                           !m_tasks.empty();
                });

                if ((stopToken.stop_requested() || m_stopRequested.load(std::memory_order_relaxed)) 
                    && m_tasks.empty()) {
                    return;
                }

                if (!m_tasks.empty()) {
                    task = std::move(m_tasks.front());
                    m_tasks.pop();
                }
            }

            if (task) {
                task();
            }
        }
    }

    std::vector<std::jthread> m_workers;
    std::queue<std::function<void()>> m_tasks;
    std::mutex m_queueMutex;
    std::condition_variable m_cv;
    std::atomic<bool> m_stopRequested{false};
};

} // namespace Tessera

