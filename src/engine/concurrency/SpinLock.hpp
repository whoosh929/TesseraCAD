#pragma once

#include <atomic>

namespace Tessera {

/// C++20 low-overhead atomic flag spinlock (fallback only, used strictly where mutexes are prohibited)
class SpinLock {
public:
    SpinLock() = default;
    ~SpinLock() = default;

    SpinLock(const SpinLock&) = delete;
    SpinLock& operator=(const SpinLock&) = delete;

    void Lock() noexcept {
        while (m_flag.test_and_set(std::memory_order_acquire)) {
            // C++20 atomic_flag::wait suspends the hardware execution thread efficiently
            m_flag.wait(true, std::memory_order_relaxed);
        }
    }

    bool TryLock() noexcept {
        return !m_flag.test_and_set(std::memory_order_acquire);
    }

    void Unlock() noexcept {
        m_flag.clear(std::memory_order_release);
        m_flag.notify_one();
    }

    void lock() noexcept { Lock(); }
    void unlock() noexcept { Unlock(); }
    bool try_lock() noexcept { return TryLock(); }

private:
    std::atomic_flag m_flag = ATOMIC_FLAG_INIT;
};

} // namespace Tessera
