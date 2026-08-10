/**
 * P10-4 threading unit checks (docs/P10_RECON.md 3부 §3).
 *
 * Dependency-free assert-style binary in the tests/bit_identity/
 * extract_hash.cpp mold: no gtest, exits nonzero on the first failure,
 * total runtime well under 2 s on the success path.
 *
 * SCOPE NOTE (stated up front, per the validation strategy): constructing a
 * real LocalMapping requires an Atlas, a BAEpochs and an IMappingOptimizer
 * (heavyweight map/backend-layer types with their own threads of concerns).
 * These checks therefore exercise a minimal fixture class -- MiniMapper --
 * that replicates LocalMapping's P10-4 queue+CV+flag PROTOCOL verbatim:
 * the same three mutexes (NewKFs/Stop/Finish+Reset), the same condition
 * variables and predicates, the same notify points, the same Run() loop
 * skeleton (queue branch evaluated before Stop() every iteration, park wait
 * on `!stopped || finishRequested`, ResetIfRequested position, 3ms-capped
 * timed queue wait, SetFinish sets stopped). They validate the PROTOCOL
 * SHAPE, not the LocalMapping class itself.
 *
 * Every check runs under a ~2 s watchdog (std::async + wait_for): a protocol
 * bug that turns a CV wake into a deadlock shows up as a bounded FAIL, never
 * as a hung test binary.
 *
 * Checks:
 *   (a) insert wakes a parked consumer in <50 ms -- fixture built with a
 *       deliberately HUGE timed net (30 s) so only the notify_one can
 *       explain a fast pickup (with the production 3 ms net the timeout
 *       alone would pass this check vacuously);
 *   (b) RequestStop -> stopped convergence (production 3 ms net: the net IS
 *       the stop-pickup mechanism by design -- this check documents it);
 *   (c) Release wakes the parked thread (park wait is untimed: only the
 *       notify can explain the wake);
 *   (d) reset handshake completes while idle; and invariant 5: a reset
 *       requested WHILE PARKED stays pending until Release;
 *   (e) finish wakes the park (untimed wait) and the thread joins.
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <list>
#include <mutex>
#include <thread>

using namespace std::chrono;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::fprintf(stderr, "CHECK FAILED: %s (%s:%d)\n", #cond,        \
                         __FILE__, __LINE__);                                \
            std::_Exit(1);                                                   \
        }                                                                    \
    } while (0)

// Run `f` on a helper thread and fail hard if it does not complete within
// `budget` -- the watchdog that turns a lost-wakeup deadlock into a bounded
// failure. (On timeout the stuck threads are abandoned; _Exit skips their
// joins deliberately.)
template <class F>
static void bounded(const char* name, F&& f, milliseconds budget = milliseconds(2000))
{
    auto fut = std::async(std::launch::async, std::forward<F>(f));
    if (fut.wait_for(budget) != std::future_status::ready) {
        std::fprintf(stderr, "FAIL %s: watchdog timeout (%lld ms)\n", name,
                     static_cast<long long>(budget.count()));
        std::_Exit(1);
    }
    fut.get();  // propagate exceptions as std::terminate -> nonzero exit
    std::fprintf(stderr, "ok  %s\n", name);
}

// ---------------------------------------------------------------------------
// MiniMapper: LocalMapping's P10-4 synchronization protocol, stripped of all
// SLAM. Field-for-field mirror of the production members it models:
//   queue_/muNewKFs_/condNewKFs_   <-> mlNewKeyFrames/mMutexNewKFs/mCondNewKFs
//   stopped_/stopRequested_/notStop_/muStop_/condStop_
//                                  <-> mbStopped/.../mMutexStop/mCondStop
//   finishRequested_(atomic)/finished_/muFinish_
//                                  <-> mbFinishRequested/mbFinished/mMutexFinish
//   resetRequested_(atomic)/muReset_/condReset_
//                                  <-> mbResetRequested/mMutexReset/mCondReset
// netMs_ parameterizes the Run() tail's timed net (production value: 3).
// ---------------------------------------------------------------------------
class MiniMapper
{
public:
    explicit MiniMapper(long netMs = 3) : netMs_(netMs) {}

    void Run()
    {
        {
            std::unique_lock<std::mutex> lk(muFinish_);
            finished_ = false;
        }
        while (true) {
            if (CheckNewItems()) {
                ProcessNewItem();
            } else if (Stop()) {
                // park: predicate mirrors LocalMapping::Run -- atomic finish
                // flag, NO mMutexFinish (canonical order), NO reset flags
                // (invariant 5).
                {
                    std::unique_lock<std::mutex> lk(muStop_);
                    condStop_.wait(lk, [&] {
                        return !stopped_ ||
                               finishRequested_.load(std::memory_order_relaxed);
                    });
                }
                if (CheckFinish())
                    break;
            }

            ResetIfRequested();

            if (CheckFinish())
                break;

            {
                std::unique_lock<std::mutex> lk(muNewKFs_);
                condNewKFs_.wait_for(lk, milliseconds(netMs_),
                                     [&] { return !queue_.empty(); });
            }
        }
        SetFinish();
    }

    void Insert(int v)
    {
        {
            std::unique_lock<std::mutex> lk(muNewKFs_);
            queue_.push_back(v);
        }
        condNewKFs_.notify_one();  // after unlock, like InsertKeyFrame
    }

    void RequestStop()
    {
        std::unique_lock<std::mutex> lk(muStop_);
        stopRequested_ = true;
    }

    bool Stop()
    {
        std::unique_lock<std::mutex> lk(muStop_);
        if (stopRequested_ && !notStop_) {
            stopped_ = true;
            condStop_.notify_all();
            return true;
        }
        return false;
    }

    void WaitUntilStopped()
    {
        std::unique_lock<std::mutex> lk(muStop_);
        condStop_.wait(lk, [&] { return stopped_; });
    }

    void Release()
    {
        {
            std::unique_lock<std::mutex> lk(muFinish_);  // Finish -> Stop (R6)
            std::unique_lock<std::mutex> lk2(muStop_);
            if (finished_)
                return;
            stopped_ = false;
            stopRequested_ = false;
            {
                std::unique_lock<std::mutex> lk3(muNewKFs_);
                queue_.clear();
            }
        }
        condStop_.notify_all();
    }

    void RequestReset()
    {
        {
            std::unique_lock<std::mutex> lk(muReset_);
            resetRequested_.store(true, std::memory_order_relaxed);
        }
        std::unique_lock<std::mutex> lk(muReset_);
        condReset_.wait(lk, [&] {
            return !resetRequested_.load(std::memory_order_relaxed);
        });
    }

    // Non-blocking probe used by check (d)'s invariant-5 half.
    bool ResetPending()
    {
        std::unique_lock<std::mutex> lk(muReset_);
        return resetRequested_.load(std::memory_order_relaxed);
    }

    void RequestFinish()
    {
        {
            std::unique_lock<std::mutex> lk(muFinish_);
            finishRequested_.store(true, std::memory_order_relaxed);
        }
        { std::unique_lock<std::mutex> lk(muStop_); }  // hand-off, see prod comment
        condStop_.notify_all();
    }

    int Consumed() const { return consumed_.load(std::memory_order_relaxed); }

private:
    bool CheckNewItems()
    {
        std::unique_lock<std::mutex> lk(muNewKFs_);
        return !queue_.empty();
    }

    void ProcessNewItem()
    {
        {
            std::unique_lock<std::mutex> lk(muNewKFs_);
            queue_.pop_front();
        }
        consumed_.fetch_add(1, std::memory_order_relaxed);
    }

    void ResetIfRequested()
    {
        bool executed = false;
        {
            std::unique_lock<std::mutex> lk(muReset_);
            if (resetRequested_.load(std::memory_order_relaxed)) {
                executed = true;
                {
                    std::unique_lock<std::mutex> lkQ(muNewKFs_);
                    queue_.clear();
                }
                resetRequested_.store(false, std::memory_order_relaxed);
            }
        }
        if (executed)
            condReset_.notify_all();
    }

    bool CheckFinish()
    {
        return finishRequested_.load(std::memory_order_relaxed);
    }

    void SetFinish()
    {
        {
            std::unique_lock<std::mutex> lk(muFinish_);
            finished_ = true;
            std::unique_lock<std::mutex> lk2(muStop_);
            stopped_ = true;  // finish-aware waiters converge on stopped_
        }
        condStop_.notify_all();
    }

    const long netMs_;

    std::list<int> queue_;
    std::mutex muNewKFs_;
    std::condition_variable condNewKFs_;

    bool stopped_ = false;
    bool stopRequested_ = false;
    bool notStop_ = false;
    std::mutex muStop_;
    std::condition_variable condStop_;

    std::atomic<bool> finishRequested_{false};
    bool finished_ = true;
    std::mutex muFinish_;

    std::atomic<bool> resetRequested_{false};
    std::mutex muReset_;
    std::condition_variable condReset_;

    std::atomic<int> consumed_{0};
};

// Poll an arbitrary condition with a deadline (checks run inside the
// watchdog, so a tight poll is fine and keeps the fixture free of extra
// test-only synchronization).
template <class Pred>
static bool eventually(Pred&& p, milliseconds deadline = milliseconds(1500))
{
    const auto t1 = steady_clock::now() + deadline;
    while (steady_clock::now() < t1) {
        if (p())
            return true;
        std::this_thread::sleep_for(microseconds(200));
    }
    return p();
}

int main()
{
    // (a) Insert wakes a parked consumer in <50 ms. Net = 30 s: the timeout
    // cannot explain the pickup, only condNewKFs_.notify_one can.
    bounded("a_insert_wakes_consumer", [] {
        MiniMapper m(30000);
        std::thread t(&MiniMapper::Run, &m);
        std::this_thread::sleep_for(milliseconds(30));  // settle into the wait
        const auto t0 = steady_clock::now();
        m.Insert(1);
        CHECK(eventually([&] { return m.Consumed() == 1; }, milliseconds(1000)));
        const auto us =
            duration_cast<microseconds>(steady_clock::now() - t0).count();
        std::fprintf(stderr, "    insert->consume latency: %lld us\n",
                     static_cast<long long>(us));
        CHECK(us < 50000);
        m.RequestFinish();  // finish also wakes the 30 s net? NO -- see below
        // RequestFinish only notifies condStop_; the consumer may be inside
        // the 30 s queue wait. Nudge it the way production would be nudged
        // by any queue activity. (Production keeps a 3 ms net precisely so
        // finish never needs this; the huge net is test-only.)
        m.Insert(0);
        t.join();
    });

    // (b) RequestStop -> stopped convergence under the PRODUCTION 3 ms net:
    // no notify targets the queue wait on stop -- the timed net is the
    // designed pickup mechanism, bounded at ~3 ms.
    bounded("b_requeststop_converges", [] {
        MiniMapper m;  // netMs = 3, production value
        std::thread t(&MiniMapper::Run, &m);
        std::this_thread::sleep_for(milliseconds(10));
        m.RequestStop();
        m.WaitUntilStopped();  // CV wait; woken by Stop() from the loop
        m.RequestFinish();
        t.join();
    });

    // (c) Release wakes the parked thread. The park wait is UNTIMED: only
    // Release's notify_all can wake it. Proof of resumption: a fresh insert
    // is consumed after Release.
    bounded("c_release_wakes_park", [] {
        MiniMapper m;
        std::thread t(&MiniMapper::Run, &m);
        m.RequestStop();
        m.WaitUntilStopped();
        m.Release();
        m.Insert(1);
        CHECK(eventually([&] { return m.Consumed() == 1; }));
        m.RequestFinish();
        t.join();
    });

    // (d) Reset handshake completes while the loop idles; and invariant 5:
    // a reset requested while PARKED stays pending until Release.
    bounded("d_reset_handshake", [] {
        MiniMapper m;
        std::thread t(&MiniMapper::Run, &m);
        m.RequestReset();  // idle loop services within ~one 3 ms iteration

        // Invariant 5: park, then request a reset from a side thread -- it
        // must still be pending after 100 ms (the park does NOT service
        // resets), and must complete once Release lets the loop run.
        m.RequestStop();
        m.WaitUntilStopped();
        auto resetFut =
            std::async(std::launch::async, [&] { m.RequestReset(); });
        std::this_thread::sleep_for(milliseconds(100));
        CHECK(m.ResetPending());  // still blocked while parked
        CHECK(resetFut.wait_for(seconds(0)) != std::future_status::ready);
        m.Release();
        CHECK(resetFut.wait_for(milliseconds(1000)) ==
              std::future_status::ready);
        m.RequestFinish();
        t.join();
    });

    // (e) Finish wakes the park (untimed wait -- only RequestFinish's
    // notify can wake it) and the thread joins via SetFinish.
    bounded("e_finish_wakes_park_and_joins", [] {
        MiniMapper m;
        std::thread t(&MiniMapper::Run, &m);
        m.RequestStop();
        m.WaitUntilStopped();
        m.RequestFinish();
        t.join();  // park -> CheckFinish -> break -> SetFinish
        CHECK(m.Consumed() == 0);
    });

    std::fprintf(stderr, "threading_checks: ALL PASS\n");
    return 0;
}
