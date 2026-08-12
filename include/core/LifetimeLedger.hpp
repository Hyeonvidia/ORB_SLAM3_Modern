/**
* This file is part of ORB-SLAM3 (Modern). GPLv3, same terms as the project.
*/

#ifndef ORB_SLAM3_LIFETIME_LEDGER_HPP
#define ORB_SLAM3_LIFETIME_LEDGER_HPP

// P12-L0-b lifetime-event ledger (docs/P12_L0_DESIGN.md): measures the
// bad-then-read window — for each probed site, how long after an object's
// SetBadFlag it is still being read. Diagnostic-only: every macro expands
// to ((void)0) unless the tree is configured with -DLIFETIME_TRACE=1
// (benchmark/scripts/lifetime_build.sh); gate binaries carry no probe
// code (md5-identical binaries are the inertness proof). This is the
// TSAN/ASan diagnostic-tree policy, NOT a FixLevel runtime flag — the #20
// same-binary rule governs behavior fixes in gate builds, which this is not.

#ifdef LIFETIME_TRACE

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <unordered_map>

namespace ORB_SLAM3
{

// Mutex-guarded global ledger. The design doc sketches per-thread ring
// buffers; the pilot site classes (solver-local, shutdown path) are cold,
// so one leaf mutex is contention-free there — revisit before probing the
// hot Optimizer class (L2 stage 3). The ledger mutex is a leaf: never
// held while acquiring any other lock (Stamp/Probe bodies only).
class LifetimeLedger
{
public:
    static LifetimeLedger& I() { static LifetimeLedger g; return g; }

    // L0-D1: one global op counter — bumped at SetBadFlag and AddKeyFrame —
    // so seq_delta reads as "map operations since this object died".
    void Bump() { mSeq.fetch_add(1, std::memory_order_relaxed); }

    void StampBad(char kind, unsigned long id)
    {
        const unsigned long long s = mSeq.fetch_add(1, std::memory_order_relaxed) + 1;
        std::lock_guard<std::mutex> l(mMx);
        mBad[key(kind, id)] = Stamp{s, Clock::now()};
    }

    // Record a read of a bad object at `site`.
    void Probe(const char* site, char kind, unsigned long id)
    {
        const unsigned long long now = mSeq.load(std::memory_order_relaxed);
        const auto t = Clock::now();
        std::lock_guard<std::mutex> l(mMx);
        const auto it = mBad.find(key(kind, id));
        Event e;
        e.site = site; e.kind = kind; e.id = id;
        if (it != mBad.end())
        {
            e.seqDelta = now - it->second.seq;
            e.msDelta = std::chrono::duration<double, std::milli>(t - it->second.t).count();
        }
        else  // bad before the ledger saw it (e.g. PostLoad-restored map)
        {
            e.seqDelta = ~0ull; e.msDelta = -1.0;
        }
        if (mEvents.size() >= kCap) { mEvents.pop_front(); ++mDropped; }  // L0-D2 drop-oldest
        mEvents.push_back(e);
    }

    ~LifetimeLedger() { Flush(); }

private:
    using Clock = std::chrono::steady_clock;
    struct Stamp { unsigned long long seq; Clock::time_point t; };
    struct Event { const char* site; char kind; unsigned long id;
                   unsigned long long seqDelta; double msDelta; };
    static constexpr size_t kCap = 1u << 20;

    static unsigned long long key(char kind, unsigned long id)
    { return (static_cast<unsigned long long>(kind) << 56) ^ id; }

    void Flush()
    {
        const char* path = std::getenv("LIFETIME_TRACE_OUT");
        std::FILE* f = path ? std::fopen(path, "w") : stderr;
        if (!f) f = stderr;
        std::fprintf(f, "site,kind,id,seq_delta,ms_delta\n");
        for (const Event& e : mEvents)
            std::fprintf(f, "%s,%c,%lu,%llu,%.3f\n",
                         e.site, e.kind, e.id, e.seqDelta, e.msDelta);
        std::fprintf(f, "# events=%zu dropped=%llu bad_stamped=%zu\n",
                     mEvents.size(), (unsigned long long)mDropped, mBad.size());
        if (path && f != stderr) std::fclose(f);
    }

    std::atomic<unsigned long long> mSeq{0};
    std::mutex mMx;
    std::unordered_map<unsigned long long, Stamp> mBad;
    std::deque<Event> mEvents;
    unsigned long long mDropped = 0;
};

}  // namespace ORB_SLAM3

#define LT_SEQ_BUMP() ORB_SLAM3::LifetimeLedger::I().Bump()
#define LT_STAMP_BAD(kind, id) ORB_SLAM3::LifetimeLedger::I().StampBad((kind), (id))
#define LT_PROBE_BAD(site, kind, id) ORB_SLAM3::LifetimeLedger::I().Probe((site), (kind), (id))

#else  // !LIFETIME_TRACE — gate builds: macros vanish, header adds nothing

#define LT_SEQ_BUMP() ((void)0)
#define LT_STAMP_BAD(kind, id) ((void)0)
#define LT_PROBE_BAD(site, kind, id) ((void)0)

#endif  // LIFETIME_TRACE

#endif  // ORB_SLAM3_LIFETIME_LEDGER_HPP
