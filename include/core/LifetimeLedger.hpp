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
//
// v2 (P12-L2 class-3 precondition): per-thread ring buffers, no shared
// state on the record path. The v1 mutex-guarded ledger measurably delayed
// LocalMapping — enough to systematically shift the T22 InitializeIMU
// landing window (benchmark/lifetime/pilot_e4032eb/perturb.verdict). Now
// StampBad and Probe are a thread-local ring append (single-writer, no
// lock); the stamp->probe delta join moved into lifetime_report.py. The
// only shared atomic is the op counter (relaxed fetch_add).

#ifdef LIFETIME_TRACE

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <vector>

namespace ORB_SLAM3
{

class LifetimeLedger
{
public:
    static LifetimeLedger& I() { static LifetimeLedger g; return g; }

    // L0-D1: one global op counter — bumped at SetBadFlag and AddKeyFrame —
    // so seq deltas read as "map operations since this object died".
    void Bump() { mSeq.fetch_add(1, std::memory_order_relaxed); }

    void StampBad(char kind, unsigned long id)
    {
        const unsigned long long s = mSeq.fetch_add(1, std::memory_order_relaxed) + 1;
        Tls().write(Rec{"", 'S', kind, id, s, NowMs()});
    }

    // Record a read of a bad object at `site` (caller already checked isBad).
    void Probe(const char* site, char kind, unsigned long id)
    {
        Tls().write(Rec{site, 'P', kind, id,
                        mSeq.load(std::memory_order_relaxed), NowMs()});
    }

    ~LifetimeLedger() { Flush(); }

private:
    using Clock = std::chrono::steady_clock;
    struct Rec { const char* site; char type; char kind; unsigned long id;
                 unsigned long long seq; double ms; };

    // Stamps and probes separate per thread. Stamps are the JOIN KEYS for
    // the report-side delta computation — dropping one silently orphans
    // every later probe of that object (measured: L2's ~65k MP-culling
    // stamps overflowed a shared 64k ring and orphaned 903 probe joins),
    // so stamps go to a growable vector (bounded by objects created per
    // run, ~MBs, diagnostic tree). Probes keep the L0-D2 drop-oldest ring;
    // drops are counted and reported so truncation is never silent.
    struct Ring
    {
        static constexpr size_t kCap = 1u << 16;
        std::array<Rec, kCap> buf;
        unsigned long long total = 0;
        std::vector<Rec> stamps;
        void write(const Rec& r)
        {
            if (r.type == 'S') { stamps.push_back(r); return; }
            buf[total % kCap] = r; ++total;
        }
    };

    // First touch per thread: allocate + register (one-time mutex; never on
    // the steady-state record path). Rings leak deliberately — threads may
    // outlive flush ordering, and this is a diagnostic tree.
    Ring& Tls()
    {
        static thread_local Ring* tls = nullptr;
        if (!tls)
        {
            tls = new Ring();
            std::lock_guard<std::mutex> l(mMx);
            mRings.push_back(tls);
        }
        return *tls;
    }

    double NowMs()
    {
        return std::chrono::duration<double, std::milli>(Clock::now() - mT0).count();
    }

    void Flush()
    {
        const char* path = std::getenv("LIFETIME_TRACE_OUT");
        std::FILE* f = path ? std::fopen(path, "w") : stderr;
        if (!f) f = stderr;
        std::fprintf(f, "type,site,kind,id,seq,ms\n");
        std::lock_guard<std::mutex> l(mMx);
        unsigned long long nOut = 0, nDropped = 0;
        for (Ring* r : mRings)
        {
            for (const Rec& e : r->stamps)
            {
                std::fprintf(f, "%c,%s,%c,%lu,%llu,%.3f\n",
                             e.type, e.site, e.kind, e.id, e.seq, e.ms);
                ++nOut;
            }
            const unsigned long long lo = r->total > Ring::kCap ? r->total - Ring::kCap : 0;
            nDropped += lo;
            for (unsigned long long i = lo; i < r->total; ++i)
            {
                const Rec& e = r->buf[i % Ring::kCap];
                std::fprintf(f, "%c,%s,%c,%lu,%llu,%.3f\n",
                             e.type, e.site, e.kind, e.id, e.seq, e.ms);
                ++nOut;
            }
        }
        std::fprintf(f, "# records=%llu dropped=%llu threads=%zu\n",
                     nOut, nDropped, mRings.size());
        if (path && f != stderr) std::fclose(f);
    }

    std::atomic<unsigned long long> mSeq{0};
    const Clock::time_point mT0 = Clock::now();
    std::mutex mMx;                // ring registry + flush only, never records
    std::vector<Ring*> mRings;
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
