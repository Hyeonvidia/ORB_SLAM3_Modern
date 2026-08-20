/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BAEPOCHS_H
#define BAEPOCHS_H

#include <map>
#include "map/MapTypes.hpp"  // R4b: MapPointPtr

namespace ORB_SLAM3
{

class KeyFrame;
class MapPoint;

// Local-BA epoch marks, externalized from the KeyFrame/MapPoint member fields
// mnBALocalForKF / mnBAFixedForKF / MapPoint::mnBALocalForKF (P5 group C).
//
// Unlike the per-call MergeScratch (P5-G), this table is PERSISTENT: it is
// owned by System (one instance per System, alive for the process) and is
// threaded through LocalMapping and LoopClosing into
// Optimizer::LocalBundleAdjustment / LocalInertialBA / MergeInertialBA (which
// stamp entries with the current KF id for per-call dedup) and into
// Optimizer::FullInertialBA(bFixLocal=true), which READS marks left behind by
// a PREVIOUS local-BA call (comparisons of the form `>= maxKFid-1`) to decide
// which keyframe vertices to fix. The values therefore cross call boundaries
// by design and must not be reset between calls (LocalInertialBA clears some
// of its own marks to 0 at the end, exactly as the old fields did).
//
// Concurrency: the old member fields were unlocked shared state written by the
// LocalMapping thread and the LoopClosing thread. In practice accesses never
// overlap: LoopClosing stops LocalMapping (RequestStop/isStopped) before
// MergeInertialBA / the welding LocalBundleAdjustment, and the GBA thread's
// FullInertialBA runs with bFixLocal=false, which never touches the marks.
// This table intentionally reproduces that racy-but-serialized upstream
// contract WITHOUT adding a lock (locking would change timing semantics);
// see docs/OWNERSHIP.md for the inherited concurrency contracts.
struct BAEpochs
{
    std::map<KeyFramePtr, unsigned long> localForKF;   // was KeyFrame::mnBALocalForKF
    std::map<KeyFramePtr, unsigned long> fixedForKF;   // was KeyFrame::mnBAFixedForKF
    std::map<MapPointPtr, unsigned long> mpLocalForKF; // was MapPoint::mnBALocalForKF

    // Lookup with default 0: the old member fields were zero-initialized in
    // the KeyFrame/MapPoint constructors, so an absent entry must compare
    // exactly like a never-stamped field.
    // Key is either a raw KeyFramePtr or a MapPointPtr (R4b slice 1). A
    // MapPointPtr key is a deliberate STRONG pin: a tombstoned MapPoint
    // stamped here stays alive with its epoch mark, so stale-entry address
    // reuse (freed MP, new MP at the same address) cannot occur.
    template<typename K>
    static unsigned long get(const std::map<K, unsigned long>& m, const K& p)
    {
        auto it = m.find(p);
        return (it != m.end()) ? it->second : 0ul;
    }
};

} // namespace ORB_SLAM3

#endif // BAEPOCHS_H
