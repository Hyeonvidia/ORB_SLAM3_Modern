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

// R4b (2026-08-20): forward smart-pointer aliases for the map layer.
//
// Slice 1: MapPoint is shared_ptr-managed (all holders strong, C-lite).
// Slice 2: KeyFrame is shared_ptr-managed too. Ownership model (C-lite,
// strong-by-default with DELIBERATE cycle breaks):
//   * Map::mspKeyFrames / Map::mspMapPoints are THE owners (strong).
//   * KF -> MP match slots: strong (slice 1, unchanged).
//   * MP -> KF back-references (mObservations keys) stay RAW KeyFrame*:
//     a strong edge here would close the KF<->MP cycle and leak the whole
//     graph on Map::clear(). Contract: every raw key is scrubbed under the
//     MapPoint's lock by KeyFrame::SetBadFlag() BEFORE the KF can be
//     released (the KF is Map-pinned until SetBadFlag completes), so a key
//     present in the map is always alive; accessors wrap keys into
//     KeyFramePtr pins under the same lock before handing them out.
//   * KF -> KF covisibility stays RAW (mutual edges = cycles); same
//     scrub-on-SetBadFlag contract; accessors pin under mMutexConnections.
//   * Spanning tree: child->parent STRONG (bad-KF parent-chain walks in
//     trajectory saving / UpdateFrameIMU need parents alive),
//     parent->children WEAK. IMU chain: mPrevKF STRONG, mNextKF WEAK.
//     Loop/merge edges WEAK (loop partners are latch-pinned in the Map by
//     protocol; merge partners may be culled and simply expire).
//   * KFDB inverted file stays RAW (erase-on-SetBadFlag contract; weak
//     would double entry size in ~1M word lists).
// A bad (tombstoned) object stays alive for as long as anything still
// holds a strong handle, which preserves the pre-migration tombstone
// liveness contract; SetBadFlag() remains the removal protocol.
// See docs/OWNERSHIP.md ("R4b 슬라이스 1/2") for the full ownership table.

#ifndef ORB_SLAM3_MAP_TYPES_H
#define ORB_SLAM3_MAP_TYPES_H

#include <memory>
#include <mutex>

namespace ORB_SLAM3
{

class MapPoint;
class KeyFrame;

// Strong (owning/pinning) handle to a MapPoint. Pass as const MapPointPtr&,
// store by value. Raw MapPoint* must not be stored beyond a pinned scope.
using MapPointPtr = std::shared_ptr<MapPoint>;

// Strong (owning/pinning) handle to a KeyFrame (R4b slice 2). Same rules.
// Ordered containers of KeyFramePtr sort by the stored raw pointer
// (std::less<shared_ptr> compares .get()), so iteration order is identical
// to the pre-migration raw-pointer containers.
using KeyFramePtr = std::shared_ptr<KeyFrame>;

// Weak handle for the deliberate cycle breaks (parent->children, mNextKF,
// loop/merge edges). lock() before use; a null result means the target was
// reclaimed (equivalent to the pre-migration "rewired away" null).
using KeyFrameWeakPtr = std::weak_ptr<KeyFrame>;

// Mutex-guarded shared-pointer slot with an atomic-like API so the P10-2
// call-site shape (`.load(mo)` / `.store(p, mo)`) survives verbatim while
// every read returns a strong pin. Used for the two cross-thread slots that
// were std::atomic<KeyFrame*> since P10-2 (Tracking::mpLastKeyFrame,
// LocalMapping::mpCurrentKeyFrame): std::atomic<shared_ptr> is C++20 but
// lock-based in libstdc++ anyway, and a named mutex keeps the semantics
// obvious. The memory-order argument is accepted and ignored (the mutex
// provides stronger ordering than any of the old atomic orders).
template <class T>
class SharedSlot
{
public:
    SharedSlot() = default;
    SharedSlot(std::nullptr_t) {}

    std::shared_ptr<T> load(std::memory_order = std::memory_order_seq_cst) const
    {
        std::lock_guard<std::mutex> lk(mMutex);
        return mPtr;
    }
    void store(std::shared_ptr<T> p, std::memory_order = std::memory_order_seq_cst)
    {
        std::lock_guard<std::mutex> lk(mMutex);
        mPtr = std::move(p);
    }

private:
    mutable std::mutex mMutex;
    std::shared_ptr<T> mPtr;
};

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_MAP_TYPES_H
