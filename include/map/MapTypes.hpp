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

// R4b slice 1 (2026-08-20): forward smart-pointer aliases for the map layer.
//
// MapPoint is shared_ptr-managed. Ownership model (C-lite, deliberate):
//   * Map::mspMapPoints is THE owner (strong). All other holders (KeyFrame
//     match slots, Frame slots, local-map/loop vectors, optimizer pins,
//     side tables) are ALSO strong on purpose: a bad (tombstoned) MapPoint
//     stays alive for as long as anything still references it, which
//     preserves the pre-migration tombstone liveness contract exactly.
//     SetBadFlag() remains the removal protocol; nothing is reclaimed
//     while still referenced.
//   * KeyFrame stays a raw pointer in this slice (R4b slice 2).
// See docs/OWNERSHIP.md ("R4b slice 1") for the full ownership table.

#ifndef ORB_SLAM3_MAP_TYPES_H
#define ORB_SLAM3_MAP_TYPES_H

#include <memory>

namespace ORB_SLAM3
{

class MapPoint;

// Strong (owning/pinning) handle to a MapPoint. Pass as const MapPointPtr&,
// store by value. Raw MapPoint* must not be stored beyond a pinned scope.
using MapPointPtr = std::shared_ptr<MapPoint>;

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_MAP_TYPES_H
