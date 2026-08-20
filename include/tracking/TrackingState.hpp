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

#ifndef ORB_SLAM3_TRACKINGSTATE_HPP
#define ORB_SLAM3_TRACKINGSTATE_HPP

namespace ORB_SLAM3
{

// R4c: Tracking::eTrackingState promoted to a scoped enum at namespace scope
// so FrameDrawer can name the TYPE despite the Tracking.hpp <-> FrameDrawer.hpp
// include cycle (a class-nested enum cannot be forward-declared). Tracking
// re-exports the old spelling (`using eTrackingState = TrackingState;` plus
// `using enum`), so every Tracking::OK / Tracking::eTrackingState site is
// unchanged.
enum class TrackingState : int {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    RECENTLY_LOST = 3,
    LOST = 4,
    OK_KLT = 5
};

}

#endif // ORB_SLAM3_TRACKINGSTATE_HPP
