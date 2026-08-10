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

#ifndef IVIEWERHOST_H
#define IVIEWERHOST_H

// P10-6: the only System surface the Viewer actually consumes (docs/
// P10_RECON.md 2부 item 7), cut to the P7-1b IResetRequester discipline:
// zero project includes and zero forward declarations — no ORB_SLAM3 type
// appears in any signature, so this header can never re-enter the
// System.hpp <-> Viewer.hpp include cycle it exists to dissolve. <string>
// is the sole dependency (std::string cannot legally be forward-declared).
// The sensor-enum constants the Viewer also used to read from System
// (mpSystem->MONOCULAR/...) are deliberately NOT an interface method: they
// are compile-time facts, injected as a ctor bool instead.

#include <string>

namespace ORB_SLAM3
{

// Narrow viewer-host interface, implemented by System. Every method is
// called from the VIEWER thread (Pangolin menu handlers in Viewer::Run).
class IViewerHost
{
public:
    virtual ~IViewerHost() = default;

    // Localization-mode toggles. Non-blocking latches (System: mMutexMode
    // flags), consumed at the top of the next System::Track* call, where
    // the tracking thread performs the actual LocalMapping stop/release.
    virtual void ActivateLocalizationMode() = 0;
    virtual void DeactivateLocalizationMode() = 0;

    // IResetRequester semantics reused verbatim (see IResetRequester.hpp):
    // non-blocking latch (System: mbResetActiveMap under mMutexReset),
    // consumed by the tracking thread one frame later. MUST NOT be made
    // synchronous.
    virtual void RequestResetActiveMap() = 0;

    // Latch ONLY — NEVER joins. Sets the shutdown request latch and asks
    // the worker threads to finish; the joins (and SaveAtlas) belong to the
    // main thread's end-of-example System::Shutdown(). This is what removes
    // the menuStop viewer-thread-self-join hazard structurally (P10-5 kept
    // a get_id() guard for it; that guard survives as belt-and-braces).
    virtual void RequestShutdown() = 0;

    // Synchronous trajectory saves, executed ON the calling (viewer) thread
    // while other threads may still be winding down — upstream menuStop
    // behavior, preserved deliberately.
    virtual void SaveTrajectoryEuRoC(const std::string& filename) = 0;
    virtual void SaveKeyFrameTrajectoryEuRoC(const std::string& filename) = 0;
};

} // namespace ORB_SLAM3

#endif // IVIEWERHOST_H
