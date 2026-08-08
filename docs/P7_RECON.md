# P7 정찰 보고 — Tracking 계층 (2026-08-09)

3-agent 병렬 조사 원문. P7 구현 중 이 지도와 어긋나는 발견은 이 문서를 갱신할 것.

> **최우선 제약**: `LocalMapping::InitializeIMU`(LocalMapping 스레드)가
> `LocalMapping.cpp:1440`에서 `mpTracker->mState = Tracking::OK`를 **뮤텍스 없이**
> 직접 쓴다. 즉 상태머신은 Tracking 단독 소유가 아니다. 어떤 캡슐화·리팩토링도
> 이 교차 스레드 전이를 보존해야 하며, 이것이 P9의 R-1 회귀와 같은 부류의 함정이다.

## A. 상태머신 전수 지도 (전이표 골든 레퍼런스)

# Tracking state machine — exhaustive map (P7 golden reference)

Files: `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/src/tracking/Tracking.cpp`, `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/include/tracking/Tracking.hpp`, `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/src/core/System.cpp`, `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/src/mapping/LocalMapping.cpp`, `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/src/viz/FrameDrawer.cpp`

Site census in `Tracking.cpp`: **38 grep hits = 15 writes to `mState` (incl. ctor mem-init) + 21 read lines (25 distinct comparisons) + 1 write to `mLastProcessedState` + 1 commented-out read (L2136)**. Outside `Tracking.cpp`: 3 more reads + 1 read in `System.cpp`, 2 reads + **1 cross-thread write** in `LocalMapping.cpp`.

---

## 1. Every site, grouped by function

### 1.1 `Tracking::Tracking` (ctor, L44)

| Line | Kind | Value / expression | Guard |
|---|---|---|---|
| 45 | **W** | `mState(NO_IMAGES_YET)` | unconditional mem-init |

`mLastProcessedState` is **not** in the mem-init list and has no default initializer → indeterminate until L1022. Benign only because its sole reader (`FrameDrawer::Update`) runs after L1022 on the same thread.

### 1.2 `Tracking::GrabImageMonocular` (L721)

| Line | Kind | Expression | Effect / guard |
|---|---|---|---|
| 741 | R ×2 | `mState==NOT_INITIALIZED \|\| mState==NO_IMAGES_YET \|\| (lastID-initID)<mMaxFrames` | `mSensor==MONOCULAR`: pick `mpIniORBextractor` (2× features) vs `mpORBextractorLeft` |
| 748 | R ×2 | `mState==NOT_INITIALIZED \|\| mState==NO_IMAGES_YET` | `mSensor==IMU_MONOCULAR`: same choice, **without** the `lastID-initID` grace clause |
| 756 | R | `mState==NO_IMAGES_YET` | `t0 = timestamp` |

`GrabImageStereo` (L609) and `GrabImageRGBD` (L675) contain **zero** `mState` reads — the ini-extractor selection is monocular-only.

### 1.3 `Tracking::Track` (L949) — the core

| Line | Kind | Value / expression | Guarding condition |
|---|---|---|---|
| 973 | R | `mState!=NO_IMAGES_YET` | enables the two timestamp sanity checks (L975 regression, L983 >1.0 s jump) |
| 1017 | R | `mState==NO_IMAGES_YET` | — |
| **1019** | **W** | `NOT_INITIALIZED` | `mState==NO_IMAGES_YET` |
| **1022** | **W** | `mLastProcessedState = mState` | unconditional snapshot of frame-entry state |
| 1054 | R | `mState==NOT_INITIALIZED` | dispatch to `StereoInitialization()` (STEREO/RGBD/IMU_STEREO/IMU_RGBD) or `MonocularInitialization()` |
| 1067 | R | `mState!=OK` | init did not complete → `mLastFrame=mCurrentFrame; return` |
| 1094 | R | `mState==OK` | inside `!mbOnlyTracking`; normal front-end branch |
| **1119** | **W** | `LOST` | `!mbOnlyTracking && mState==OK && !bOK && mCurrentFrame.mnId<=(mnLastRelocFrameId+mnFramesToResetIMU) && sensor∈{IMU_MONOCULAR,IMU_STEREO,IMU_RGBD}` |
| **1124** | **W** | `RECENTLY_LOST` (+`mTimeStampLost=ts` L1125) | same, `else if pCurrentMap->KeyFramesInMap()>10` |
| **1129** | **W** | `LOST` | same, `else` (≤10 KFs in map) |
| 1136 | R | `mState==RECENTLY_LOST` | `!mbOnlyTracking && mState!=OK`; sets `bOK=true` optimistically |
| **1150** | **W** | `LOST` (+`bOK=false`) | RECENTLY_LOST && inertial && `mCurrentFrame.mTimeStamp-mTimeStampLost > time_recently_lost` (**5.0**, ctor L48, never read from settings) |
| **1163** | **W** | `LOST` (+`bOK=false`) | RECENTLY_LOST && non-inertial && `ts-mTimeStampLost > 3.0f` && `!Relocalization()` |
| 1169 | R | `mState==LOST` (`else if`) | → `ResetActiveMap()` (<10 KFs) or `CreateMapInAtlas()`, null `mpLastKeyFrame`, `return` |
| 1194 | R | `mState==LOST` | inside `mbOnlyTracking`; → `bOK=Relocalization()` |
| **1298** | **W** | `OK` | `bOK` (post-`TrackLocalMap`) |
| 1299 | R | `mState==OK` (`else if`) | `!bOK` |
| **1310** | **W** | `RECENTLY_LOST` | `!bOK && mState==OK && inertial`; L1304-1307 first issues `mpSystem->ResetActiveMap()` if `!isImuInitialized() \|\| !GetIniertialBA2()` |
| **1313** | **W** | `RECENTLY_LOST` | `!bOK && mState==OK && !inertial` ("visual to lost") |
| — | — | L1317 `mTimeStampLost = mCurrentFrame.mTimeStamp` | both 1310/1313 arms |
| 1360 | R | `bOK \|\| mState==RECENTLY_LOST` | gates motion-model update, VO cleanup, temporal-MP delete, `NeedNewKeyFrame`/`CreateNewKeyFrame` |
| 1403 | R | `mState==RECENTLY_LOST` | `bNeedKF && !bOK && mInsertKFsLost && inertial` → insert KF while lost |
| 1426 | R | `mState==LOST` | → ≤10 KFs: `ResetActiveMap()`+`return`; inertial && `!isImuInitialized()`: `ResetActiveMap()`+`return`; else `CreateMapInAtlas()`+`return` |
| 1455 | R ×2 | `mState==OK \|\| mState==RECENTLY_LOST` | gates trajectory recording |
| 1464 | R | `mlbLost.push_back(mState==LOST)` | `mCurrentFrame.isSet()` |
| 1472 | R | `mlbLost.push_back(mState==LOST)` | `!mCurrentFrame.isSet()` |

### 1.4 `Tracking::StereoInitialization` (L1490)

| Line | Kind | Value | Guard |
|---|---|---|---|
| **1598** | **W** | `OK` | `mCurrentFrame.N>500`; for IMU_STEREO/IMU_RGBD additionally both preintegrations present (L1496) and, unless `mFastInit`, `‖avgA_cur−avgA_last‖ ≥ 0.5` (L1502) |

### 1.5 `Tracking::CreateInitialMapMonocular` (L1681)

| Line | Kind | Value | Guard |
|---|---|---|---|
| **1811** | **W** | `OK` | reached only if `medianDepth>=0 && pKFcur->TrackedMapPoints(1)>=50`; otherwise L1747 `mpSystem->ResetActiveMap()` + `return` **leaves `mState==NOT_INITIALIZED`** |

### 1.6 `Tracking::CreateMapInAtlas` (L1817)

| Line | Kind | Value | Guard |
|---|---|---|---|
| **1826** | **W** | `NO_IMAGES_YET` | unconditional; also `mbCreatedMap=true` (L1854), `mbVelocity=false`, `mbVO=false`, `mbReadyToInitializate=false`, IMU preintegrator recreated (L1838-1842), `mpLastKeyFrame`/`mpReferenceKF` nulled, `mLastFrame`/`mCurrentFrame` default-constructed |

### 1.7 `Tracking::TrackLocalMap` (L2104)

| Line | Kind | Expression | Effect |
|---|---|---|---|
| 2136 | — | `// if(!mbMapUpdated && mState == OK)` | **commented out**; live code is `if(!mbMapUpdated)` |
| 2188 | R | `(mnMatchesInliers>10) && (mState==RECENTLY_LOST)` | early `return true` — lenient acceptance while recently lost (all sensors) |

### 1.8 `Tracking::NeedNewKeyFrame` (L2219)

| Line | Kind | Expression | Effect |
|---|---|---|---|
| 2337 | R | `(((inliers<75 && inliers>15) \|\| mState==RECENTLY_LOST) && mSensor==IMU_MONOCULAR)` | `c4=true` → force KF. Note the "MODIFICATION_2" comment; IMU_STEREO/IMU_RGBD are **excluded** |

### 1.9 `Tracking::SearchLocalPoints` (L2498)

| Line | Kind | Expression | Effect |
|---|---|---|---|
| 2565 | R ×2 | `mState==LOST \|\| mState==RECENTLY_LOST` | projection search radius `th=15` |

### 1.10 `Tracking::Reset` (L2934) / `Tracking::ResetActiveMap` (L2995)

| Line | Kind | Value | Guard |
|---|---|---|---|
| **2973** | **W** | `NO_IMAGES_YET` | unconditional (`Reset`); also `KeyFrame::nNextId=0`, `Frame::nNextId=0`, `mnLastRelocFrameId=0`, `mlbLost.clear()` |
| **3032** | **W** | `NO_IMAGES_YET` (comment: `//NOT_INITIALIZED;`) | unconditional (`ResetActiveMap`); frame ids **not** reset, `mnLastRelocFrameId=mCurrentFrame.mnId` (L3070) |

### 1.11 Outside `Tracking.cpp`

| File:line | Kind | Expression | Context |
|---|---|---|---|
| `System.cpp:308` | R | `mTrackingState = mpTracker->mState` | end of `TrackStereo`, under `mMutexState` |
| `System.cpp:380` | R | same | end of `TrackRGBD` |
| `System.cpp:456` | R | same | end of `TrackMonocular` |
| `System.cpp:1341` | R | `mpTracker->mState==Tracking::LOST` | `System::isLost()`, only if `mpAtlas->isImuInitialized()`; the `\|\|RECENTLY_LOST` disjunct is commented out |
| `LocalMapping.cpp:204` | R | `mpTracker->mState==Tracking::OK` | gates VIBA1/VIBA2 stage advance in `Run()`; **not** under `mMutexMapUpdate` |
| `LocalMapping.cpp:466` | R | `mbInertial && mpTracker->mState==RECENTLY_LOST && GetIniertialBA2()` | `bCoarse` for `SearchForTriangulation` |
| **`LocalMapping.cpp:1440`** | **W** | `mpTracker->mState = Tracking::OK` | end of `InitializeIMU()`, **from the local-mapping thread**; under `mMutexMapUpdate` (lock taken at L1328, function scope to L1446) |
| `FrameDrawer.cpp:406/411/436` | R | `pTracker->mLastProcessedState` | see §3 |

---

## 2. Derived transition table

States actually held by `mState`: `NO_IMAGES_YET(0)`, `NOT_INITIALIZED(1)`, `OK(2)`, `RECENTLY_LOST(3)`, `LOST(4)`. **`SYSTEM_NOT_READY(-1)` and `OK_KLT(5)` are never assigned.**

| # | From | To | Site | Trigger | Sensor |
|---|---|---|---|---|---|
| T0 | — | `NO_IMAGES_YET` | `Tracking.cpp:45` | construction | all |
| T1 | `NO_IMAGES_YET` | `NOT_INITIALIZED` | `1019` | unconditional at frame entry | all |
| T2 | `NOT_INITIALIZED` | `NOT_INITIALIZED` | (no write) | init preconditions unmet (`N<=500`; or mono: `<100` keys / `<100` matches / reconstruct failed) | all |
| T3 | `NOT_INITIALIZED` | `OK` | `1598` | `N>500` (+ IMU excitation gate) | STEREO, RGBD, IMU_STEREO, IMU_RGBD |
| T4 | `NOT_INITIALIZED` | `OK` | `1811` | two-view reconstruct + `medianDepth>=0` + `TrackedMapPoints(1)>=50` | MONOCULAR, IMU_MONOCULAR |
| T5 | `NOT_INITIALIZED` | `NO_IMAGES_YET` *(deferred 1 frame)* | `1747` → `3032` | bad monocular init (`medianDepth<0 \|\| tracked<50`) | MONO(-inertial) |
| T6 | `OK` | `OK` | `1298` | `bOK` (front-end + `TrackLocalMap` succeeded) | all |
| T7 | `OK` | `LOST` | `1119` | `!bOK && mnId<=mnLastRelocFrameId+mnFramesToResetIMU` | inertial — **UNREACHABLE**, see F1 |
| T8 | `OK` | `RECENTLY_LOST` | `1124` | `!bOK` (front-end failed) && `KeyFramesInMap()>10` | all |
| T9 | `OK` | `LOST` | `1129` | `!bOK` (front-end failed) && `KeyFramesInMap()<=10` | all |
| T10 | `OK` | `RECENTLY_LOST` | `1310` | front-end OK but `TrackLocalMap()==false`; **plus** deferred `ResetActiveMap` if `!isImuInitialized() \|\| !GetIniertialBA2()` | inertial |
| T11 | `OK` | `RECENTLY_LOST` | `1313` | front-end OK but `TrackLocalMap()==false` | visual |
| T12 | `RECENTLY_LOST` | `OK` | `1298` | inertial: `PredictStateIMU()` + `TrackLocalMap()` (accepted at `2188` with >10 inliers) / visual: `Relocalization()` + `TrackLocalMap()` | all |
| T13 | `RECENTLY_LOST` | `RECENTLY_LOST` | (no write) | `bOK==false` and no timeout — `1299`'s `else if(mState==OK)` does not fire, so the state persists | all |
| T14 | `RECENTLY_LOST` | `LOST` | `1150` | `ts − mTimeStampLost > 5.0 s` | inertial |
| T15 | `RECENTLY_LOST` | `LOST` | `1163` | `ts − mTimeStampLost > 3.0 s` **and** `Relocalization()` failed | visual |
| T16 | `LOST` | `NO_IMAGES_YET` | `1441` → `1826` | `KeyFramesInMap()>10` && (visual \|\| map IMU-initialized) → `CreateMapInAtlas()` | all |
| T17 | `LOST` | `LOST`, then `NO_IMAGES_YET` *(deferred)* | `1430`/`1437` → `3032` | `KeyFramesInMap()<=10`, or inertial && `!isImuInitialized()` | all |
| T18 | any | `NO_IMAGES_YET` | `980` → `1826` | `mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp` (timestamp regression); IMU queue cleared | all |
| T19 | any | `NO_IMAGES_YET` | `999` → `1826` | `Δt>1.0 s` && `isInertial()` && `isImuInitialized()` && `GetIniertialBA2()` | inertial |
| T20 | any | `NO_IMAGES_YET` *(deferred)* | `995`/`1005` → `3032` | `Δt>1.0 s` && inertial && (`!GetIniertialBA2()` or `!isImuInitialized()`) | inertial |
| T21 | any | `NO_IMAGES_YET` *(deferred)* | `963` → `3032` | `mpLocalMapper->mbBadImu` at Track() entry | inertial |
| T22 | any | **`OK`** | `LocalMapping.cpp:1440` | end of `InitializeIMU()` — **asynchronous, from the local-mapping thread** | inertial |
| T23 | any | `NO_IMAGES_YET` | `System.cpp:1358` → `3032`, or `System::ChangeDataset` → `CreateMapInAtlas` → `1826` | `SLAM.ChangeDataset()` between dataset segments (8 Examples call this) — synchronous, between frames | all |
| T24 | any | `NO_IMAGES_YET` *(deferred)* | `Viewer.cpp:249` → `3032` | GUI "Reset" button, from the viewer thread | all |
| T25 | any | `NO_IMAGES_YET` | `2973` | `Tracking::Reset()` — **unreachable in-tree**, see F8 | all |

**Deferred-reset semantics (critical).** `System::ResetActiveMap()` (`System.cpp:496-500`) only sets `mbResetActiveMap`. The flag is consumed at the **top of the next** `System::TrackStereo/TrackRGBD/TrackMonocular` (L285-296 / L360-370 / L435-446), which calls `Tracking::ResetActiveMap()` → `mState=NO_IMAGES_YET`. So every `mpSystem->ResetActiveMap()` inside `Track()` leaves `mState` unchanged for the remainder of that frame. `CreateMapInAtlas()` is synchronous and changes `mState` immediately.

### Reachability findings

**F1 — T7 (`OK→LOST` at L1119) is dead code.** `mnFramesToResetIMU` is `0` (`Tracking.hpp:308`; the P2-2 note documents that the new-format settings path never assigned it). The guard collapses to `mCurrentFrame.mnId <= mnLastRelocFrameId`. `mnLastRelocFrameId` is only written by `Relocalization()` (L2927, current frame id) and `ResetActiveMap()` (L3070); `Frame::nNextId` is monotonic and never rolled back on the reachable paths. L1119 sits inside the `mState==OK` branch, where `Relocalization()` is never called, so `mnId` is always strictly greater. **Behavioral consequence:** with the legacy value (`mMaxFrames`), an inertial sequence that lost tracking shortly after a relocalization went straight to `LOST` (→ map reset / new map). In this build it goes to `RECENTLY_LOST` instead whenever `KeyFramesInMap()>10`. This is a live semantic difference from upstream's legacy config path and must be a conscious decision in the refactor. (The same `mnFramesToResetIMU==0` collapse also neuters L1322, L1338 and makes L2017/L2129 fire only on the exact relocalization frame.)

**F2 — `LOST` never survives a `Track()` boundary.** Every write of `LOST` (1119/1129/1150/1163) sets or leaves `bOK==false`, so control always reaches L1426, and there is no `return` between L1131 and L1426 on a reachable path. L1426 always resets or creates a new map and returns. Therefore:
- **L1169 `else if (mState == LOST)` (and its whole body, L1170-1187) is unreachable.**
- **L1194 `if(mState==LOST)` (localization-mode relocalization) is unreachable**: `LOST` is only written inside `!mbOnlyTracking` branches, and cannot arrive from a previous frame. Reachable only if `InformOnlyTracking(true)` were called while the tracker was already `LOST`, which cannot happen (Track() returns with the reset pending).
- **L2565's `mState==LOST` disjunct can never be true**: `SearchLocalPoints` runs only from `TrackLocalMap`, which runs only when `bOK==true`, i.e. `mState ∈ {OK, RECENTLY_LOST}`. Only the `RECENTLY_LOST` disjunct is live.

**F3 — `mlbLost` is always `false`.** L1464/L1472 evaluate `mState==LOST` inside the `if(mState==OK || mState==RECENTLY_LOST)` gate at L1455 — a comparison against a value the variable provably cannot hold. `mlbLost` is consumed by `System::SaveTrajectoryTUM/EuRoC/KITTI` (`System.cpp:584, 697, 802, 917`), `Tracking::SaveSubTrajectory` (L3102) and `ResetActiveMap` (L3053), so the "lost" column of every saved trajectory is uniformly false. Frames in `LOST`/`NOT_INITIALIZED` are simply *absent* from the lists rather than marked.

**F4 — `OK_KLT` is completely dead.** Grep over `src/`, `include/`, `Examples/`: the only occurrence is the enum declarator `Tracking.hpp:127`. Never written, never compared. Safe to delete, or to reserve deliberately for a KLT front-end.

**F5 — `SYSTEM_NOT_READY` is never held by `Tracking::mState`.** It exists only as `FrameDrawer`'s own sentinel (`FrameDrawer.cpp:32`, tested at 65/217, rendered at 355). `Tracking` has no "vocabulary loading" state.

**F6 — externally observable `LOST` window.** Because of F2, `System::mTrackingState` and `System::isLost()` can observe `LOST` **only** when `Track()` early-returned at L1431 or L1438 (the two deferred-reset arms of L1426). In the `CreateMapInAtlas` arm (L1441) the published state is already `NO_IMAGES_YET`. Combined with `isLost()`'s `!isImuInitialized() → false` short-circuit, **`System::isLost()` is constant `false` for all pure-visual runs.**

**F7 — `bool bOK;` (L1081) is uninitialized.** It is safe today only because `{OK, RECENTLY_LOST, LOST}` is exhaustive at L1094-1188 and the `LOST` arm returns. Any state added by the refactor (or making the `LOST` arm fall through) makes L1280/L1293 read an indeterminate value.

**F8 — `Tracking::Reset()` (and its L2973 write) is unreachable in-tree.** `System::mbReset` is only set by `System::Reset()`, which has no callers in `src/`, `include/`, or `Examples/`. Only `ResetActiveMap` is live. Note `Reset()` is the only path that rewinds `Frame::nNextId`/`KeyFrame::nNextId`.

---

## 3. `mLastProcessedState`

- **Written exactly once**, `Tracking.cpp:1022`, at the top of `Track()` — *after* the `NO_IMAGES_YET→NOT_INITIALIZED` promotion (L1019) and *before* any tracking work. It is therefore the **frame-entry state**, and it can never hold `NO_IMAGES_YET`.
- **Never read inside `Tracking`.**
- **Sole reader: `FrameDrawer::Update`** (`FrameDrawer.cpp:406`, `411`, `436`), called from **exactly one place**, `Tracking.cpp:1356` — which is *inside* the "already initialized" `else` branch and *after* `mState` has already been mutated for this frame (L1298/1310/1313). The parallel call at `Tracking.cpp:1065` is **commented out**. `FrameDrawer` copies it into its own `int mState` (`FrameDrawer.hpp:68`) under `FrameDrawer::mMutex`; the Viewer thread renders from that copy (`DrawFrame` L64-99, `DrawRightFrame` L216-237, `DrawTextInfo` L334-357).

**Can it disagree with `mState`? Yes, on every transition frame:**

| Frame-entry state (`mLastProcessedState`) | Post-frame `mState` | What the drawer shows |
|---|---|---|
| `OK` | `RECENTLY_LOST` (T8/T10/T11) | "SLAM MODE \| Maps/KFs/MPs/Matches" + green/blue keypoints — i.e. a *healthy* overlay for a frame whose pose was rejected |
| `OK` | `LOST` (T9) | same healthy overlay; then the map is reset |
| `RECENTLY_LOST` | `OK` (T12) | drawer state `3`, which matches **no** branch in `DrawFrame` (L70/77/96) or `DrawTextInfo` (L334-355) → no keypoints drawn, **blank status line** |
| `RECENTLY_LOST` | `RECENTLY_LOST` (T13) | same blank/empty rendering |

**No caller depends on the disagreement.** Nothing reads `mLastProcessedState` for control flow; `System` publishes `mState` (not `mLastProcessedState`) and `LocalMapping` reads `mState`. The original rationale — that `Update` copies `mvIniKeys`/`mvIniMatches`, which belong to the pre-tracking state — no longer applies, because:

**F9 — the `NOT_INITIALIZED` visualization path is entirely dead.** Since `Update` is only called from the initialized branch, `mLastProcessedState==NOT_INITIALIZED` (L406) never fires, `FrameDrawer::mState` never becomes `NOT_INITIALIZED`, and the init match-line drawing (`FrameDrawer.cpp:113-149`, `251+`) plus the `" TRYING TO INITIALIZE "` text (L336) are unreachable. Combined with F2, `FrameDrawer::mState ∈ {SYSTEM_NOT_READY, NO_IMAGES_YET, OK, RECENTLY_LOST}` only — so the `Tracking::LOST` branches at `FrameDrawer.cpp:96`, `234` and the `" TRACK LOST. TRYING TO RELOCALIZE "` text (L351) are **also dead**. The `NO_IMAGES_YET` text is reachable only via `FrameDrawer`'s internal `SYSTEM_NOT_READY→NO_IMAGES_YET` self-assignment (L65-66 / L217-218), never from `Tracking`.

**Refactor guidance:** `mLastProcessedState` can be modelled as an explicit `state_at_frame_entry` snapshot taken once per `Track()`. It is not a second state machine and nothing consumes the divergence. Uninitialized-in-ctor should be fixed regardless.

---

## 4. Per-sensor divergence

`System::eSensor`: `MONOCULAR=0, STEREO=1, RGBD=2, IMU_MONOCULAR=3, IMU_STEREO=4, IMU_RGBD=5`. "inertial" below = `{IMU_MONOCULAR, IMU_STEREO, IMU_RGBD}`.

| Concern | Visual (MONO/STEREO/RGBD) | Inertial |
|---|---|---|
| Ini-extractor selection (L741/748) | MONO only, with `(lastID-initID)<mMaxFrames` grace period after init; STEREO/RGBD never switch | IMU_MONOCULAR only, **no** grace clause; IMU_STEREO/IMU_RGBD never switch |
| Init routine (L1056) | STEREO/RGBD → `StereoInitialization` (1 frame, `N>500`); MONO → `MonocularInitialization` (two-view) | IMU_STEREO/IMU_RGBD → `StereoInitialization` + preintegration present + `‖ΔavgA‖≥0.5` unless `mFastInit`; IMU_MONOCULAR → `MonocularInitialization` + aborts if ref frame older than 1.0 s (L1638) |
| Timestamp jump >1.0 s (L983-1010) | **ignored** — visual runs keep tracking straight through a gap | `isInertial()` maps always reset or fork the map and `return` (T19/T20) |
| `OK` → immediate `LOST` (L1116-1119) | n/a | gated on inertial; **dead** (F1) |
| `RECENTLY_LOST` handling (L1136-1167) | `Relocalization()` every frame; escapes to `LOST` only after **3.0 s** *and* a failed reloc | `PredictStateIMU()` if the map is IMU-initialized (else `bOK=false` immediately); hard timeout **5.0 s** (`time_recently_lost`, ctor-hardcoded) → `LOST` regardless of matches |
| `OK→RECENTLY_LOST` on `TrackLocalMap` failure | L1313, pure state flip | L1310, **plus** `mpSystem->ResetActiveMap()` (L1307) when `!isImuInitialized() \|\| !GetIniertialBA2()` |
| KF insertion while `RECENTLY_LOST` (L1403) | never | `mInsertKFsLost` (`Settings::insertKFsWhenLost`, `Settings.cpp:436/439`, defaults **true**) |
| `NeedNewKeyFrame` c4 on `RECENTLY_LOST` (L2337) | never | **IMU_MONOCULAR only** — IMU_STEREO/IMU_RGBD deliberately excluded ("MODIFICATION_2") |
| `LOST` recovery (L1426-1443) | ≤10 KFs → reset; else new map | ≤10 KFs → reset; `!isImuInitialized()` → reset (L1433-1439); else new map |
| `TrackLocalMap` accept thresholds (L2185-2216) | `<30` inliers → fail | IMU_MONOCULAR `<15` (IMU-init) / `<50` (not); IMU_STEREO/IMU_RGBD `<15`. `RECENTLY_LOST` + `>10` inliers short-circuits to `true` for **all** sensors (L2188) |
| `mState=OK` forced externally | never | `LocalMapping::InitializeIMU` L1440 (T22) |
| `bCoarse` triangulation (`LocalMapping.cpp:466`) | never | `mbInertial && RECENTLY_LOST && GetIniertialBA2()` |
| `System::isLost()` | constant `false` (`!isImuInitialized()` short-circuit, L1337) | can return `true` in the F6 window |
| `OK_KLT` | dead | dead |

---

## 5. Entanglement with map resets

**Two mechanisms with different timing.**

| Mechanism | Effect on `mState` | Timing |
|---|---|---|
| `CreateMapInAtlas()` (L1817) | `NO_IMAGES_YET` **immediately** (L1826) + `mbCreatedMap=true` (L1854) + rebuilds `mpImuPreintegratedFromLastKF` + nulls `mpLastKeyFrame`/`mpReferenceKF` + `mLastFrame=mCurrentFrame=Frame()` | synchronous, inside `Track()` |
| `mpSystem->ResetActiveMap()` (`System.cpp:496`) | **none this frame**; sets `mbResetActiveMap` → next `System::TrackX` calls `Tracking::ResetActiveMap()` → `NO_IMAGES_YET` (L3032) | deferred by one frame |

**Every reset-adjacent site:**

| Site | Mechanism | State when `Track()` returns |
|---|---|---|
| `963` (`mbBadImu`) | deferred | unchanged (whatever it was) → next frame `NO_IMAGES_YET` |
| `980` (timestamp regression) | `CreateMapInAtlas` | `NO_IMAGES_YET` |
| `995` / `1005` (jump, pre-BA2 / pre-IMU-init) | deferred | unchanged → next frame `NO_IMAGES_YET` |
| `999` (jump, BA2 done) | `CreateMapInAtlas` | `NO_IMAGES_YET` |
| `1176` / `1179` (`LOST` branch) | both | **unreachable** (F2) |
| **`1307`** (inertial, `TrackLocalMap` failed, map not mature) | deferred, **then `mState=RECENTLY_LOST` at 1310** | `RECENTLY_LOST` — **the most entangled site**: the frame continues to completion in `RECENTLY_LOST`, updates the motion model (L1360), may insert a keyframe (L1403) and pushes a trajectory entry (L1455) **into a map that is about to be wiped**, then the reset lands one frame later |
| `1430` / `1437` (`mState==LOST`) | deferred + `return` | `LOST` — the only externally visible `LOST` window (F6) |
| `1441` (`mState==LOST`, mature map) | `CreateMapInAtlas` + `return` | `NO_IMAGES_YET` |
| `1747` (bad monocular init) | deferred + `return` | `NOT_INITIALIZED` → caught by L1067 → `return`; reset lands next frame |
| `Viewer.cpp:249` (GUI reset) | deferred, from the **viewer thread** | applied at the next `System::TrackX` |
| `System.cpp:1358` (`ChangeDataset`) | **`Tracking::ResetActiveMap()` called directly**, synchronously (`<12` KFs), else `CreateMapInAtlas()` | between frames, on the caller's thread |

**`mbCreatedMap` lifecycle:** ctor `false` → `CreateMapInAtlas` sets `true` (L1854) → read at L1024 as `!mbCreatedMap` to **skip `PreintegrateIMU()` for exactly one frame** → cleared unconditionally at L1038. That is its entire purpose: after a map fork, `mLastFrame` is a default `Frame` (timestamp 0), so preintegrating against it would integrate a bogus span.

**F10 — `Tracking::ResetActiveMap()` does not set `mbCreatedMap`, and does not rebuild `mpImuPreintegratedFromLastKF`.** `CreateMapInAtlas` does both (L1838-1842, L1854); `ResetActiveMap` (L2995-3084) does neither, while still doing `mCurrentFrame = Frame(); mLastFrame = Frame();` (L3072-3073) and leaving `mlQueueImuData` untouched. Consequence on the first frame after a deferred `ResetActiveMap` on an inertial sequence: `mState==NO_IMAGES_YET`, `mbCreatedMap==false` → L1024 runs `PreintegrateIMU()` with `mCurrentFrame.mpPrevFrame` pointing at a default-constructed `mLastFrame` (`mTimeStamp==0`), against the *pre-reset* `mpImuPreintegratedFromLastKF`. Worth verifying against a run before the refactor; the asymmetry between the two reset paths is real regardless.

**F11 — concurrency around `mState`.** `Track()` holds `pCurrentMap->mMutexMapUpdate` only from **L1041** onward, but reads/writes `mState` at L973, L1017, L1019 and L1022 **before** taking it. `LocalMapping::InitializeIMU` writes `mState=OK` under that same mutex (L1328→L1440), so the L1054-1444 region is protected, but the L973-1022 prologue races with it. `LocalMapping::Run` L204 reads `mState` with no lock at all. Additionally, `CreateMapInAtlas` calls `mpAtlas->CreateNewMap()` while `Track()` still holds the *old* map's mutex, so the guard no longer covers the new map (mitigated only because every `CreateMapInAtlas` call site returns immediately after). A State-pattern refactor should make the state object's ownership and locking explicit rather than inheriting this.

---

## Refactor checklist (what the State pattern must preserve or deliberately change)

1. Live states: 5 (`NO_IMAGES_YET`, `NOT_INITIALIZED`, `OK`, `RECENTLY_LOST`, `LOST`). Drop or reserve `SYSTEM_NOT_READY` and `OK_KLT`.
2. `LOST` is a **transient, same-call** state, never observed at frame entry (F2). Modelling it as a persistent state would resurrect L1169/L1194/L2565 with untested semantics.
3. Preserve the deferred-vs-immediate reset split, or make both synchronous — but note that T10's "reset requested, keep tracking as `RECENTLY_LOST` for the rest of the frame" is load-bearing for the current EuRoC/TUM-VI baselines.
4. Decide explicitly on F1 (`mnFramesToResetIMU==0` killing `OK→LOST`) — it is the highest-impact behavioral delta from upstream's legacy config path, and it lives in the monocular-inertial regression area already flagged in the project memory.
5. `mLastProcessedState` → an explicit frame-entry snapshot; fix the missing ctor initialization; decide whether to revive the `NOT_INITIALIZED` drawer path (uncomment `Tracking.cpp:1065`) or delete the dead `FrameDrawer` branches (F9).
6. `bool bOK;` at L1081 must become initialized or be replaced by an explicit per-state return.
7. `mlbLost` (F3) is a genuine output bug in trajectory files, independent of the refactor.

## B. System 역참조 분석 (DI 설계)

## 1. Every `mpSystem` site in `src/tracking/Tracking.cpp`

Declaration: `include/tracking/Tracking.hpp:281` — `System* mpSystem;` (protected). Initialised at `src/tracking/Tracking.cpp:47` from ctor param `System* pSys` (`Tracking.hpp:63`, `Tracking.cpp:44`).

**11 live call sites** (the user's "~10" undercounts the trajectory group: 3 live calls, not 2, plus 1 commented out).

### A. `ResetActiveMap()` — 8 sites, all inside `Tracking::Track()` (949) except the last

| file:line | Enclosing fn | Guard / calling context | Returns after? |
|---|---|---|---|
| `Tracking.cpp:963` | `Track()` | `if(mpLocalMapper->mbBadImu)` — LocalMapping raised bad-IMU flag; very first block of `Track()`, before `pCurrentMap` is even fetched | **yes** (`:964`) |
| `Tracking.cpp:995` | `Track()` | timestamp jump > 1.0 s, inertial, IMU initialised, **but** `!pCurrentMap->GetIniertialBA2()` | yes (`:1007`) |
| `Tracking.cpp:1005` | `Track()` | same jump branch, IMU **not** initialised | yes (`:1007`) |
| `Tracking.cpp:1176` | `Track()` | `mState == LOST` && `pCurrentMap->KeyFramesInMap() < 10` (else `CreateMapInAtlas()`) | yes (`:1186`) |
| `Tracking.cpp:1307` | `Track()` | `!bOK && mState==OK`, inertial, `!isImuInitialized() \|\| !GetIniertialBA2()` | **NO** — falls through, sets `mState=RECENTLY_LOST` (`:1310`) and continues the whole rest of `Track()` |
| `Tracking.cpp:1430` | `Track()` | post-KF-creation, `mState==LOST` && `KeyFramesInMap() <= 10` | yes (`:1431`) |
| `Tracking.cpp:1437` | `Track()` | `mState==LOST`, inertial, `!pCurrentMap->isImuInitialized()` | yes (`:1438`) |
| `Tracking.cpp:1747` | `CreateInitialMapMonocular()` (1681) | `medianDepth<0 \|\| pKFcur->TrackedMapPoints(1)<50` — bad monocular init | yes (`:1748`) |

`Tracking.cpp:1307` is the one that matters for any refactor: it is the only site whose semantics depend on the call being **non-blocking and deferred**. A naive "make it synchronous" would reset the map underneath the remainder of `Track()`.

### B. Trajectory saving — 3 live sites, both `SaveSubTrajectory` overloads

```
Tracking.cpp:3185  void Tracking::SaveSubTrajectory(string frames, string kf, string strFolder)
Tracking.cpp:3187      mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
Tracking.cpp:3188      //mpSystem->SaveKeyFrameTrajectoryEuRoC(...);   <-- commented out upstream
Tracking.cpp:3191  void Tracking::SaveSubTrajectory(string frames, string kf, Map* pMap)
Tracking.cpp:3193      mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
Tracking.cpp:3195      mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
```

Declared under a literal `//DEBUG` comment at `Tracking.hpp:104-106`.

---

## 2. `System::ResetActiveMap` — request/execute trace

**It is a latch, not a call.**

```
src/core/System.cpp:496-500
void System::ResetActiveMap() {
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;          // returns immediately
}
```

Flags: `mMutexReset`, `mbReset`, `mbResetActiveMap` at `include/core/System.hpp:228-230`; init `System.cpp:43`.

**Consumers** — identical "Check reset" block at the head of all three entry points, *before* `GrabImage*`:

- `System.cpp:282-296` (`TrackStereo`)
- `System.cpp:357-371` (`TrackRGBD`)
- `System.cpp:432-447` (`TrackMonocular`, with the extra `cout << "SYSTEM-> Reseting active map in monocular case"` at `:443`)

`mbReset` takes priority and clears `mbResetActiveMap` too (`:288-289/363-364/438-439`).

### Thread identity (the crucial part)

| Role | Thread |
|---|---|
| **Requester**, all 8 Tracking sites | The **tracking thread**, which is the *app's main thread* — Tracking has no thread of its own (`System.hpp:221-222`: "The Tracking thread *lives* in the main execution thread that creates the System object"). Chain: `main → SLAM.TrackMonocular → Tracking::GrabImageMonocular (Tracking.cpp:721) → Track() (949)`. |
| **Requester**, 9th site | The **Viewer thread** (`src/viz/Viewer.cpp:249`, `menuReset` button). |
| **Executor** | The **same tracking/main thread**, at `System.cpp:293 / 368 / 444`, i.e. **at the top of the *next* `Track*()` call — one frame later**. The requesting frame is abandoned (7 of 8 sites `return` immediately). |
| **Blocked-on threads** | `Tracking::ResetActiveMap` (`Tracking.cpp:2995`) then busy-waits on the **Viewer thread** (`:2998-3003`), the **LocalMapping thread** (`:3010 RequestResetActiveMap`), and the **LoopClosing thread** (`:3016`). |

So: request is asynchronous and cheap; execution is synchronous-on-the-tracking-thread, one frame deferred, and blocks on three other threads.

**Synchronous bypass path exists.** `System::ChangeDataset()` (`System.cpp:1354-1365`) calls `mpTracker->ResetActiveMap()` **directly**, no flag — invoked by the app between `Track*()` calls (`Examples/Monocular/mono_euroc.cc:182`, and 7 more). Also `System.cpp:287/362/437` call `mpTracker->Reset()` the same way.

### Two latent defects worth capturing in the P7 design (pre-existing, inherited from upstream)

1. **`mMutexReset` is held across the entire reset.** The `unique_lock` at `System.cpp:434` covers the `mpTracker->ResetActiveMap()` call at `:444`. That call busy-waits for `mpViewer->isStopped()`. If the Viewer thread is simultaneously at `Viewer.cpp:249` (`mpSystem->ResetActiveMap()`), it blocks on `mMutexReset` and therefore never reaches its `Stop()` check at `Viewer.cpp:267` → **hard deadlock**. Narrow window (requires the Reset button in that iteration), but real.
2. **`mMutexReset` also guards `mbShutDown`** (`System.cpp:504-507` `Shutdown`, `:551-554` `isShutDown`). Unrelated concerns share one lock, so `isShutDown()` from any thread can block behind a full active-map reset.

---

## 3. `System::SaveTrajectoryEuRoC` from Tracking — dead debug code

`Tracking::SaveSubTrajectory` has **zero callers**:

- in this repo: only the 2 declarations (`Tracking.hpp:105-106`) and 2 definitions (`Tracking.cpp:3185, 3191`);
- in upstream `/Users/jhpark/VSLAM/ORB_SLAM3`: identical — declarations at `include/Tracking.h:106-107`, definitions at `src/Tracking.cc:4068, 4074`, **no call sites**.

It is **not** behind `REGISTER_TIMES` or `REGISTER_LOOP` — it is unguarded dead code marked `//DEBUG`. (The genuinely `REGISTER_TIMES`-gated Tracking↔System traffic runs the *other* direction: `System::InsertRectTime/InsertResizeTime/InsertTrackTime` at `System.hpp:166-170` push into `mpTracker->vd*_ms`.)

**The coupling here is a full round trip.** `System::SaveTrajectoryEuRoC` (`System.cpp:649`) reads Tracking's own members back out:

```
System.cpp:695-697   mpTracker->mlpReferences / mlFrameTimes / mlbLost
System.cpp:705-706   mpTracker->mlRelativeFramePoses
```

So `Tracking → System → Tracking`, to consume data Tracking already owns (`Tracking.hpp:151-154`). No mutex is taken on those lists in either direction; the only reason it isn't a data race today is that this path is never executed. If it *were* called from inside `Track()`, it would iterate `mlRelativeFramePoses` mid-mutation.

---

## 4. How wide is the accidental coupling?

`mpSystem` is a raw pointer to the full `System` class, so Tracking can reach **every public member** — 26 methods across 5 unrelated concerns:

| Concern | Reachable API |
|---|---|
| **Re-entrant frame ingest** | `TrackStereo`, `TrackRGBD`, `TrackMonocular` — Tracking could recursively re-enter itself |
| Lifecycle | `Shutdown`, `isShutDown` |
| Mode | `ActivateLocalizationMode`, `DeactivateLocalizationMode` |
| Reset | `Reset`, `ResetActiveMap` ← *only one actually used* |
| I/O | `SaveTrajectoryTUM`, `SaveKeyFrameTrajectoryTUM`, `SaveTrajectoryEuRoC` ×2, `SaveKeyFrameTrajectoryEuRoC` ×2, `SaveTrajectoryKITTI`, `SaveDebugData` ← *3 used, all dead* |
| Introspection | `MapChanged`, `GetTrackingState`, `GetTrackedMapPoints`, `GetTrackedKeyPointsUn`, `GetTimeFromIMUInit`, `isLost`, `isFinished`, `GetImageScale` — **all of which just forward back to `mpTracker`** |
| Other | `ChangeDataset` (→ calls `mpTracker->ResetActiveMap()`/`CreateMapInAtlas()` synchronously), `InsertRectTime/InsertResizeTime/InsertTrackTime` (`REGISTER_TIMES`) |

**Does anything depend on that width? No.** Only `ResetActiveMap` + 3 save methods are touched. The needed surface is **1 method** (once the dead debug code is deleted) out of 26 → ~96 % of the reachable surface is accidental.

**Separate, type-level dependency that DI does not remove:** `Tracking.cpp` uses `System::eSensor` enumerators **64 times** (`System::IMU_MONOCULAR` ×41, `IMU_STEREO` ×34, `IMU_RGBD` ×33, `MONOCULAR` ×12, `STEREO` ×9, `RGBD` ×5 — overlapping counts per line). Note `Tracking::mSensor` is declared `int` (`Tracking.hpp:134`), not `System::eSensor`, so the header is already clean; this is a `.cpp`-only dependency.

**Parallel finding for the same P7 sweep:** `LocalMapping::mpSystem` (`include/mapping/LocalMapping.hpp:144`) is assigned at `src/mapping/LocalMapping.cpp:36` and **never read**. It is 100 % dead — delete the member and the `System* pSys` ctor param (`LocalMapping.hpp:47`) outright. `Viewer::mpSystem` (`Viewer.hpp:72`) is *legitimately* used (6 sites, `Viewer.cpp:110,178,183,245,249,256,259,262,263`): Viewer is the interactive front-end and System is its app facade.

---

## 5. Proposed minimal split

### 5.1 Delete, don't abstract, the trajectory half

`ITrajectorySink` should **not** be created. `SaveSubTrajectory` is dead in this repo *and* upstream, is `//DEBUG`-marked, and its only effect is to make System read Tracking's private lists back out. Delete `Tracking.hpp:104-106` and `Tracking.cpp:3185-3196`. That removes 3 of the 11 sites for free and eliminates the `Tracking → System → Tracking` cycle entirely.

*If* the project insists on keeping the hook, the ISP-clean form is (no cycle risk — needs only `class Map;` and `<string>`):

```cpp
// include/io/ITrajectorySink.hpp
namespace ORB_SLAM3 {
class Map;
class ITrajectorySink {
public:
    virtual ~ITrajectorySink() = default;
    virtual void SaveFrameTrajectory(const std::string& filename) const = 0;
    virtual void SaveFrameTrajectory(const std::string& filename, Map* pMap) const = 0;
    virtual void SaveKeyFrameTrajectory(const std::string& filename, Map* pMap) const = 0;
};
}
```
…but it still can't work without System reaching back into `mpTracker->ml*`, so it does not actually decouple. Recommend deletion.

### 5.2 `IResetRequester` + a `ResetLatch` value member

```cpp
// include/core/IResetRequester.hpp   — ZERO includes, zero forward declarations
#ifndef IRESETREQUESTER_H
#define IRESETREQUESTER_H
namespace ORB_SLAM3 {

// P7-1: the only System surface Tracking actually consumes. Naming follows the
// existing LocalMapping::RequestResetActiveMap / LoopClosing::RequestResetActiveMap
// convention: "Request" = non-blocking latch, not the work itself.
class IResetRequester
{
public:
    virtual ~IResetRequester() = default;

    // Non-blocking. Latches a request consumed by the tracking thread at the top
    // of the next System::Track*() call (System.cpp:293/368/444). Idempotent
    // within one frame interval. MUST NOT be made synchronous: Tracking.cpp:1307
    // continues executing Track() after requesting.
    virtual void RequestResetActiveMap() = 0;
};

} // namespace ORB_SLAM3
#endif
```

One method only. `System::Reset()` is external-API-only (no internal caller anywhere) — leaving it out is the ISP-correct choice and matches P6 design rule "no interface wider than the consumer".

**Implementor — follow the `BAEpochs`/`G2oBackend` precedent, not `System : public IResetRequester`:**

```cpp
// include/core/ResetLatch.hpp
#include "core/IResetRequester.hpp"
#include <mutex>
namespace ORB_SLAM3 {
class ResetLatch final : public IResetRequester
{
public:
    enum class Pending { None, Full, ActiveMap };

    void RequestResetActiveMap() override;  // IResetRequester
    void RequestFullReset();                // System::Reset() forwards here
    Pending Take();                         // read+clear under lock, then RELEASE
private:
    std::mutex mMutex;
    bool mbReset = false;
    bool mbResetActiveMap = false;
};
}
```

```cpp
// include/core/System.hpp — next to mBAEpochs (:203) / mBackend (:209)
ResetLatch mResetLatch;
```
`System::Reset()`/`System::ResetActiveMap()` (`System.cpp:490-500`) become one-line forwarders, so the public API and `Viewer.cpp:249` are untouched. `mMutexReset`/`mbReset`/`mbResetActiveMap` (`System.hpp:228-230`) are deleted; `mbShutDown` gets its own `mMutexShutdown`.

**Why the value member beats `System : public IResetRequester`:** exact `BAEpochs mBAEpochs` / `G2oBackend mBackend` precedent (`System.hpp:203/209`); no vtable added to `System`; the latch becomes unit-testable in isolation; and the `Take()`-then-release shape **fixes the deadlock in §2.1** — `System.cpp:432-447` becomes

```cpp
switch(mResetLatch.Take()) {            // lock released here
  case ResetLatch::Pending::Full:      mpTracker->Reset();          break;
  case ResetLatch::Pending::ActiveMap: mpTracker->ResetActiveMap(); break;
  case ResetLatch::Pending::None:                                   break;
}
```
Flag it as a **separate, explicitly labelled step**, not folded into the "behaviour change 0" phase: it is a real (if narrow) semantic change under Viewer interaction. Deterministic headless EuRoC ATE is unaffected.

**Injection — mirrors `src/core/System.cpp:178-180` exactly:**

```cpp
// Tracking.hpp:56 area — alongside `class ITrackingOptimizer;`
class IResetRequester;

// Tracking.hpp:63 — System* pSys replaced by IResetRequester*
Tracking(IResetRequester* pResetRequester, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer,
         MapDrawer* pMapDrawer, Atlas* pAtlas, KeyFrameDatabase* pKFDB,
         const string &strSettingPath, const int sensor, Settings* settings,
         ITrackingOptimizer* pOptimizer, const string &_nameSeq = std::string());

// Tracking.hpp:280-281 — replaces `System* mpSystem;`
IResetRequester* mpResetRequester;

// System.cpp:178
mpTracker = new Tracking(&mResetLatch, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                         mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_,
                         static_cast<ITrackingOptimizer*>(&mBackend), strSequence);
```
Plus a mechanical `mpSystem->ResetActiveMap()` → `mpResetRequester->RequestResetActiveMap()` at the 8 lines in §1A. `Tracking.cpp:26` already includes `backend/ITrackingOptimizer.hpp`; add `core/IResetRequester.hpp` beside it.

Optional same-sweep cleanups: delete `LocalMapping::mpSystem` + its `System* pSys` param (§4); switch `Viewer.cpp:249` to the latch too.

### 5.3 The include cycle — yes, this breaks it, but DI alone is not sufficient

Current cycle: `System.hpp:31 → Tracking.hpp:36 → System.hpp` and `System.hpp:39 → Viewer.hpp:25 → Tracking.hpp:26 → Viewer.hpp`.

Authoritative fact: **only two headers include `core/System.hpp`** — `Tracking.hpp:36` and `Viewer.hpp:26`. Both use `System` **only through a pointer**, and both already carry the forward declaration (`Tracking.hpp:54`, `Viewer.hpp:37`). Four steps:

1. **`Tracking.hpp:36`** — delete `#include "core/System.hpp"`. Removable *today*, before any DI; after DI even the pointer is gone.
2. **`Tracking.hpp:26`** — delete `#include "viz/Viewer.hpp"`. `Viewer` is used only as `Viewer* mpViewer` (`:284`) and `SetViewer(Viewer*)` (`:80`); fwd decl already at `:49`. `Tracking.cpp` then needs `#include "viz/Viewer.hpp"` (it calls `RequestStop`/`isStopped`/`Release` at `Tracking.cpp:2940-2942, 2990, 3000-3002, 3081`).
3. **`Viewer.hpp:26`** — delete; move to `Viewer.cpp`. Precedent already in-tree: `LocalMapping.hpp:36` holds `class System;` with no include.
4. **`Tracking.cpp`'s 64 `System::eSensor` uses** — the last tie. **Also `LoopClosing.cpp` (13 uses) currently gets `System.hpp` only transitively via `Tracking.hpp:36`** and will fail to compile after step 1 unless handled.

Two options for step 4:

- **Option A (recommended for P7-1, minimal diff):** add an explicit `#include "core/System.hpp"` to `src/tracking/Tracking.cpp` and `src/closing/LoopClosing.cpp`. The **header** cycle is fully broken; the class `Tracking` has no `System` coupling; only the TU-level include cost remains.
- **Option B (defer):** extract `include/core/SensorType.hpp` with a namespace-scope `enum eSensor { MONOCULAR=0, … }` and re-export it inside `System` — C++17 (`CMakeLists.txt:94`) makes this zero-churn for the ~120 `System::MONOCULAR`-style call sites, including all 22 `Examples/*.cc`:
  ```cpp
  class System {
  public:
      using eSensor = ORB_SLAM3::eSensor;
      static constexpr eSensor MONOCULAR     = ORB_SLAM3::MONOCULAR;
      static constexpr eSensor IMU_MONOCULAR = ORB_SLAM3::IMU_MONOCULAR;  // …6 total
  ```
  Caveat: unscoped enumerators at namespace scope pollute `ORB_SLAM3` with `MONOCULAR`/`STEREO`/`RGBD` (grep confirms no current collision).

**Does the P6-1 `ILoopOptimizer` trap recur? No — and the fix retires it.** P6-1 failed because `ILoopOptimizer.hpp` needed `LoopClosing::KeyFrameAndPose`, and `LoopClosing.hpp → Tracking.hpp → Viewer.hpp → System.hpp → G2oBackend.hpp → ILoopOptimizer.hpp` closed the loop (`docs/P6_DESIGN.md:19-25`). `IResetRequester.hpp` is structurally immune: **it includes nothing and forward-declares nothing** — there is no ORB_SLAM3 type in its signature. Better still, steps 2+3 sever `Tracking.hpp → Viewer.hpp → System.hpp`, which is the exact edge that forced the P6-1 workaround. Once that edge is gone, `ILoopOptimizer` may include `closing/LoopClosing.hpp` directly and drop the duplicated `ILoopOptimizer::KeyFrameAndPose` typedef + the narrow `g2o/types/sim3/sim3.h` include (`ILoopOptimizer.hpp:34`, comment at `:29-33`). Worth listing as a P7-1 follow-on — the comment block in `ILoopOptimizer.hpp` must be updated or it becomes stale documentation.

### 5.4 Net result

- 11 `mpSystem` call sites → **8 mechanical rewrites + 3 deletions**; member `System* mpSystem` deleted.
- Reachable System surface from Tracking: **26 methods → 1**.
- `System.hpp` includers: **2 headers + 2 sources → 0 headers + 3-4 sources**.
- Files touched: `Tracking.hpp`, `Tracking.cpp`, `System.hpp`, `System.cpp`, `Viewer.hpp`, `Viewer.cpp`, `LoopClosing.cpp`, `LocalMapping.hpp`, `LocalMapping.cpp`, + 2 new headers (`IResetRequester.hpp`, `ResetLatch.hpp`) and 1 new source (`ResetLatch.cpp`). **No `Examples/` change.**
- Gate: same as P6 Phase A — clean build log (`p*_build.log` lineage) + EuRoC ATE identical to the P6 baseline, since the deferred-latch semantics are preserved byte-for-byte (the lock-scope fix, if included, must be gated separately).

## C. IMU 큐 계약

# IMU data-flow contract in `Tracking` (as implemented, P7 prep — descriptive only)

All paths below are under `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/`. Line numbers are current HEAD.

---

## 1. `mlQueueImuData` lifecycle

**Declaration:** `include/tracking/Tracking.hpp:237` (`std::list<IMU::Point>`), guard `mMutexImuQueue` at `:241`, scratch buffer `mvImuFromLastFrame` at `:240` (tracking-thread-private, not guarded).

### Producer
- Sole writer: `Tracking::GrabImuData()` — `src/tracking/Tracking.cpp:773-777`. Takes `mMutexImuQueue`, `push_back`. No timestamp validation, no ordering enforcement, no size cap (unbounded growth if `Track()` stops draining).
- Called only from `System`, in a loop over the batch handed in by the driver:
  - `src/core/System.cpp:299-300` (`TrackStereo`, gated on `IMU_STEREO`)
  - `src/core/System.cpp:374-375` (`TrackRGBD`, gated on `IMU_RGBD`)
  - `src/core/System.cpp:449-451` (`TrackMonocular`, gated on `IMU_MONOCULAR`)
  - In all three, the push loop runs **after** the reset check block (`System.cpp:284-297 / 359-372 / 433-447`) and **immediately before** `mpTracker->GrabImage*()` (`:303 / :453`), i.e. push and drain are two statements apart.

### Threading (important, and not what the mutex suggests)
- **Every shipped driver is single-threaded with respect to this queue.** `Examples/Monocular-Inertial/mono_inertial_euroc.cc:171-185` accumulates all IMU samples with `t <= t_cam[ni]` into a local `vImuMeas`, then calls `SLAM.TrackMonocular(im, tframe, vImuMeas)` at `:195` from the same main-loop thread. `Examples/Stereo-Inertial/stereo_inertial_euroc.cc:167-176` / `:185` is identical.
- The ROS drivers are also single-threaded *at this boundary*: `Examples_old/ROS/ORB_SLAM3/src/ros_mono_inertial.cc` has a real IMU callback thread (`ImuGrabber::GrabImu`, `:186-190`) but it fills its **own** `imuBuf` under its **own** `mBufMutex`; the sync thread (`:101`, `SyncWithImu` at `:141-178`) drains that buffer into `vImuMeas` and calls `TrackMonocular` at `:178`. Same for `ros_stereo_inertial.cc`.
- **Contract consequence:** `GrabImuData` is always called on the tracking thread, before `Track()` for the frame whose interval the samples belong to. `mMutexImuQueue` is therefore currently uncontended; it exists so that a driver *may* push from a sensor-callback thread. Nothing else in the codebase assumes an asynchronous producer.

### Consumer
- Sole reader/drainer: `Tracking::PreintegrateIMU()` — `Tracking.cpp:779-890`, called from `Track()` at `:1024-1037`, gated on inertial sensor **and** `!mbCreatedMap` (so the first `Track()` after `CreateMapInAtlas()` deliberately skips preintegration; `mbCreatedMap` is cleared at `:1038`).
- Drain loop `:798-830`, lock scope `:802-827` (re-acquired every iteration). Front-only inspection, three cases:
  - `m->t < prevFrame.t - mImuPer` → **popped and discarded** (`:807-810`)
  - `m->t < curFrame.t - mImuPer` → **popped and copied** into `mvImuFromLastFrame` (`:811-815`)
  - otherwise → **copied but NOT popped**, then `break` (`:816-820`). This first "future" sample is intentionally kept in the queue: it is the right interpolation endpoint of this interval and becomes the left endpoint of the next.
- `mImuPer` is hard-coded to `0.001` at `:570` (the `1.0/mImuFreq` form is commented out with a `TODO`), so the two thresholds are "frame timestamp minus 1 ms".
- Integration `:840-881`: `n = size-1` intervals, endpoints linearly extrapolated to `prevFrame.mTimeStamp` (`i==0`, `:844-853`) and `curFrame.mTimeStamp` (`i==n-1`, `:860-869`); each sample is fed to **both** objects (`:879`, `:880`).
- Publication `:883-885`: `mpImuPreintegratedFrame`, `mpImuPreintegrated`, `mpLastKeyFrame` on `mCurrentFrame`, then `setIntegrated()` at `:887` (the handshake `LocalMapping` waits on, see §5).

### Out-of-order / late measurements
- **Late (arrives after its frame was processed):** it is simply absent from that frame's interval. On the next frame it is at the queue front and is *dropped* by `:807-810` if it predates the new previous-frame timestamp. There is no wait-for-late-IMU: the `bSleep` retry is dead (see B3).
- **Out of order (older sample pushed behind a newer one):** the queue is never sorted and only the front is inspected, so the drain `break`s at the newer sample and leaves the older one behind it. On a subsequent frame the stale sample is either discarded by `:807-810`, or — if it still falls inside the new interval — appended to `mvImuFromLastFrame` **out of order**, producing a negative `tstep` at `:846-874`. `Preintegrated::IntegrateNewMeasurement` (`src/backend/ImuTypes.cpp:177-235`) has no `dt` guard, so this silently corrupts `dT`, `dP`, `dV`, `avgA` and the covariance. **Timestamp monotonicity of the pushes is an unchecked precondition of the contract.**
- **Duplicate timestamps** produce `tstep == 0` and a division by `tab == 0` at `:846-851` / `:862-866`.

### Clearing
- The queue is cleared in exactly one place: `Track():978-979`, under the lock, on the "frame older than previous frame" branch, immediately before `CreateMapInAtlas()`.
- It is **not** cleared by `Reset()`, `ResetActiveMap()`, `CreateMapInAtlas()`, or the timestamp-jump branch (`:983-1008`). See §4.

---

## 2. The two preintegration objects

| | `Frame::mpImuPreintegratedFrame` (from last **frame**) | `Frame::mpImuPreintegrated` / `Tracking::mpImuPreintegratedFromLastKF` (from last **KF**) |
|---|---|---|
| Declared | `include/map/Frame.hpp:268` | `Frame.hpp:263`, `Tracking.hpp:234` |
| Created | `Tracking.cpp:838`, one per frame, bias = `mLastFrame.mImuBias`, calib = `mCurrentFrame.mImuCalib` | `Tracking.cpp:579, 1511, 1626, 1774, 1841, 2399` |
| Owner | `Frame` value member — but `Frame` has **no destructor** (`Frame.hpp:71` commented out) and the copy ctor aliases the pointer (`src/map/Frame.cpp:59`) | `Tracking` owns it until a `KeyFrame` is built from the frame that carries it; `KeyFrame::KeyFrame(Frame&)` copies the raw pointer (`src/map/KeyFrame.cpp:54`) |
| Deleted | **never** (see B4) | `Tracking.cpp:1509, 1624, 1840` only |
| Handed to backend | `Optimizer::PoseInertialOptimizationLastFrame` (`src/backend/Optimizer.cpp:5017`), edge built at `:5200`; also read in `Tracking.cpp:925-929` (`PredictStateIMU`, `!mbMapUpdated` branch) and `:1502` (stereo-init `avgA` gate) | `Optimizer::PoseInertialOptimizationLastKeyFrame` (`:4633`), edge at `:4814`; per-KF copies feed `LocalInertialBA` (`:617, :658`), `FullInertialBA` (`:2747, :2765`), `InertialOptimization` (`:3284-3302, :3458-3473, :3611`), merge BA (`:4341-4359`); `LocalMapping` merges them on KF culling (`src/mapping/LocalMapping.cpp:1032, :1041`) and reads them in `InitializeIMU` (`:1239-1245`) |

**Ownership-transfer points (the from-KF object):**
- Monocular init — `Tracking.cpp:1772-1774`: `pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;` then the member is re-pointed at a fresh object seeded with `GetUpdatedBias()`. `pKFini`'s copy is explicitly nulled at `:1688` to avoid two KFs claiming one object.
- Steady state — `CreateNewKeyFrame():2379` builds the KF (which copies the pointer via `KeyFrame.cpp:54`), then `:2399` re-points the member at a fresh object seeded with `pKF->GetImuBias()`. **No `delete` here, and none is correct**: ownership moved to the KF.
- Stereo/RGBD init — `Tracking.cpp:1511-1512` creates the object and assigns it to `mCurrentFrame` only; `pKFini` at `:1528` then copies that same pointer while `mpImuPreintegratedFromLastKF` still holds it. This is the one window where Tracking and a live `KeyFrame` alias the same object (see B7).
- KFs are never freed: `Map::clear()` (`src/map/Map.cpp:214-234`) tombstones instead of deleting, and `KeyFrame` has no destructor — consistent with `docs/OWNERSHIP.md`. So per-KF preintegrations live for the process lifetime by design.

**Bias coupling:** `Frame::SetNewBias` (`Frame.cpp:421-425`) forwards to `mpImuPreintegrated->SetNewBias`. At `Track():1015` this is a no-op on the object because `mCurrentFrame.mpImuPreintegrated` is still `NULL` there (`PreintegrateIMU` only publishes it at `:884`, later, at `:1029`). At `UpdateFrameIMU():3125-3126` it is **not** a no-op and mutates the shared from-KF object from the LocalMapping thread (see B12).

---

## 3. Every `new IMU::Preintegrated` / `delete` in Tracking

| Site | What | Paired delete? |
|---|---|---|
| `Tracking.cpp:579` (`newParameterLoader`) | initial `mpImuPreintegratedFromLastKF`, zero bias | only if a later reset path frees it; **leaks at shutdown** (`~Tracking()` at `:491-495` is empty; `mpImuCalib` from `:577` leaks too) |
| `Tracking.cpp:838` (`PreintegrateIMU`) | per-frame from-last-frame object | **never deleted — one leak per frame** |
| `Tracking.cpp:1331` (`Track`, "save frame for IMU reset") | `new IMU::Preintegrated(...)` plus `new Frame` ×2 at `:1327-1328`, all stored in a **local** `pF` | **immediate leak**; currently unreachable because `mnFramesToResetIMU == 0` (see §5) |
| `Tracking.cpp:1509 → 1511` (`StereoInitialization`) | delete + recreate from-KF object | paired |
| `Tracking.cpp:1624 → 1626` (`MonocularInitialization`) | delete + recreate | paired |
| `Tracking.cpp:1774` (`CreateInitialMapMonocular`) | recreate after handing the old one to `pKFcur` | no delete — correct, ownership moved |
| `Tracking.cpp:1840 → 1841` (`CreateMapInAtlas`) | delete + recreate | paired |
| `Tracking.cpp:2399` (`CreateNewKeyFrame`) | recreate after handing the old one to `pKF` | no delete — correct **iff** the KF actually received it (see B6) |

The only other `delete` in the file is `:1392` (temporal `MapPoint`s), unrelated.

---

## 4. Reset paths

| Path | Queue | `mpImuPreintegratedFromLastKF` | `mCurrentFrame` / `mLastFrame` |
|---|---|---|---|
| `Reset()` `:2934-2993` | **untouched** | **untouched** (stale object survives a full atlas reset) | both `= Frame()` (`:2982, :2984`) |
| `ResetActiveMap()` `:2995-3084` | **untouched** | **untouched** | both `= Frame()` (`:3072-3073`); `mnLastRelocFrameId = mCurrentFrame.mnId` at `:3070` (read **before** the reassignment) |
| `CreateMapInAtlas()` `:1817-1855` | **untouched** | `delete` + fresh zero-bias object `:1838-1842` | both `= Frame()` (`:1850-1851`); sets `mbCreatedMap = true` (`:1854`) so the next `Track()` skips `PreintegrateIMU` |
| `Track():975-982` (timestamp goes backwards) | **cleared** under lock `:978-979` | via the `CreateMapInAtlas()` that follows | via `CreateMapInAtlas()` |
| `Track():983-1008` (>1 s jump) | **not** cleared | only if the branch routes to `CreateMapInAtlas()` (`:999`); the `ResetActiveMap()` routes (`:995, :1005`) leave both stale | — |
| IMU re-initialization (`LocalMapping::InitializeIMU` / `ScaleRefinement`) | untouched | untouched — objects are *rebiased/reintegrated in place* (`Optimizer.cpp:3359, :3524`) and rescaled via `UpdateFrameIMU` (`Tracking.cpp:3097`) | `UpdateFrameIMU` rewrites poses/velocities of both frames |

**Stale-consumption windows (descriptive, not a proposal):**
1. After `ResetActiveMap()` the queue still holds samples from the old map and `mpImuPreintegratedFromLastKF` still holds the old map's accumulated deltas. `mbCreatedMap` is **not** set, so the very next `Track()` runs `PreintegrateIMU` (`:1029`) and integrates those samples into the stale object (`:879`). The window closes only when `MonocularInitialization():1624-1626` or `StereoInitialization():1509-1511` deletes and recreates it. Since `mState` is `NO_IMAGES_YET`→`NOT_INITIALIZED` throughout, no `KeyFrame` and no backend edge can consume the stale accumulation — but the object handed to `mCurrentFrame` at `:884` during that window is stale, and `PredictStateIMU():908-912` would read its `dT` if it were reachable.
2. `CreateInitialMapMonocular()` can bail at `:1744-1749` (`ResetActiveMap`) **after** `pKFcur` has already copied `mCurrentFrame.mpImuPreintegrated` and **before** the ownership transfer at `:1772-1774`, leaving the member and a (tombstoned) KF aliasing one object until `MonocularInitialization():1624` deletes it.
3. The stereo-init alias window described in §2 / B7.

---

## 5. `mnFramesToResetIMU`, `mnFirstImuFrameId`, `mpImuPreintegratedFromLastKF`

- `mnFramesToResetIMU` is declared `int mnFramesToResetIMU = 0;` at `Tracking.hpp:308` and is **never assigned anywhere** (verified repo-wide). Upstream's legacy flat-YAML branch set it to `mMaxFrames` (= fps); upstream's new-`Settings` branch left it uninitialized. The explicit `= 0` here removes the UB and pins the value, as documented at `Tracking.cpp:51-56`. With `0` the five read sites behave as:
  - `Track():1116` — `mnId <= mnLastRelocFrameId` → only the reloc frame itself goes `LOST` instead of `RECENTLY_LOST`.
  - `Track():1322` — `mnId < mnLastRelocFrameId + 0 && mnId > 0` is unsatisfiable → the "save frame, imu needs reset" block (`:1326-1332`) is **dead code**, which is what currently suppresses its triple leak.
  - `Track():1338` — fires exactly on `mnId == mnLastRelocFrameId`, calling `ResetFrameIMU()`, which is an **empty stub** (`:943-946`, `// TODO To implement...`). So the whole "reset frame IMU after relocalization" mechanism is inert.
  - `TrackWithMotionModel():2017` — IMU-only prediction is accepted from one frame after relocalization instead of after `fps` frames.
  - `TrackLocalMap():2129` — the inertial pose optimizers are used immediately after relocalization instead of falling back to visual `PoseOptimization` for `fps` frames.
- `mnFirstImuFrameId` (`Tracking.hpp:307`) is **written once** (`:3167`, end of `UpdateFrameIMU`) and **never read** — dead member. It is not the pairing partner of `mnFramesToResetIMU` any more.
- `mpImuPreintegratedFromLastKF` and `mnFramesToResetIMU` therefore no longer interact at all: the only code that would have re-seeded a preintegration on a relocalization-triggered IMU reset is the dead block at `:1326-1332` plus the empty `ResetFrameIMU()`. The member's lifetime is driven solely by KF creation (`:2399`), map creation (`:1841`), and the two init paths (`:1511`, `:1626`, `:1774`).
- `UpdateFrameIMU` (`:3097-3168`), called from `LocalMapping::InitializeIMU` at `src/mapping/LocalMapping.cpp:1290` (inside `mMutexMapUpdate`) and `:1301` (**outside** it), and from `ScaleRefinement` at `:1498` (inside), is the only cross-thread mutator of Tracking's IMU state: it rescales `mlRelativeFramePoses`, sets `mLastBias`/`mpLastKeyFrame`, rebiases both frames, blocks on `mCurrentFrame.imuIsPreintegrated()` (`:3128-3131`), and rewrites both frames' pose/velocity from the from-KF preintegration.

---

## Latent bugs found (file:line, no fixes proposed)

**B1 — `Tracking.cpp:833-836`:** the `n == 0` path (`"Empty IMU measurements vector!!!"`) returns **without** calling `mCurrentFrame.setIntegrated()`, unlike the other two early exits (`:785`, `:794`). That frame's `mbImuPreintegrated` stays `false` forever, so `UpdateFrameIMU():3128-3131` can spin in `while(!mCurrentFrame.imuIsPreintegrated()) usleep(500);` until the tracking thread overwrites `mCurrentFrame`.

**B2 — `Tracking.cpp:790-791`:** `mlQueueImuData.size()` is read twice **outside** `mMutexImuQueue` (the lock is only taken at `:802`). Benign today (single-threaded producer), a real race the moment a driver pushes from a callback thread.

**B3 — `Tracking.cpp:824-825`:** `break;` followed by `bSleep = true;` — the assignment is unreachable, so `if(bSleep) usleep(500)` at `:828-829` can never fire. The wait-for-late-IMU behaviour the code appears to implement does not exist.

**B4 — `Tracking.cpp:838` + `Frame.hpp:71` + `Frame.cpp:59`:** the per-frame `IMU::Preintegrated` is allocated once per frame and never deleted anywhere in the repo (`grep mpImuPreintegratedFrame` finds no `delete`). `Frame`'s destructor is commented out and its copy ctor aliases the pointer, so there is no owner. Steady leak proportional to frame count in all three inertial modes. (`Frame::mpMutexImu`, `new`'d at `Frame.cpp:179, 261, 362, 1095`, leaks the same way.)

**B5 — `Tracking.cpp:1327-1331`:** `new Frame` ×2 + `new IMU::Preintegrated` stored only in the local `pF`; leaked on every execution. Currently unreachable only because `mnFramesToResetIMU == 0` makes the `:1322` guard unsatisfiable.

**B6 — `Tracking.cpp:2399` (with `KeyFrame.cpp:54`):** the no-delete re-point is only safe when `mCurrentFrame.mpImuPreintegrated == mpImuPreintegratedFromLastKF`, i.e. when `PreintegrateIMU` reached `:884`. If it took any early exit (`:782-787`, `:791-796`, `:833-836`), `mCurrentFrame.mpImuPreintegrated` is still `NULL`, so (a) the accumulated from-KF object is orphaned and leaked at `:2399`, and (b) the new `KeyFrame` gets a `NULL mpImuPreintegrated` while still having `mPrevKF` set (`:2390`) — which `LocalMapping.cpp:1032` and `:1041` (`pKF->mNextKF->mpImuPreintegrated->MergePrevious(...)`, no null check) and `Optimizer.cpp:617` (guarded only by `bImu` flags) will dereference.

**B7 — `Tracking.cpp:1509 / 1624 / 1840` vs `1511-1512 + 1528`:** after `StereoInitialization`, `pKFini->mpImuPreintegrated` and `mpImuPreintegratedFromLastKF` alias the same object until the first `CreateNewKeyFrame():2399`. Any `delete` reached inside that window — most plausibly `CreateMapInAtlas():1840`, reachable from `Track():980` (timestamp went backwards) or `:999` (>1 s jump) — frees an object still reachable from a `KeyFrame` that is in the Atlas and possibly in `LocalMapping`'s queue. `KeyFrame` has no destructor, so nothing else ever frees it either. (Mono init avoids this via the explicit transfer at `:1772-1774`; `InitializeIMU` needs 10 KFs (`LocalMapping.cpp:1148-1153`), so the backend cannot touch it in that window — but `LocalMapping`'s culling/merge paths and `Release()`/`InitializeIMU` KF deletion at `LocalMapping.cpp:873, 1436, 1505` can.)

**B8 — `Tracking.cpp:1612-1613` + `:1624`:** `mInitialFrame` and `mLastFrame` are copied from `mCurrentFrame` *before* `delete mpImuPreintegratedFromLastKF`, so both retain a dangling `mpImuPreintegrated`. `mLastFrame` is overwritten a few statements later (`Track():1069`) and `mInitialFrame.mpImuPreintegrated` is never read, so it is not currently dereferenced — but the dangling alias is real. Same shape at `StereoInitialization:1509` w.r.t. `mLastFrame` (repaired at `:1582`).

**B9 — `Tracking.cpp:2934-2993` and `:2995-3084`:** neither `Reset()` nor `ResetActiveMap()` clears `mlQueueImuData` or re-seeds `mpImuPreintegratedFromLastKF`, while both null `mpLastKeyFrame`/`mpReferenceKF` and default-construct both frames. Only the `:978-979` branch clears the queue and only `CreateMapInAtlas()` re-seeds the preintegration.

**B10 — `src/map/Frame.cpp:41-47`:** the default `Frame()` ctor does **not** initialize `mpMutexImu` (`Frame.hpp:323`, no in-class initializer), unlike the four real ctors (`:179, :261, :362, :1095`). `Tracking.cpp:3128` calls `mCurrentFrame.imuIsPreintegrated()`, which locks `*mpMutexImu` (`Frame.cpp:1003-1007`). Default frames are installed at `Tracking.cpp:1850-1851`, `:2982-2984`, `:3072-3073`, so a `UpdateFrameIMU` arriving in that window locks an indeterminate pointer.

**B11 — `Tracking.cpp:807/811` after `ResetActiveMap():3073`:** the same default `Frame()` also leaves `mTimeStamp` (`Frame.hpp:199`) indeterminate. `ResetActiveMap` does **not** set `mbCreatedMap`, so the next `Track()` runs `PreintegrateIMU` with `mCurrentFrame.mpPrevFrame == &mLastFrame` (non-null, so the `:782` guard does not fire) and compares queue timestamps against that indeterminate value. (`CreateMapInAtlas` is immune because `mbCreatedMap = true` at `:1854` skips one preintegration.)

**B12 — `src/backend/ImuTypes.cpp:177` vs `:263`:** `IntegrateNewMeasurement` takes **no** lock on `Preintegrated::mMutex` while `SetNewBias`/`GetDelta*` do. `Tracking.cpp:879` integrates into `mpImuPreintegratedFromLastKF` on the tracking thread; `UpdateFrameIMU():3125-3126` calls `SetNewBias` on frames aliasing that same object from the LocalMapping thread, and the call site `LocalMapping.cpp:1301` is **outside** `mMutexMapUpdate` (unlike `:1290` and `:1498`). Data race on `b`/`db` concurrent with `dR/dV/dP/C` updates.

**B13 — `src/backend/Optimizer.cpp:5200` vs `:5213, :5220`:** `PoseInertialOptimizationLastFrame` builds the `EdgeInertial` from `pFrame->mpImuPreintegratedFrame` (frame-to-frame interval) but takes the gyro/acc random-walk information blocks from `pFrame->mpImuPreintegrated->C` (KF-to-frame interval). Upstream asymmetry, preserved here; noting it because the two objects are not interchangeable and any P7 refactor that unifies them changes the optimizer weights.

**B14 — `Tracking.cpp:3167` / `Tracking.hpp:307`:** `mnFirstImuFrameId` is write-only (dead member).

**B15 — `Tracking.cpp:943-946` + `:1338-1342`:** `ResetFrameIMU()` is an empty stub, so the `"RESETING FRAME!!!"` path performs no reset.
