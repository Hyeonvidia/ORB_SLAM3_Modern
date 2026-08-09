# P10 정찰: 스레딩 현대화 — 동기화 전수 지도 · 이행 설계 · 검증 전략

2026-08-10, 3-agent 병렬 정찰 종합. phase-9 (ac72110) 기준.
아래 3부는 에이전트 원문 그대로 보존한다 (수치·file:line 전부 현행 트리 검증됨).

**요지**:
- 신규 발견 **R6 ABBA 데드락**: `LocalMapping::Release()`가 Stop→Finish 순서,
  `SetFinish()`는 Finish→Stop — 오늘도 발생 가능. P10-2에서 해소.
- **C++17로 충분** — jthread/stop_token은 편의일 뿐, atomic_ref(C++20)는 g2o의
  평문 `*_forceStopFlag` 읽기와 공존 시 여전히 데이터 레이스라 건전성 기각.
- `mbAbortBA`/`mbStopGBA`의 `bool*` 파이프라인은 **OrbLevenberg 섀도 브리지**로
  해결 (atomic 원본 → solve() 진입/탈출 시 섀도 갱신 → g2o에는 섀도 포인터).
  서브모듈 무수정 원칙 유지.
- **TSAN 컨테이너 실증 완료** (gcc11 libtsan0 기설치, arm64 네이티브, 레이스
  검출 확인). 성공 기준은 TSAN-clean이 아니라 **시그니처 대장 델타**(툼스톤
  계약의 154 사이트는 소거 불가 잡음 — 단계가 지운다고 주장한 시그니처의 소멸
  + 신규 시그니처 0).
- 6 게이트 커밋: P10-1 abort 플래그 atomic화(순수 안전) → P10-2 원자화/락 폐쇄
  일괄(R1/R3/R-b/R5/R-a/R-c/R-d/R-f/R6) → P10-3 R2 단독(EmptyQueue 순서) →
  P10-4 LM 큐 CV(**케이던스 변경 스텝** — ATE 민감) → P10-5 LC CV+GBA 수명+
  Shutdown 조인 순서 복원 → P10-6 Viewer DI(IViewerHost, RequestShutdown 래치).
  모든 UB를 대기 구조 변경 **전에** 제거 → P10-4/5의 게이트 회귀는 순수 타이밍
  으로만 이분된다.
- 사전 캡처(P10-0): TSAN 기준 대장(T1/T2/T3) + 골든 큐 트레이스(env-gated
  계측 추가 후 phase-9 동작에서 채취).

---

# ═══ 1부: 동기화 전수 지도 (sync-inventory) ═══

All line numbers verified against the current working tree (post-phase-9). Thread key: **T** = Tracking (caller thread of `System::Track*`), **LM** = LocalMapping::Run, **LC** = LoopClosing::Run, **GBA** = transient LoopClosing::RunGlobalBundleAdjustment, **V** = Viewer::Run, **S** = System API caller (usually same thread as T).

---

# P10 Thread/Synchronization Inventory — ORB_SLAM3_Modern

## (1) Threads: creation, loops, poll intervals, exit protocols

### Creation / join / detach sites

| Thread | Created | Joined | Detached | Notes |
|---|---|---|---|---|
| LM | `src/core/System.cpp:188` (`new thread(&LocalMapping::Run, ...)`) | never | never | `System.hpp:233` raw ptr; no `~System()`; leaks at process exit |
| LC | `src/core/System.cpp:202` | never | never | `System.hpp:234` |
| V | `src/core/System.cpp:221` (only if `bUseViewer`) | never | never | `System.hpp:235`; `mpTracker->SetViewer` :222 and `mpViewer->both = ...` :223 execute AFTER launch (see §3 startup races) |
| GBA | `src/closing/LoopClosing.cpp:537` (CorrectLoop spawn), `:1077` (MergeLocal respawn) | P9-3 reap only: `:531-536` and `:1071-1076` (join+delete completed predecessor immediately before respawn) | abort path: `detach(); delete; mpThreadGBA=NULL` at `:324-329` (CorrectLoop), `:566-571` (MergeLocal), `:1107-1112` (MergeLocal2) | `LoopClosing.hpp:185`. A GBA that completes and is never respawned is never joined (Shutdown wait is commented out) |
| T | not created — lives in the thread that calls `System::TrackStereo/TrackRGBD/TrackMonocular` (`System.hpp:231-232`) | — | — | |
| transient workers | `Frame.cpp:123-124`, `:1042-1043` (stereo extraction, join immediately); `TwoViewReconstruction.cpp:109-110` | immediate | — | intra-Tracking, not P10 targets |

### Top-level loops, poll intervals, exit

**LocalMapping::Run** `src/mapping/LocalMapping.cpp:64-274`
- Entry: `mbFinished = false` at `:66` **without mMutexFinish** (LM-side analogue of LC race R-a).
- Iteration: `SetAcceptKeyFrames(false)` :71 → if `CheckNewKeyFrames() && !mbBadImu` :74 process one KF (:83-242, hands KF to LC at :242) → else if `Stop() && !mbBadImu` :251 **park loop** `while(isStopped() && !CheckFinish()) usleep(3000)` :254-257, break if CheckFinish :258 → `ResetIfRequested()` :262 → `SetAcceptKeyFrames(true)` :265 → break if CheckFinish :267 → **poll `usleep(3000)`** :270.
- Exit: `SetFinish()` :273 = `mbFinished=true` + `mbStopped=true` under mMutexFinish+mMutexStop (`:1130-1136`).
- Protocol fns: RequestStop `:802-808` (mbStopRequested under mMutexStop, then mbAbortBA=true under mMutexNewKFs), Stop `:810-821`, isStopped `:823-827`, stopRequested `:829-833`, Release `:835-848` (**drains queue with `delete` while holding only mMutexStop+mMutexFinish — R1**), SetNotStop `:862-872`, InterruptBA `:874-877` (**lock-free**), RequestReset `:1033-1052` (caller spins 3ms), RequestResetActiveMap `:1054-1074` (3ms), ResetIfRequested `:1076-1116`, RequestFinish `:1118-1122`, CheckFinish `:1124-1128`, isFinished `:1138-1142`, PurgeNewKeyFramesAfterInertialInit `:1144-1156` (**lock-free queue mutation, R3**), IsInitializing `:1160-1163` (**lock-free**), GetCurrKFTime `:1166-1174` (**lock-free mpCurrentKeyFrame deref**).

**LoopClosing::Run** `src/closing/LoopClosing.cpp:95-274`
- Entry: `mbFinished = false` at `:97` **without mMutexFinish** (R-a).
- Iteration: `CheckNewKeyFrames()` :106 → PlaceRecognition detection (:112) → merge-before-loop consumption (:122-196 merge, :198-258 loop; scale-abort `continue` at :148 = D1 shape) → `ResetIfRequested()` :264 → break if CheckFinish :266 → **poll `usleep(5000)`** :270.
- Exit: `SetFinish()` :273 = mbFinished=true only (`:1608-1612`) — unlike LM it does NOT set a stopped flag.
- Protocol fns: InsertKeyFrame `:276-281` (drops KF id 0), CheckNewKeyFrames `:283-287`, PopNewKeyFrame `:293-305` (+`SetNotErase` latch), RequestReset `:1341-1357` (spins 5ms), RequestResetActiveMap `:1359-1376` (3ms), ResetIfRequested `:1378-1414` (**mutates queue under mMutexReset, not mMutexLoopQueue — R-b**), RequestFinish `:1596-1600`, CheckFinish `:1602-1606`, isFinished `:1614-1618` (no live caller — Shutdown wait commented).

**Viewer::Run** `src/viz/Viewer.cpp:58-280`
- Entry: `mbFinished=false; mbStopped=false` :60-61 **without locks**.
- Iteration: Pangolin render; paced by `cv::waitKey(mT)` :235 (mT = 1e3/fps, :44) — no usleep in the main path. Menu handling :176-265 calls into System/Tracking (§4). Park: `if(Stop()) while(isStopped()) usleep(3000)` :267-273. Break on CheckFinish :275.
- Exit: `SetFinish()` :279 (`:294-298`).
- Protocol fns: RequestFinish `:282-286`, CheckFinish `:288-292`, isFinished `:300-304`, RequestStop `:306-311`, isStopped `:313-317`, Stop `:319-335` (finish wins over stop), Release `:337-341`.

**LoopClosing::RunGlobalBundleAdjustment** `src/closing/LoopClosing.cpp:1416-1594` (one-shot, no loop)
- Optimizes with `&mbStopGBA` polled raw by the optimizer (:1436/:1438 — R-e).
- Epoch check: first read of `mnFullBAIdx` **lock-free** at :1452 (R-d); recheck under mMutexGBA :1459-1461. **Early returns at :1460-1461 and :1463-1464 skip `:1591-1592`, leaving `mbRunningGBA=true` forever** (stale-true; abort path never resets it either).
- If not stopped: `RequestStop()` :1471 then **finish-aware wait** `while(!isStopped() && !isFinished()) usleep(1000)` :1474-1477, apply under `pActiveMap->mMutexMapUpdate` :1480, `mpLocalMapper->Release()` :1577, then `mbFinishedGBA=true; mbRunningGBA=false` :1591-1592 under mMutexGBA.

**System::Shutdown** `src/core/System.cpp:502-549` — current state:
- `:504-507` set `mbShutDown=true` under **mMutexReset**.
- `:511-512` `mpLocalMapper->RequestFinish(); mpLoopCloser->RequestFinish()` — fire and forget.
- `:513-518` Viewer `RequestFinish` + `isFinished` wait (`usleep(5000)` :517) — **entirely commented out** (Viewer thread is never asked to exit).
- `:520-533` the LM/LC/GBA wait loop (`isFinished()`/`isRunningGBA()`, `usleep(5000)` :532) — **entirely commented out**.
- `:535-539` `SaveAtlas` runs immediately → live data race with still-running LM/LC/GBA.
- No thread joins anywhere; no `~System`.
- `mbShutDown` effects: early-return gate **only in TrackMonocular** `:389-393`; `isShutDown()` `:551-554` (used by realsense example main loops). **TrackStereo/TrackRGBD have no shutdown gate** (asymmetry).
- Shutdown callers: example `main`s (T thread) and **Viewer menuStop `Viewer.cpp:259`** (V thread calls Shutdown then SaveTrajectoryEuRoC :262-263 while T may still be tracking).

## (2) Mutex inventory (per class: guards → takers)

### System (`include/core/System.hpp`)
| Mutex | Guards | Takers |
|---|---|---|
| `mMutexReset` :238 | `mbReset`, `mbResetActiveMap`, `mbShutDown` | T (Track* reset-check `System.cpp:284-296/:358-371/:433-447`, mono shutdown gate :389-393); V via `Reset`/`ResetActiveMap` :490-500; Shutdown writer :504-507 (T or V); `isShutDown` :551-554 |
| `mMutexMode` :243 | `mbActivateLocalizationMode`, `mbDeactivateLocalizationMode` | T (Track* mode-check :259-280/:333-355/:409-430 — **held across the spin-wait on LM stop**); V via Activate/DeactivateLocalizationMode :465-475 |
| `mMutexState` :254 | `mTrackingState`, `mTrackedMapPoints`, `mTrackedKeyPointsUn` | T (Track* tail :307-311/:379-383/:455-458); any caller of GetTrackingState/GetTrackedMapPoints/GetTrackedKeyPointsUn :1246-1268 |

### LocalMapping (`include/mapping/LocalMapping.hpp`)
| Mutex | Guards | Takers |
|---|---|---|
| `mMutexNewKFs` :184 | `mlNewKeyFrames`; *partially* `mbAbortBA` | T InsertKeyFrame `LocalMapping.cpp:276-281`, KeyframesInQueue `hpp:107-110`; LM CheckNewKeyFrames :284-288, ProcessNewKeyFrame pop :292-296; LC via EmptyQueue :332-336; LC/GBA/S RequestStop 2nd lock :806-807. **Not taken by**: Release drain :843-845 (R1), PurgeNewKeyFramesAfterInertialInit :1146-1155 (R3), ResetIfRequested clears :1086/:1101 (protocol-protected), InterruptBA :876, Run's `mbAbortBA=false` :103 |
| `mMutexStop` :191 | `mbStopped`, `mbStopRequested`, `mbNotStop` | LM Stop/isStopped/stopRequested :810-833, SetFinish 2nd lock :1134-1135; T SetNotStop :862-872 + NeedNewKeyFrame reads `Tracking.cpp:2331`; LC RequestStop×4 (`LoopClosing.cpp:312/:575/:1010/:1116`); GBA :1471; Release :837 (LC :541/:942/:1062/:1268/:1278/:1577, T-System :277/:352/:427) |
| `mMutexAccept` :194 | `mbAcceptKeyFrames` | LM SetAcceptKeyFrames :856-860 (Run :71/:265); T AcceptKeyFrames :850-854 (NeedNewKeyFrame `Tracking.cpp:2354`) |
| `mMutexReset` :158 | `mbResetRequested`, `mbResetRequestedActiveMap`, `mpMapToReset` (+ queue/mTinit/mbBadImu cleared while held) | T RequestReset* :1033-1074 (spin); LM self-request Run :141-145; LM ResetIfRequested :1076-1116 |
| `mMutexFinish` :164 | `mbFinishRequested`, `mbFinished` | S RequestFinish :1118-1122; LM CheckFinish/SetFinish/isFinished :1124-1142; Release early-return :838-840; GBA isFinished :1474. **Run entry write :66 bypasses it** |

### LoopClosing (`include/closing/LoopClosing.hpp`)
| Mutex | Guards | Takers |
|---|---|---|
| `mMutexLoopQueue` :160 | `mlpLoopKeyFrameQueue` (+ `mpCurrentKF`/`mpLastMap` set during pop) | LM InsertKeyFrame `LoopClosing.cpp:276-281`; LC CheckNewKeyFrames :283-287, PopNewKeyFrame :293-305. **Not taken by** ResetIfRequested mutations :1389/:1400-1409 (R-b) |
| `mMutexReset` :134 | `mbResetRequested`, `mbResetActiveMapRequested`, `mpMapToReset` (+ queue mutation, R-b) | T RequestReset* :1341-1376 (spin); LC ResetIfRequested :1378-1414 |
| `mMutexFinish` :140 | `mbFinishRequested`, `mbFinished` | S RequestFinish :1596-1600; LC CheckFinish/SetFinish/isFinished :1602-1618. **Run entry write :97 bypasses it (R-a)** |
| `mMutexGBA` :184 | `mbRunningGBA`, `mbFinishedGBA`, `mbStopGBA`, `mnFullBAIdx`, `mpThreadGBA` | LC abort blocks :319-329/:561-571/:1102-1112, `isRunningGBA` `hpp:72-75`; GBA epoch-recheck/apply/completion :1459-1593. **Not held for**: spawn writes :523-525 + :531-537, :1067-1077 (R-c); GBA first epoch read :1452 (R-d); optimizer polling `&mbStopGBA` :1436/:1438 (R-e) |

### Viewer (`include/viz/Viewer.hpp`)
| Mutex | Guards | Takers |
|---|---|---|
| `mMutexFinish` :88 | `mbFinishRequested`, `mbFinished` | V CheckFinish/SetFinish :288-298, Stop 2nd lock :322; S RequestFinish/isFinished **dead** (Shutdown block commented). **Run entry writes :60-61 bypass it** |
| `mMutexStop` :92 | `mbStopped`, `mbStopRequested` | T RequestStop/isStopped/Release `Viewer.cpp:306-341` (from `Tracking.cpp:3036-3038/:3086/:3096-3098/:3177`); V Stop :319-335 + park isStopped :269 |

### Tracking (`include/tracking/Tracking.hpp`)
| Mutex | Guards | Takers |
|---|---|---|
| `mMutexImuQueue` :328 | `mlQueueImuData` | T only in this repo: GrabImuData `Tracking.cpp:870-874` (called from System::Track* :300/:374-375/:450-451 — same thread), PreintegrateIMU :898-924, clear :1075-1076. Becomes cross-thread only with an external async IMU feeder |
| `mMutexTraceState` :513 | trace **sink only** (diagnostic, ORB_TRACE_STATE) — explicitly NOT `mState_` | T + LM (via NotifyImuInitialized :80-83 / TraceStateTransition :92-108) |
| `mMutexStop` :464 | REGISTER_LOOP-only stop flags | not built (ifdef; Track's park at :1574-1583 is dead) |

### Data layer
| Mutex | Guards | Takers |
|---|---|---|
| `Map::mMutexMapUpdate` `Map.hpp:148` | coarse map-surgery gate | T (Track `Tracking.cpp:1138`); LM via optimizer apply (`Optimizer.cpp:1592/:1867/:2200`) + ImuInitializer (`ImuInitializer.cpp:141/:179/:334`); LC (`LoopClosing.cpp:371/:821-822/:952/:1026-1027/:1130/:1151/:1166-1167/:1297/:1326`); GBA (:1480 + `Optimizer.cpp:3030/:4559/:5689`) |
| `Map::mMutexMap` :209 | map containers/counters | all 4 threads via accessors |
| `Map::mMutexPointCreation` :151 | point-creation serialization | T (stereo/RGBD/init point creation) + LM CreateNewMapPoints |
| `KeyFrame::mMutexPose/Connections/Features` `KeyFrame.hpp:449-451` | pose / covisibility+erase latch / observations | all threads. SetNotErase/SetErase latch under mMutexConnections (`KeyFrame.cpp:549-567`); deferred SetBadFlag completes on LC thread via SetErase |
| `MapPoint::mMutexPos/Features/Map` + static `mGlobalMutex` `MapPoint.hpp:194,236-238` | position / observations; global = generation-consistent snapshot for pose-only opt | all threads; mGlobalMutex: T pose-only optimization `Optimizer.cpp:984/:4687/:5071` vs `MapPoint::SetWorldPos` `MapPoint.cpp:118` |
| `Atlas::mMutexAtlas` `Atlas.hpp:173` | `mspMaps`, `mpCurrentMap`, imu flags | all threads. **Lock-free methods (inherited race, FixLevel list)**: AddKeyFrame `Atlas.cpp:113-117`, AddMapPoint :119-123, AddCamera :125-164, GetAllCameras :166-169, SetViewer :107-111 (no caller), SetMapBad :270-276 (LC), RemoveBadMaps :278-286 (LC). **GetCurrentMap spins `usleep(3000)` at :264-265 while HOLDING mMutexAtlas** — `ChangeMap` :89-92 needs the same mutex: deadlock window if a thread catches the bad-map interval during merge |
| `KeyFrameDatabase::mMutex` `KeyFrameDatabase.hpp:72` | inverted file | LC/PlaceRec add (`PlaceRecognition.cpp:53/:60/:67/:151/:199`) + DetectNBestCandidates `KeyFrameDatabase.cpp:120-128`; LM+LC via `KeyFrame::SetBadFlag`→erase (`KeyFrame.cpp:676`); T DetectRelocalizationCandidates :256-263, clearMap :88-90. **`clear()` :82-86 takes NO lock** (T Reset `Tracking.cpp:3057` vs LC add — P10 item) |
| `IMU::Preintegrated::mMutex` `ImuTypes.hpp:256` | preintegration state + bias | T (integrate/predict), LM/backend (SetNewBias during inertial opt), `ImuTypes.cpp:170-317` |
| `Frame::mpMutexImu` `Frame.hpp:330` (heap, per-frame) | `mbImuPreintegrated` | T setIntegrated `Frame.cpp:1009-1012` vs LM/LC spin via imuIsPreintegrated :1003-1006 (`Tracking.cpp:3224-3227`) |
| `FrameDrawer::mMutex` `FrameDrawer.hpp:74` | frame snapshot | T Update (`Tracking.cpp:1453`; `FrameDrawer.cpp:372`) vs V DrawFrame (`FrameDrawer.cpp:63/:215`) |
| `MapDrawer::mMutexCamera` `MapDrawer.hpp:65` | camera pose | T SetCurrentCameraPose `MapDrawer.cpp:351-353` (`Tracking.cpp:1455/:1471/:1693/:1904`) vs V GetCurrentOpenGLCameraMatrix :357-361 |

PlaceRecognition (`mPlaceRec`, LC member) holds **no mutexes** — LC-thread-only state.

## (3) Cross-thread channels

### Queue 1: `LocalMapping::mlNewKeyFrames` (`hpp:178`, mMutexNewKFs, no CV, LM polls 3ms)
- **Producers (all T)** via InsertKeyFrame `:276-281` (push + `mbAbortBA=true`): StereoInitialization `Tracking.cpp:1677` (**no SetNotStop**), CreateInitialMapMonocular `:1875-1876` ×2 (**no SetNotStop**; + lock-free `mFirstTs` write :1877), CreateNewKeyFrame `:2586` (SetNotStop bracket :2472/:2588).
- **Consumers**: LM ProcessNewKeyFrame `:290-330` (pop under lock, BoW/`Atlas::AddKeyFrame` :328 outside); **LC as second consumer** via EmptyQueue `:332-336` (called `LoopClosing.cpp:313/:582/:1132`); ImuInitializer with-processing drains (`ImuInitializer.cpp:79-84/:182.../:314...` — locked via CheckNewKeyFrames/ProcessNewKeyFrame).
- **Destructive drains (disposal asymmetry, keep)**: Release `:843-845` delete-only (R1 + dangling `mpLastKeyFrame`); PurgeNewKeyFramesAfterInertialInit `:1144-1156` SetBadFlag→guarded delete, lock-free (R3); ResetIfRequested `:1086/:1101` clear-only (protocol-protected).
- Backpressure advisory: `mbAcceptKeyFrames` hint + T override path `Tracking.cpp:2438-2461` (InterruptBA + queue-depth<3 stereo; `IsInitializing()` bypass :2442).

### Queue 2: `LoopClosing::mlpLoopKeyFrameQueue` (`hpp:158`, mMutexLoopQueue, no CV, LC polls 5ms)
- **Producer**: LM Run `LocalMapping.cpp:242` → InsertKeyFrame `LoopClosing.cpp:276-281` (drops id 0). Only main-path KFs — EmptyQueue/IMU-drain KFs never reach LC (preserved asymmetry).
- **Consumer**: LC PopNewKeyFrame `:293-305` (+SetNotErase latch).
- Reset mutates queue under **mMutexReset** `:1389/:1400-1409` (R-b).

### Request/response flag families
| Family | Flags (owner class) | Requester → Responder | Sites |
|---|---|---|---|
| LM stop | `mbStopRequested`/`mbStopped`/`mbNotStop`; response-clear = Release | LC×4, GBA, T(S)×3 → LM | req `LoopClosing.cpp:312/:575/:1010/:1116/:1471`, `System.cpp:263/:338/:413`; ack LM Stop :251/:810-821; release :541/:942/:1062/:1268/:1278/:1577, `System.cpp:277/:352/:427` |
| LM reset | `mbResetRequested(ActiveMap)` + requester spin | T → LM | `LocalMapping.cpp:1033-1074` ↔ :262/:1076-1116 |
| LC reset | same pattern | T → LC | `LoopClosing.cpp:1341-1376` ↔ :264/:1378-1414 |
| finish | `mbFinishRequested`/`mbFinished` per LM/LC/V | S → LM/LC (V commented) | above; only LM's SetFinish also sets mbStopped |
| localization mode | `mbActivate/DeactivateLocalizationMode` (System) | V → T | `Viewer.cpp:178/:183` → `System.cpp:259-280/...` |
| system reset | `mbReset`/`mbResetActiveMap` (System) | V (or S) → T | `Viewer.cpp:249`, `System.cpp:490-500` → :283-296/... |
| GBA control | `mbStopGBA` + `mnFullBAIdx` epoch + `mbRunningGBA`/`mbFinishedGBA` | LC ↔ GBA | spawn :521-538/:1064-1078; abort :316-331/:559-573/:1100-1113; epoch self-destruct :1452-1464 |
| backpressure | `mbAcceptKeyFrames` | LM → T | :71/:265 ↔ `Tracking.cpp:2354` |
| BA preemption | `mbAbortBA` (plain bool, `hpp:186`) | T/LC/GBA → LM optimizer | writes :280(T, locked)/:806-807(locked)/:876(T, **lock-free**)/:103(LM clear, lock-free); polled raw `&mbAbortBA` :149/:154 (R4) |

### Unsynchronized cross-thread variable ledger (consolidated R1-R5, R-a..R-f + new)
| # | Variable | Writer(s) | Reader(s) | Sites |
|---|---|---|---|---|
| R1 | queue vs Release | T push (init, no SetNotStop) | LM/LC/T-S Release delete-drain | `LocalMapping.cpp:843-845` vs `Tracking.cpp:1677/:1875-1876` |
| R2 | ordering: EmptyQueue before park-wait | LC | LM+LC both in ProcessNewKeyFrame | `LoopClosing.cpp:312-313` vs wait :333-337 (MergeLocal :575-582 and MergeLocal2 :1116-1132 are correct order) |
| R3 | lock-free purge | LM (via ImuInitializer) | vs T insert gate passing during `bInitializing` | `LocalMapping.cpp:1146-1155`; gate `Tracking.cpp:2442/:2469` |
| R4 | `mbAbortBA` | T/LC/LM mixed-discipline | optimizer raw-ptr poll | `LocalMapping.hpp:186`; :103/:280/:807/:876; poll :149/:154 |
| R5 | `mbBadImu` | LM :144, reset :1093/:1106 | T `Tracking.cpp:1057`; LM :74/:251 | no mutex (`hpp:117`) |
| R5 | `bInitializing` | ImuInit :77/:135/:289/:329 | T via IsInitializing :1160-1163 (`Tracking.cpp:2442/:2469`) | no mutex |
| R5 | `mFirstTs` | **two writer threads**: T `Tracking.cpp:1877`, LM `ImuInitializer.cpp:73` | LM :74/:116; S `System.cpp:1272-1274` | `hpp:115` |
| R5 | `mpCurrentKeyFrame` | LM :294 (locked) | S/T via GetCurrKFTime :1166-1174 (lock-free) | |
| R5 | `mbResetRequested` early reads | T (locked write) | LM lock-free `ImuInitializer.cpp:39/:300` | |
| R-a | `mbFinished=false` at Run entry | LC :97, **LM :66, V :60-61** | vs isFinished readers | no lock |
| R-b | LC queue dual-mutex | LC reset path | vs LM producer | `LoopClosing.cpp:1389/:1400-1409` vs :278-280 |
| R-c | GBA spawn flags+ptr | LC no lock | GBA/LC | :523-525,:531-537/:1067-1077 |
| R-d | `mnFullBAIdx` first read | GBA | vs LC increments (locked) | :1452 |
| R-e | `mbStopGBA` | LC (locked) | optimizer raw `bool*` | :1436/:1438 |
| R-f | `mpTracker->GetLastKeyFrame()` | T `Tracking.cpp:2591` (+cross write :3219!) | LC :1138/:1152/:1274 lock-free (`Tracking.hpp:106-108`) | plus Request* after SetFinish = infinite spin |
| new | `Tracking::mState_` | T SetState (`hpp:492-498`, plain write) + **LM** via NotifyImuInitialized `Tracking.cpp:80-83`/`ImuInitializer.cpp:288` | lock-free GetState `hpp:179`: T everywhere, **LM** `LocalMapping.cpp:202`, S `System.cpp:1285` | mMutexTraceState guards sink only |
| new | `Tracking::mnMatchesInliers` | T :2256/:2269 | LM via GetMatchesInliers :3276-3279 (`LocalMapping.cpp:148`) | |
| new | `Tracking::t0IMU` | **LM** `ImuInitializer.cpp:160` | T (IMU-reset-window logic) | `Tracking.hpp:243` |
| new | `Tracking::mbStep`/`bStepByStep` | V `Viewer.cpp:190/:195/:201` | T spin `Tracking.cpp:1046-1054` | no lock (`hpp:231/:379`) |
| new | `Tracking::UpdateFrameIMU` whole body | **LM** (`ImuInitializer.cpp:145/:156/:339`) and **LC** (`LoopClosing.cpp:1138/:1152`) mutate `mLastBias`, `mpLastKeyFrame`, `mlRelativeFramePoses` scaling, `mLastFrame`/`mCurrentFrame` biases | vs T using the same members concurrently | `Tracking.cpp:3193-3268` |
| new | `Viewer::both` | S ctor `System.cpp:223` **after** V launch :221 | V :219 | startup race |
| new | `LocalMapping::mbFarPoints/mThFarPoints` | S ctor `System.cpp:189-196` **after** LM launch :188 | LM `LocalMapping.cpp:659`, T `Tracking.cpp:2664` | startup race (benign-in-practice) |
| new | Atlas lock-free methods | LM AddKeyFrame :328→`Atlas.cpp:113`, LM/T AddMapPoint, LC SetMapBad/RemoveBadMaps | all | FixLevel list |
| new | `mbFinishedGBA` | GBA :1591 (locked) | **no reader** (isFinishedGBA deleted P9-1) | near-dead |
| dead | `usleep(500)` at `Tracking.cpp:926` | — | — | unreachable: `break` precedes `bSleep=true` at :919-923 (empty-queue exit). The IMU-wait poll is dead code |

## (4) System ↔ Viewer cycle and Tracking ↔ Viewer interplay

**Include cycle**: `Viewer.hpp:26` includes `core/System.hpp`; `System.hpp:39` includes `viz/Viewer.hpp` (broken only by include guards + fwd decls `Viewer.hpp:37`). `Viewer.hpp:25` also includes `Tracking.hpp`; `System.hpp:31-36` includes all module headers.

**Viewer → System** (all from V thread, `src/viz/Viewer.cpp`):
- `ActivateLocalizationMode()` :178, `DeactivateLocalizationMode()` :183, :245 (menuReset), :256 (menuStop)
- `ResetActiveMap()` :249 (menuReset)
- `Shutdown()` :259 (menuStop) → V thread runs the whole shutdown incl. SaveAtlas
- `SaveTrajectoryEuRoC("CameraTrajectory.txt")` :262, `SaveKeyFrameTrajectoryEuRoC(...)` :263 — V thread reads the whole map while T/LM/LC may still run
- enum access `mpSystem->MONOCULAR/...` :110

**System → Viewer**: create+launch `System.cpp:220-221`; `mpTracker->SetViewer` :222; `mpViewer->both=` :223 (post-launch); Shutdown's RequestFinish/isFinished **commented** :513-518 → nothing ever finishes the Viewer thread.

**Tracking → Viewer** (T thread): `RequestStop` + `isStopped` spin (`Tracking.cpp:3036-3038` Reset, :3096-3098 ResetActiveMap); `Release` :3086/:3177. (Tracking's resets are triggered from Track*/System — T thread — or from V's menuReset via System flag relay.)

**Viewer → Tracking** (V thread): `SetStepByStep` `Viewer.cpp:190/:195`; `mpTracker->mbStep = true` :201; `mpTracker->mSensor` :110; `GetImageScale()` :115.

**Drawer relay**: T writes FrameDrawer::Update (`Tracking.cpp:1453`) / MapDrawer::SetCurrentCameraPose; V reads DrawFrame/DrawRightFrame (`Viewer.cpp:217-220`) / GetCurrentOpenGLCameraMatrix :122 — both properly mutexed (see §2).

## (5) Who-waits-on-whom matrix (spin-waits, with release condition)

| # | Waiter | Waits on (state, owner) | Site | Poll | Released by |
|---|---|---|---|---|---|
| W1 | LC | LM `mbStopped` | `LoopClosing.cpp:334-337` (CorrectLoop — **after** EmptyQueue, R2), :577-580 (MergeLocal), :1012-1015 (MergeLocal 2nd window), :1118-1121 (MergeLocal2) | 1ms | LM Run :251 Stop(). Hazard: if LM already finished, SetFinish sets mbStopped=true → proceeds; but if `mbBadImu` quirk, mbStopped set while LM keeps looping |
| W2 | GBA | LM `mbStopped` **or** `mbFinished` (finish-aware) | `LoopClosing.cpp:1474-1477` | 1ms | LM Stop or SetFinish (only finish-aware LM wait in the codebase) |
| W3 | T (in System::Track*, **holding mMutexMode**) | LM `mbStopped` | `System.cpp:266-269`, :341-344, :416-419 | 1ms | LM Stop. Blocks any concurrent V mode toggle (same mutex) |
| W4 | T | LM `mbResetRequested(ActiveMap)` cleared | `LocalMapping.cpp:1042-1050`, :1064-1072 | 3ms | LM ResetIfRequested :262 only — **parked LM never services it** (invariant 5: spins until LC/S Release); LM after SetFinish → infinite spin |
| W5 | T | LC `mbResetRequested(ActiveMap)` cleared | `LoopClosing.cpp:1348-1356` (5ms), :1367-1375 (3ms) | 5/3ms | LC ResetIfRequested :264; after LC SetFinish → infinite spin (R-f note) |
| W6 | T | Viewer `mbStopped` | `Tracking.cpp:3037-3038`, :3097-3098 | 3ms | V Stop() :267 in render loop (requires V alive and not finish-requested — Stop returns false if mbFinishRequested :324) |
| W7 | V (self-park) | own `mbStopped` cleared | `Viewer.cpp:269-272` | 3ms | T `Viewer::Release` `Tracking.cpp:3086/:3177` |
| W8 | LM (park) | own `mbStopped` cleared, or finish | `LocalMapping.cpp:254-257` | 3ms | Release from LC (:541/:942/:1062/:1268/:1278), GBA (:1577), T-S (System.cpp:277/:352/:427); or RequestFinish |
| W9 | LM and LC (inside `Tracking::UpdateFrameIMU`) | T frame `mbImuPreintegrated` | `Tracking.cpp:3224-3227` | 500µs | T `Frame::setIntegrated` (`Frame.cpp:1009-1012`) — cross-thread mutation + wait on the tracker's live frame |
| W10 | any thread in `Atlas::GetCurrentMap` | current map not bad — **while holding mMutexAtlas** | `Atlas.cpp:264-265` | 3ms | LC `ChangeMap` :89-92 — which needs mMutexAtlas → deadlock window during merge's SetMapBad→ChangeMap interval |
| W11 | T (step-by-step) | V `mbStep` | `Tracking.cpp:1052-1054` | 500µs | V `Viewer.cpp:201` |
| W12 | S Shutdown | (nothing — all waits commented) | `System.cpp:513-533` | — | current behavior: SaveAtlas races LM/LC/GBA; GBA abort elsewhere is fire-and-forget (epoch :1452-1461 self-destruct; stale `mbRunningGBA=true` possible via early returns :1460-1464) |

**CV-placement takeaway for the implementer**: the wait edges cluster on five predicates — (a) LM queue non-empty (LM Run poll + producers), (b) LM stopped/finished (W1/W2/W3/W8: one `condition_variable` on mMutexStop, notified by Stop/Release/SetFinish), (c) LM/LC reset-consumed (W4/W5: CV on each mMutexReset), (d) Viewer stopped/released (W6/W7: CV on Viewer mMutexStop), (e) LC queue non-empty (LC Run poll). W9/W10/W11 need per-object treatment (Frame imu flag CV; Atlas current-map CV must fix the lock-held spin; step flag is debug-only). Everything in §3's ledger must become atomic or be pulled under the owning mutex before the CVs are trustworthy, and Shutdown needs the W12 waits reinstated (jthread/stop_token) before SaveAtlas.
---

# ═══ 2부: 이행 설계 (migration-design) ═══

All evidence gathered from the current tree (post-P9, `phase-9`-era line numbers verified today). Design follows, per item.

---

# P10 Threading Modernization — Design

Scope constraint: core stays `-std=c++17` `CXX_EXTENSIONS OFF` (CMakeLists.txt:96/137/147); Thirdparty/third_party flags untouched. Everything below is implementable with `std::thread` + `condition_variable` + `atomic`; the per-item "C++20 delta" states what C++20 would buy — nothing *requires* it.

**Canonical lock order (new, documented in OWNERSHIP.md):** `mMutexFinish → mMutexStop → mMutexNewKFs`; `mMutexReset → mMutexLoopQueue`; `mMutexAccept`, `mMutexGBA` standalone. Two existing violations must be dissolved first (see Discovered R6 and item 4).

**Discovered in this recon (new, not in the ledgers) — R6 ABBA deadlock:** `LocalMapping::Release()` locks `mMutexStop → mMutexFinish` (LocalMapping.cpp:835-838) while `SetFinish()` locks `mMutexFinish → mMutexStop` (:1130-1136). A Release from LC/System racing LM's exit can deadlock today. Fix: reorder Release to `Finish → Stop` (matches SetFinish). Goes in step P10-2.

---

## (1) LocalMapping::Run 3ms poll → CV

**Current:** Run loop LocalMapping.cpp:64-274; tail poll `usleep(3000)` :270; park loop :254-257; duty-cycle `SetAcceptKeyFrames(false/true)` :71/:265; producer `InsertKeyFrame` :276-281 (push + `mbAbortBA=true` under `mMutexNewKFs`); Tracking reads hint at Tracking.cpp:2354, inserts at Tr:1677/1875-1876/2586.

**Proposed:**
- Add `std::condition_variable mCondNewKFs` (paired with `mMutexNewKFs`). `InsertKeyFrame`: push under lock, then `notify_one()` after unlock (`mbAbortBA` store moves out of the lock once atomic — item 4).
- Run tail becomes `{ unique_lock lk(mMutexNewKFs); mCondNewKFs.wait_for(lk, 3ms, [&]{ return !mlNewKeyFrames.empty(); }); }`. **Recommendation: keep the 3ms timed net permanently** for this one wait. Rationale: the Run loop's other wake reasons (stop/finish/reset/mbBadImu quirk) live under three *other* mutexes; a pure untimed wait would need those flags in the predicate (lock-nesting or atomics for all) and makes invariant-4 (`mbBadImu` set, queue non-empty, work branch skipped) an instant-wakeup busy loop where upstream burns a 3ms sleep. The timed net preserves every non-queue latency at exactly today's ≤3ms bound while making KF pickup immediate. An optional later tightening (pure wait + atomic request-flag predicate) can be its own gated commit.
- **Park loop** (:254-257): `mCondStop.wait(lk(mMutexStop), [&]{ return !mbStopped || mbFinishRequested.load(); })`. `Release()` (mbStopped=false) and `RequestFinish()` notify `mCondStop`. Predicate must NOT read `CheckFinish()` under `mMutexStop` (would nest Stop→Finish against SetFinish's Finish→Stop) — hence **`mbFinishRequested` becomes `std::atomic<bool>`** (write in RequestFinish :1118-1122 keeps or drops the lock, read lock-free; `mbFinished` stays under `mMutexFinish` because SetFinish couples it with mbStopped).
- **Invariant 5 preserved verbatim:** the park predicate deliberately excludes reset flags — a Tracking reset still blocks until Release, now on a CV instead of a spin.
- **Drain-beats-stop preserved structurally:** the queue branch (:74) is evaluated before `Stop()` (:251) every iteration regardless of which notify woke the loop; `SetNotStop` latch (:862-872) unchanged under `mMutexStop`.
- **SetAcceptKeyFrames — yes, it stays a plain guarded bool.** Single writer (LM), single reader (T at Tr:2354), already under `mMutexAccept`, and no CV predicate depends on it. What changes is its *duty cycle*: today the accept=true window is the 3ms tail sleep; under CV a queued KF wakes LM instantly, shrinking the window. That is exactly the intended, gate-judged timing change (c1b `bLocalMappingIdle` will read true less often under load → KF admission cadence shifts).
- **ResetIfRequested servicing:** unchanged position (:262, every iteration). Requester side: `RequestReset`/`RequestResetActiveMap` (:1033-1074) drop the 3ms spin for `mCondReset.wait(lk(mMutexReset), [&]{ return !mbResetRequested; })` (resp. ActiveMap flag); `ResetIfRequested` does `mCondReset.notify_all()` after clearing. Semantics identical (producer-synchronous, invariant 6). Preserved quirk: a reset requested while LM is parked still waits for Release; a reset requested after SetFinish still never returns (upstream hang, documented, FixLevel candidate — do NOT silently add a finished-escape, that changes observable protocol).

**C++17:** fully feasible. **C++20 delta:** `jthread`+`stop_token` would subsume the mbFinishRequested plumbing; `atomic::wait/notify` could replace mCondStop. Convenience only.
**Risk:** HIGH (cadence-changing) for the queue CV; LOW for the reset handshake. **Validation:** paired smoke + KITTI07 mid-gate; queue-depth + accept-hint-hit-rate counters (temporary trace) compared before/after; TSan smoke.

## (2) LoopClosing::Run 5ms poll → CV; reset spins; R-b

**Current:** poll `usleep(5000)` LoopClosing.cpp:270; producer `InsertKeyFrame` :276-281 (`mMutexLoopQueue`); consumer pop `PopNewKeyFrame` :293-305 (`mMutexLoopQueue`); reset spins :1348-1356 (5ms) / :1367-1375 (3ms); **R-b**: `ResetIfRequested` :1378-1414 mutates `mlpLoopKeyFrameQueue` (:1389, :1400-1409) holding only `mMutexReset`.

**Proposed:**
- `mCondLoopQueue` + `wait_for(lk(mMutexLoopQueue), 5ms, [&]{ return !mlpLoopKeyFrameQueue.empty(); })` at :270; notify in `InsertKeyFrame`. Same timed-net rationale as LM (finish/reset flags live under other mutexes; 5ms bound preserved for them).
- Reset handshakes → `mCondLCReset` on `mMutexReset`, identical pattern to LM. Preserved quirk (R-f second half): RequestReset* after LC SetFinish still never returns — document, don't fix here.
- **R-b fix — unify to `mMutexLoopQueue`:** `ResetIfRequested` keeps `mMutexReset` for the flags and additionally takes `mMutexLoopQueue` (inner) for the two queue mutations. Lock order `mMutexReset → mMutexLoopQueue`; no reverse nesting exists anywhere (producer and PopNewKeyFrame take only the queue mutex) — verified. The old "reset-spin protocol protects it" defense stops being load-bearing.

**C++17:** feasible. **Risk:** LOW-MEDIUM (loop-detection trigger timing shifts by up-to-5ms→~0 pickup — mildly ATE-relevant on loop sequences). **Validation:** KITTI00 loop-detection count (P9-4 golden traces reusable) + paired KITTI07.

## (3) Stop protocol → CV; R2 as its own step

**Current spin-waiters on LM `isStopped`:** LC CorrectLoop :334-336, MergeLocal :577-579, MergeLocal2 :1118-1120, GBA :1474-1477 (`!isStopped && !isFinished`), System localization-mode :266-269/:341-344/:416-419.

**Proposed:** `LocalMapping::WaitUntilStopped()` = `mCondStop.wait(lk(mMutexStop), [&]{ return mbStopped; })`. Notify sites: `Stop()` on success (:815), `SetFinish()` (:1135 sets mbStopped=true — so the GBA finish-aware variant collapses to the same predicate; identical to today where SetFinish makes isStopped() true), `Release()` (for the park loop). Preserved quirk (invariant 4): with `mbBadImu`, Stop() still sets mbStopped and notifies while Run keeps looping — external waiters proceed exactly as today.

**R2 fix — own commit, minimal diff:** CorrectLoop currently `RequestStop()` :312 → `EmptyQueue()` :313 → GBA abort :316-331 → wait :334-337. Move the `EmptyQueue()` call to after the stopped-wait (i.e., after :337), matching MergeLocal's order (:575 → :577-579 → :582) and MergeLocal2 (:1116-1120 → :1132). Same KF set processed, same thread (LC), no concurrent `ProcessNewKeyFrame` window. Do this while the wait is still the 1ms poll, *before* the CV migration, so each is separately bisectable.

**C++17:** feasible. **C++20 delta:** `atomic<bool>::wait` could replace WaitUntilStopped entirely (mbStopped as atomic + futex wait) — cosmetic. **Risk:** R2 move LOW-MEDIUM (drain now strictly after park; KF→map timing shifts ≤ one LM iteration; mildly ATE-relevant); CV conversion LOW. **Validation:** R2 step gets its own smoke + KITTI07 pair; assert (debug-only) in `ProcessNewKeyFrame` that caller is LM-thread-or-LM-parked (temporary instrumentation retired at phase end).

## (4) mbAbortBA / mbStopGBA — the bool* pipeline

**Evidence of how the flag reaches g2o:** our Optimizer hands the raw pointer to `SparseOptimizer::setForceStopFlag(bool*)` at Optimizer.cpp:207, 538, 1332, 2989, 3660, 4512. g2o polls it at two places: the outer iteration guard `!terminate()` (third_party/g2o/g2o/core/sparse_optimizer.cpp:415; `terminate()` derefs `_forceStopFlag`, sparse_optimizer.h:183) and the LM failed-step retrial loop (optimization_algorithm_levenberg.cpp:147). **OrbLevenberg does NOT currently wrap the flag path** — it only overrides `solve()` for the Raul stop criterion (OrbLevenberg.hpp:53-76); the premise "we already wrap g2o's forceStop" is false, but `solve()` is called once per outer iteration by every `optimize()`, which makes OrbLevenberg the natural sound interception point. Our own manual derefs: Optimizer.cpp:848-849, 1534-1535, 3860-3870, 4515-4516. Signatures we own: IMappingOptimizer.hpp:47/51/65, ILoopOptimizer.hpp:96/102/111/116, Optimizer.hpp + defs :179/188/518/1244/2526/3641/4091, G2oBackend forwarders. Callers: LM `&mbAbortBA` (LocalMapping.cpp:149/154), GBA `&mbStopGBA` (LoopClosing.cpp:1436/1438), MergeLocal/MergeLocal2 *local* `bool bStop`/`bStopFlag` (:927/:931/:1275 — single-thread, never set, dead-in-practice), ImuInitializer passes `NULL` (ImuInitializer.cpp:171/173).

**Assessment:**
- (a) plain bool + documentation: **rejected.** T writes (`InterruptBA` :874-877 lock-free; `InsertKeyFrame` :280) race LM/g2o reads; it is the flagship UB this phase exists to remove.
- (c) `std::atomic_ref` (C++20): **rejected on soundness, not just standard-version.** [atomics.ref] requires *all* concurrent accesses to go through atomic_ref while any exists; g2o's plain `*_forceStopFlag` read concurrent with an atomic_ref store is still a data race. C++20 does not actually solve this problem.
- `reinterpret_cast<bool*>(&atomic)`: UB (works on our ABIs, but we don't need it).
- **(b) recommended, with a shadow bridge in OrbLevenberg:**
  - `std::atomic<bool> mbAbortBA` (LocalMapping.hpp:186) and `std::atomic<bool> mbStopGBA` (LoopClosing.hpp:183). All 18-odd sites we control switch to `const std::atomic<bool>* pbStopFlag` + `load(memory_order_relaxed)`.
  - OrbLevenberg gains `bindAbortFlag(const std::atomic<bool>*)`, a private `bool mShadowStop = false`, and `bool* shadowPtr()`. `solve()` refreshes `mShadowStop` from the atomic at entry **and** exit. Each Optimizer site (when `pbStopFlag`): `solver->bindAbortFlag(pbStopFlag); optimizer.setForceStopFlag(solver->shadowPtr());`. Only the optimizing thread writes/reads the shadow → no race; the cross-thread edge is the atomic load. RequestStop (:802-808) drops its `mMutexNewKFs` nesting (the lock existed only for the mbAbortBA write) — this also clears the `Stop→NewKFs` lock-order violation, prerequisite for item 1.
  - Abort granularity: the outer `terminate()` check reads a value sampled at the end of the previous `solve()` — equal to upstream to within µs. The retrial-loop poll sees the solve-entry sample — worst case the remaining trials of one LM iteration (bounded by `_maxTrialsAfterFailure`) of extra latency vs upstream. Timing-only; flagged for the paired gate.
  - The two LC dead-local flags become local `std::atomic<bool>` (still never set — behavior preserved); ImuInitializer keeps `nullptr`.
  - Patching g2o's signature is prohibited (pinned submodule, DIVERGENCES #10 policy) — the bridge is the only fully-conformant C++17 *or* C++20 route.

**Risk:** LOW-MEDIUM (mechanical fan-out ~8 files; semantic surface = abort latency only). **Validation:** TSan smoke must show the InterruptBA race gone; temporary OrbLevenberg counter (iterations-after-abort) diffed on KITTI07; paired smoke.

## (5) R5 / R-a / R-c / R-d / R-f — per-site cheapest sound fix

| Site | Writers / readers | True race? | Fix |
|---|---|---|---|
| `mbBadImu` (LM.hpp:117) | LM :144, :1093/:1106 / T Tracking.cpp:1057, LM Run :74/:251 | Yes | `std::atomic<bool>`, relaxed |
| `bInitializing` (LM.hpp:201) | ImuInitializer.cpp:77/135/289/329 / T via IsInitializing (LM.cpp:1160, Tr:2442/2469) | Yes | `std::atomic<bool>`, relaxed |
| `mFirstTs` (LM.hpp:115) | T Tr:1877 + LM ImuInit.cpp:73 / ImuInit.cpp:74/116, System.cpp:1272-1274 | Yes (torn double possible) | `std::atomic<double>` (lock-free arm64/x86-64), relaxed load/store |
| `mpCurrentKeyFrame` (LM.hpp:180) | LM under mMutexNewKFs (LM.cpp:293-296) / System::GetTimeFromIMUInit → GetCurrKFTime (LM.cpp:1166-1175) lock-free | Yes (pointer + deref of T-constructed KF) | `std::atomic<KeyFrame*>`, store-release in ProcessNewKeyFrame, load-acquire in GetCurrKFTime (pairs with queue-mutex handoff to make KF construction visible); same-thread reads relaxed |
| `mbFinished=false` at Run entry (LM.cpp:66, LC.cpp:97 — R-a) | LM/LC entry / System isFinished | Yes, benign-in-practice | take `mMutexFinish` for the write (2 lines); NOT atomic — couples with mbStopped in SetFinish |
| `mbResetRequested*` early guards (now inside ImuInitializer guards) + park/Run predicate needs | T + LM | Yes | request flags → `std::atomic<bool>` (consistent with mbFinishRequested from item 1); acked/`mpMapToReset` stay under mMutexReset |
| LC GBA spawn-flag block (R-c): `mbRunningGBA/mbFinishedGBA/mbStopGBA` writes at LC:523-525, 1067-1069 lock-free vs mMutexGBA readers (isRunningGBA :72-75, GBA tail :1459-1592) | LC / GBA, T | Yes | scope `unique_lock(mMutexGBA)` around the three flag writes at both spawn sites. `mpThreadGBA` needs **no lock**: LC-thread-confined while LC lives, System-confined after LC join (item 6); reap-join stays outside the lock |
| `mnFullBAIdx` (R-d): lock-free read LC:1452, ++ under lock :322/:564/:1105 | LC / GBA | Yes | `std::atomic<int>`; increments stay inside the existing mMutexGBA scopes |
| `mpTracker->GetLastKeyFrame()` from LC thread (R-f): LC:1138/:1152/:1274 | T writes mpLastKeyFrame / LC reads | Yes | Tracking-side: `std::atomic<KeyFrame*> mpLastKeyFrame` (store-release on write, load-acquire in accessor); cheapest sound fix without inventing a Tracking mutex |

**C++17:** all feasible (`atomic<double>` and `atomic<KeyFrame*>` are lock-free on target ABIs — add `static_assert(std::atomic<double>::is_always_lock_free)`). **C++20 delta:** none needed. **Risk:** LOW (no scheduling change). **Validation:** TSan EuRoC MH01 short run before/after — the report diff *is* the acceptance criterion; bit-gate expected to hold (no float-path change).

## (6) GBA thread lifetime + System::Shutdown restoration

**Current:** raw `new thread` LC:537/:1077 with reap-at-spawn join (P9-3, :531-536/:1071-1076); abort = flag + epoch + `detach/delete/null` (:319-329/:561-570/:1102-1111); epoch self-destruct :1452/:1459-1461; **stale `mbRunningGBA=true`** on the epoch-abort return path (:1460-1461 exits without clearing); Shutdown wait fully commented (System.cpp:513-533) → SaveAtlas (:535-539) can race a live GBA; LC::Run exit (:273) ignores a live GBA; `std::thread*` members System.hpp:233-235.

**Proposed:**
- `std::thread mThreadGBA` value member. Epoch discipline kept verbatim (atomic `mnFullBAIdx` + under-lock recheck :1459-1461).
- **Abort path: drop `detach`** (keep flag + epoch, leave the thread joinable). The aborter still returns immediately (observable fire-and-forget preserved); the *next spawn's* reap-join may now block until the aborted GBA actually exits (≤ one BA iteration) — the price of making every GBA reapable. Deadlock audit: the exiting GBA's tail takes `mMutexGBA` (:1459); therefore the reap-join executes **outside** any `mMutexGBA` scope (spawn flags re-locked after the join).
- Stale `mbRunningGBA` fix: scope-exit guard in `RunGlobalBundleAdjustment` clearing `mbRunningGBA/mbFinishedGBA` under `mMutexGBA` on *all* return paths. This un-sticks the respawn gate after an abort race — strictly a defect fix but it changes respawn outcomes in that corner → own DIVERGENCES entry, gate-judged.
- **System::Shutdown exact ordering** (uncommented and redesigned):
  1. Idempotence latch: atomic exchange on `mbShutDown`; second caller skips teardown (joins remain naturally idempotent via `joinable()`).
  2. Viewer: `RequestFinish()`; join `mptViewer` **only if** `std::this_thread::get_id() != mptViewer.get_id()` (menuStop-path Shutdown runs on the viewer thread — unconditional join self-deadlocks; this is why upstream commented the wait. Item 7 removes the hazard properly via RequestShutdown).
  3. `mpLocalMapper->RequestFinish(); mpLoopCloser->RequestFinish();` (both notify their CVs so parked/waiting threads wake).
  4. **Join LM** (SetFinish sets mbStopped + notifies mCondStop, so LC/GBA stop-waiters can never hang on a dead LM).
  5. **Join LC** (an in-flight CorrectLoop/Merge completes; its `Release()` is a no-op on finished LM :839-840, upstream semantics).
  6. **GBA custody transfer + reap:** LC is dead ⇒ System has exclusive access to `mThreadGBA`. `{ lock(mMutexGBA); mbStopGBA=true; ++mnFullBAIdx; }` then `if (joinable) join`. GBA cannot hang: its stop-wait predicate (`mbStopped||mbFinished`, :1474) is already true, its Release a no-op.
  7. **Only now** SaveAtlas — provably zero live LM/LC/GBA/Viewer ⇒ boost serialization cannot race a map mutation. This is the "SaveAtlas never races a live GBA" invariant, achieved by join-ordering, not by locks.
  - System dtor calls Shutdown() if not latched; `mptLocalMapping/mptLoopClosing/mptViewer` become `std::thread` values.
- **LC::Run exit accounting:** no code at LC exit — the custody-chain rule replaces it ("mThreadGBA: LC-confined while LC alive; System-confined after step 5; reaped before step 7"), documented in OWNERSHIP.md.

**C++17:** feasible. **C++20 delta:** `jthread`+`stop_token` would collapse RequestFinish/joins into destructor order and let the GBA loop observe cancellation without our flag — the single strongest C++20 simplification in P10, but nothing here needs it. **Risk:** MEDIUM (teardown ordering; no steady-state impact). **Validation:** shutdown-under-load harness — send finish mid-KITTI00 (GBA running) N times, assert clean exit + SaveAtlas loads back; ASan run of the same.

## (7) System ↔ Viewer cycle cut

**Exact surface the Viewer consumes** (Viewer.cpp): `ActivateLocalizationMode` :178, `DeactivateLocalizationMode` :183/:245/:256, `ResetActiveMap` :249, `Shutdown` :259, `SaveTrajectoryEuRoC` :262, `SaveKeyFrameTrajectoryEuRoC` :263, plus sensor-enum constants via `mpSystem->MONOCULAR/...` :110 (static enum values — replace with a `bool` computed at ctor or pass the sensor value; no interface method needed).

**Recommendation: narrow-interface DI, P7-1b pattern** (`IResetRequester` precedent, include/core/IResetRequester.hpp):

```cpp
class IViewerHost {                       // implemented by System
    virtual void ActivateLocalizationMode() = 0;
    virtual void DeactivateLocalizationMode() = 0;
    virtual void RequestResetActiveMap() = 0;   // reuse IResetRequester semantics (latch)
    virtual void RequestShutdown() = 0;          // latch ONLY — never joins (see below)
    virtual void SaveTrajectoryEuRoC(const std::string&) = 0;
    virtual void SaveKeyFrameTrajectoryEuRoC(const std::string&) = 0;
};
```

Zero-include header per the IResetRequester discipline; cuts Viewer.hpp:26's `#include "core/System.hpp"`. Critically, `Shutdown` becomes `RequestShutdown` (latch; the main thread's normal end-of-Example `Shutdown()` performs the joins) — this removes the viewer-thread-self-join hazard from item 6 structurally instead of by get_id() special-case (keep the get_id() guard anyway as belt-and-braces). Trajectory saves stay synchronous on the viewer thread (upstream behavior).
**Viewer thread protocol modernization:** Tracking's spin `RequestStop` + `while(!isStopped()) usleep` (Tracking.cpp:3036-3037/:3096-3097, Release :3086/:3177) → `Viewer::WaitUntilStopped()` CV on `mMutexStop`; Viewer park loop (Viewer.cpp:267-271) → CV wait on `!mbStopped || finishRequested`; `Release`/`RequestFinish` notify. `mbStopTrack` (:94, :345) — audit: if any cross-thread writer remains, atomic; else delete. Viewer `std::thread` value owned by System (item 6).

**Risk:** LOW (viewer is off in gates). **Validation:** build both with/without viewer; manual EuRoC GUI smoke (menu stop, localization toggle, reset).

## (8) Step ordering — 6 gated commits

| Step | Content | Class | Mid-check | ATE-timing-sensitive? |
|---|---|---|---|---|
| **P10-1** | Item 4 in full: `mbAbortBA`/`mbStopGBA` → atomic, `const std::atomic<bool>*` signature fan-out (I*Optimizer, Optimizer, G2oBackend, callers), OrbLevenberg shadow bridge; RequestStop loses its mMutexNewKFs nest | pure-safety | build + TSan smoke + smoke gate + KITTI07 pair | No (abort-latency µs-shift only — verify via pair) |
| **P10-2** | Item 5 table complete (atomics R5/R-a/R-c/R-d/R-f, request-flag atomics) + lock closures: R1 (`Release` queue ops under mMutexNewKFs), R3 residue (`PurgeNewKeyFramesAfterInertialInit` LM.cpp:1144-1156 under mMutexNewKFs), R-b (reset→queue-mutex unification), **R6 ABBA reorder in Release** | pure-safety | build + TSan (acceptance: race-report diff clean) + smoke; bit-gate expected to hold | No |
| **P10-3** | R2 alone: CorrectLoop `EmptyQueue` moved after the stopped-wait (LC:313 → after :337), matching MergeLocal | protocol-order | smoke + KITTI07 pair + KITTI00 loop-count check | Mild (drain deferred ≤1 LM iteration) |
| **P10-4** | Item 1: LM queue CV (`wait_for` 3ms net) + park CV + LM reset handshake CV + `WaitUntilStopped()` adopted by all 7 spin-waiters (LC×3, GBA, System×3) | cadence | smoke + **full paired KITTI07 + one inertial pair** (P8 #20 lesson: boundary p-value ⇒ add that mode's pairs immediately) | **YES — the KF-admission-cadence step** (accept-hint duty cycle shrinks, pickup latency →0) |
| **P10-5** | Item 2 (LC queue CV 5ms net + LC reset CV) + item 6 (owned `std::thread` GBA, no-detach abort, stale-mbRunningGBA scope guard [own DIVERGENCES entry], Shutdown join-order restoration, System thread values, dtor) | cadence-light + teardown | smoke + KITTI00 loop count + shutdown-under-load ×N + ASan | Mild (loop-trigger pickup latency); teardown ATE-neutral |
| **P10-6** | Item 7: IViewerHost DI, RequestShutdown latch, Viewer CV protocol, `mbStopTrack` audit | pure-safety | build ±viewer + GUI manual smoke | No |
| **Phase gate** | full gate v2.2.1 4 rounds (KITTI00) + stereo-inertial pairs, paired-interleaved → tag phase-10 | | | |

Every intermediate state is runnable: P10-1/2 change no scheduling; P10-3 is a one-call move under the old polling; P10-4 introduces CVs while all counterpart flags are already atomic/lock-clean from P10-2; P10-5's Shutdown depends only on P10-4's notify infrastructure; P10-6 is independent. Sequencing rationale: all UB is dead *before* any wait is restructured, so any P10-4/5 gate regression bisects to pure timing, never to a latent race.

**Cross-cutting validation hooks:** (i) TSan build target (core only, Thirdparty suppressed) run at P10-1/2/4/5; (ii) temporary protocol trace counters (queue depth at insert, accept-hint hit rate, aborted-BA iteration count, park entries) behind a macro, diffed across steps, removed at phase end; (iii) the existing "Local Mapping STOP/RELEASE" stdout lines double as protocol-order traces for R2/park verification; (iv) shutdown-under-load harness from P10-5 kept as a permanent test.

**Known-preserved quirks (do not "fix" silently):** invariant-4 mbBadImu park-skip; invariant-5 no-reset-while-parked; reset-after-finish permanent block (LM and LC); LC queue asymmetry (EmptyQueue/IMU-drain KFs never reach LC); Release's delete-without-SetBadFlag dangling risk (needs T-side cooperation — still deferred, now tracked as the sole surviving R1-adjacent item); D1/D5/D6, L1/L2 latch leaks (FixLevel backlog, unrelated to threading).
---

# ═══ 3부: 검증 전략 (validation-strategy) ═══

All evidence gathered and the TSAN feasibility probe was run live in the actual dev container. Report follows.

---

# P10 Validation Strategy — Threading Modernization Evidence Design

Premise confirmed in-repo: the phase intentionally changes timing, and DIVERGENCES #20 already proved this codebase's ATE moves ±9-13% from *layout-only* changes in stereo-inertial. The paired ATE gate therefore judges only outcome quality, never mechanism. The strategy below adds mechanism evidence (TSAN signature deltas), liveness evidence (bounded-time + unit checks), and latency evidence (queue trace) — all with current tooling; the only new code is ~40 lines of env-gated tracing (P7-1a pattern) and one small assert-style test binary.

## 1. ThreadSanitizer feasibility — CONFIRMED empirically, no Dockerfile change needed

Probed live in the dev container (`docker compose run --rm dev`):
- gcc 11.4.0, **libtsan0 11.4 already installed** (came as a gcc-11 dependency despite `--no-install-recommends`), `/usr/lib/gcc/aarch64-linux-gnu/11/libtsan.{a,so}` present.
- aarch64 native (no qemu), 16 cores, **46 GB VM memory** (shadow-memory headroom is a non-issue), `vm.mmap_rnd_bits=18` (well inside GCC-11 TSAN's supported ASLR entropy — the known high-entropy TSAN breakage does not apply; no `setarch -R` workaround needed).
- Minimal two-thread race program compiled with `g++ -fsanitize=thread` **ran and detected the race (exit 66)** in the container. Feasibility is proven, not assumed.

**Build integration** — separate tree, command-line injection only, zero CMakeLists changes:
```
cmake -S /workspace -B /build/tsan -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_C_FLAGS="-fsanitize=thread -fno-omit-frame-pointer -g -O1" \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread -fno-omit-frame-pointer -g -O1" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread" -DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=thread"
```
- `CMAKE_CXX_FLAGS` propagates to **all** subdirectories — core, examples, `Thirdparty/DBoW2`, `third_party/g2o` all get instrumented. This does **not** violate the P1 "don't touch Thirdparty flags" gate finding: that constraint protects numeric parity of the *gate* build; the TSAN tree is diagnostic-only, never ATE-judged, and lives in `/build/tsan` beside `/build/cmake` (gate build untouched; ccache keys differ, no pollution).
- **DBoW2 must be instrumented** — yes: `KeyFrame::ComputeBoW` writes `mBowVec/mFeatVec` *inside* DBoW2 code while instrumented threads read them; a race with one side in uninstrumented code is invisible to TSAN. **g2o must be instrumented** for R4/R-e (the `mbAbortBA`/`mbStopGBA` raw-`bool*` polling happens inside the optimizer loop). Global injection covers both for free.
- Shared-lib constraint satisfied: every in-tree `.so` and every executable carries `-fsanitize=thread`; the main executables are instrumented (TSAN's hard requirement).
- **Uninstrumented remainder**: Pangolin (baked into image), OpenCV, TBB, X11. These hide happens-before edges → false-positive noise in Viewer/cv paths. Mitigations: `OPENCV_FOR_THREADS_NUM=1` (kills the cv worker-pool noise; timing distortion is irrelevant in TSAN runs), a suppressions file grown from first triage (`called_from_lib:libpango*`, `called_from_lib:libopencv*`), and keep the Viewer ON (P10 touches Viewer; xvfb headless is fine — TSAN intercepts pthreads, not GL).

**TSAN smoke recipe** (proposed `benchmark/scripts/tsan_smoke.sh`, invoke binaries directly so truncated EuRoC timestamp files can be passed — no code change):
```
TSAN_OPTIONS="history_size=7 second_deadlock_stack=1 exitcode=0 log_path=<rundir>/tsan suppressions=benchmark/tsan/suppressions.txt"
```
| run | scenario | triggers | est. runtime (compute-bound: pacing `usleep((T-ttrack))` vanishes once ttrack>T) |
|---|---|---|---|
| T1 | euroc_stereo MH01, truncated timestamps ~45 s (first ~900 lines) | init + steady LM + backpressure: **R4 fires every run** (guaranteed baseline signal), R1-class window | ~8–15 min |
| T2 | kitti_stereo 07 **full** (1101 frames, 1 loop) | the only cheap loop trigger: **R2, R-c, R-d, R-f**, GBA lifetime | ~10–20 min |
| T3 | euroc_stereo_inertial MH01, truncated ~75 s | IMU init + VIBA1/2 drains: **R3, R5** | ~10–18 min |
| T4 (optional) | euroc_mono_inertial truncated | reset storm (14 resets/run per golden trace): **R-b**, reset protocol | ~10 min |

TSAN build ~15–25 min first time (cold ccache for new flags). Per-step subset T1+T2 ≈ 25–40 min; full suite ≈ 45–75 min. Give TSAN runs their own generous `timeout` (45 min/run) and never run them concurrently with gate rounds (16-core contention finding in DIVERGENCES).

**Expected pre-existing noise — and the correct success criterion.** The tombstone contract (154 lock-free `isBad()` deref sites, Atlas lock-free methods, scribble fields — OWNERSHIP.md calls these *contract, not lock*) guarantees TSAN output is **not reducible to zero** in P10. Therefore the criterion is a **signature-set diff**, not TSAN-clean:
1. Normalize reports to signatures (dedup by top-frame pair).
2. Before any migration commit, capture the **baseline ledger** from the phase-9 tag binary rebuilt under TSAN, and map each documented race (R1–R5, R-a..R-f) to observed signatures. Honest expectations: R4 always fires; R2/R-c/R-d/R-f fire on T2; R3/R5 on T3; R-a likely silent; **R1 and R-b are narrow-window and may never fire** — their elimination evidence is protocol-by-construction (lock/CV code) plus no-new-signatures, and the ledger must say so rather than claim a TSAN delta.
3. Per step: ledgered signatures the step claims to fix are gone; no new signatures at migrated sites; unrelated tombstone noise unchanged.
4. Keep `report_thread_leaks` on: the never-joined LC/LM threads and detached GBA produce leak reports today — their disappearance is direct evidence for the P10 Shutdown protocol (OWNERSHIP.md GBA section).
5. GCC-11 libtsan's lock-order-inversion detection is weak — treat any deadlock report as bonus; liveness evidence comes from §3.

## 2. What existing gates still prove under intentional timing change

- **Paired full gate v2.2.1 + SI 8v8**: still the outcome arbiter. Its interleaved-paired design controls machine state, not timing — it proves *distributional ATE equivalence under each arm's own timing*. Because P10 shifts which frames become KFs, expect real shifts especially in stereo-inertial (highest KF insertion rate, queue-depth<3 backpressure rule — the #20 mechanism); the gate can say "quality not worse", never "timing change was the intended one". Necessary, not sufficient. Keep the standing SI 8v8 (#20 protocol, re-measured clean in P9: −7.7%, p=0.677).
- **Bit gate: verified unaffected.** `tests/bit_identity/extract_hash.cpp` is a single-image, thread-free harness (confirmed by inspection); the production stereo path does run two extraction threads (`Frame.cpp:121-124`) but on disjoint extractor instances and outputs, so the feature layer stays deterministic. P10 doesn't touch it. Under P10 the bit gate only proves the rebuild didn't disturb the deterministic layer (flag/codegen parity) — keep as the cheap per-step tripwire it already is.
- **Smoke gate**: completion + coarse ATE — valid per-step, freshness guard already present.
- **Free bonus check**: run.log already prints "Map 0 has N KFs"; #20 established the SI band (122–132). A step that exits the historical KF-count band is a timing-behavior red flag even when ATE passes. Zero new code.
- **What none of them prove**: interleaving correctness / absence of data-race UB (→ TSAN, §1), deadlock/livelock freedom and shutdown liveness (→ §3), latency changes masked by sensor pacing (→ §4). Golden state traces (P7-1a) add a cheap shape invariant for visual modes (3-line healthy shape); mono-inertial trace is nondeterministic — compare shape statistics only.

## 3. Deadlock / liveness evidence (all cheap)

- **Bounded-time completion**: wall clock is pacing-dominated and therefore *stable* — measured from gate.log: EuRoC modes 3.4–3.7 min/run, KITTI 00 ~8.9 min. Add a `RUN_TIMEOUT` env to `docker/scripts/run_slam.sh` wrapping the binary in coreutils `timeout` (EuRoC 360 s, KITTI 07 300 s, KITTI 00 900 s ≈ 1.6× golden). Exit 124 = hang = hard FAIL, cleanly distinguished from the tolerated teardown crash (project policy judges by result files; a hang produces neither files nor exit).
- **Shutdown-clean check**: keep the result-file policy, add two observations per run: (a) both "End of saving" lines present in run.log, (b) wall time from the "Shutdown" log line to process exit. P10's join/jthread work may legitimately turn the known teardown SIGABRT/SIGSEGV into clean exit — record the transition as an improvement in DIVERGENCES, don't gate on exit 0 yet.
- **6-mode bounded-time table at phase end**: the full gate already timestamps every run in gate.log — a small awk in the verdict step yields per-mode runtime vs golden ±10%. Zero extra runs.
- **CV-wakeup unit checks**: no gtest infra exists (tests/ = bit_identity + backend_equiv only). Justified minimal addition: P10 introduces new primitives (CV'd queue, stop/finish latch) — one assert-style binary `tests/threading/` (same dependency-free style as extract_hash), 4–6 bounded-wait checks each under a watchdog: insert wakes parked consumer (<50 ms), RequestStop→isStopped converges, **reset-during-parking is now serviced** (the documented P8 protocol quirk that CV migration fixes), finish wakes parked thread, destructor joins. Runs in <1 s, wired into every step's build. This is the only place a unit harness pays for itself in P10: it tests the new primitive, not the SLAM.
- The two documented non-deadlock spins (park-loop reset non-service; R-f RequestReset-after-SetFinish permanent spin) never trigger in gate scenarios — their evidence is the unit checks plus code-level protocol argument, stated explicitly in the phase report.

## 4. Runtime-duration instrumentation — the "CV didn't add latency" observable

Wall clock is useless (pacing-dominated). Cheapest credible metric: **KF dequeue latency + queue depth**, via the proven P7-1a env-gate pattern (`ORB_TRACE_QUEUE=1`, off-by-default one-bool-test, file/stderr sink):
- Stamp `steady_clock` at `InsertKeyFrame` push (parallel timestamp queue under the same mutex — no KeyFrame field changes), read at `ProcessNewKeyFrame` pop; at `SetFinish` emit summary: n, p50/p95/max dequeue latency, max queue depth, total loop iterations, empty-poll iterations. ~30–40 lines in LocalMapping; reuse for the LC queue (5 ms poll).
- Expected signature: **before** = latency floor ~1.5–3 ms (3 ms usleep poll) + backlog; **after** = sub-0.1 ms wakeup with unchanged backlog; empty-poll iteration count collapses (~333/s → ~0). This is simultaneously the direct latency proof and the backpressure-behavior probe (queue-depth<3 is the SI-sensitive mechanism per #20).
- Capture golden queue traces per mode from the phase-9 tag binary → `benchmark/golden/queue_traces/`, mirroring `state_traces/`.
- Zero-code companions: KF-count band (§2), per-run wall time (§3). Optional non-judged: `/usr/bin/time -v` CPU seconds (polling removal shaves a small stable amount).

## 5. Evidence packages

**Capture order first**: before commit S1, from the phase-9 tag — (a) TSAN baseline ledger (T1+T2+T3), (b) golden queue traces. These are the "before" halves of every delta below.

Assumed step split (S1 LM queue/backpressure CV → S2 LM stop/reset/finish flags + atomics [R4, R5] → S3 LC run-loop + queue [R-a, R-b, R2] → S4 GBA flags + lifetime [R-c..R-f] → S5 Shutdown/jthread/join protocol → S6 Viewer split + Tracking spin-waits). Per-step package:

| evidence | S1 LM queue | S2 LM flags | S3 LC loop | S4 GBA | S5 Shutdown | S6 Viewer/T |
|---|---|---|---|---|---|---|
| build + flags evidence (build.ninja rules) | x | x | x | x | x | x |
| bit gate | x | x | x | x | x | x |
| smoke gate, timeout-wrapped + KF-count band | x | x | x | x | x | x |
| threading unit checks (from S1 on) | x | x | x | x | x | x |
| kitti_stereo 07 loop smoke (loop confirmed, P9-3 method) | — | — | x | x | x | — |
| TSAN delta T1+T2 vs ledger | x | x | x | x | x | x |
| TSAN T3 (inertial) | — | x | — | x | — | — |
| queue-trace delta (golden vs step) | x | x | x | — | — | — |
| mid-phase SI probe, 4 pairs (`test_si_at.sh` pattern — port to benchmark/scripts per #20 note) | x | x | — | — | — | — |
| thread-leak report delta | — | — | — | x | x | x |

Per-step cost ≈ 40–60 min (bit 2 + smoke 4 + kitti07 6 + TSAN subset 25–40 + units <1; SI probe +30 where used).

**Phase-end package** (~4.5–5.5 h total): full gate v2.2.1, 4 rounds, KITTI_SEQ=00 + standing SI 8v8 · TSAN before/after summary table — one row per R1–R5/R-a..R-f: {expected signature, fired-before? (or "narrow-window: not observable"), gone-after?, evidence type}, plus new-signature scan and thread-leak before/after · 6-mode bounded-time table vs golden ±10% · shutdown matrix per mode {clean / tolerated-crash / hang} · queue-latency before/after medians per mode · state-trace shape check (visual modes) · DIVERGENCES entries for every behavioral delta (park-loop reset service, Shutdown wait restoration, teardown-crash disappearance if any) · phase-10 tag.

Key file paths: `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/docker/Dockerfile`, `docker/scripts/container_build.sh` (TSAN tree goes beside its `/build/cmake`), `docker/scripts/run_slam.sh` (add RUN_TIMEOUT), `benchmark/scripts/{smoke_gate.sh,bit_gate.sh,full_gate_paired.sh}` (unchanged; awk addition to verdict step), `benchmark/golden/state_traces/README.md` (pattern to copy for queue traces), `docs/OWNERSHIP.md` + `docs/P8_RECON.md` §4 + `docs/P9_RECON.md` §4 (race→signature ledger source), `docs/DIVERGENCES.md` #20 (SI 8v8 protocol + `test_si_at.sh` porting note).