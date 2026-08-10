# P11 정찰: 최종 옵션 페이즈 — 수명 계약 · FixLevel · 백엔드/어휘/직렬화

2026-08-11, 3-agent 병렬 정찰 종합. phase-10 (5c57017) 기준.
아래 3부는 에이전트 원문 그대로 보존한다.

**확정 실행 계획** (3부 권고 수렴):

| 순서 | 워크스트림 | 규모 | 요지 |
|---|---|---|---|
| P11-V | DBoW2 바이너리 어휘 로드 (1안) | S | 실측 2.09s→~0.2s(엣지에서 더 큼), 게이트 중립 by construction, .osa 체크섬은 텍스트 앵커 유지 |
| P11-A | **Frame/IMU 메시지 패싱** + 스칼라 atomic + B3(Release 드레인 #19형 강등) + Atlas 락 배치 | M | T3 잔존 레이스 클래스(~14 시그니처) 전소가 목표 — TSAN T3가 수용 오라클. W9 스핀/B1 행 위험 은퇴. 스냅샷이 아니라 메시지 패싱(UpdateFrameIMU는 변이가 목적) |
| P11-F0~F6 | **FixLevel 워크스트림** | M | 런타임 플래그(동일 바이너리 ON/OFF — #20 교훈이 설계 동인: ifdef면 레이아웃 교란으로 ON 검증 오염), level-0 무해성 증명 프로토콜, F1 #9(3줄, s_true=2.0 픽스처)→F2 #24→F3 #21+L1/L2+D5(+멀티맵 스모크)→F4 #5→F5 #13(LDLT 벤더드 헤더)→F6 #6+#3 |
| 페이즈 게이트 | 전 플래그 OFF 풀 게이트 + SI 8v8 | — | **SI 청정 3연속 시 #20 CLOSED** |
| **P12 이월** | GTSAM(검증 체제 분기+선행 조건 미비 — 버전 결정만 D-기록), 직렬화 재설계(수명 결정의 하류 — 먼저 하면 2번 지불), 툼스톤/shared_ptr 에픽(C-lite는 XL 비용에 회수 0, C-full은 152개 만료 결정에 오라클 부재) | — | 결합 관계를 OWNERSHIP에 기록: shared_ptr ↔ 직렬화는 한 결정, GTSAM ISP 동결 전에 확정할 것 |

핵심 판단 근거: P10이 지명한 잔존 위험 2건(T3 Frame 클래스, Release 드레인)이
정확히 P11-A의 표적이고 152-사이트 계약을 건드리지 않고 제거 가능하다. 툼스톤은
결함 백로그가 아니라 **작동하는 계약**(P1-P10 무사고)이며, XL 규모 diff는 #20
실증에 의해 게이트로 귀속 불가능하다.

---

# ═══ 1부: 수명/스마트포인터 이행 (lifetime-migration) ═══

All evidence gathered from the working tree (post-phase-10), the TSAN ledgers in `benchmark/tsan/`, and docs. Report follows.

---

# P11 Smart-Pointer / Lifetime-Contract Migration — Scoping Report

Basis: phase-10 tip of `/Users/jhpark/VSLAM/ORB_SLAM3_Modern`, TSAN ledgers `benchmark/tsan/baseline_b0f829a/` + `step_P10-2_T3.ledger`, `docs/OWNERSHIP.md`, `docs/IMU_CONTRACT.md`, `docs/P10_RECON.md`.

## (1) Tombstone reality post-P10

### 1a. Lock-free `isBad()` deref census — verified today: **152 sites, 15 files** (OWNERSHIP.md's "154 (13 files)" is a stale P8 count; recommend updating)

| File | Count | Executing thread(s) | Pattern class |
|---|---|---|---|
| src/backend/Optimizer.cpp | 62 | LM, GBA, LC, T | **Local raw-pointer vectors** built under map/pose locks, then deref'd lock-free across the whole optimize+recover span |
| src/closing/LoopClosing.cpp | 20 | LC | Loop/merge correction sets, SearchAndFuse, essential-graph prep |
| src/features/ORBmatcher.cpp | 11 | T, LM | Hot inner matching loops over `MapPoint*` |
| src/tracking/Tracking.cpp | 10 | T (+**LM/LC**: the `while(pKF->isBad()) pKF=pKF->GetParent()` walk inside `UpdateFrameIMU` runs on LM/LC threads) | Local-map update, relocalization, parent-chain walks |
| src/mapping/LocalMapping.cpp | 10 | LM | Culling, purge guard (#19), neighbor scans |
| src/core/System.cpp | 10 | S/V | All in `Save*Trajectory*` — parent-chain walks over `mlpReferences`; post-Shutdown safe since P10-5 join chain; mid-run viewer save remains contract-only |
| src/map/Map.cpp | 8 | S (PreSave) | Serialization prep — now effectively single-threaded (SaveAtlas after join chain) |
| src/map/KeyFrame.cpp | 7 | all | Graph maintenance derefs of *other* KF/MP while holding own `mMutexConnections` |
| src/closing/PlaceRecognition.cpp | 5 | LC | Candidate filtering |
| src/viz/MapDrawer.cpp | 2 | **V** | Draw loops — the Viewer-thread contract reads |
| src/mapping/ImuInitializer.cpp | 2 | LM | Inertial-init KF list |
| src/map/MapPoint.cpp | 2 | all | `Replace`/`GetReplaced` chain |
| src/recognition/KeyFrameDatabase.cpp | 1 | LC/T | The #23-guarded candidate loop |
| src/geometry/Sim3Solver.cpp | 1 | LC | Solver-local |
| src/geometry/MLPnPsolver.cpp | 1 | T | Solver-local |

Note the contract's precise shape: `isBad()` itself locks the object's own mutex (KeyFrame.cpp:679, MapPoint.cpp:301). The hazard is **object lifetime, not `mbBad` visibility** — every one of the 152 sites derefs a stored raw pointer whose liveness is guaranteed only by "nothing ever deletes" (OWNERSHIP rule 1). Any migration that introduces true reclamation must re-prove all 152.

### 1b. Remaining delete sites (KF/MP surface)

| Site | What | Status |
|---|---|---|
| `src/mapping/LocalMapping.cpp:960` | **Release() queue drain: raw `delete`, no SetBadFlag** | Since P10-2 under `mMutexNewKFs` (R1 list-corruption fixed) but the **dangling risk is intact**: `Tracking::mpLastKeyFrame`, `Frame::mpLastKeyFrame`, and (stereo/RGBD) MapPoint observations created in `CreateNewKeyFrame` all alias the deleted KF. The sole surviving documented UAF-risk delete. |
| `src/mapping/LocalMapping.cpp:1338-1346` | IMU-init purge: SetBadFlag → `isBad()`-guarded delete | #19 pattern, UAF→leak demoted (P8-3), under lock since P10-2 |
| `LocalMapping::ResetIfRequested` ×2 | clear-only (intentional leak) | Safe by protocol (T spins during reset) |
| `src/tracking/Tracking.cpp:1489` | temporal MapPoints delete | T-thread-confined, never registered in map — safe, out of scope |
| Non-KF/MP: Tracking preintegrated ×3, `Atlas` dtor maps, `Optimizer.cpp:5459` (`mpcpi` frame scribble), `Map.cpp:53` thumbnail | — | Out of scope |

Plus the load path: boost deserialization **creates** KF/MP with `new` (ownership originates in the archive), and culling-deferred SetBadFlag completes on the **LC thread** via `SetErase` — tombstoning itself is cross-thread.

### 1c. Frame-object / IMU-contract race class (the T3 residue)

`step_P10-2_T3.ledger` shows the surviving class exactly as predicted: ~14 signatures, all one shape — **`Frame::operator=` (T, inlined at include/map/Frame.hpp:53) racing LM/LC-thread mutation of `Tracking::mCurrentFrame`/`mLastFrame`**:

- vs `Frame::SetImuPoseVelocity` (:441/:449/:450), `Frame::SetNewBias` (:422), `Frame::imuIsPreintegrated` (:1005/:1006) — all reached from `Tracking::UpdateFrameIMU` (Tracking.cpp:3193-3268) called on **LM** (ImuInitializer.cpp:145/156/339) and **LC** (LoopClosing.cpp:1216/1230);
- vs `ImuInitializer::InitializeIMU:160` (`t0IMU` write + `mCurrentFrame.mTimeStamp` read, no lock);
- plus `Frame::isInFrustum` vs `LocalInertialBA`, `SetState|GetState`, `TrackLocalMap|GetMatchesInliers`.

Why locks can't fix it as-is: 3 of 5 `UpdateFrameIMU` calls *are* under `mMutexMapUpdate` (ImuInitializer:141, LoopClosing:1208/1229), and one is deliberately outside (ImuInitializer:156). But T constructs/copies frames **before** Track()'s lock at Tracking.cpp:1138 (`mCurrentFrame = Frame(...)` at :746-850, PreintegrateIMU, `mLastFrame = Frame(mCurrentFrame)` sites) — so even the "locked" calls race T's pre-lock pipeline. `Frame` has ~100 members; per-Frame locking is impractical and W9 (LM spinning on T's live frame `imuIsPreintegrated`, Tracking.cpp:3224-3227, with B1's missing-`setIntegrated` hang risk) is itself part of the defect.

**What eliminates the class — message-passing, not snapshotting.** Snapshotting fails structurally: `UpdateFrameIMU`'s purpose is *mutation* of T's frames, not reading them. The sound redesign:

- LM/LC post an `ImuUpdateMsg {scale, bias, baseKF, t0IMU-if-first-init}` into a single mutex'd slot; T applies it (the existing `UpdateFrameIMU` body, now T-thread-only) immediately after acquiring `mMutexMapUpdate` at Track():1138, before any frame state is consumed.
- The W9 spin dissolves (T applies after its own preintegration naturally — also retires hang risk B1); `t0IMU` rides in the message; `mState_` underlying int and `mnMatchesInliers` become atomics (they are pull-reads, not protocol).
- Net effect: **Frame objects become T-thread-confined** (FrameDrawer copy already mutexed). Semantics delta: the rescale/rebias lands at T's next lock acquisition instead of mid-LM — a bounded staleness window (T's pre-lock section only) that upstream already exhibits *nondeterministically*; ApplyScaledRotation-vs-frame-consistency is preserved because T can't touch world-frame state without the same lock. Gate-judged, DIVERGENCES entry required.

## (2) Full shared_ptr migration — design sketch and blockers

Fan-out measured: **1,279 occurrences** of `KeyFrame*`/`MapPoint*` across src/+include/.

Blockers:

1. **Boost serialization**: `Map::mvpBackupKeyFrames/mvpBackupMapPoints` (Map.hpp:173-174, serialized at :63-64) and `Atlas::mvpBackupMaps` (Atlas.hpp:68/154) are **raw-pointer vectors serialized through the pointer** — boost's `new` on load is where ownership originates. Switching to `shared_ptr` changes the archive format (map-file break; version/converter needed) and every PreSave/PostLoad reconnection signature. KeyFrame-level refs are already ID-flattened (`mvBackupMapPointsId`, `mBackupParentId`, `mBackupPrevKFId`… KeyFrame.hpp:120-158) — the break is confined to the Map/Atlas layer but is unavoidable. **This couples the migration to the P11 serialization-redesign candidate — they are one decision.**
2. **Static ID counters** (`ar & Map::nNextId; Frame::nNextId; KeyFrame::nNextId; MapPoint::nNextId; GeometricCamera::nNextId`, Atlas.hpp:71-75): not a pointer blocker per se, but the load path's object-identity plumbing must produce shared_ptr end-to-end.
3. **`mpReplaced` chain** (MapPoint.hpp:225): MP→MP links with no static acyclicity guarantee; strong links can cycle (leak-equivalent to tombstone, so tolerable — but it means shared_ptr **cannot deliver reclamation** here without a cycle audit that doesn't exist).
4. **Spanning tree**: `mpParent`/`mspChildrens` (KeyFrame.hpp:417-418) are mutually referential. The parent-chain walk over *bad* KFs (`UpdateFrameIMU`, `Save*Trajectory*` ×6, LoopClosing) requires bad KFs' parents to stay alive ⇒ **child→parent must be strong, parent→children weak**. SetBadFlag's re-parenting loop then runs on `weak_ptr::lock()`.
5. **IMU chain**: `mPrevKF`/`mNextKF` (KeyFrame.hpp:355-356) doubly-linked; `MergePrevious` in culling needs prev alive ⇒ **prev strong, next weak**, with culling rewiring both directions through lock() checks.
6. **KF↔MP cycle**: `KeyFrame::mvpMapPoints` vs `MapPoint::mObservations`+`mpRefKF`. One side must be weak; but the 152-site contract reads *both* sides on bad objects (e.g. `TrackedMapPoints`, trajectory refKFs) ⇒ whichever side goes weak converts those sites into expiry decisions.
7. **Optimizer local vectors** (62 sites, plus `BAEpochs` side-tables keyed `map<KeyFrame*,…>`): copying shared_ptr in hot loops = refcount churn in the 53-81% bottleneck. The sound pattern is a **pin-set at function entry** (lock weak/copy shared once, raw internally) — but that's a per-function contract to design and prove, ×17 optimizer functions ×2 backends once GTSAM lands.
8. **KFDB inverted file** (`std::vector<list<KeyFrame*>>`, KeyFrameDatabase.hpp:69, ~1M words): weak_ptr doubles entry size and adds atomic ops in the hottest query loops; strong pins bad KFs beyond the current erase-on-SetBadFlag semantics (deferred-erase KFs pinned until LC's SetErase). Either choice is measurable.
9. **enable_shared_from_this**: 12 `this`-passing sites (KeyFrame.cpp 10, MapPoint.cpp 2 — SetBadFlag/EraseObservation/AddObservation plumbing) ⇒ both classes need `enable_shared_from_this` + factory functions (`shared_from_this()` is UB in ctors; creation sites: 3 KF + 6 MP + boost load path).

**The decisive fork**: (a) *C-lite* — shared_ptr with strong pins preserving tombstone liveness: nothing is ever freed (same as today), so you pay the full 1,279-site fan-out, archive break, and hot-loop churn for type-documented custody with **zero reclamation benefit**; (b) *C-full* — weak links + true reclamation on SetBadFlag: resurrects the exact UAF question the tombstone exists to avoid, as 152 individually-judged expiry decisions, each an observable behavior change (bad-but-readable → expired-unreadable). C-full is the thing OWNERSHIP rule 1 forbids without per-site lifetime proofs.

## (3) Realistic increments

- **A (Frame/IMU message-passing + scalar atomics)** — eliminates the entire T3 residual class without touching KF/MP custody. Self-contained: 5 call-site conversions, 1 message struct, 3 atomics, W9/B1 retirement. TSAN T3 is a direct acceptance oracle (the ~14 `Frame::operator=` signatures must vanish, no new ones).
- **B (queue-KF custody)** — two sub-options, composable:
  - **B3 (disposal demotion, the actual risk payoff)**: change `Release()`'s drain (LocalMapping.cpp:960) from raw delete to the #19 pattern (SetBadFlag → `isBad()`-guarded delete). For un-admitted queue KFs, `EraseKeyFrame`/KFDB-erase are no-ops; observation detach removes the dangling MP back-references raw delete leaves today. UAF→leak demotion identical in kind to P8-3; **does not touch the 152-site contract at all**. Pathological-window-only behavior delta → DIVERGENCES entry.
  - **B1 (unique_ptr through `mlNewKeyFrames` until `AddKeyFrame`)**: compiler-enforced custody documentation. Honest limit: it does **not** remove the dangle by itself (Tracking's aliases are non-owning), and the ResetIfRequested intentional leak needs explicit `release()`. Value = typed custody + making every drain's disposal decision explicit in code. Behavior-identical if done carefully.
- **C (full shared_ptr)** — see (2); only rational bundled with the serialization redesign, and only *before* GTSAM freezes the ISP interface signatures (or keep ISP interfaces raw + documented pin-set convention, which is my recommendation regardless).

## (4) Effort / gate risk / what gates can still prove

| Increment | Effort | Gate risk | What the gates prove |
|---|---|---|---|
| A: IMU message-passing (+`mState_`/`mnMatchesInliers`/`t0IMU`) | **M** (2-3 gated commits) | MEDIUM — inertial-init application timing shifts; judged on paired **mono-inertial + SI 8v8** | Bit gate holds (feature layer untouched); **TSAN T3 signature-class elimination is the mechanism proof**; ATE pairs prove outcome-not-worse |
| B3: Release-drain UAF→leak | **S** (1 commit) | LOW — window unreached in gates | Smoke + bit gate hold; evidence is by-construction + DIVERGENCES entry (same standard as #19/#23/#25) |
| B1: unique_ptr queue custody | **S/M** | ~0 if leak-preserving | Bit gate + smoke; behavior-identical by design |
| C-lite: shared_ptr, strong pins | **XL** (1,279 sites, archive break, factories, KFDB/BAEpochs key types) | **HIGH** — #20 proved layout-only churn moves SI ATE +9-13%; a diff this size makes any gate failure unattributable, violating the bisectable-increment discipline | ATE gates prove only "not worse"; **no gate proves lifetime correctness**; equivalence tests break if optimizer signatures change |
| C-full: weak links + reclamation | **XL+** | UNACCEPTABLE — 152 observable expiry decisions, no oracle | Nothing existing can prove it; would need ASan+deterministic replay infra that doesn't exist |

## Recommendation: **partial custody + document-and-defer the full migration**

**Do in P11**: A (M) + B3 (S), optionally B1 (S/M), as separately gated commits in that order (A first — it retires the named P10 carry-over and has the strongest oracle).

Evidence for this split:
1. The two *actual* residual risks P10 named are exactly A's and B3's targets: the T3 Frame class (OWNERSHIP P10-2 NOT-fixed list; ~14 live TSAN signatures in `step_P10-2_T3.ledger`) and the Release-drain dangle (OWNERSHIP "업스트림 그대로의 댕글링 위험", explicitly deferred to "T측 협조" — which A's thread-confinement supplies). Both are killable without entering the 152-site surface.
2. The tombstone is a *working* contract, not a defect backlog: P10's own success criterion already treats the 152 sites as stable noise ("소거 불가 잡음"), and zero gate incidents trace to it across P1-P10.
3. Full migration's only real payoff over tombstone (memory reclamation) requires C-full, which converts a proven contract into 152 unproven expiry decisions; C-lite pays XL cost for no reclamation. Neither passes a cost/evidence test under the #20 finding that this codebase's ATE gate cannot attribute XL-sized diffs.
4. Deferral is not free-floating: record in OWNERSHIP.md that the shared_ptr decision is **coupled to the serialization redesign** (shared archive break) and must be settled **before** GTSAM ISP signatures freeze — until then, ISP interfaces stay raw-pointer with the pin-set convention documented, so a later migration doesn't fan out ×2 backends.

Key files: `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/src/tracking/Tracking.cpp` (:3193-3268 UpdateFrameIMU, :1138 lock, :746-850 pre-lock frame ctors), `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/src/mapping/ImuInitializer.cpp` (:141-160), `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/src/mapping/LocalMapping.cpp` (:960 Release drain, :1338-1346 #19 purge), `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/include/map/{Map.hpp,KeyFrame.hpp,MapPoint.hpp}` (serialization/chain fields), `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/benchmark/tsan/step_P10-2_T3.ledger` (live Frame-class signatures), `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/docs/{OWNERSHIP.md,IMU_CONTRACT.md,DIVERGENCES.md}`.
---

# ═══ 2부: FixLevel 설계·후보 감사 (fixlevel-design) ═══

# FixLevel Design & Candidate Audit — ORB_SLAM3_Modern (phase-10 tip, 5c57017)

## 1. Candidate enumeration (verified file:line as of phase-10 tip)

### FL-9 — EdgeInertialGS scale-update divergence (DIVERGENCES #9) — STANDING FIRST PRIORITY
- **Sites**: `include/backend/G2oTypes.hpp:314-316` (`VertexScale::oplusImpl` — multiplicative `s*exp(w)`); `src/backend/G2oTypes.cpp:715-717` (`EdgeInertialGS::linearizeOplus` — additive Jacobian `∂r/∂s` without the chain-rule factor). Twin copy in `tests/backend_equiv/reference_backend/G2oTypes.cpp` (same shape, stays unfixed as the OFF-reference).
- **Fix sketch** (3 lines, in `linearizeOplus`; `s` is already in scope at :707):
  ```cpp
  const double sj = FixFlags::I().scaleJacobianChainRule ? s : 1.0;  // cached bool, see mechanism
  _jacobianOplus[7].block<3,1>(3,0) = sj * (Rbw1*(VV2->estimate()-VV1->estimate()));
  _jacobianOplus[7].block<3,1>(6,0) = sj * (Rbw1*(VP2->estimate().twb-VP1->estimate().twb-VV1->estimate()*dt));
  ```
  (Chain rule for the multiplicative update: `∂r/∂w = ∂r/∂s · s`. At s≈1 identical, which is why the bug only bites when scale is badly wrong.)
- **Blast radius**: **mono-inertial only.** Scale vertex is free only at `src/backend/Optimizer.cpp:3284` (`InertialOptimization` full variant, `setFixed(!bMono)` at :3290-region) and `:3601` (`InertialOptimization(Rwg,scale)`, reached via `ImuInitializer::ScaleRefinement` at `ImuInitializer.cpp:324`, gated `mbMonocular` at `LocalMapping.cpp:261-262`). Stereo/RGBD-inertial: vertex fixed → Jacobian never enters the normal equations → provably no effect.
- **Testability**: excellent. Existing fixture `tests/backend_equiv/common/EquivFixtures.hpp:104-170` (`kScaleGravity`) documents s_true=1.1 was **forced by this bug**. ON-config test: same fixture with s_true=2.0 asserting convergence (property test vs ground truth, not twin-equivalence — reference backend keeps the bug by design). Plus `EQUIV_INERTIAL_DEBUG=1` manual repro (`EquivMain.cpp:252`). Gate: paired MI EuRoC (the judgment domain). Maps directly to the Remastered mono-inertial regression lead.

### FL-24 — raw mbFixScale into OptimizeSim3 (DIVERGENCES #24)
- **Site**: `src/closing/PlaceRecognition.cpp:510` (BoW path; intent comment at :508-509). The relaxed `bFixedScale` is already computed at :447-449 **and is in scope** (both inside the `:444 if(numBoWMatches >= nBoWMatches)` block).
- **Fix sketch**: one token — pass `bFixedScale` instead of `mHost.mbFixScale` at :510 (matching the other 3 sites, e.g. :324-327).
- **Blast radius**: mono-inertial pre-BA2 window only (Sim3 refinement runs with scale free instead of fixed). Numeric change in early-MI loop/merge candidates; all other modes identical (`bFixedScale == mbFixScale` there).
- **Testability**: gate-visible in MI mode; no unit fixture needed (shares FL-9's validation runs).

### FL-21 — poisoned-consume composite in loop detection (DIVERGENCES #21)
- **Sites**: (arm A) `src/closing/LoopClosing.cpp:167-174` — merge scale-abort `continue` escapes with `LoopCh().detected` intact and skips `mpLastCurrentKF` update (:287); stale `LoopCh().slw` (anchored to previous KF) is consumed at :230 next iteration. (arm B) `src/closing/PlaceRecognition.cpp:108` — loop reffine failure calls `ChannelDecayStep(..., /*bClearDetected=*/false)` (merge twin at :137 passes `true`). Side asymmetries preserved: merge advance keeps `numNotFound` (:130), BoW seed carries `numNotFound` (:289-305), merge `mps` evidence never read.
- **Fix sketch** (either arm breaks the composite; ship both, one flag):
  - at `LoopClosing.cpp:174`, before `continue`: `if(FixFlags::I().loopStateHygiene && mPlaceRec.LoopCh().detected) mPlaceRec.WipeLoopOnMergePriority();`
  - at `PlaceRecognition.cpp:108`: `ChannelDecayStep("loop", mLoopCh, /*bClearDetected=*/FixFlags::I().loopStateHygiene);`
- **Blast radius**: LC thread only; changes which KF consumes a loop hypothesis in the pathological conjunction (multi-map + inertial scale outside [0.90,1.1] + concurrent loop DETECTED + subsequent reffine fail). Single-map gate scenarios: unreachable, bit-identical.
- **Testability**: **the full composite is NOT reliably reachable in a gate scenario.** The merge path itself *is* reachable: `mono_inertial_euroc.cc:49` / `stereo_inertial_euroc.cc:50` already support multi-sequence (`num_seq`), and `System::ChangeDataset` (`System.cpp:1375`) does `CreateMapInAtlas` → multi-map → cross-map merges on MH01+MH02 (same hall). But the scale-abort branch additionally needs merge scale outside [0.90,1.1] — in a well-initialized run scale≈1, so the conjunction is luck-dependent. Answer: (i) **unit-test the typed machine** (P9-4 `DetectionChannel` + trace hooks make ChannelBoWSeed/Advance/DecayStep sequences drivable headlessly — assert `detected` cleared at the right steps); (ii) add an MH01+MH02 multi-session run as an integration smoke (assert via channel trace that ≥1 merge consumed and no stale-anchor consume), not as a paired-ATE gate.

### FL-L1/L2 — SetNotErase latch leaks (OWNERSHIP, P9_RECON D2/D3)
- **Site**: `src/closing/PlaceRecognition.cpp:289-305` (`ChannelBoWSeed`; leak comment :284-288). L1: seeding overwrites `c.matchedKF`/`c.lastCurrentKF` without `SetErase` on the old latch → permanent pin, culling blocked. L2: seed with `nBestNumCoincidences==0` latches a KF the decay path (guarded `numCoincidences > 0` at :80/:114) can never release → orphan latch + stale `numNotFound` carryover.
- **Fix sketch** (one flag): at seed entry: `if(FixFlags::I().latchHygiene){ if(c.matchedKF) c.matchedKF->SetErase(); if(c.lastCurrentKF && c.lastCurrentKF!=mHost.mpCurrentKF) c.lastCurrentKF->SetErase(); c.numNotFound=0; }` and skip `SetNotErase`/seed when `nBestNumCoincidences==0`.
- **Blast radius**: KF-culling eligibility in LC-active runs (a previously pinned KF may now be culled → downstream covisibility/graph deltas possible in any mode with loop closing). Memory/graph health win on long multi-map sessions; cosmetic for short gates.
- **Testability**: unit test on the machine (seed twice, assert old latch released; seed with cnt=0, assert no latch). Gate: standard paired runs to bound ATE impact.

### FL-D5 — LC reset doesn't clear the detection machine (OWNERSHIP D5)
- **Site**: `src/closing/LoopClosing.cpp:1458-1523` (`ResetIfRequested`; D5-visibility trace at :1471-1472 and :1488-1491 — queue cleared, channels survive pointing into the torn-down map for up to 2 KFs).
- **Fix sketch**: add `PlaceRecognition::ResetChannels()` = `ChannelWipe("loop",...)` + `ChannelWipe("merge",...)` (def :272-282, already releases latches via SetErase — which also completes deferred SetBadFlag, the #19 completion path); call it in both reset branches under the flag.
- **Blast radius**: only runs that reset (tracking-lost map resets, `ChangeDataset` with <12 KFs). Prevents cross-map reffine attempts against a destroyed map.
- **Testability**: unit test (populate channels, fire reset, assert wiped). Deterministic mid-run reset is hard to force in EuRoC gates; multi-session smoke covers `ChangeDataset`-reset side. The existing `TraceReset` lines give the assertion hook.

### FL-13 — SimplicialLDLT→LLT robustness regression (DIVERGENCES #13, F3)
- **Sites**: `src/backend/Optimizer.cpp:1654-1659` and `:1947-1952` (both `OptimizeEssentialGraph` overloads: `LinearSolverEigen<BlockSolver_7_3>` + `setUserLambdaInit(1e-16)`); watch-comments :1876, :2207 (chi2 no-move detector already instrumented per #15).
- **Fix sketch**: vendored header `include/backend/LinearSolverEigenLDLT.hpp` (copy of upstream g2o `linear_solver_eigen.h` with `SimplicialLLT`→`SimplicialLDLT`, submodule untouched — same pattern as `OrbLevenberg`); at the two 7_3 sites choose solver by flag. Restrict to the two pose-graph sites (the other 10 Eigen sites run with real damping; LLT failure there is recoverable by λ growth).
- **Blast radius**: loop/merge essential-graph corrections in gauge-poor cases (early loops). When LLT succeeds anyway → different factorization → fp-level diffs in *every* loop correction (KITTI 00 gate territory). This is the widest-radius fix numerically.
- **Testability**: hard to trigger the failure in a gate; validation = (i) unit fixture: hand-built near-semidefinite 7_3 pose graph where LLT fails and LDLT solves, asserting the ON path moves chi2; (ii) KITTI 00 (4-5 loops) paired ON-vs-OFF; (iii) the #15 "pose graph didn't move" stderr counter as the field detector in both arms.

### FL-5 — Frame::mb read-before-init (DIVERGENCES #5)
- **Sites**: `include/map/Frame.hpp:216` (`float mb;` no initializer); stereo ctor `src/map/Frame.cpp:140` calls `ComputeStereoMatches` → `:822` `minZ = mb` (garbage) → `:824` `maxD = mbf/minZ`; assignment only at `:167`.
- **Fix sketch**: under flag, hoist `mb = mbf / K.at<float>(0,0);` above :140 (or hoist the whole fx/…/mb block :160-167).
- **Blast radius**: stereo + RGBD(-inertial) stereo-match search range **on every frame** (fresh Frame object per frame = fresh indeterminate read; genuine UB and a latent nondeterminism source the golden baseline merely got lucky with). Gate-measurable directly on stereo modes.
- **Testability**: trivially unit-testable (construct stereo Frame, assert match count deterministic across two constructions with poisoned stack); paired stereo/KITTI gate for ATE impact.

### FL-3 — mnFramesToResetIMU legacy semantics (DIVERGENCES #3)
- **Sites**: `include/tracking/Tracking.hpp:403` (`= 0` canonicalized); consumers `src/tracking/Tracking.cpp:1213, 1419, 1435, 2114, 2226`; provenance comment `:125-135`.
- **Fix sketch**: config-time value only — in `newParameterLoader`: `mnFramesToResetIMU = FixFlags::I().legacyImuResetWindow ? mMaxFrames : 0;` **Zero hot-path cost by construction** (no branch at any read site).
- **Blast radius**: inertial modes after relocalization only (restores the fps-sized post-reloc IMU-reset window of the legacy parser).
- **Testability**: gate scenarios rarely relocalize; validation = MI/SI paired runs (expect null) + a TUM-VI or induced-occlusion reloc scenario if the user ever turns it on. Low-value switch; ship for completeness.

### FL-6 — Rectified calibration2_ pre-resize synthesis (DIVERGENCES #6)
- **Sites**: `src/core/Settings.cpp:337-348` (Rectified clones calibration1_ in `readCamera2`, before resize); resize blocks `:381-389` and `:401-414` explicitly exclude `cameraType_ != Rectified` from calibration2_ scaling.
- **Fix sketch**: under flag, drop the `cameraType_ != Rectified` exclusion in both resize blocks (or re-clone calibration2_ from post-resize calibration1_ at the end of `readImageInfo`).
- **Blast radius**: only Rectified + `Camera.newWidth/newHeight` configs — **no gate yaml uses this** (KITTI/EuRoC unresized). Config-time only, zero runtime cost.
- **Testability**: pure unit test (parse a Rectified+resize yaml, assert calibration2_ == calibration1_). No gate needed beyond the standard OFF-inertness run.

### Enumerated but recommended OUT of FixLevel (different mechanism class)
- **Release() dangling delete** (`src/mapping/LocalMapping.cpp:960`, OWNERSHIP "특수 경로"): recommend an **unconditional #19-style safety demotion** (SetBadFlag → `if(isBad()) delete` → else leak, same pattern as `PurgeNewKeyFramesAfterInertialInit` :1328/:1344), not a flag — precedent: #19/#22/#23/#25/#26 all shipped unconditional because normal-path behavior is identical and the alternative is UAF. Observable difference exists only in the UAF scenario.
- **Atlas unlocked add-methods** (`src/map/Atlas.cpp:113-125`): pure threading safety (take `mMutexAtlas`), numerics-invisible — belongs in the P11 Frame/IMU-contract threading batch (P10-2 style), not FixLevel. A mutex behind a runtime flag is an anti-pattern.
- **Tombstone contract** (152 current lock-free `isBad()` deref sites in src/, was 154): not switchable — a lifetime redesign epic (shared_ptr/hazard-pointer), P12+ candidate alongside GTSAM/serialization.
- **Frame-object/IMU-contract races** (P10-2 NOT-fixed: `Frame::operator=` vs `UpdateFrameIMU`, `t0IMU` write at `ImuInitializer.cpp:160`, `mState_`, `mnMatchesInliers`): P11 redesign scope, not a FixLevel switch.
- **mGlobalMutex**: OWNERSHIP says status-quo (perf note only) — excluded.

## 2. Mechanism recommendation: (b) per-fix runtime flags in one write-once global struct, with a `FixLevel` preset key as sugar; reject (c) ifdef

**Decisive argument against compile-time ifdef**: DIVERGENCES #20 proved this codebase's stereo-inertial ATE moves +9-13% from *semantically inert* binary-layout changes. With ifdefs, ON-vs-OFF are different binaries → every ON validation is polluted by the exact layout confound #20 documented. With runtime flags, ON-vs-OFF is **the same binary, same layout** — the paired gate then measures only the fix's semantic effect. This converts the P1/#20 lesson from a threat into the design driver.

**Design**:
- `include/core/FixFlags.hpp`: plain aggregate `struct FixFlags { bool scaleJacobianChainRule=false; bool loopStateHygiene=false; bool latchHygiene=false; bool lcResetWipe=false; bool ldltPoseGraph=false; bool stereoMbInit=false; bool legacyImuResetWindow=false; bool rectifiedResizeCal2=false; static const FixFlags& I(); }` — written exactly once by `System` ctor from `Settings` **before any thread spawns**, `const` thereafter (no atomics needed; happens-before via thread creation).
- **Yaml**: per-fix keys `Fix.ScaleJacobian: 1` etc. (all optional, default 0), plus `FixLevel: 0|1|2` preset — 0 = none (default, gate), 1 = curated safe set (state-hygiene fixes: FL-21, L1/L2, D5, FL-5, FL-6), 2 = all (adds FL-9, FL-24, FL-13, FL-3). Individual keys override the preset. Gate yamls carry neither key → level-0 by absence.
- **Provenance**: stderr banner + one comment line in saved trajectory headers listing non-default flags; gate scripts grep-assert "no Fix.* active" in gate arms.
- **Hot-path budget**: audit shows **only FL-9 sits in a hot loop** (per-edge per-iteration `linearizeOplus`). Everything else is config-time (FL-3, FL-6), per-frame-ctor (FL-5), per-LC-KF (FL-21/L1/L2/D5), or per-optimizer-setup (FL-13). For FL-9, cache the bool in the edge at construction (`const bool mbFixScaleJac = FixFlags::I().scaleJacobianChainRule;`) → one register test per Jacobian eval, branch perfectly predicted; do NOT use the `*(flag ? s : 1.0)` form at level 0 (extra multiply changes the instruction stream more than a test+branch, even though `x*1.0` is bit-exact).
- **Level-0 inertness proof protocol** (one-time, at the infrastructure commit): (i) bit gate (nondeterminism exit code propagates); (ii) paired full gate OFF-vs-phase-10-tip, all 5 modes, expect clean — this is the real proof, per project doctrine that only paired runs prove anything; (iii) assembly spot-check: `objdump -d` of the two touched hot functions (`EdgeInertialGS::linearizeOplus`, stereo Frame ctor) diffed against phase-10 build — confirm the only delta is the guarded test+branch; accept and document residual layout drift under the #20 disposition (measured, not blocked); (iv) unit test asserting `FixFlags` defaults are all-false and gate yamls parse to level-0.

## 3. Priority order (fixes land OFF; order = implementation order and ON-validation investment)

1. **FL-9** — standing first priority; cheap (3 lines), mono-inertial only, directly the Remastered regression mechanism; best testability of the set (s_true=2.0 fixture is a decisive property test).
2. **FL-24** — one token, same subsystem and same validation runs as FL-9; the two together make the MI early-window story coherent.
3. **FL-21** — cheap, prevents essential-graph poisoning (highest-consequence latent bug in the set); unit-testable now that the machine is typed; gate can't reach it, which is exactly why the flag+unit-test route is right.
4. **FL-L1/L2 + FL-D5** — cheap latch/reset hygiene, same file, same test harness as FL-21; valuable for long multi-map sessions, cosmetic for gates.
5. **FL-5** — cheap; kills a genuine UB/nondeterminism source in every stereo frame; directly gate-measurable (stereo, KITTI).
6. **FL-13** — most expensive (vendored solver header) and widest numeric radius; high value for loop robustness but rare trigger and weakest A/B story — do after the cheap wins, with the chi2-no-move counter as its field metric.
7. **FL-6** — cheap+cosmetic (unit-test-only validation, no gate exposure).
8. **FL-3** — cheapest mechanically (config value), but lowest value: legacy semantics of debatable merit; ship last for completeness.

Plus non-FixLevel P11 items in parallel: Release() safety demotion (unconditional), Atlas locking with the Frame/IMU-contract batch.

## 4. Proposed P11 commit slicing (this workstream)

- **P11-F0**: FixFlags infrastructure + Settings parse + provenance banner + all-false unit test + DIVERGENCES "FixLevel registry" section (table: flag ↔ DIVERGENCES/OWNERSHIP ID ↔ level preset). Gate: bit + smoke + **paired full OFF vs phase-10 tip** (the inertness proof) + objdump spot-check note. No fix code yet — proves the mechanism inert before any fix rides it.
- **P11-F1**: FL-9 behind flag + reference-backend note (twin intentionally unfixed) + s_true=2.0 ON-fixture (property: converges; OFF-fixture s_true=1.1 twin-equivalence unchanged). OFF: smoke+bit. ON validation: MI paired ON-vs-OFF same-binary 4v4 (extend to 8v8 if boundary-p, per #20 lesson).
- **P11-F2**: FL-24 behind flag (rides FL-9's MI validation protocol; separate commit, separate DIVERGENCES delta).
- **P11-F3**: FL-21 + L1/L2 + D5 behind flags (three flags, one subsystem) + PlaceRecognition machine unit tests (seed/advance/decay/wipe/reset sequences via trace hooks) + `benchmark/scripts/multimap_smoke.sh` (MH01+MH02 single-session MI/SI using existing `num_seq`, asserts merge-consumed + no stale-anchor via channel trace). OFF: standard gate.
- **P11-F4**: FL-5 behind flag + deterministic-ctor unit test. OFF gate; ON: stereo+KITTI paired.
- **P11-F5**: FL-13 — `LinearSolverEigenLDLT.hpp` vendored header behind flag + near-semidefinite pose-graph unit fixture + chi2-no-move counter surfaced in gate logs. ON: KITTI 00 paired.
- **P11-F6**: FL-6 + FL-3 (config-time pair) + unit tests. OFF gate only.
- Phase-boundary: full gate v2.2 (KITTI 00), all flags OFF — also the third clean SI measurement that would CLOSE #20 per its P10 note.

## 5. ON-config validation story (per user-enabled fix)

Each ON decision gets: (a) **same-binary paired interleaved gate**, ON-arm vs OFF-arm, on the fix's judgment modes (FL-9/24: MI(+SI null-check); FL-5: stereo/KITTI; FL-13: KITTI 00; FL-21/L1/L2/D5: multimap smoke + standard modes), 4v4 minimum with the #20 escalation rule (boundary p → 8v8); (b) a **DIVERGENCES delta entry** recording the measured ON effect (the "gate can't see it → document it" duty applies in reverse); (c) where the gate can't reach the behavior (FL-21, D5, FL-6, FL-3), the unit/fixture test is the primary evidence and the gate run only bounds collateral. FL-9 additionally keeps both fixtures permanently: s_true=1.1 twin-equivalence (OFF contract vs reference backend) and s_true=2.0 convergence (ON contract) — the pair pins both semantics against regression in either direction.
---

# ═══ 3부: 어휘/GTSAM/직렬화·순서 권고 (backend-vocab-serial) ═══

All four scoping questions answered from source; one in-container measurement taken. Findings below.

---

## (1) DBoW2 binary vocabulary load (option 1 — confirmed design)

**Vintage check: no binary path exists.** The vendored `TemplatedVocabulary.h` load/save surface is exactly: `loadFromTextFile` / `saveToTextFile` (lines 241/247, impls 1338/1429) plus cv::FileStorage `save`/`load` (.yml, slower than text — irrelevant). **No `loadFromBinaryFile`/`saveToBinaryFile`** — this is the pre-community-patch vintage. The patch must be written, modeled on the well-known ORB_SLAM2 binary-voc patch.

**Vendored tracking confirmed:** `Thirdparty/DBoW2` is 194 regular tracked files (`git ls-files`), no `.gitmodules` entry — the submodules live under `third_party/` (dorian3d DBoW2 submodule exists there but is **not in the build**; `CMakeLists.txt:36` uses `add_subdirectory(Thirdparty/DBoW2)`, only `third_party/g2o` is built from submodules). In-tree patching is allowed; record as a DIVERGENCES intentional-deviation entry.

**Measured load time (kills the folklore number):** built a 20-line timer in-container (orbslam3-modern-dev, arm64) against `/build/cmake/Thirdparty/DBoW2/libDBoW2.so` and `/build/ORBvoc.txt` (145 MB, 971,815 words, k=10 L=6):
```
loadFromTextFile ok=1 size=971815 seconds=2.09  (run 1)
loadFromTextFile ok=1 size=971815 seconds=2.06  (run 2, warm)
```
The ~30-50s figure does not reproduce on this hardware (it dates from 2017-era x86/debug builds). Expect roughly 4-8s on Jetson Orin, a few seconds on iPhone. So the patch's value is real (iPhone cold-start budget, Orin field restarts, and the per-gate-run tax across paired interleaved runs) but it is a ~2s→~0.2s win here, not a 40s win. This lowers its priority from "must" to "cheap S-class win."

**Minimal patch design:**
- `saveToBinaryFile`/`loadFromBinaryFile` added to the vendored header only (~120 lines): fixed header (magic+format-version, m_k, m_L, scoring, weighting, node count, F::L), then fixed 45-byte records per node (parent i32, isLeaf u8, 32-byte FORB descriptor, weight f64). Load = one buffered read + linear fill. ~49 MB file, projected 0.1-0.4s here.
- **Converter: first-run cache, not a tool.** `System.cpp` has two `loadFromTextFile` call sites (lines 106 and 128 of `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/src/core/System.cpp`) — factor into one `LoadVocabulary` helper: try `<vocpath>.bin`, on miss load text then write the .bin beside it, fallback to text on any .bin open/parse failure. This lands the cache at `/build/ORBvoc.txt.bin` on the persistent `orbmodern-build` volume automatically (`docker/scripts/run_slam.sh` sets `VOC=/build/ORBvoc.txt`; `container_build.sh` extracts it there) — zero workflow change.
- **Trap to record:** `SaveAtlas`/`LoadAtlas` MD5-checksum `mStrVocabularyFilePath` as TEXT_FILE (System.cpp:1423) for session-compat. Keep `mStrVocabularyFilePath` pointing at the .txt (the .bin is a derived cache, never passed as the path) so .osa checksums stay stable. Declare the .bin host-local (no endianness portability contract).
- **Validation:** by construction the in-memory vocabulary is bitwise identical (binary stores the exact parsed values); add a unit round-trip (load text → save bin → load bin → compare nodes) plus one paired smoke run. Gate-neutral.
- **Effort: S** (header patch + System helper + DIVERGENCES entry + round-trip test).

## (2) GTSAM second backend

**The 18 methods** (4 + 5 + 9 declarations; GlobalBundleAdjustment and FullInertialBA are each declared on two interfaces with one override — 16 unique implementations):

| # | Method (interface) | GTSAM feasibility |
|---|---|---|
| 1 | PoseOptimization (ITrk) | Natural — projection factors on Pose3; 4-round robust inlier reclassification is a manual loop around the optimizer |
| 2-3 | PoseInertialOptimizationLastKeyFrame / LastFrame (ITrk) | Natural core (CombinedImuFactor is canonical) **and the mpcpi contract is feasible**: the 15x15 Hessian extraction (`Optimizer.cpp:5043/5458` → `Frame::mpcpi` ConstraintPoseImu, consumed as EdgePriorPoseImu at 5258) maps to GTSAM `JointMarginal`/`LinearContainerFactor` — this marginalize-then-prior pattern is what LinearContainerFactor exists for. Numerics won't match g2o's hand-rolled H accumulation |
| 4/17 | GlobalBundleAdjustment (ITrk+ILoop) | Natural; per-iteration abort (`pbStopFlag` via the OrbLevenberg shadow bridge) needs a manual `iterate()` loop — public API, fine. Note Tracking calls it too (mono-init full BA, Tracking.cpp:1832), not just the GBA thread |
| 5 | LocalBundleAdjustment (IMap) | Natural |
| 6 | LocalInertialBA (IMap) | Natural (IMU factor chain; bLarge/bRecInit thresholds host-side) |
| 7-8 | InertialOptimization full + scale/gravity (IMap) | Custom-moderate — no stock gravity-direction+scale factor; EdgeInertialGS equivalent is a small custom factor (and #9's divergent scale update is exactly the code you'd be reimplementing — fix-or-preserve decision forced) |
| 9/18 | FullInertialBA (IMap+ILoop) | Natural, custom bias priors |
| 10 | OptimizeSim3 (ILoop) | Custom-heavy — Similarity3 exists as a Lie group but there are no stock Sim3 projection factors; two-way reprojection edges, inlier trimming, and the 7x7 mAcumHessian output all custom |
| 11-12 | OptimizeEssentialGraph loop + merge (ILoop) | Custom-heavy — BetweenFactor\<Similarity3\> is instantiable, but the 6/7DoF switch, `setUserLambdaInit(1e-16)`, and the documented **#13 LDLT-vs-LLT solver cliff** make this the most parity-sensitive numerics in the codebase; GTSAM's linear solvers are a third behavior |
| 13 | OptimizeEssentialGraph4DoF (ILoop) | Custom-heavy — no 4DoF (yaw+t) pose type in GTSAM; custom manifold + factor |
| 14 | MergeInertialBA (ILoop) | Natural-moderate |
| 15 | LocalBundleAdjustment welding (ILoop) | Natural |
| 16 | InertialOptimization bias-only (ILoop) | Easy |

This matches the repo's own recon: `PROJECT_PLAN.md:17-18` already records that `GetHessian*()` marginalization is not stock in any backend, and GTSAM has no Sim3 essential-graph pipeline — KB8 fisheye projection factors are also custom.

**Partial-backend architecture: clean, with one composition trick.** System owns `G2oBackend mBackend` by value (System.hpp:248) but injects **three separate interface pointers** (System.cpp:183/188/202) — the wiring already supports different objects per interface, and the backends are stateless (all-const, state extracted to BAEpochs/GBAResult/MergeScratch in P5), so mixing is safe. The wrinkle is the dual-declared methods: a GtsamBackend covering ITracking+IMapping still must implement FullInertialBA and GBA, which the LoopClosing/GBA thread also runs via ILoop — naive partial coverage gives the same map global BA from two different optimizers depending on caller. **Clean cut: GtsamBackend implements pose/local-window ops natively and delegates GBA + FullInertialBA to an internal G2oBackend member.** System holds both by value; config flag picks pointers. No interface changes needed.

**Effort:** partial GtsamBackend (ITrk+IMap, globals delegated) = **L**; full ILoopOptimizer parity = **XL** (custom Sim3/4DoF manifolds + factors + #13 sensitivity). Hidden cost either way: **it forks the validation regime** — bit-parity paired gates are impossible against a different optimizer; needs the per-method equivalence harness that PROJECT_PLAN lesson #4/D1 mandates (that harness alone is M), then statistical ATE-envelope gating.

## (3) Serialization redesign

**Current surface:** `System::SaveAtlas/LoadAtlas` (System.cpp:1411-1516; text/binary boost archives, `oa << mpAtlas` whole-graph pointer serialization, MD5 vocabulary checksum) + intrusive `serialize()` in Atlas (9 `ar&` sites), Map (15), KeyFrame (63), MapPoint (31), ImuTypes (31), GeometricCamera+Pinhole/KB8 — ~150 fields — plus `io/SerializationUtils.hpp` helpers and the PreSave/PostLoad ID-backup dance (mvBackupMapPointsId, mBackupConnectedKeyFrameIdWeights, mBackupParentId... then PostLoad pointer rebuild).

**What a redesign targets:** (a) **versioning** — every `serialize(ar, version)` ignores `version`; fields are positional, so any added field (the mfScale class of problem — KeyFrame.hpp:79 serializes it inline) silently invalidates all prior .osa with no migration path; (b) **raw-pointer flattening done twice** — boost object-tracking of the pointer graph AND the hand-rolled Backup-ID fields coexist; a redesign keeps only ID-based records (PreSave already computes them) and drops boost entirely; (c) **portability** — binary_archive is not cross-architecture, blocking Jetson↔iPhone session exchange, and boost::serialization is exactly the dependency GTSAM 4.3 is removing (plan D7 explicitly says decide these together); (d) the vocabulary-checksum coupling from item (1).

**Effort: M** for a clean-break versioned chunked-binary format reusing PreSave/PostLoad, **L** if legacy-.osa migration loading is required. **Key sequencing fact: it is downstream of the lifetime workstream** — if Frame/KeyFrame ownership becomes shared_ptr/tombstone-redesigned, the PreSave/PostLoad and every intrusive serialize gets rewritten again. Doing serialization before lifetime means paying for it twice.

## (4) Recommended P11 ordering (all five workstreams)

**In-scope for P11, in order:**
1. **DBoW2 binary voc (S)** — week one, isolated, gate-neutral by construction, immediate Jetson/iPhone startup value. Quick win that de-risks nothing else.
2. **FixLevel backlog (M)** — the behavioral heart of the phase; each item its own paired-run gate + DIVERGENCES update. Internal order: the **LoopClosing state-machine cluster first (#21 poisoned-consume, latch leaks L1/L2, reset D5)** as one coherent sub-package, **before** the lifetime agent's tombstone work touches the same files (LoopClosing.cpp is contested territory — sequencing this avoids merge churn); then #9 EdgeInertialGS (direct mono-inertial-init robustness — the highest user-value fix for the iPhone ambition, and its resolution informs any future GTSAM InertialOptimization port); #13 LDLT→LLT (modern-g2o robustness cliff); #24 raw mbFixScale (trivial); #3 mnFramesToResetIMU legacy semantics last (config-surface nicety, cuttable).
3. **Lifetime: Frame/IMU-contract races, then tombstone contract (L/XL)** — the phase anchor and the actual on-device crash class; start early in parallel with 1-2, but land the tombstone (154-site) portion only after the LoopClosing fixlevel cluster. Build in a mid-phase checkpoint: if tombstone slips, P11 still tags coherently with Frame-contract + fixlevel + voc done and tombstone rolled to P12.

**Defer past P11 — both remaining candidates:**
- **GTSAM backend → P12 (or later).** Three independent reasons: it forks the gate methodology mid-phase (statistical envelope vs bit-parity — poison for a coherent P11 tag); its edge-target value is speculative until profiling shows g2o optimization time is a bottleneck on Orin/iPhone (no such evidence in the repo); and prerequisite work isn't done (per-method equivalence harness per plan lesson #4, plus the #9/#13 fixlevel resolutions that define what "correct" even means for the factors you'd port). If P11 wants a token step: pin the GTSAM version decision (4.2.2-with-Boost vs waiting for 4.3's Boost removal) as a documented D-decision only.
- **Serialization redesign → P12, sequenced after lifetime.** It's downstream of the ownership model (else it's built twice) and plan D7 already ties it to the GTSAM 4.3 Boost decision. The one interaction to honor in P11: the voc patch must keep .osa checksums text-anchored (covered in (1)).

**Coherent P11 tag story:** "on-device trustworthiness on the g2o backend — lifetime contracts settled, documented-defect burn-down, instant startup" — every item gate-disciplined under the existing paired-run regime, no validation-methodology fork.

Key files: `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h`, `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/src/core/System.cpp`, `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/include/backend/{ITrackingOptimizer,IMappingOptimizer,ILoopOptimizer,G2oBackend}.hpp`, `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/include/map/{KeyFrame,MapPoint,Map,Atlas}.hpp`, `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/docs/{DIVERGENCES.md,OWNERSHIP.md,PROJECT_PLAN.md}`. Timing tool (reusable): `/private/tmp/claude-501/-Users-jhpark-VSLAM/0fff52ab-3205-45b4-852d-8390f95b1114/scratchpad/voc_timer.cpp`.