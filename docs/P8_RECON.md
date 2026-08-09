# P8 정찰: LocalMapping + IMU 초기화

2026-08-09, 3-agent 병렬 정찰 (queue-protocol / imu-init / surface-cleanup) 종합.
대상: `src/mapping/LocalMapping.cpp` (1547줄) + `include/mapping/LocalMapping.hpp` (211줄).
모든 file:line은 phase-7 (7766362) 기준.

스레드 표기: **T** = Tracking, **LM** = LocalMapping::Run, **LC** = LoopClosing::Run,
**GBA** = LoopClosing::RunGlobalBundleAdjustment (detached), **S** = System.

---

## 1. mlNewKeyFrames 생명주기

`std::list<KeyFrame*> mlNewKeyFrames` (hpp:173), `mMutexNewKFs` 보호 (hpp:179).
무한 큐, condition variable 없음 — 소비자는 3ms usleep 폴링.

**생산자 (전부 T, 전부 `InsertKeyFrame` 경유** — push_back + `mbAbortBA=true`, cpp:287-292):

| 호출처 | SetNotStop 가드 |
|---|---|
| `Tracking::StereoInitialization` (Tr:1681) | **없음** |
| `Tracking::CreateInitialMapMonocular` (Tr:1879-1880, ×2) | **없음** (+`mFirstTs` 직접 쓰기 Tr:1881) |
| `Tracking::CreateNewKeyFrame` (Tr:2591) | 있음 (Tr:2477/2593 브래킷) |

**소비자 (처리 동반)**:
- 주 경로: Run() 반복당 1개 (`ProcessNewKeyFrame` cpp:301-341 — pop은 뮤텍스 하에서,
  BoW/관측 연결/`Atlas::AddKeyFrame`(cpp:340, 여기서부터 툼스톤 체제)은 뮤텍스 밖).
- 함수 중간 드레인(처리 동반): `InitializeIMU` cpp:1222-1227, 1332-1337(맵 뮤텍스 하),
  `ScaleRefinement` cpp:1473-1478.
- `EmptyQueue()` (cpp:343-347): **LC 스레드에서만** 호출 (LC:981, 1265, 1851) — 제2 소비자.

**처리 없는 드레인 = 3종 처분 비대칭** (통일 시도 시 "큐 잔류 KF는 아직 맵에 없음" 불변식 주의):

| 사이트 | 처분 | 위험 |
|---|---|---|
| `Release()` cpp:873-875 | **delete만** (SetBadFlag 없음) | T의 `mpLastKeyFrame`/`mpReferenceKF` 댕글링 (OWNERSHIP.md) |
| `InitializeIMU` cpp:1434-1439, `ScaleRefinement` cpp:1508-1513 | SetBadFlag → delete | SetBadFlag 조기 return(초기 KF/mbNotErase)에도 delete 실행 → 맵 자료구조 댕글링 |
| `ResetIfRequested` cpp:1116, 1135 | clear만 (의도적 누수) | 없음 (T가 스핀 대기 중이라 생산자 정지) |

## 2. 배압 플래그 소유권 표

| 플래그 | 뮤텍스 | 쓰기 (스레드) | 읽기 (스레드) |
|---|---|---|---|
| `mbAcceptKeyFrames` | mMutexAccept | LM만 (Run 상단 false :74 / 하단 true :276) | T (Tr:2359 — c1b 판정) |
| `mbStopRequested` | mMutexStop | true: RequestStop :835 (LC×4, GBA, S×3); false: Release :872 | LM (:843, :127), T (Tr:2336) |
| `mbStopped` | mMutexStop | true: Stop :845 (LM), SetFinish :1171; false: Release :871 | LM 파킹 :265, T:2336, LC 대기 5곳, S 대기 3곳 |
| `mbNotStop` | mMutexStop | T만 (SetNotStop :899) | LM (Stop :843 — KF 생성 중 파킹 차단) |
| `mbAbortBA` | **없음** (평범 bool, hpp:181) | InsertKeyFrame :291(T), RequestStop :837, InterruptBA :904(T, 무락); false: Run :106 | SearchInNeighbors :751/:784, KFCulling :1056, 옵티마이저 내부 루프(`&mbAbortBA` :152/:157) |
| `mbResetRequested*` | mMutexReset | T (RequestReset* — 소비될 때까지 스핀), LM 자가요청 :144-147 | LM :1106; **무락 조기 읽기** :1182, :1459 |
| `mbFinishRequested/ed` | mMutexFinish | S (RequestFinish), LM (SetFinish :1166) | LM, GBA (LC:2338) |
| `mbBadImu` | **없음** | LM (:147 true, :1125/:1142 false) | LM Run :77/:262, **T (Tr:1061 무동기화)** |
| `bInitializing` | **없음** | LM (:1220, :1281, :1447, :1492) | T via IsInitializing (Tr:2447/2474) |

**Run() 프로토콜 불변식** (P8-2에서 헤더에 문서화):
1. **배압은 권고적**: `mbAcceptKeyFrames`는 듀티사이클 힌트. T는 c3/c4 조건이면
   `InterruptBA`+큐깊이<3(스테레오)으로 무시하고 삽입 (Tr:2447-2459).
2. **드레인이 정지보다 우선**: 큐 비기 전엔 파킹 안 함. `RequestStop`→`isStopped` 대기는
   암묵적으로 전체 드레인 대기 (LC가 `EmptyQueue`로 직접 비우지 않는 한).
3. **STOPPED 탈출은 외부만**: `Release()` 또는 종료. `SetNotStop(true)` 실패(이미 정지)는
   T가 KF를 통째로 버리라는 신호 (Tr:2477-2478).
4. **Stop/mbBadImu 특이점**: `else if(Stop() && !mbBadImu)` (:262)에서 Stop()이 먼저 평가 →
   mbBadImu=true면 `mbStopped`는 세팅되되 파킹 루프는 스킵 — 외부 대기자는 isStopped()를
   보고 진행하는데 LM은 계속 돈다. 복구는 자가요청한 리셋뿐.
5. **파킹 중 리셋 미서비스**: 파킹 루프(:265-268)는 ResetIfRequested를 안 부름 →
   T의 RequestReset은 LC/S가 Release할 때까지 스핀 (지연 위험, 데드락은 아님).
6. **리셋은 생산자-동기**: RequestReset*이 T를 스핀시키므로 리셋 중 큐 무락 clear는
   뮤텍스가 아니라 프로토콜이 보호.

## 3. LoopClosing 안무 (P9 입력)

- LC 큐 생산자는 단 1곳: Run :253 → `LoopClosing::InsertKeyFrame` (id 0 드롭).
- 4개 stop-the-mapper 시퀀스: CorrectLoop (LC:974), MergeLocal (LC:1224, Release/재정지 2회),
  MergeLocal2 (LC:1795), RunGlobalBundleAdjustment (LC:2280, finish-aware 대기; LM 종료 후엔
  Release가 no-op → 큐 KF 미삭제).
- **P9 보존 비대칭**: `EmptyQueue`/`InitializeIMU`/`ScaleRefinement`가 소비한 KF는 맵에는
  들어가지만 **LC 큐에는 절대 안 들어감** — 루프/병합 탐지 미실행. Run :253 경로만 LC 도달.

## 4. 상속 레이스 (문서화 대상, 수정은 P10 스레딩 현대화)

| # | 내용 | 위치 |
|---|---|---|
| R1 | `Release()`가 mMutexNewKFs 없이 큐 순회+delete; 초기화 생산자 2곳은 SetNotStop 미사용이라 push_back과 경합 가능 (std::list UB) | cpp:873-875 vs Tr:1681/1879 |
| R2 | CorrectLoop이 `RequestStop` 후 **파킹 대기 전에** `EmptyQueue` 호출 → 두 스레드가 ProcessNewKeyFrame 동시 실행 가능 (MergeLocal/2는 순서 올바름) | LC:980-981 vs LC:1001 |
| R3 | IMU 드레인 2곳이 mMutexNewKFs 없이 순회+delete; `SetImuInitialized()`(:1305) 이후 bInitializing=true인 동안에도 T의 삽입 게이트(Tr:2474)가 통과 → 동시 삽입 가능 | cpp:1434-1439, 1508-1513 |
| R4 | `mbAbortBA` 평범 bool을 무락으로 읽고 씀 (InterruptBA는 락 없음, 옵티마이저 내부 루프가 포인터로 폴링) | hpp:181 |
| R5 | 무동기 교차 읽기 모음: `mbBadImu`(Tr:1061), `bInitializing`(Tr:2447/2474), `mbResetRequested`(cpp:1182/1459), `mFirstTs`(Tr:1881 쓰기 vs cpp:1216 읽기), `mpCurrentKeyFrame`(Sys:1330) | — |

## 5. 사장 코드 (P8-1 삭제 목록, 전수 grep 재검증 완료)

**호출자 없는 함수 체인**: `System::SaveDebugData` (Sys:1254-1307, 선언 System.hpp:152) —
유일한 독자였던 `mInitSect`(항상 0)/`mCostTime`(미기록)/`mcovInertial`(미기록)/`mInitTime` 동반 삭제.
이 함수가 mScale/mRwg/mbg/mba가 public인 유일한 이유였음 (P8-4에서 ImuInitializer로 이동).

**죽은 멤버 (hpp)**: `mMutexImuInit`(:85, 유일 참조가 주석 처리된 락 cpp:1458), `mIdxInit`(:96),
`mnKFs`(:97), `mnMatchesInliers`(:99 — T:2285의 교차 스레드 쓰기 포함; **Tracking 자체의 동명
멤버는 활발히 사용 중이므로 LM 쪽만**), `mInitFr`(:102, Sys:189 쓰기 포함), `mIdxIteration`(:103),
`strSequence`(:104 — 생성자 `_strSeqName` 파라미터가 통째로 무시됨; 파라미터+호출 인자 동반 삭제),
`mbNotBA1/2`(:106-107, 쓰기 전용), `mbWriteStats`(:110), `mpSystem`(:144, P7-1b 잔재 —
`System* pSys` 파라미터+Sys:185 인자 동반 삭제), `infoInertial`(:196 → cpp 로컬로 강등;
백엔드 `InertialOptimization` 본문이 covInertial을 전혀 안 쓰는 것 확인), `mNumLM`/`mNumKFCulling`(:197-198),
`countRefinement`(:202), `f_lm`(:205). 미사용 접근자 `GetCurrKF()`(:83, cpp:1542).

**죽은 로컬 (cpp)**: `borrar`(:362/:385), `countStereo`/`countStereoGoodProj`/`countStereoAttempt`/
`totalStereoPts`(:432-435 + 증가 5곳; 이에 따라 `bPointStereo`도 사장), `IMU::Bias b`(:1230),
ScaleRefinement의 `N`(:1480)/`so3wg`(:1496)/`t_inertial_only`(:1515), 소비처 없는 타이밍 포인트
t0/t1/t4/t5(:1273/:1276/:1310/:1324), t0/t1/t2/t3(:1485/:1487/:1499/:1506).

**살아있음 (삭제 금지)**: `mFirstTs`(Sys:1330 GetTimeFromIMUInit + Tr:1881), `mbBadImu`,
`mbFarPoints`/`mThFarPoints`(Sys:190-197 쓰기, cpp:687 + Tr:2669 읽기), REGISTER_TIMES 벡터
(ifdef 하에서 T가 읽음 — 단 :249가 timeKFCulling_ms를 vdLBASync_ms에 넣는 상속 버그 존재,
우리는 REGISTER_TIMES 미빌드이므로 방치).

## 6. IMU 초기화 구조 (P8-4 추출 설계 입력)

**3단계 스테이징** (Run :183-231): 첫 init `(1e2, 1e10|1e5, true)` (mono|else) → VIBA1
`mTinit>5` `(1.f, 1e5, true)` → VIBA2 `mTinit>15` `(0.f, 0.f, true)`. VIBA1/VIBA2의
mono/else 분기는 **완전 동일** (흔적 코드, P8-3에서 접기). ScaleRefinement는
`mTinit<50` 게이트 안의 (25,25.5)∪(35,35.5)∪(45,45.5) 창만 실제 도달 (55/65/75 창은 도달 불가).

**InitializeIMU 흐름**: 가드(리셋/KF수/시간창) → `bInitializing=true` → 큐 드레인#1 →
중력방향/속도 시딩(첫 init) → `InertialOptimization`(관성 전용) → 스케일 가드 →
`ApplyScaledRotation`+`UpdateFrameIMU`(맵 뮤텍스) → 첫 init이면 `SetImuInitialized`+
`t0IMU` 교차 쓰기 → `FullInertialBA`(GBAResult, P5-D) → 맵 뮤텍스 하 큐 드레인#2 +
스패닝 트리 전파(tcwBefGBA 로컬 맵) → 큐 퍼지(SetBadFlag+delete) →
`NotifyImuInitialized()`(P7-1a) → `bInitializing=false`.

**교차 스레드 접점 (추출 시 그대로 보존)**: `mpTracker->mLastFrame.mTimeStamp` 읽기(:1271 —
P8-1에서 mInitTime과 함께 제거됨), `mCurrentFrame.mTimeStamp` 읽기+`t0IMU` 쓰기(:1306),
`UpdateFrameIMU`(:1291/:1302/:1504), `NotifyImuInitialized`(:1446).

**헤더/구현 파라미터명 불일치**: hpp:191 `bFirst` vs cpp:1180 `bFIBA` — FIBA 게이팅이 실체이므로
추출 시 `bFIBA`로 정렬. 디폴트 인자는 사용처 없음 (3곳 모두 전체 인자 전달).

**추출 경계 (동작 불변)**: `ImuInitializer`가 mRwg/mbg/mba/mScale (+로컬 infoInertial)을 private
소유; mFirstTs/mTinit/bInitializing은 접근자로 노출(외부 소비자: Tr:1881, Sys:1330, Run 게이트).
큐는 LocalMapping이 좁은 콜백(드레인/퍼지/현재KF)으로 제공 — mMutexNewKFs를 collaborator에
노출하지 않으면서 기존 드레인 의미(문서화된 위험 포함)를 자구 보존. BAEpochs는 이 경로에서
순수 통과 배관 (두 FullInertialBA 호출 모두 bFixLocal=false → 에포크 마크 미접촉).

## 7. P8 실행 계획

| 단계 | 내용 | 검증 |
|---|---|---|
| P8-1 | §5 사장 코드 전수 삭제 (LM + SaveDebugData 체인 + Tr:2285/Sys:189 촉수) | 빌드 + 비트 게이트 + 스모크 게이트 |
| P8-2 | §2 프로토콜 불변식 헤더 문서화, OWNERSHIP.md에 §4 레이스/§1 처분 비대칭/§3 P9 비대칭 기록 | 문서 전용 |
| P8-3 | 댕글링 delete 가드 (SetBadFlag 후 `isBad()` 확인 시에만 delete — DIVERGENCES 신규 항목, UAF→누수 강등), VIBA 동일 분기 접기, `const float bMonocular`→bool, 헤더 include 정리 | 빌드 + 스모크 |
| P8-4 | ImuInitializer 추출 (§6 경계) | 빌드 + 스모크 + 관성 2모드 확인 |
| 게이트 | 풀 게이트 v2.2.1 4라운드 (KITTI_SEQ=00), 관성 모드 중점 판정 → phase-8 태그 | 페어드 인터리브 |

수정하지 않는 것: R1-R5 레이스 (P10), Release의 delete-no-SetBadFlag (P10, T측 협조 필요),
파킹 중 리셋 미서비스 (P10 condition_variable 전환 시 자연 해소), REGISTER_TIMES 버그 (미빌드).
