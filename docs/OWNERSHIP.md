# 데이터 계층 소유권·수명 계약 (P5-3, 2026-08-07 / P8-2 증보 2026-08-09)

정찰 전수조사 기반(3-agent, journal: wf_b582e188-52e; P8 정찰 wf_1fdd629d-52f).
이 문서는 **현행 계약의 기술**이며, 스마트 포인터 이행은 P6+ 등가성 인프라
이후의 백로그다. LocalMapping 큐/배압 프로토콜의 규범적 기술은
`include/mapping/LocalMapping.hpp` 상단 주석(P8-2), 전체 정찰은 docs/P8_RECON.md.

## R4b 슬라이스 1: MapPoint shared_ptr 이행 (2026-08-20)

**MapPoint는 이제 `MapPointPtr = std::shared_ptr<MapPoint>` 관리다**
(include/map/MapTypes.hpp). 목적은 유지보수성(타입으로 문서화된 소유권) —
회수는 보너스다. **C-lite**: 모든 보유자가 의도적으로 strong이라 툼스톤
수명 계약이 그대로 보존된다(bad MP는 참조가 남아 있는 동안 살아 있고,
SetBadFlag가 여전히 제거 프로토콜). KeyFrame은 이 슬라이스에서 raw 유지
(슬라이스 2 예정).

### 소유권 표 (member → type → strong/weak → 근거)

| 보유자 | 타입 | 강도 | 근거 |
|---|---|---|---|
| `Map::mspMapPoints` | `std::set<MapPointPtr>` | **strong (THE owner)** | 맵 수명 동안의 정본 소유자. SetBadFlag → EraseMapPoint가 여기서 제거 |
| `KeyFrame::mvpMapPoints` | `std::vector<MapPointPtr>` | strong | 툼스톤 의미론: SetBadFlag와 슬롯 소거 사이(및 무락 리더의 사본)에 bad MP 생존 보장 |
| `MapPoint::mObservations` 키 | `KeyFrame*` (raw) | — | KF는 슬라이스 2. 값이 아닌 키; KF 툼스톤 계약이 수명 보장 |
| `MapPoint::mpRefKF`, `mpHostKF` | `KeyFrame*` (raw) | — | 슬라이스 2 |
| `MapPoint::mpReplaced` | `MapPointPtr` | strong | Replace 체인 보존(궤적 복원·CheckReplacedInLastFrame). 순환 시 누수 = 툼스톤과 등가라 허용 |
| `Frame::mvpMapPoints` | `std::vector<MapPointPtr>` | strong | 임시(temporal) MP의 유일한 장기 보유자이기도 함 |
| `Tracking::mlpTemporalPoints` | `std::list<MapPointPtr>` | strong | **수동 delete 루프 은퇴** — 임시 MP는 refcount로 소멸. mLastFrame이 삭제된 임시 MP raw 포인터를 들고 있던 업스트림 댕글링 창 폐쇄 |
| `Tracking::mvpLocalMapPoints` 등 로컬맵 | `std::vector<MapPointPtr>` | strong | 스레드-로컬 핀 |
| `LocalMapping::mlpRecentAddedMapPoints` | `std::list<MapPointPtr>` | strong | 컬링 대기 핀 |
| `LoopClosing::mvpLoopMapPoints` 등 | `std::vector<MapPointPtr>` | strong | LC 스레드 핀 |
| `Map::mvpReferenceMapPoints`, drawer 사본 | strong | strong | Viewer가 든 사본이 곧 핀(임시 MP 뷰어 UAF도 함께 폐쇄) |
| `BAEpochs::mpLocalForKF` / `GBAResult::mps` / `MergeScratch::mps` 키 | `MapPointPtr` | strong | **주소 재사용 방지**: 해제 가능해진 세계에서 raw 키가 남으면 stale 에포크 오판 가능 — strong 키는 툼스톤과 등가인 영구 핀 |
| Optimizer 로컬(핀-셋) | `MapPointPtr` 컨테이너 | strong | 함수 진입 시 1회 핀, 반복 루프는 raw g2o 정점만 접촉(Optimizer.cpp 상단 주석) |
| `Map::mvpBackupMapPoints` | `std::vector<MapPoint*>` (raw) | non-owning | **직렬화 전용**: boost 아카이브 레이아웃을 pre-R4b와 바이트 호환으로 유지. PreSave가 .get() 채움, PostLoad가 정확히 1회 shared_ptr로 래핑 후 clear |

### 계약 변경/불변 사항

- **무락 `isBad()` 참조 계약(152사이트)**: 불변 — 무락 리더가 들던 raw 사본이
  이제 shared_ptr 사본(=핀)이라 수명이 구조적으로 보장된다. 단, **공유 컨테이너
  슬롯 자체의 무락 동시 read/write는 금지**(shared_ptr 인스턴스 경합은 refcount
  파손): 슬롯 접근은 기존 뮤텍스 규율(P8–P10 감사) 하에서만, 무락 통화는
  스레드-로컬 사본으로.
- **enable_shared_from_this**: SetBadFlag/Replace가 `Map::EraseMapPoint`에
  `shared_from_this()`를 넘김. 모든 생성은 make_shared 6곳 + boost 로드 경로의
  PostLoad 1회 래핑 — 생성자에서 shared_from_this 호출 없음.
- **해제되는 것**: 맵 소멸/clear() 시 어떤 KF 슬롯에도 남지 않은 MP,
  프레임 교체 시 임시 MP, 컬링된 bad MP(모든 핀 소멸 후). KF가 raw로 누수되는
  동안 KF 슬롯이 든 MP는 계속 산다(실질 회수는 슬라이스 2에서 확대).
- **.osa**: 형식 무변경(위 표의 backup 행 + src/core/System.cpp SaveAtlas 주석).
- 기존(업스트림 계승) Map::PreSave의 순회-중-소거 self-invalidation 가능성은
  raw 시절과 동일 클래스로 유지(신규 위험 아님).

## 할당·해제 지도

| 객체 | 생성자(유일) | 정상 경로 해제 |
|---|---|---|
| KeyFrame | Tracking (3곳: 초기화 2, CreateNewKeyFrame) | **없음 — tombstone** |
| MapPoint | LocalMapping 삼각측량 + Tracking (초기화/신규KF/임시) | **없음 — tombstone** (임시 포인트만 Tracking이 delete) |
| Map | Atlas::CreateNewMap | Atlas 소멸자에서만 (bad 맵은 의도적 누수 — RemoveBadMaps의 delete 주석화) |
| Atlas | System | 없음 (프로세스 수명) |

특수 경로 delete 4곳(맵 편입 전 큐 KF 정리 등)은 LocalMapping/Tracking에 있으며,
과거 **2곳이 업스트림 그대로의 댕글링 위험**이었다 — 현재 둘 다 강등 완료:
- `LocalMapping::Release()`: ~~큐 KF를 SetBadFlag 없이 delete~~ →
  **P11-A(B3)에서 #19 패턴으로 강등 완료**(SetBadFlag → isBad() 확인부
  delete, DIVERGENCES #29). 미편입 KF에서 Erase류는 no-op, 관측 해제가
  CreateNewKeyFrame산 MP 역참조 댕글링을 제거. 마지막 잔존 raw delete였음.
- `LocalMapping::ScaleRefinement()/InitializeIMU()`: SetBadFlag가 초기 KF /
  mbNotErase에서 조기 return해도 delete는 실행 — mspKeyFrames 댕글링 가능
  (**P8-3에서 UAF→누수로 강등**: SetBadFlag 후 isBad() 확인 시에만 delete,
  DIVERGENCES #19)

### 큐 드레인 3종 처분 비대칭 (P8-2 기록)

`mlNewKeyFrames`를 처리 없이 비우는 세 경로는 처분이 서로 다르며, 통일하려면
"큐 잔류 KF는 아직 맵에 없음(AddKeyFrame은 ProcessNewKeyFrame에서)" 불변식을
같이 봐야 한다:

| 경로 | 처분 | 비고 |
|---|---|---|
| Release() | SetBadFlag→delete (P11-A, #29) | 舊 raw delete의 댕글링 강등 |
| InitializeIMU/ScaleRefinement 말미 | SetBadFlag→delete | P8-3 가드 |
| ResetIfRequested | clear만 (의도적 누수) | 리셋 중 T는 스핀 대기라 안전 |

또한 **LC 큐 비대칭**: Run() 주 경로가 소비한 KF만 LoopClosing 큐에 전달된다.
EmptyQueue/InitializeIMU/ScaleRefinement가 소비한 KF는 맵에는 들어가지만
루프/병합 탐지는 영원히 안 돈다 — P9에서 이 비대칭을 그대로 보존할 것.

## LocalMapping 상속 레이스 대장 (P8-2, 수정은 P10)

전부 업스트림 계승. 게이트에서 무증상이며, 수정은 P10 스레딩 현대화에서
condition_variable/atomic 전환과 함께 일괄 처리한다 (상세 file:line은
docs/P8_RECON.md §4):

- **R1** `Release()`가 mMutexNewKFs 없이 큐 순회+delete. 초기화 생산자 2곳
  (StereoInitialization/CreateInitialMapMonocular)은 SetNotStop 미사용이라
  push_back과 경합 가능 (std::list 동시 변형 UB).
- **R2** `LoopClosing::CorrectLoop`이 RequestStop 후 **파킹 대기 전에**
  EmptyQueue 호출 — LM/LC 두 스레드가 ProcessNewKeyFrame 동시 실행 가능.
  (MergeLocal/MergeLocal2는 대기 후 드레인으로 순서 올바름.)
- **R3** IMU 드레인 2곳이 mMutexNewKFs 없이 순회+delete. SetImuInitialized
  이후 bInitializing 중에도 T의 삽입 게이트가 통과해 동시 삽입 가능.
- **R4** `mbAbortBA`가 평범 bool — InterruptBA는 무락 쓰기, 옵티마이저 내부
  루프는 포인터 폴링. atomic<bool> 전환은 옵티마이저 시그니처(bool*)와 함께
  P10에서.
- **R5** 무동기 교차 읽기 모음: mbBadImu(T), bInitializing(T),
  mbResetRequested(LM 조기 가드 2곳), mFirstTs(T 쓰기 vs LM 읽기),
  mpCurrentKeyFrame(System::GetTimeFromIMUInit).

프로토콜 특이점 2건(버그 아님, 동작 계약): ① mbBadImu=true면 Stop()이
mbStopped를 세팅하고도 파킹 루프를 건너뛴다 — 외부 대기자는 isStopped()를 보고
진행하지만 LM은 계속 돈다(복구는 자가요청 리셋). ② 파킹 루프는
ResetIfRequested를 서비스하지 않는다 — T의 RequestReset은 LC/S의 Release까지
스핀(지연 위험, 데드락 아님).

## LoopClosing 상속 레이스 대장 (P9-2, 수정은 P10)

LC 스레드판 R-목록. 상세 file:line은 docs/P9_RECON.md §4 및 정찰 원문:

- **R-a** `Run()` 진입부 `mbFinished=false`가 mMutexFinish 없이 기록됨.
- **R-b** **루프 큐가 뮤텍스 2개에 이중 보호**: ResetIfRequested는 mMutexReset만
  들고 큐를 변형하고, 생산자 InsertKeyFrame은 mMutexLoopQueue를 든다 — 같은
  리스트를 다른 락이 지킨다(리셋 스핀 프로토콜이 실질 방어).
- **R-c** GBA **스폰** 시 플래그 4종(mbRunningGBA/mbFinishedGBA/mbStopGBA/
  mpThreadGBA)을 무락 기록(CorrectLoop/MergeLocal 재스폰). 중단 경로는 락 있음.
- **R-d** GBA 스레드가 `mnFullBAIdx` 에포크를 무락으로 1차 읽기(재확인은 락 하).
- **R-e** `mbStopGBA`를 옵티마이저 내부 루프가 raw bool* 폴링(업스트림 의미).
- **R-f** LC 스레드가 `mpTracker->GetLastKeyFrame()`을 무락 호출(MergeLocal2·GBA).
  또한 RequestReset*이 LC의 SetFinish 이후 호출되면 영구 스핀.

### P10-2 수정 기록 (2026-08-10) — 위 대장 중 순수-안전 클래스 일괄 해소

**정준 잠금 순서 (신규, 규범)**: `mMutexFinish → mMutexStop → mMutexNewKFs`;
`LM mMutexReset → mMutexNewKFs`; `LC mMutexReset → mMutexLoopQueue`;
`mMutexAccept`/`mMutexGBA` 단독. 역방향 중첩 금지
(LocalMapping.hpp Thread Synch 주석과 동일 문구).

FIXED (각 1줄 메커니즘):
- **R1**: `Release()`의 큐 순회+delete+clear가 mMutexNewKFs(최내곽)를 추가로
  든다 — 초기화 생산자 push_back과의 std::list 동시 변형 소거.
- **R3**: `PurgeNewKeyFramesAfterInertialInit()` 전체가 mMutexNewKFs 하에서
  실행 (SetBadFlag는 KF/Map 계층 뮤텍스만 취해 역중첩 없음).
- **R6**: `Release()` 잠금 순서를 Stop→Finish에서 **Finish→Stop**으로 재배열
  — SetFinish와의 ABBA 데드락 소거.
- **R-a**: LM/LC `Run()` 진입부 `mbFinished=false`를 mMutexFinish 하에 기록
  (atomic 아님 — SetFinish가 mbStopped와 결합).
- **R-b**: LC `ResetIfRequested`의 큐 변형 2곳이 mMutexLoopQueue(내곽)를
  추가로 든다 — 생산자와 같은 락으로 통일.
- **R-c**: GBA 스폰 플래그 3종(mbRunningGBA/mbFinishedGBA/mbStopGBA) 기록을
  양 스폰 지점에서 mMutexGBA 스코프로 감쌈 (reap-join과 mpThreadGBA는 락
  밖 — GBA 꼬리가 mMutexGBA를 취하므로 락 하 join은 데드락 가능).
- **R-d**: `mnFullBAIdx` → atomic<int>; 증가는 기존 mMutexGBA 스코프 유지,
  GBA의 무락 1차 읽기는 relaxed load.
- **R-f**(전반부): `Tracking::mpLastKeyFrame` → atomic<KeyFrame*>; 쓰기
  store-release, GetLastKeyFrame load-acquire, T 내부 읽기 relaxed.
  (후반부 SetFinish-후-영구-스핀은 미수정 — 문서화된 프로토콜 특이점.)
- **R5 플래그류**: `mbBadImu`·`bInitializing`·`mbResetRequested(ActiveMap)`·
  `mbFinishRequested` → atomic<bool> relaxed; `mFirstTs` → atomic<double>
  (is_always_lock_free static_assert); `mpCurrentKeyFrame` →
  atomic<KeyFrame*> (ProcessNewKeyFrame store-release ↔ GetCurrKFTime
  load-acquire, LM 내부 relaxed). 큐 clear 2곳(ResetIfRequested)도
  mMutexNewKFs로 폐쇄 (프로토콜 보호였으나 균일화).

명시적으로 NOT fixed (P10-2 범위 밖) — P11-A 처리 현황:
- ~~**Frame 객체/IMU-계약 레이스**~~ → **P11-A에서 메시지 패싱으로 FIXED**
  (DIVERGENCES #28): LM/LC의 UpdateFrameIMU 직접 호출 5곳이
  `Tracking::PostImuUpdate(ImuUpdateMsg)` 게시로, 적용은 T가 Track() 상단
  mMutexMapUpdate 획득 직후 1곳에서. `t0IMU`는 T-한정 복귀(bFirstInit
  플래그가 대체), `mState_` 저장소·`mnMatchesInliers`는 atomic relaxed
  풀-리드. **Frame 객체 = T-스레드 한정**(FrameDrawer 복사는 기존 뮤텍스).
  TSAN T3의 `Frame::operator=` 시그니처 클래스 소멸 확인
  (benchmark/tsan/step_P11-A_T3.ledger).
- **툼스톤 계약 클래스**: 152개(재계수) 무락 isBad() 역참조,
  `_Rb_tree` 집합 복사 잡음 — 계약(락 아님), P12+ 백로그. Atlas 무락
  메서드군은 **P11-A에서 잠금 완료**(AddKeyFrame/AddMapPoint/AddCamera/
  GetAllCameras/SetMapBad/RemoveBadMaps가 mMutexAtlas 취득; PreSave/
  PostLoad/배선 세터는 단일 스레드 경로로 문서화, Atlas.cpp 주석).
  MapPoint 스크래치 필드 교차 읽기(LocalInertialBA의 mTrackDepth 등,
  T3 잔존 시그니처 1건)는 이 클래스에 남는다.
- **R2**(EmptyQueue 순서)는 P10-3, **R-e**는 P10-1에서 기해소, GBA 수명/
  Shutdown은 P10-5.

## GBA 스레드 수명 (P9-2 기록, P10-5 해소)

- 생성 2곳(CorrectLoop 스폰, MergeLocal 재스폰), ~~join은 어디에도 없음~~ →
  P9-3에서 스폰 직전 reap-join(DIVERGENCES #22), **P10-5에서 완전 해소**(아래).
- ~~중단은 파이어앤포겟: mbStopGBA + 에포크 증가 + detach/delete~~ →
  **P10-5: 중단 경로에서 detach 제거** — 플래그 + 에포크만 남기고 스레드는
  joinable로 유지. 중단자는 여전히 즉시 반환(관측 가능한 fire-and-forget
  보존); 대가는 다음 스폰의 reap-join이 중단된 GBA의 실제 종료까지(≤ BA
  1회 반복) 블록할 수 있다는 것. 에포크 불일치 return의 **mbRunningGBA
  영구 잔류는 scope-exit 가드로 수정**(모든 return 경로에서 mMutexGBA 하에
  mbFinishedGBA=true/mbRunningGBA=false — DIVERGENCES #25).
- ~~System::Shutdown의 대기 루프가 통째로 주석 처리~~ → **P10-5: Shutdown
  join 순서 복원**. 원자 exchange 래치(1회만 해체 실행) → Viewer
  RequestFinish+조건부 join(viewer 스레드 자신이면 self-join 회피, ~System이
  뒷정리) → LM/LC RequestFinish(CV 통지) → **LM join**(SetFinish가
  mbStopped 세팅+통지라 LC/GBA의 stop 대기가 죽은 LM에 수렴) → **LC join**
  → GBA 커스터디 스텝 → **그 후에야 SaveAtlas**(살아있는 LM/LC/GBA 0 —
  boost 직렬화가 맵 변형과 경쟁 불가; 락이 아니라 join 순서로 달성).
  System의 스레드 3종은 `std::thread` 값 멤버, ~System이 백스톱 reaper.
- **mThreadGBA 커스터디 체인 (규범)**: `std::thread` 값 멤버(포인터 아님).
  ① LC 살아있는 동안 — LC 스레드 전속(스폰/reap-join 전부 LC 스레드,
  reap-join은 항상 mMutexGBA **밖**: GBA 꼬리와 #25 가드가 그 락을 취함;
  join → 플래그 기록 → 스폰 순서 필수, 가드의 정리가 새 스폰 플래그를
  덮지 않도록) ② System::Shutdown이 LC를 join한 뒤 — System 전속
  ③ `LoopClosing::StopAndJoinGBA()`(Shutdown 6단계 전용)가 mMutexGBA 하에
  stop+에포크 증가 후 락 밖에서 join — SaveAtlas 전에 반드시 reap.
  LC::Run 종료부에는 GBA 코드가 없다 — 이 체인 규칙이 그것을 대체한다.

## Viewer/Shutdown 경로 (P10-6 기록)

- Viewer는 System 파사드 대신 **IViewerHost**(6메서드, 제로-include)를 본다 —
  System↔Viewer 순환의 마지막 절단. menuStop은 **RequestShutdown 래치 전용**
  (조인 없음): 요청 래치(mbShutDown)와 티어다운 래치(mbShutdownDone)가 분리되어
  뷰어 스레드는 절대 자기 자신을 join하지 않고, 실제 조인+SaveAtlas는
  메인 스레드의 Shutdown() 1회가 수행한다(~System이 최종 안전망, viewer-first
  reap). 파킹 술어의 finish 암은 DIVERGENCES #26.

## SetNotErase/SetErase 래치 프로토콜 (P9-2 기록)

- 래치는 refcount가 아니라 **불리언**(중첩 래치 붕괴). 획득은 LC 2곳뿐
  (큐 팝 시 현재 KF, BoW 최적 매치). 해제(SetErase)는 `mspLoopEdges`가 빌 때만
  — **mspMergeEdges는 미검사**: 루프엣지 KF는 영구 핀, 병합엣지 KF는 재컬링
  가능(MergeLocal2는 AddMergeEdge 자체를 안 함). 컬링이 미룬 SetBadFlag는
  **LC 스레드의 SetErase에서 완결**된다(#19 가드의 완결 경로).
- **누수 L1**: ACCUM 중 BoW 재탐지가 매치 KF 포인터를 SetErase 없이 덮어씀 →
  구 후보 영구 핀(컬링 차단). **누수 L2**: BoW 약시딩이 cnt=0으로 시딩 가능 →
  해제 경로가 cnt>0 가드라 도달 불가 → 영구 래치+고아 앵커. 둘 다 업스트림
  계승, FixLevel 후보(docs/P9_RECON.md D2/D3).
- **리셋 미소거(D5)**: LC의 ResetIfRequested는 큐만 지우고 탐지 상태머신
  (카운터/앵커/매치 KF 래치)은 안 지운다 — 활성맵 리셋 후 최대 2 KF 동안
  파괴된 맵으로 교차 reffine 시도. P9-4 타입드 머신이 리셋을 구독해 트레이스로
  가시화한다(동작 보존).

## Tombstone은 버그가 아니라 동시성 계약이다

`SetBadFlag()`는 그래프 연결·관측·DB 색인만 해제하고 메모리는 남긴다(mbBad).
근거: 4개 스레드가 락 없이 보관한 포인터로 `isBad()`를 역참조하는 지점이
**src/ 전체 152곳(15파일)** 존재하고(P11_RECON 1부 1a 재계수 2026-08-11 —
파일별 표는 그 문서; 종전 "154곳 13파일"은 P8 시점 수치), mpReplaced 체인·
스패닝 트리 부모·Optimizer 로컬 벡터가 bad 객체를 계속 가리킨다. raw delete
도입은 이 152곳 전부의 수명 증명 없이는 UAF다. 따라서:

- **규칙 1**: KeyFrame/MapPoint를 delete하지 말 것. 제거는 SetBadFlag 경유.
- **규칙 2**: 보관한 포인터는 사용 전 isBad() 확인이 관례(락 아님 — 계약).
- **규칙 3**: MapPoint 대체는 Replace()/GetReplaced() 체인을 따라갈 것.

**실측 증거 (P12-L0-b, 2026-08-12)**: 이 계약이 실제로 하중을 받는다는
분포 실측이 benchmark/lifetime/pilot_e4032eb/에 있다 — 셧다운 궤적 저장의
부모 체인 워크가 죽은 KF 645개를 사망 후 최대 146초까지 2,772회 읽는다
(리셋 스톰 시나리오). "죽은 객체 읽기"는 버그가 아니라 기능임의 숫자 증명.

**B1(큐 unique_ptr 커스터디) 처분 (P12-L1, 2026-08-12): 스킵 확정.**
mlNewKeyFrames는 raw list로 유지한다. 실위험(드레인 댕글)은 P11의 B3
(#19형 가드 강등, LocalMapping.cpp:960-978)가 이미 은퇴시켰고, P11_RECON의
자체 평가대로 B1 단독으로는 어떤 위험도 제거하지 못한다(Tracking 별칭이
비소유라 댕글 불변) — 타입 문서화 가치만 남는데 그 대가가 disposal 의미론
변경 위험이다. 큐 커스터디의 활자화가 다시 필요해지면 C-full 결정과 함께
재론한다.

## Pin-set 관례 (P12-L1 확정 — GTSAM ISP 동결 전 필수 기록)

Optimizer 계열 함수(ITrackingOptimizer/IMappingOptimizer/ILoopOptimizer의
전 시그니처)는 KeyFrame*/MapPoint*를 **raw 포인터로 받고, 앞으로도 그렇게
유지한다** — 어떤 수명 이행이 오더라도 ISP 시그니처는 불변이다. 이행 시
바뀌는 곳은 단 하나, 함수 진입부다:

- **관례**: 각 최적화 함수는 진입 시 작업 집합을 로컬 벡터로 1회 수집
  (pin)하고, 내부 핫루프는 전부 raw로 돈다. 툼스톤 체제에서 pin은 암묵적
  (아무것도 해제되지 않으므로 수집 자체가 pin이다). 장래 C-full 체제에서는
  이 수집 지점만 shared_ptr 복사/weak lock으로 바뀌고 루프 본문·시그니처는
  그대로다 — refcount 트래픽이 LM 병목(LBA 54~76%, benchmark/times/
  g0_profile_f095359)에 진입하지 않는 유일한 배치.
- **이유**: 시그니처가 스마트포인터로 바뀌면 이후 백엔드 추가(GTSAM G1)마다
  이행 팬아웃이 ×2가 된다(P11_RECON 권고 4). 시그니처 동결 + 진입부 관례가
  그 결합을 끊는다.
- 신규 최적화 함수·신규 백엔드 구현은 이 관례를 따를 것(리뷰 체크 항목).

## 직렬화 제약 (소유권 변경 시 필수 고려)

- Map/Atlas가 raw 포인터 vector(mvpBackup*)를 boost로 직렬화 — 로드 시
  boost가 new로 재생성(소유권이 아카이브 경로에서 발생). 정적 ID 카운터
  (Map::nNextId 등)도 직렬화됨.
- 교차 참조는 PreSave에서 ID로 평탄화, PostLoad에서 재연결. 필드 삭제/이동은
  맵 파일 포맷 변경이므로 별도 버전 처리 필요 (예: mfScale).

## 뮤텍스 요약 (정찰 (2)절)

- 정준 잠금 순서: MapUpdate → Connections → Features → Pose / Global → Pos
  (역방향 위반 관측 0건 — 신규 코드도 이 순서를 지킬 것)
- `MapPoint::mGlobalMutex`: pose-only 최적화 동안 전 포인트 위치의 세대 일관
  스냅샷 제공 — per-instance로 축소 시 혼합-세대 좌표 최적화가 되므로 **현상
  유지** (성능 이슈로만 기록)
- Atlas 무락 메서드군(AddKeyFrame/AddMapPoint/AddCamera/GetAllCameras/
  SetMapBad/RemoveBadMaps) — **P11-A에서 mMutexAtlas 잠금 완료**(순수
  스레딩, 수치 불가시). 잠금 순서는 기존 Atlas→{Map,KF}-계층 에지만 추가
  (clearMap/GetCurrentMap 선례), MapUpdate→Atlas 순서는 호출부 선례 유지.
  PreSave/PostLoad/배선 세터-게터는 단일 스레드 경로라 의도적 무락
  (PreSave는 잠긴 SetMapBad/RemoveBadMaps를 호출하므로 스스로 mMutexAtlas를
  들면 안 됨 — Atlas.cpp 주석).
