# IMU 데이터플로 계약 — Tracking ↔ System ↔ LocalMapping (P7-1c, 2026-08-09)

`docs/P7_RECON.md` §C를 현재 HEAD(P7-1a/b 반영, 95a41f9) 기준으로 재검증·정제한
**기술(記述) 계약**이다. 여기 적힌 것은 전부 "지금 그렇게 동작한다"이지 "그래야
한다"가 아니다. §6의 결함들은 업스트림에서 bug-for-bug로 계승된 것으로,
**이 단계(P7)에서 고치지 않는다** — 수정은 FixLevel로만 제공한다
(docs/DIVERGENCES.md 방법론 참조).

정찰(§C) 대비 라인 이동: P7-1a/b가 `Tracking.cpp` 앞부분에 코드를 추가해
대부분의 사이트가 **약 +101행** 밀렸다. 실질 변경은 한 곳 —
`LocalMapping.cpp`의 `mpTracker->mState=Tracking::OK` 직접 쓰기가
`mpTracker->NotifyImuInitialized()` 호출(`LocalMapping.cpp:1446`)로 대체됐다
(동일 쓰기, 동일 스레드, 동일 무동기화). 본 문서의 모든 라인은 재검증 완료.

**P11-A 갱신 (2026-08-11, DIVERGENCES #28)**: `UpdateFrameIMU`는 더 이상
교차 스레드로 호출되지 않는다 — LM/LC는 `Tracking::PostImuUpdate(ImuUpdateMsg)`
로 게시하고, T가 Track() 상단 mMutexMapUpdate 획득 직후 스스로 적용한다
(§5 갱신부). 이에 따라 §5의 대기 스핀(W9)은 삭제, §6의 B1 행 위험과 B12의
교차 스레드 암은 **은퇴**했다(각 행의 취소선 주석 참조). 라인 번호는 P11-A
편집으로 `Tracking.cpp` 후반부가 +수십 행 밀렸다 — 본문 라인은 P7 시점
기준을 유지하고, 갱신부만 상대 서술로 적는다.

---

## 1. `mlQueueImuData` — 생산자/소비자/뮤텍스 프로토콜

**선언**: `include/tracking/Tracking.hpp:320` (`std::list<IMU::Point>`),
가드 `mMutexImuQueue` `:328`, 배출 버퍼 `mvImuFromLastFrame` `:324`
(추적 스레드 전용, 무가드 — 뮤텍스 보호 대상이 **아님**).

### 생산자 (유일)

`Tracking::GrabImuData()` — `src/tracking/Tracking.cpp:874-878`.
락 잡고 `push_back` 한 줄. 타임스탬프 검증 없음, 정렬 없음, 크기 상한 없음
(`Track()`이 배출을 멈추면 무한 성장).

호출자는 `System`의 세 진입점뿐이며, 항상 **리셋 처리 블록 뒤, `GrabImage*`
직전**에 드라이버가 넘긴 배치를 루프로 밀어 넣는다:

| 진입점 | push 루프 | 직후의 drain |
|---|---|---|
| `System::TrackStereo` | `src/core/System.cpp:301-302` | `:305` |
| `System::TrackRGBD` | `System.cpp:376-377` | `:379` |
| `System::TrackMonocular` | `System.cpp:452-453` | `:456` |

### 스레딩 — 뮤텍스가 암시하는 것과 실제

- **동봉된 모든 드라이버는 이 경계에서 단일 스레드다.**
  `Examples/Monocular-Inertial/mono_inertial_euroc.cc:172-195`: 메인 루프가
  `t ≤ t_frame`인 샘플을 로컬 `vImuMeas`에 모아 `SLAM.TrackMonocular(im,tframe,vImuMeas)`
  를 같은 스레드에서 호출. `Examples/Stereo-Inertial/stereo_inertial_euroc.cc:167-185` 동일.
- ROS 드라이버(`Examples_old/ROS/ORB_SLAM3/src/ros_mono_inertial.cc`)는 IMU 콜백
  스레드가 있지만 **자체** `imuBuf`/`mBufMutex`(`:45-46`)에 쌓고, sync 스레드
  (`:101`, `SyncWithImu` `:141-178`)가 그것을 배출해 `TrackMonocular`를 호출한다.
  즉 `GrabImuData`는 여기서도 추적 스레드에서 불린다.
- **계약**: `GrabImuData`는 현재 항상 추적 스레드에서, 해당 프레임 구간의 샘플을
  프레임 처리 직전에 공급받는다. `mMutexImuQueue`는 현재 무경합이며, 비동기
  생산자를 *허용하기 위해* 존재한다. 단, B2(§6) 때문에 비동기 생산자는 아직
  실제로는 안전하지 않다 — 뮤텍스는 약속이지 보증이 아니다.

### 소비자 (유일)

`Tracking::PreintegrateIMU()` — `Tracking.cpp:880-991`. 호출은 `Track():1130`
한 곳, 게이트는 관성 센서 **그리고** `!mbCreatedMap`(`:1125`; 맵 포크 직후 첫
프레임은 의도적으로 건너뜀, `mbCreatedMap`은 `:1139`에서 무조건 해제).

배출 루프 `:899-931`, 락 범위 `:903-928`(반복마다 재획득), front만 검사:

| 조건 (`m` = front) | 동작 | 라인 |
|---|---|---|
| `m->t < prevFrame.t − mImuPer` | **pop, 폐기** | `:908-911` |
| `m->t < curFrame.t − mImuPer` | **pop, `mvImuFromLastFrame`에 복사** | `:912-916` |
| 그 외 (미래 샘플) | **복사만 하고 pop 안 함**, `break` | `:917-921` |

첫 "미래" 샘플을 큐에 남기는 것은 의도된 동작: 이번 구간의 우측 보간 끝점이자
다음 구간의 좌측 끝점이 된다. `mImuPer`는 `:671`에서 `0.001` 하드코딩
(`1.0/mImuFreq` 형태는 TODO 주석으로 봉인).

적분 `:941-982`: `n = size−1` 구간, 양 끝점을 프레임 타임스탬프로 선형 외삽
(`i==0` `:945-954`, `i==n-1` `:961-970`), 각 샘플을 **두 객체 모두**에 공급
(`:980` from-KF, `:981` from-frame). 발행 `:984-986`
(`mpImuPreintegratedFrame`/`mpImuPreintegrated`/`mpLastKeyFrame`을
`mCurrentFrame`에), `setIntegrated()` `:988` — `UpdateFrameIMU`가 대기하는
핸드셰이크(§5).

### 순서/지각(遲刻) 가정 — 무검증 전제조건

- **지각 샘플**(프레임 처리 후 도착): 그 프레임 구간에서 그냥 빠진다. 다음
  프레임에서 front에 있다가 새 prev 타임스탬프보다 오래됐으면 `:908-911`이
  버린다. 지각 대기 로직은 없다(`bSleep` 죽은 코드, B3).
- **역순 샘플**(새 샘플 뒤에 옛 샘플 push): 큐는 정렬되지 않고 front만 보므로
  drain이 새 샘플에서 `break`하고 옛 샘플이 뒤에 남는다. 이후 프레임에서
  폐기되거나, 새 구간 안이면 **역순으로** `mvImuFromLastFrame`에 들어가 음수
  `tstep`을 만든다. `Preintegrated::IntegrateNewMeasurement`
  (`src/backend/ImuTypes.cpp:177-235`)에는 `dt` 가드가 없어 `dT/dP/dV/avgA/C`가
  조용히 오염된다. **push 타임스탬프의 단조성은 검사 없는 계약 전제조건이다.**
- **중복 타임스탬프**: `tstep==0` → `:947`/`:963`의 `tab==0` 나눗셈.

### 비움

큐를 비우는 곳은 단 하나: `Track():1079-1080` (락 하에), "프레임 타임스탬프
역행" 분기에서 `CreateMapInAtlas()` 직전. `Reset()`, `ResetActiveMap()`,
`CreateMapInAtlas()`, >1s 점프 분기(`:1084-1111`)는 큐를 건드리지 **않는다**(§4).

---

## 2. 소유권 표 — 세 개의 preintegration 핸들

| | `Frame::mpImuPreintegratedFrame` (직전 **프레임**부터) | `Tracking::mpImuPreintegratedFromLastKF` (직전 **KF**부터) | `Frame::mpImuPreintegrated` | `KeyFrame::mpImuPreintegrated` |
|---|---|---|---|---|
| 선언 | `include/map/Frame.hpp:275` | `include/tracking/Tracking.hpp:312` | `Frame.hpp:266` | `include/map/KeyFrame.hpp:364` |
| new | `Tracking.cpp:939` (프레임당 1개, bias=`mLastFrame.mImuBias`) | `:680, 1612, 1727, 1875, 1942, 2500` | 직접 new 없음 — **항상 FromLastKF의 별칭** (`:985, 1613, 1728`) | 직접 new 없음 — `KeyFrame::KeyFrame(Frame&)`이 포인터 복사 (`src/map/KeyFrame.cpp:54`) |
| delete | **없음** (B4: 프레임당 1개 누수) | `:1610, 1725, 1941`만 (null 가드 있음) | 없음 (별칭이므로 정당) | **없음** — KF는 프로세스 수명 (`Map::clear()`는 tombstone, docs/OWNERSHIP.md) |
| 소유자 | 사실상 없음: `Frame` 소멸자는 주석 처리(`Frame.hpp:71`), 복사 생성자는 포인터 별칭(`Frame.cpp:59`) | `Tracking` — 단, 그 프레임으로 KF가 만들어지는 순간 소유권이 KF로 이동 | — | KF (영구) |
| 백엔드 인계 | `Optimizer::PoseInertialOptimizationLastFrame` (`src/backend/Optimizer.cpp:5018`, 에지 `:5201`); `PredictStateIMU` `!mbMapUpdated` 분기 (`Tracking.cpp:1026-1030`); 스테레오 초기화 avgA 게이트 (`:1603`) | (별칭 경유) `PoseInertialOptimizationLastKeyFrame` (`:4634`, 에지 `:4815`) | 좌동 | `LocalInertialBA` (`:618, :659`), `FullInertialBA` (`:2748-2799`), `InertialOptimization` (`:3285-3303, :3459-3474, :3612`, in-place `Reintegrate` `:3360, :3525`), merge BA (`:4342-4385`); KF 컬링 시 `MergePrevious` (`src/mapping/LocalMapping.cpp:1033, :1042`); `InitializeIMU` (`:1240-1246`) |

**소유권 이동 지점 (from-KF 객체):**

1. **정상 상태** — `CreateNewKeyFrame()`: `:2480`에서 KF 생성(= `KeyFrame.cpp:54`가
   `mCurrentFrame.mpImuPreintegrated` 포인터 복사) → `:2500`에서 멤버를 새 객체로
   재지정(bias=`pKF->GetImuBias()`). **delete 없음이 올바름**: 소유권이 KF로 이동.
   단 B6(§6)의 전제 — `PreintegrateIMU`가 `:985`까지 도달했어야 함 — 이 붙는다.
2. **단안 초기화** — `CreateInitialMapMonocular():1873`에서
   `pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF` 명시 이전 후
   `:1875`에서 재지정(bias=`GetUpdatedBias()`). `pKFini` 쪽 별칭은 `:1789`에서
   명시적으로 null — 한 객체를 두 KF가 주장하는 것을 막는다.
3. **스테레오/RGBD 초기화** — `StereoInitialization():1612-1613`이 새 객체를
   만들어 `mCurrentFrame`에만 별칭시키고, `:1629`의 `pKFini`가 같은 포인터를
   복사한다. **Tracking과 살아있는 KF가 같은 객체를 별칭하는 유일한 창**
   (다음 `CreateNewKeyFrame():2500`까지; B7).

**바이어스 결합**: `Frame::SetNewBias`(`Frame.cpp:420-425`)는 `mImuBias` 대입 후
`mpImuPreintegrated`가 있으면 전달한다. `Track():1116`에서는 아직
`mpImuPreintegrated==NULL`(발행은 `:985`, 즉 `:1130`의 `PreintegrateIMU` 내부)
이라 객체에는 no-op. `UpdateFrameIMU`의 두 프레임 rebias에서는 no-op이
**아니며** 공유 preintegration 객체를 변이시킨다 — P11-A부터 이 호출도 T
스레드에서 실행되므로 교차 스레드 레이스가 아니다(B12 행의 은퇴 주석).

---

## 3. `new IMU::Preintegrated` / `delete` 전수 대장 (현재 HEAD)

| 사이트 | 무엇 | 짝지어진 delete? |
|---|---|---|
| `Tracking.cpp:680` (`newParameterLoader`) | 최초 from-KF 객체, zero bias | 이후 리셋 경로가 지우면 짝; **종료 시 누수** (`~Tracking()` `:592-596` 비어 있음; `:678`의 `mpImuCalib`도 동일 누수) |
| `Tracking.cpp:939` (`PreintegrateIMU`) | 프레임당 from-frame 객체 | **전무 — 프레임당 1개 누수** (B4) |
| `Tracking.cpp:1428-1432` (`Track`, "save frame for IMU reset") | `new Frame` ×2 + `new IMU::Preintegrated`, 로컬 `pF`에만 저장 | **즉시 누수**; 단 `mnFramesToResetIMU==0`으로 현재 도달 불가 (§5, B5) |
| `Tracking.cpp:1610 → 1612` (`StereoInitialization`) | delete(널 가드) + 재생성 | 짝 |
| `Tracking.cpp:1725 → 1727` (`MonocularInitialization`) | delete(널 가드) + 재생성 | 짝 |
| `Tracking.cpp:1875` (`CreateInitialMapMonocular`) | `pKFcur`에 이전 후 재생성 | delete 없음 — 올바름 (소유권 이동) |
| `Tracking.cpp:1941 → 1942` (`CreateMapInAtlas`) | delete(널 가드) + 재생성 | 짝 |
| `Tracking.cpp:2500` (`CreateNewKeyFrame`) | KF에 인계 후 재생성 | delete 없음 — KF가 실제로 받았을 때**만** 올바름 (B6) |

파일 내 다른 delete는 `:1493`(임시 MapPoint)뿐, 무관.
(정찰 대비: 세 delete 사이트의 `if(mpImuPreintegratedFromLastKF)` 널 가드는
정찰 표기에서 생략됐던 것으로, 코드 변경이 아니라 기록 보완이다.)

---

## 4. 리셋 경로와 stale 창 — 전부 INHERITED (여기서 고치지 않음)

| 경로 | 큐 | `mpImuPreintegratedFromLastKF` | `mCurrentFrame`/`mLastFrame` |
|---|---|---|---|
| `Reset()` `:3035-3094` | **불변** | **불변** (전체 아틀라스 리셋을 stale 객체가 생존) | 둘 다 `= Frame()` (`:3083, :3085`) |
| `ResetActiveMap()` `:3096-3185` | **불변** | **불변** | 둘 다 `= Frame()` (`:3173-3174`); `mnLastRelocFrameId = mCurrentFrame.mnId`는 재대입 **전**에 읽음 (`:3171`) |
| `CreateMapInAtlas()` `:1918-1956` | **불변** | delete + zero-bias 재생성 `:1939-1943` | 둘 다 `= Frame()` (`:1951-1952`); `mbCreatedMap=true` (`:1955`) → 다음 `Track()`은 `PreintegrateIMU` 건너뜀 |
| `Track():1076-1083` (타임스탬프 역행) | **비움** (락 하 `:1079-1080`) | 후속 `CreateMapInAtlas()` 경유 | 좌동 |
| `Track():1084-1111` (>1s 점프) | 비우지 **않음** | `CreateMapInAtlas()` 경로(`:1100`)만 재생성; `RequestResetActiveMap()` 경로(`:1096, :1106`)는 둘 다 stale 방치 | — |
| IMU 재초기화 (`LocalMapping::InitializeIMU`/`ScaleRefinement`) | 불변 | 불변 — 객체를 제자리에서 rebias/reintegrate (`Optimizer.cpp:3360, :3525`), `UpdateFrameIMU`(`Tracking.cpp:3198`)로 스케일 반영 | `UpdateFrameIMU`가 두 프레임의 pose/velocity 재작성 |

리셋 요청은 **1프레임 지연 실행**임에 주의: `mpResetRequester->RequestResetActiveMap()`
(P7-1b, 舊 `mpSystem->ResetActiveMap()`)은 래치만 세우고, 실행은 다음
`System::Track*` 진입부(`System.cpp:284-297/359-372/434-449`)에서
`Tracking::ResetActiveMap()`으로 일어난다.

**Stale 소비 창 (기술이지 제안 아님 — INHERITED):**

1. 지연 `ResetActiveMap()` 직후: 큐에는 옛 맵의 샘플, `mpImuPreintegratedFromLastKF`
   에는 옛 맵의 누적 델타가 남는다. `mbCreatedMap`이 세워지지 **않으므로** 바로
   다음 `Track()`이 `PreintegrateIMU`(`:1130`)를 실행해 stale 객체에 적분한다
   (`:980`). 창은 `MonocularInitialization():1725-1727` 또는
   `StereoInitialization():1610-1612`의 재생성으로 닫힌다. 그동안 `mState`는
   `NO_IMAGES_YET→NOT_INITIALIZED`라 KF·백엔드 에지가 stale 누적을 소비할 수는
   없지만, `:985`에서 프레임에 별칭된 객체 자체는 stale이다. B11(기본 생성
   `Frame`의 미정 `mTimeStamp` 비교)이 같은 창에서 겹친다.
2. `CreateInitialMapMonocular():1845-1850`이 `pKFcur`(`:1786`)가 이미 포인터를
   복사한 **뒤**, 소유권 이전(`:1871-1875`) **전**에 bail할 수 있다 — 멤버와
   (tombstone된) KF가 한 객체를 별칭한 채 `MonocularInitialization():1725`의
   delete까지 지속.
3. 스테레오 초기화 별칭 창 (§2 항목 3, B7).

---

## 5. `mnFramesToResetIMU == 0` 상호작용 (docs/DIVERGENCES.md #3 교차참조)

`Tracking.hpp:400`에서 `= 0` 고정, 대입 전무(리포 전체 확인). 업스트림 레거시
경로는 `mMaxFrames`(=fps), 신형 Settings 경로는 미초기화(UB)였다 — 골든 기준선
실측 동작(=0)의 정본화이며, 레거시 의미 복원은 FixLevel 대상 (DIVERGENCES #3).

`0` 고정이 다섯 읽기 사이트에 미치는 효과 (현재 라인):

| 사이트 | 효과 |
|---|---|
| `Track():1217` | `mnId <= mnLastRelocFrameId`로 붕괴 → 재추적 직후 `OK→LOST` 강등은 사실상 도달 불가 (P7_RECON §A F1) |
| `Track():1423` | "save frame, imu needs reset" 블록(`:1425-1433`) **죽은 코드** → B5의 3중 누수가 현재 억제되는 유일한 이유 |
| `Track():1439` | `mnId == mnLastRelocFrameId`에서만 발화, 호출되는 `ResetFrameIMU()`(`:1044-1047`)는 **빈 스텁** → "재추적 후 IMU 리셋" 메커니즘 전체가 무력 (B15) |
| `TrackWithMotionModel():2118` | IMU 단독 예측이 재추적 1프레임 후부터 허용 (레거시: fps 프레임 후) |
| `TrackLocalMap():2230` | 관성 pose 최적화기가 재추적 직후부터 사용 (레거시: fps 프레임 동안 시각 `PoseOptimization` 폴백) |

`mnFirstImuFrameId`(`Tracking.hpp:399`)는 `:3268`(`UpdateFrameIMU` 말미)에서
한 번 쓰이고 **읽히지 않는 죽은 멤버** (B14). 따라서
`mpImuPreintegratedFromLastKF`와 `mnFramesToResetIMU`는 더 이상 상호작용하지
않는다: 멤버 수명은 KF 생성(`:2500`), 맵 생성(`:1942`), 초기화 경로
(`:1612, :1727, :1875`)만으로 결정된다.

~~`UpdateFrameIMU`(`:3198-3269`)는 Tracking IMU 상태의 유일한 교차 스레드
변이자다.~~ **P11-A로 폐지된 서술** — 현행 계약: 舊 호출 5곳
(`ImuInitializer.cpp` ×3, `LoopClosing.cpp` MergeLocal2 ×2)은
`Tracking::PostImuUpdate(ImuUpdateMsg{scale,bias,pBaseKF,bFirstInit,t0Imu})`
게시로 대체됐고(단일 뮤텍스 슬롯 `mPendingImuUpdate`/`mMutexImuUpdate`,
scale 곱 합성·나머지 최신 우선·bFirstInit OR), T가 Track() 상단
mMutexMapUpdate 획득 직후 `ApplyPendingImuUpdate()`로 적용한다(= 기존
UpdateFrameIMU 본문, 이제 **T-스레드 전용**: `mlRelativeFramePoses` 리스케일,
`mLastBias`/`mpLastKeyFrame` 설정, 두 프레임 rebias, from-KF preintegration
pose/velocity 재작성). `mCurrentFrame.imuIsPreintegrated()` 대기 스핀(W9)은
적용 지점이 T 자신의 PreintegrateIMU 이후라 **삭제**(B1 행 위험 은퇴).
`t0IMU` 스탬프는 bFirstInit 플래그를 받아 T가 자기 현재 프레임 시각으로
찍는다(유계 편차, write-only 멤버라 관측 불가). 미적용 메시지는
Reset/ResetActiveMap/CreateMapInAtlas에서 폐기된다(프레임 기본 생성 복귀와
동기 — 리셋 2경로는 LM 핸드셰이크로 업스트림 등가, 포크 경로는 B10/B11
크래시 창의 드롭 강등). 이로써 Tracking의 프레임 객체·IMU 상태에 대한
교차 스레드 변이자는 0이 됐다(NotifyImuInitialized의 mState_ 쓰기만 남고,
그 저장소는 atomic).

---

## 6. FixLevel 후보 — 잠재 결함 대장 (수정 금지, 기록만)

정찰 §C의 B1-B15를 현재 HEAD 라인으로 갱신. 전부 업스트림 계승(INHERITED).

| # | 위치 (현재 HEAD) | 요약 |
|---|---|---|
| B1 | `Tracking.cpp:934-937` | `n==0` 조기 리턴이 `setIntegrated()` 누락 (다른 두 조기 리턴 `:886, :895`와 비대칭). ~~→ `UpdateFrameIMU():3229-3232`가 무한 스핀 가능~~ **P11-A에서 행 위험 은퇴**: 대기 스핀 삭제로 `imuIsPreintegrated()`의 소비자가 0 — 비대칭 자체는 보존되나 무해 (DIVERGENCES #28) |
| B2 | `Tracking.cpp:891-892` | `mlQueueImuData.size()` 2회를 락 밖에서 읽음 (락은 `:903`부터) — 비동기 생산자 등장 시 실제 레이스 |
| B3 | `Tracking.cpp:925-926` | `break;` 뒤 `bSleep=true;` 도달 불가 → `:929-930`의 지각-IMU 대기 usleep은 죽은 코드 |
| B4 | `Tracking.cpp:939` + `Frame.hpp:71` + `Frame.cpp:59` | 프레임당 from-frame 객체 누수 (소멸자 주석 처리, 복사 생성자 별칭, delete 전무). `Frame::mpMutexImu`(`Frame.cpp:179, 261, 362, 1095`에서 new) 동일 |
| B5 | `Tracking.cpp:1428-1432` | `new Frame` ×2 + `new Preintegrated`가 로컬 `pF`에만 → 실행 시 누수; `mnFramesToResetIMU==0`으로만 억제 중 |
| B6 | `Tracking.cpp:2500` (+`KeyFrame.cpp:54`) | `PreintegrateIMU` 조기 리턴 시(`:883-887, :892-897, :934-937`) `mCurrentFrame.mpImuPreintegrated==NULL` → 재지정이 (a) 누적 from-KF 객체 고아화 (b) `mPrevKF` 설정된(`:2491`) KF에 NULL preintegration → `LocalMapping.cpp:1033, :1042`(`MergePrevious`, 널 체크 없음)·`Optimizer.cpp:618` 역참조 위험 |
| B7 | `Tracking.cpp:1612-1613 + 1629` vs `:1610, :1725, :1941` | 스테레오 초기화 후 `pKFini`와 멤버가 한 객체 별칭 — 창 안에서 delete 도달 시(가장 유력: `Track():1076→CreateMapInAtlas():1941`) Atlas 내 KF가 해제된 객체 보유 |
| B8 | `Tracking.cpp:1713-1714 + 1725` | `mInitialFrame`/`mLastFrame` 복사 **후** delete → 댕글링 별칭 (`mLastFrame`은 `Track():1170`에서 곧 덮임, `mInitialFrame.mpImuPreintegrated`는 미독; 현재 비역참조). 스테레오 동형: `:1609-1610` 대비 `:1683` |
| B9 | `Tracking.cpp:3035-3094, :3096-3185` | `Reset()`/`ResetActiveMap()` 모두 큐 미비움·from-KF 미재생성 (§4) |
| B10 | `Frame.cpp:41` | 기본 `Frame()`이 `mpMutexImu` 미초기화 (`Frame.hpp:330`) — 기본 프레임 설치 창(`:1951-1952, :3083/:3085, :3173-3174`)에서 `UpdateFrameIMU():3229`가 미정 포인터 락 |
| B11 | `Tracking.cpp:908/912` (← `ResetActiveMap():3173-3174`) | 지연 리셋 뒤 `mbCreatedMap` 미설정 → 기본 `Frame`의 미정 `mTimeStamp`와 큐 타임스탬프 비교 |
| B12 | `ImuTypes.cpp:177`(무락) vs `:265`(락) | `IntegrateNewMeasurement`는 `mMutex` 미획득 — ~~추적 스레드 적분(`Tracking.cpp:980`)과 LocalMapping 스레드 `SetNewBias`의 데이터 레이스~~ **P11-A에서 교차 스레드 암 은퇴**: SetNewBias(프레임 경유)가 T 스레드로 이동해 두 접근이 동일 스레드 — 무락 비대칭 자체는 보존(비동기 생산자 등장 시 재검토) |
| B13 | `Optimizer.cpp:5201` vs `:5214, :5221` | `PoseInertialOptimizationLastFrame`이 에지는 from-frame 객체로, 랜덤워크 정보 블록은 from-KF 객체 `C`로 구성 — 업스트림 비대칭 보존. 두 객체 통합 리팩토링은 최적화 가중치를 바꾼다 |
| B14 | `Tracking.hpp:380` / `Tracking.cpp:3268` | `mnFirstImuFrameId` write-only 죽은 멤버 |
| B15 | `Tracking.cpp:1044-1047 + :1439-1443` | `ResetFrameIMU()` 빈 스텁 — "RESETING FRAME!!!" 경로 무동작 |

추가 (본 검증에서 확인): `Tracking.cpp:978-979`의
`if (!mpImuPreintegratedFromLastKF) cout << ...`는 경고만 출력하고 `:980`에서
그대로 역참조한다 — 가드가 아니라 로그다 (업스트림 동일).
