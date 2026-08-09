# P9 정찰: LoopClosing + PlaceRecognition + MapMerging

2026-08-09, 3-agent 병렬 정찰 (detect-machine / correct-merge-gba / surface-kfdb) 종합.
대상: `src/closing/LoopClosing.cpp`(2496줄) + `include/closing/LoopClosing.hpp`(261줄) +
`src/recognition/KeyFrameDatabase.cpp`. 모든 file:line은 phase-8 (cdf5031) 기준.
P8 정찰의 LC 안무 지도(OWNERSHIP.md §레이스, docs/P8_RECON.md §3)를 전제로 한다.

## 1. Run() 구조와 탐지 상태머신

한 반복에 KF 1개 소비(5ms 폴링). **병합이 루프에 우선** — 같은 KF에서 둘 다
발화하면 병합 처리(LC:127-225) 후 루프 후보는 미소비 폐기(LC:213-223).

**연속 KF 누적 메커니즘** (논문의 시간적 일관성): 채널당 앵커 KF + 연쇄 상대
Sim3. 신규 KF에서 `cnt>0`이면(LC:379) 오도메트리로 `gScw = gScl·mg2oLoopSlw`
전파(LC:383-385) → `DetectAndReffineSim3FromLastKF` 투영 검증(LC:388) → 성공 시
cnt++(:394), 앵커 교대(:395-396, SetErase 동반), 가설 갱신(:397), `cnt>=3`에서
DETECTED(:401). BoW 경로는 KF 1개 안에서 공가시 10개로 공간 검증(nNumKFs>=3,
LC:885)해 즉시 DETECTED 시딩 가능(LC:875-886).

**상태 변수 소유권 표와 상태×이벤트 전이표는 정찰 원문**(스크래치패드
p9_recon_agent3.md — 아래 §7의 보존 위치 참조)에 전문 수록. 요지:
- 상태 = 채널당 {IDLE(cnt=0), ACCUM(1-2), DETECTED(flag)}; 소거는 4곳
  (소비 :291-297/:204-211, 병합 우선 폐기 :213-223, 감쇠 :409-424/:451-464,
  스케일 중단 :149-156)
- 와이프는 카운터/플래그/벡터만 지우고 **포인터·Sim3는 절대 비우지 않음**
  (`mpLoopLastCurrentKF` 등은 생성자 미초기화 — cnt/flag 가드가 유효성을 암시)
- 비대칭 다수: 루프 reffine 실패는 `mbLoopDetected`를 안 지움(:409-424, 병합의
  :453과 비대칭), 병합 reffine 성공은 NotFound를 안 지움(:441-449, 루프의
  :402와 비대칭), BoW 시딩은 NotFound를 안 지움, `mvpMergeMPs`는 어떤 소비
  경로도 안 읽음(병합 증거는 폐기 — MergeLocal이 자체 fuse 셋 구축 :1990-2005)

## 2. 상속 결함 (탐지 계열) — 처분 지정

| # | 내용 | 처분 |
|---|---|---|
| D1 **오염 소비 합성 결함(R-1 형상)** | 병합 스케일 중단 `continue`(:158)가 `mbLoopDetected=true`를 남긴 채 반복 탈출(루프 상태 소거는 스킵된 병합-소비 경로에 있음) → 다음 KF에서 루프 reffine 실패해도 플래그 미소거 → :476이 true 반환 → **이전 KF에 앵커된 `mg2oLoopSlw`가 현재 KF의 보정 Sim3로 적용**(:236→:1019) = 본질그래프 오염. 두 비대칭의 합성 | 문서화, FixLevel 후보 1순위 (게이트 시나리오는 단일 맵이라 비발현) |
| D2 **래치 누수 L1** | ACCUM 중 BoW가 `mpLoopMatchedKF`를 SetErase 없이 덮어씀(:879-880) → 구 후보의 `mbNotErase` 영구 잔류 → 지연 SetBadFlag 영구 미완결(컬링 차단) | 문서화, FixLevel |
| D3 **래치 누수 L2** | BoW 약시딩이 `cnt=0`으로 시딩 가능(:878, 공간검증 0 + 투영 80+) → 해제 경로(:416)가 `cnt>0` 가드라 도달 불가 → 영구 래치 + 고아 앵커 상태 | 문서화, FixLevel |
| D4 **KFDB 무한 루프 행** | `DetectNBestCandidates`의 `if(pKFi->isBad()) continue;`(KFDB:731)가 :746-747의 `i++/it++`를 건너뜀 → bad KF가 리스트에 오르면 **행(전면 정지)** | **P9-3에서 가드**(#19 급 안전 강등: 반복자 전진 보장, DIVERGENCES 신규 항목) |
| D5 **리셋이 상태머신을 안 지움** | ResetIfRequested(:2249-2278)는 큐와 `mLastLoopKFid`만 소거 — 활성맵 리셋 후 카운터/앵커/매치 KF(래치 잔류)가 파괴된 맵을 가리킨 채 최대 2 KF 동안 교차 맵 reffine 시도 | 문서화, 타입드 머신이 리셋 구독해야 함(P9-4 설계 제약) |
| D6 **OptimizeSim3에 raw mbFixScale 전달** | :768-770에서 계산한 `bFixedScale`을 :772가 무시(IMU_MONO pre-BA2에서 고정스케일 강제) | 보존+문서화 (수치 결과 변화라 게이트 판별 영역) |
| D7 최다-BoW-매치 재선택 사장 | `nIndexMostBoWMatchesKF` 계산(:661-673) 후 재대입이 주석(:694) → `pKFi==pMostBoWMatchesKF` 항등 | 사장 코드로 P9-1 삭제 (동작 불변) |

## 3. 보정/병합/GBA (agent 1 전문은 p9_recon_agent1.md)

- **CorrectLoop**(:974-1222): R2 확정 — `RequestStop`(:980) 직후 `EmptyQueue`(:981)를
  **파킹 대기(:1001) 전에** 호출(MergeLocal/2는 안전 순서). GBA 중단은 파이어앤포겟
  (busy-wait 없음, 에포크 :990 자폭 의존). 조건부 GBA 스폰 게이트(:1208):
  `!isImuInitialized() || (KF<200 && 맵 1개)`. 대기(:1001-1004)는 finish-unaware
  (GBA 스레드의 :2338과 달리) — 종료 행 위험.
- **MergeLocal**(:1224-1792): 2단계 2정지창(용접 BA 사이). MergeScratch(P5-G) 현행
  확인(:1425, :1461 사장 선기록 → :1468 덮어씀). 맵 수술: 현재 맵 창을 병합 맵으로
  이주, `SetMapBad`+`ChangeId`(:1562-1566), `RemoveBadMaps`(:1790)는 delete 주석화라
  의도적 누수. GBA 재스폰 게이트가 **bad가 된 pCurrentMap을 읽고 pMergeMap에 스폰**
  (:1775-1782, 업스트림 기벽 보존).
- **MergeLocal2**(:1795-2075): 역방향 이주(병합 맵→현재 맵), `ApplyScaledRotation`
  일괄 적용 + `UpdateFrameIMU` 교차 스레드 변이(:1861-1862), BA2 미달 시 강제
  `SetIniertialBA1/2+SetImuInitialized`(:1881-1883). `bRelaunchBA` 사장(재스폰 없음).
  `mvpMergeConnectedKFs` **멤버가 병합 간 미소거 누적**(:1973-1977, cap-to-6이 이전
  병합의 stale 앞원소를 보존) — 로컬화 후보이나 동작 변화라 P9에선 문서화만.
- **GBA 스레드 수명**: 생성 2곳(:1215, :1781), join 없음. **정상 완주 시 joinable
  스레드 객체 누수**(다음 스폰이 포인터 덮어씀; delete는 중단 경로만 :992-996).
  스폰 시 플래그 4개 무락 기록(R-c). 낡은 결과 폐기 경로(:2324-2325)가
  `mbRunningGBA=true`를 영구 잔류시킬 수 있음. **System::Shutdown의 대기 루프가
  통째로 주석**(System.cpp:521-534) → 실행 중 GBA가 방치된 채 `SaveAtlas`가 맵을
  읽음 = 라이브 데이터 레이스. → 수명 정리는 P9-3(누수 join)과 P10(Shutdown 대기)
  로 분할.

## 4. Run 프로토콜 레이스 (LC판 R-목록, OWNERSHIP.md 이관 대상)

R-a `mbFinished=false` 무락(:97) · R-b **큐가 뮤텍스 2개에 이중 보호**(리셋은
mMutexReset만 들고 큐 변형 :2255/:2263 vs 생산자는 mMutexLoopQueue :318) ·
R-c GBA 스폰 플래그 무락(:1210-1215, :1778-1781; 중단 경로는 락 있음) ·
R-d `mnFullBAIdx` 무락 읽기(:2316) · R-e `mbStopGBA` 옵티마이저 내 포인터 폴링 ·
R-f `mpTracker->GetLastKeyFrame()` 무락(:1862, :2061) + Request*가 SetFinish 후
호출되면 영구 스핀. 전부 P10 이관(P8의 R1-R5와 동일 정책).

## 5. SetNotErase/SetErase 프로토콜

래치 획득은 단 2곳(:340 큐 팝, :880 BoW 최적 매치). KF 측: `SetErase`는
`mspLoopEdges` 비었을 때만 해제(**mspMergeEdges 미검사** — 루프엣지 KF는 영구 핀,
병합엣지 KF는 재컬링 가능; MergeLocal2는 AddMergeEdge 자체를 안 함). 지연
SetBadFlag는 **LC 스레드에서 완결**됨(KF:565-567) — #19 가드의 완결 경로.
실패 경로는 L1/L2(§2) 제외 전부 복원 확인. 컬링과의 상호작용은 P8 #19 문서와 정합.

## 6. 사장 코드 (P9-1 삭제 목록 — agent 2가 전수 grep 검증, 구현 시 재검증)

**LoopClosing.hpp 죽은 멤버**: `mstrFolderSubTraj`, `mnNumCorrection`→`mnCorrectionGBA`
체인, `vdPR_CurrentTime/vdPR_MatchedTime/vnPR_TypeRecogn`(매크로 밖 타이밍 벡터,
쓰기 :200-202/:230-232), `mstrFolderLoop`(REGISTER_LOOP에도 무사용),
`mnCovisibilityConsistencyTh`, 레거시 ORB-SLAM2 탐지기 상태(`ConsistentGroup`
typedef/`mvConsistentGroups`/`mvpEnoughConsistentCandidates`/`mvpCurrentMatchedPoints`),
`mScw`(cv::Mat — 삭제 시 클래스 마지막 OpenCV 타입 제거)/`mg2oScw`/`mpMatchedKF`/
`mLastLoopKFid`(3쓰기 전부 "not used" TODO)/`mg2oMergeSmw`(쓰기 전용 :171),
**public `mpViewer`**(+System.cpp:223의 죽은 쓰기), `mpORBVocabulary`(+ctor `pVoc`
파라미터 사장화 — 시그니처는 유지 검토), REGISTER_TIMES `vnLoopKFs`(푸시가 주석
:1032-1039인데 Tracking.cpp:523이 평균 냄 — ifdef 내부라 함께 정리).

**KeyFrame 죽은 필드**: `mvpLoopCandKFs/mvpMergeCandKFs`(KeyFrame.hpp:373-374) —
유일 참조가 LC:108-112의 죽은 scrub 블록. 필드+블록 동반 삭제.

**죽은 함수**: `CheckObservations`(:2077-2124, 호출자 0), `isFinishedGBA`(호출자 0),
KFDB `DetectLoopCandidates`(DEPRECATED 명기)/`DetectCandidates`/`DetectBestCandidates`
/`SetORBVocabulary`(더블포인터 const-cast 핵), KFDB 죽은 직렬화 배관(PreSave/PostLoad
선언만, `mvBackupInvertedFileId`, friend, 기본 ctor — Map::PostLoad가 재구축).

**죽은 로컬/블록**: Run `gSw1m`(:141), `bCheckSpatial`(:374/381), `vpConnectedKeyFrames`
(:486), reffine `vpMatchedMP`(:569-570), BoW `nIndexMostBoWMatchesKF`(:661-671, §2 D7)/
`spCheckKFs`(:726)/바운딩박스 블록(:786-818)/`Tc_w..vector_dist`(:839-842)/사장
`bFixedScale`(:768-770 — **D6 보존 결정과 함께**: 삭제하면 :772의 raw 전달이 그대로
정본이 됨을 주석으로 명시)/`vnStage/vnMatchesStage`+무효과 else 블록(:608-609,
:850-851, :887-899), MergeLocal `vpNewCovKFs.empty();`(:1324)/사장 선기록(:1461)/
`numPointsWithCorrection`/빈 then(:1657), MergeLocal2 `numTemporalKFs`/미사용 로컬
2개/`bRelaunchBA`/`t2,t3`/미판독 `NonCorrectedSim3`(:1926-1932)/잉여 return(:2074),
SearchAndFuse 카운터 4개, KFDB `nscores` ×6, 주석 블록 ~20곳(CheckEssentialGraph
디버그 잔재 등), `boost/algorithm/string.hpp` include, hpp의 Tracking.hpp include
(전방선언 공존 — P8-3 패턴로 사이클 절단).

**KFDB 잠복**: `clear()`만 mMutex 미획득(락 불일치), signed/unsigned 비교(:728),
`compFirst` 외부 링키지 — P9-1에서 함께 정돈(락 추가는 동작 영향 없는 안전 보강이나
보수적으로 문서화만, 락 불일치는 P10).

## 7. P9 실행 계획

| 단계 | 내용 | 검증 |
|---|---|---|
| P9-1 | §6 사장 코드 전수 삭제 (LC+KFDB+KeyFrame 필드+System 죽은 쓰기; D7 포함) | 빌드 + 비트 게이트 + 스모크 |
| P9-2 | 문서화: OWNERSHIP.md에 LC 레이스 R-a~f + GBA 수명 위험 + SetNotErase 프로토콜/누수 L1·L2 + 리셋 미소거(D5), DIVERGENCES에 D1(오염 소비)·D6(raw mbFixScale) 상속 결함 기재 | 문서 전용 |
| P9-3 | 안전 가드 2건: D4 KFDB 반복자 전진 가드(행→정상 스킵), GBA 완주 스레드 join-후-delete(누수 제거, 스폰 직전) — 각각 DIVERGENCES 신규 항목 | 빌드 + 스모크 + KITTI 07 루프 확인 |
| P9-4 | **타입드 탐지 상태머신** — 채널당 상태 페이로드 구조체(가드-암시 유효성 → 상태별 명시 페이로드), P7 SetState-with-reason 트레이싱 재사용, 리셋 구독(D5 해소는 동작 보존 범위에서 트레이스만), 전이 골든 트레이스 채취(KITTI 00) | 빌드 + 스모크 + KITTI 00 루프 5회 검출 + 골든 트레이스 대조 |
| P9-5 | PlaceRecognition 추출(탐지 5함수+큐+머신 → collaborator, P8-4 mHost 패턴), LoopCorrector/MapMerger 분리는 규모 보고 결정(과대 시 P9.5로 이월) | 빌드 + 스모크 |
| 게이트 | 풀 게이트 v2.2.1 4라운드(KITTI 00) **+ stereo-inertial 상시 +4쌍(8v8)** — #20 재계측 의무. SI 추가 악화 시 #20 재개 절차 | 페어드 인터리브 |

**보존 확인 항목**: LC 큐 비대칭(EmptyQueue/IMU 드레인 소비 KF는 LC 미도달),
병합 우선순위, MergeLocal2의 `mvpMergeConnectedKFs` 누적(문서화만), R2 순서(P10),
D1/D6 수치 동작. **P8 #20 교훈 적용**: 경계 p값은 즉시 해당 모드 쌍 추가.

정찰 원문 3부: 세션 스크래치패드 `p9_recon_agent{1,2,3}.md`
(워크플로 wf_ab715de9-1c4 journal에도 보존).
