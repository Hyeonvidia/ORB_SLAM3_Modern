# TSAN 기준 대장 (P10-0, 이행 전 캡처 @ b0f829a)

캡처 2026-08-10, dev-tsan 서비스(seccomp=unconfined), 시나리오 정의는
`benchmark/scripts/tsan_smoke.sh`, 파서는 `tsan_ledger.py`(최상위 프레임 쌍 dedup).
T1 euroc_stereo 45s(139s 실측) · T2 kitti07 full(243s) · T3 stereo-inertial 75s(261s).

## 문서화된 레이스와의 대조 (OWNERSHIP.md R1-R5 / R-a~f)

| 레이스 | 기준 시그니처 | 발화 | 제거 담당 |
|---|---|---|---|
| **R4** (mbAbortBA 평문 bool) | `LocalMapping::InterruptBA:913 \| Optimizer::LocalBundleAdjustment:1535` (T1), `InterruptBA \| operator new` (T2) | **T1·T2 확정 발화** | P10-1 (atomic+섀도 브리지) |
| **R5/IMU-계약 클래스** (P7-1c) | `Frame::operator= \| {SetImuPoseVelocity, SetNewBias, imuIsPreintegrated, ImuInitializer::InitializeIMU:160, Tracking::UpdateFrameIMU:3255-3263}` | **T3 확정 발화** (11+ 시그니처) | 플래그류(mFirstTs 등)는 P10-2; **Frame 객체 자체의 교차 복사 레이스는 P10 범위 밖** — 툼스톤/계약 클래스로 존치, P11 이월 |
| R6 (Release↔SetFinish ABBA) | lock-order-inversion 더미에 매몰(아래 한계) | 판별 불가 | P10-2 (수정은 확정적, 증거는 by-construction) |
| R1, R3, R-b, R-a | 데이터 레이스 리포트 없음 | **미발화** (정찰 예측대로 좁은 창) | P10-2 — 증거는 by-construction + 신규 시그니처 0 |
| R2, R-c, R-d, R-f | LC/GBA 프레임 시그니처 없음(락 순서 더미 가능성) | 미판별 | P10-3/P10-5 |

## 안정 잡음 (전후 불변 기대 — 소거 대상 아님)

- `_Rb_tree<unsigned long>` set 복사/삽입 계열 (T1 ~90건, T2 ~180건): 공가시/ID
  집합의 무락 읽기 — OWNERSHIP.md 툼스톤 계약 클래스.
- `KeyFrame::ReplaceMapPointMatch:320` ↔ memmove/TrackedMapPoints: 동일 계급.
- `FrameDrawer::DrawTextInfo \| Update`: Viewer 계열 (P10-6 인접, 게이트 무관).
- `g2o G2OBatchStatistics` 정적 변수: 업스트림 내부, 존치.

## 도구 한계 (알려진 것)

1. **lock-order-inversion이 단일 시그니처로 붕괴** (T1 13,390 / T2 17,418건):
   최상위 프레임이 항상 인터셉터(pthread_mutex_lock)라 dedup이 무의미.
   R6 검증은 원시 로그(raw.tgz)의 Release/SetFinish 프레임 검색 + 수정
   by-construction으로 수행. tsan_ledger.py를 "최심 비인터셉터 프레임" 기준으로
   개선하는 것이 P10-1 동반 작업 후보.
2. gcc-11 TSAN의 `called_from_lib`는 **복수 라이브러리 매치 시 치명 오류**이며
   `exitcode=0`+`log_path` 조합이 이를 무증상 은폐한다 — tsan_smoke.sh가
   런타임에 라이브러리별 서프레션을 생성하는 이유 (P10-0 포스트모템).
3. Docker 기본 seccomp가 `personality(2)`를 차단해 TSAN 초기화가 죽는다 —
   dev-tsan 서비스(seccomp=unconfined)가 존재하는 이유. 진단 전용, ATE 판정
   금지.

## 스레드 누수 리포트

미조인 LM/LC/Viewer 스레드가 기준에서 리포트됨(예상대로) — P10-5의 Shutdown
조인 복원 후 소멸이 직접 증거가 된다.
