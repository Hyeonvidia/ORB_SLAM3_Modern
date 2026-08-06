# ORB-SLAM3 Modern C++ 재구현 계획 (v1, 2026-08-06)

## 0. 목표

무수정 업스트림 ORB_SLAM3 V1.0(4452a3c, GPLv3)을 **동작 보존**을 최우선으로 modern C++ / 디자인 패턴 기반으로 재구현한다.
- Thirdparty는 전부 git submodule (벤더링 금지)
- 번들 g2o(2016년경 포크)를 **최신 백엔드**로 교체 (IOptimizer 추상화 경유)
- Docker에서 EuRoC/KITTI로 매 단계 검증, 원본 대비 성능(ATE) 동등성 확보
- GitHub(Hyeonvidia)에 주요 마일스톤마다 커밋/태그

## 1. 정찰 근거 요약 (7-agent recon, 2026-08-06)

- **원본 구조**: 29,115 LOC. 4스레드(usleep 폴링, CV 전무). God class: Optimizer 5.6k(전부 static + 침습 캐시 필드), Tracking 4.1k(전방위 포인터, System 역참조). Frame static 내부파라미터(멀티카메라 불가). Config.h 죽은 코드. Atlas가 구체 카메라 직접 include(직렬화 탓). MapPoint::mGlobalMutex 전역 잠금.
- **감사 문서**: 정확도 직접 영향 버그 9건 + 결함 21건 file:line 확보. 논문 미명시 파라미터 32건 중 24건 코드 회수 완료.
- **논문 분석 문서**: 보존 불변식 10개(Tracking marginalize vs LBA fix 비대칭, 단일 DBoW2 DB, spanning tree 완전성, mature 플래그, 카메라 3-호출점 격리 등), 병목 우선순위(Local BA 53~81%), 동시성 계약.
- **Remastered 교훈 8**: ①골든 회귀 하네스 부재가 최대 실패 요인 ②업스트림 기준 커밋 부재 ③서드파티 빌드 플래그 패리티(단안-관성 회귀 유력 용의: libg2o `-march=native` 유무) ④GTSAM 빅뱅 포팅 금지(메서드별 등가성 테스트 先) ⑤A/B는 상태 격리 필수 ⑥알려진 지뢰(Atlas 자기 데드락 등) 설계 단계 제거 ⑦논문 주장 검증 테스트 ⑧산출물 커밋 위생.
- **g2o 표면**: Optimizer 17함수 전수 목록(솔버/버텍스/엣지/커널/반복수) + 커스텀 vertex/edge 20종 + 최신 g2o 이전 시 파손 지점 8건(스마트포인터 솔버 생성, VertexSBAPointXYZ→VertexPointXYZ, 헤더 경로, number_t 시그니처…) + GTSAM 매핑표. 엣지들의 `GetHessian*()`(marginalization용)은 g2o 기성 기능이 아님 — 어떤 백엔드로 가든 보존 필요.
- **백엔드 지형**: g2o 최신 태그 `20241228_git`(C++17, arm64 CI, march=native 기본 OFF). GTSAM 4.2.2 안정/4.3a2(Boost 제거 방향). GTSAM에는 Sim3 essential-graph 기성 파이프라인 없음, KB8 어안·EdgeInertialGS·marginalization는 커스텀 필요. 유지되는 modern ORB-SLAM3 재구현은 세상에 없음(stella_vslam은 SLAM2 계열·IMU 없음).

## 2. 핵심 결정 (권고안)

| # | 결정 | 권고 | 근거 |
|---|---|---|---|
| D1 | 백엔드 | **최신 g2o(태그 고정, 예: 20241228_git) 1차 → GTSAM 2차(옵션 Phase)** | 등가성 리스크 최소(1:1 포팅 경로 확보), GTSAM 직행은 Sim3 EG/KB8/marginalization 커스텀 3종 세트로 "원본만큼 성능" 검증이 어려움. Remastered 교훈 ④ |
| D2 | BoW | DBoW2 업스트림 submodule + wrapper(txt vocab 로더는 래퍼에) | direct index가 SearchByBoW에 필수(FBoW엔 없음). DBoW3 전환은 백로그 |
| D3 | Lie/기하 | Sophus 1.24.6 submodule, Eigen은 전 서브모듈 단일 버전(시스템 3.4) | Eigen ABI 정렬 혼용이 arm64 크래시 원인 |
| D4 | 라이선스 | GPLv3 | 동작 보존 재구현 = 파생 저작물 |
| D5 | 버그 정책 | **bug-for-bug 보존이 기본**, 감사 D-1 9건은 `FixLevel` 플래그 뒤에서 수정 제공, 벤치는 보존 모드로 원본과 비교 | 등가성 검증과 개선을 분리 |
| D6 | 스레딩 | 이주 기간 내내 원본 4-스레드 구조 유지, 최종 Phase에서만 현대화 | 성능/타이밍 회귀 격리 |
| D7 | 직렬화 | boost serialization은 초기 보존, Atlas save/load 재설계는 백로그(GTSAM 4.3의 Boost 제거와 함께 결정) | recon 모순 지점 |

## 3. 저장소 구성

```
<repo>/                      # GitHub Hyeonvidia/<이름>, GPLv3
├── third_party/             # 전부 submodule, 전부 태그 고정
│   ├── g2o        (RainerKuemmerle/g2o @ 20241228_git)
│   ├── DBoW2      (dorian3d/DBoW2 upstream)
│   ├── Sophus     (strasdat/Sophus @ 1.24.6)
│   ├── Pangolin   (stevenlovegrove/Pangolin @ v0.9.4 계열)
│   └── (gtsam     — Phase 10에서 추가)
│   └── wrappers/  # 무수정 업스트림 + 어댑터 (Remastered 검증된 패턴)
├── src|include/  core/ camera/ features/ map/ backend/ tracking/
│                 mapping/ recognition/ closing/ io/ viz/
├── tests/        unit/ equivalence/ synthetic/   # GTest
├── benchmark/    golden/ (원본 기준선 ATE) + scripts (evo)
├── docker/       Dockerfile, compose, Makefile   # orb_slam3_docker 패턴 재사용
└── docs/         PLAN, AUDIT_BRIEF(계승), phase 리포트
```

**커밋 규율**: 커밋 0 = pristine upstream import(`upstream-v1.0` 태그) → 이후 모든 diff가 저장소 안에서 추적 가능(Remastered 교훈 ②). Phase 완료마다 태그(`phase-N`) + 회귀 리포트 커밋. conventional commits.

## 4. 검증 하네스 (모든 Phase의 게이트)

- **골든 기준선**: 이번에 검증한 `orb_slam3_docker` 원본 궤적(EuRoC MH01 4모드 + KITTI 00 stereo/mono)을 시드로, evo(`evo_ape`)로 GT 대비 ATE RMSE 산출. 비결정성 대응 **N=5 실행 중앙값 + 범위 기록**.
- **게이트 기준(모드별)**: ATE RMSE 중앙값이 원본 중앙값 대비 **+10% 또는 절대 +5mm(EuRoC)/+0.2m(KITTI) 중 큰 쪽 이내**, 그리고 트래킹 완주(프레임 손실률 원본 동등).
- **2단 게이트 구조** (2026-08-06 확정):
  - **스모크 게이트** (반복 개발 중 상시, ~4분): `euroc_stereo MH01` 1회 완주 + ATE가
    관대한 상한(기준선 중앙값 ×2) 이내 — 총체적 파손을 즉시 감지.
    `benchmark/scripts/smoke_gate.sh`
  - **풀 게이트** (Phase 완료 시에만, ~55분): 5모드 × N=3 중앙값 vs 원본 N=5 중앙값,
    +10% 또는 절대 여유(EuRoC +5mm / KITTI +0.2m) 중 큰 쪽 이내 + 완주.
- **단계 사이클**: 모듈 정리 → 빌드 그린 → 유닛 테스트 → 스모크 게이트 → 커밋,
  Phase 완료 시 풀 게이트 → phase-N 태그. 실패 시 직전 모듈이 범인.
- **백엔드 등가성 테스트**(Phase 5 전용): 동일 입력 그래프 직렬화 → 번들 g2o vs 새 백엔드 출력 비교(허용오차), 합성 IMU 스케일/중력 시나리오(Remastered 단안-관성 회귀의 재발 방지 지점), 빌드 플래그 명시 고정.

## 5. 모듈 이주 순서 (11 Phase)

의존성 잎부터, 스레드 오케스트레이션은 마지막. 매 Phase가 "일부 신규 + 나머지 원본"으로 전체 빌드·실행 가능.

| Phase | 내용 | 완료 게이트 |
|---|---|---|
| **P0** | 저장소/서브모듈/Docker/골든 하네스 구축, evo 파이프라인, AUDIT_BRIEF 계승 | 하네스가 원본 이미지로 기준선 재산출 |
| **P1** | pristine upstream 임포트 + 새 빌드 시스템(모던 CMake, 타깃 분리)으로 빌드 그린 | 5모드 골든 회귀 통과 (= 기준선 재현) |
| **P2** | **기반 계층**: Settings 재설계(Config.h 죽은 코드 제거, 이중 파싱 경로 통합), 로깅, Converter 정리, 타입/Lie 유틸 통합(double/float 중복 제거) | 게이트 통과 |
| **P3** | **카메라 계층**: GeometricCamera 계열 정리 + CameraFactory, Frame static 내부파라미터 제거(추상화 누수 22건), TwoViewReconstruction을 카메라 모델 밖으로 | 게이트 통과 |
| **P4** | **특징 계층**: ORBextractor/ORBmatcher 순수함수화 — 원본과 출력 비트 동일성 테스트 가능한 유일한 계층 | 비트 동일 테스트 + 게이트 |
| **P5** | **데이터 계층**: MapPoint→KeyFrame→Frame→Map→Atlas→KeyFrameDatabase. 소유권 규칙 명문화, 낙서 필드(mnBALocalForKF류) 외부화, Atlas 자기 데드락 제거(CreateNewMapNoLock), mGlobalMutex 전략 재검토(동작 보존 범위 내) | 게이트 통과 |
| **P6** | **백엔드 추상화 + g2o 최신 이전**: ISP로 분할된 인터페이스(TrackingOpt/LocalBAOpt/LoopOpt/InertialOpt — 17메서드 단일 인터페이스 금지), 함수별 이전+등가성 테스트, GetHessian 계약 보존, 플래그 패리티 문서화 | 함수별 등가성 + 게이트 |
| **P7** | **Tracking**: 4.1k 상태머신 분해(State 패턴), System 역참조 제거(DI), IMU 큐 소비 정리 | 게이트 통과 |
| **P8** | **LocalMapping + IMU 초기화**: 큐/배압 계약 명시화, InitializeIMU/ScaleRefinement 분리 | 게이트(관성 모드 중점) |
| **P9** | **Recognition/LoopClosing/MapMerging**: PlaceRecognition 분리하되 loop/merge 동시검출 상태머신은 타입화된 단일 기계로(R-1 회귀 교훈), GBA 스레드 수명 명시 | 게이트 + 루프 시나리오(KITTI 00) |
| **P10** | **System/스레딩 현대화**: jthread/stop_token/condition_variable(usleep 폴링 제거), Viewer 분리 | 전 모드 최종 회귀 + 성능 비교 리포트 |
| **P11**(옵션) | GTSAM 2차 백엔드(메서드별 등가성 선행), FixLevel 벤치, DBoW3/직렬화 재설계 백로그 | — |

## 6. 리스크 목록

1. **비결정성**: 멀티스레드 타이밍으로 실행마다 궤적 상이 → N=5 중앙값 + 분포 비교로 완화.
2. **번들 g2o의 ORB 저자 수정분**: types_six_dof_expmap에 추가된 엣지들 — 업스트림에 이미 역수입됨을 확인했으나 Phase 6에서 실제 대조 필요.
3. **-march=native류 플래그**: 전 서브모듈 OFF 고정 + 컴파일 플래그를 회귀 리포트에 기록.
4. **GetHessian/marginalization**: g2o 기성 기능 아님 — 커스텀 엣지에 유지, 등가성 테스트 대상.
5. **KITTI 13-21 yaml 부재, kitti_mono는 KF 궤적만** — 골든 대상에서 제외(orb_slam3_docker에서 확인된 제약).
