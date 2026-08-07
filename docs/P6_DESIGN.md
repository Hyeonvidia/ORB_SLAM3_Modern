# P6 설계서 — 백엔드 최신 g2o 이전 (정찰 확정본, 2026-08-07)

3-agent 정찰 결과 원문. 구현 중 이 계약과 어긋나는 발견은 이 문서를 갱신할 것.

> 보정 1건: §1의 '업스트림 투영 엣지로 커스텀 대체 가능' 결론은 핀홀 전용에만
> 해당한다. ORB_SLAM3::Edge* 커스텀 엣지는 GeometricCamera(KB8 어안) 경유라
> 대체 불가 — 커스텀 유지가 마이그레이션 계약이다.
>
> 보정 2건 (P6-1 Phase A 구현 중 발견): §C의 'ILoopOptimizer.hpp가
> closing/LoopClosing.hpp를 include'하는 설계는 포함 사이클로 불성립 —
> System.hpp(G2oBackend 값 멤버)가 LoopClosing.hpp의 include 체인 내부에서
> 도달된다 (LoopClosing.hpp → Tracking.hpp → Viewer.hpp → System.hpp →
> G2oBackend.hpp → ILoopOptimizer.hpp). 실제 구현은 Converter.hpp 선례대로
> 협소한 `Thirdparty/g2o/g2o/types/sim3.h`만 include하고, LoopClosing::
> KeyFrameAndPose와 **동일 타입**의 typedef를 ILoopOptimizer 클래스 내부에
> 재선언한다 (`ILoopOptimizer::KeyFrameAndPose`) — 타입 동일성으로 시그니처
> 호환은 그대로 성립하며, 헤더 g2o 노출은 sim3.h 1개로 §C 의도(비용을
> ILoopOptimizer만 부담)를 오히려 더 좁게 달성한다.


## A. g2o 20241228_git API 계약 (헤더 실측)

# g2o 마이그레이션 계약 검증 — 핀 고정 서브모듈 실측 결과

**대상**: `/Users/jhpark/VSLAM/ORB_SLAM3_Modern/third_party/g2o` @ `20241228_git` (eec325a1, 2024-12-14 커밋) — `git describe --tags`로 확인. 모든 근거는 이 커밋의 헤더를 직접 읽은 결과.

## (1) 솔버 구성 API

| 항목 | 판정 | 근거 (file:line) |
|---|---|---|
| `g2o::BlockSolver_6_3` 존재 | VERIFIED | `g2o/core/block_solver.h:194` — `using BlockSolver_6_3 = BlockSolverPL<6, 3>;` |
| `g2o::BlockSolver_7_3` 존재 | VERIFIED | `g2o/core/block_solver.h:197` |
| `g2o::BlockSolverX` 존재 | VERIFIED | `g2o/core/block_solver.h:191` — `BlockSolverPL<Eigen::Dynamic, Eigen::Dynamic>` |
| BlockSolver 생성자가 `std::unique_ptr` 수취 | VERIFIED | `g2o/core/block_solver.h:124` — `BlockSolver(std::unique_ptr<LinearSolverType> linearSolver);` |
| `OptimizationAlgorithmLevenberg` 생성자 | VERIFIED (unique_ptr) | `g2o/core/optimization_algorithm_levenberg.h:47` — `explicit OptimizationAlgorithmLevenberg(std::unique_ptr<Solver> solver);` |
| `g2o::make_unique` 존재 | **CHANGED — 없음** | `g2o/stuff/` 전체 grep 결과 0건 (misc.h에 없음). `std::make_unique` 사용 필수 |

주의: `OptimizationAlgorithmWithHessian`의 생성자는 `Solver&` 참조 수취(`optimization_algorithm_with_hessian.h:44`)로 바뀌었으나 Levenberg 파생 클래스가 `unique_ptr`를 받아 내부에서 `m_solver`(`:96`)로 소유하므로 호출측은 `unique_ptr`만 넘기면 됨.

## (2) 정점/에지 타입명

| 항목 | 판정 | 근거 |
|---|---|---|
| `VertexSBAPointXYZ` | **CHANGED — 삭제됨** | 전체 트리 grep 0건 |
| 대체: `VertexPointXYZ` | VERIFIED | `g2o/types/slam3d/vertex_pointxyz.h:38` — `class G2O_TYPES_SLAM3D_API VertexPointXYZ : public BaseVertex<3, Vector3>`. `types_sba.h:38`이 이 헤더를 include |
| `EdgeSE3ProjectXYZ` 업스트림 존재 | VERIFIED | `g2o/types/sba/edge_project_xyz.h:38` — `BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSE3Expmap>`, 멤버 `fx=1., fy=1., cx=0.5, cy=0.5` (`:54`) — ORB-SLAM 스타일 그대로 |
| `EdgeSE3ProjectXYZOnlyPose` | VERIFIED | `g2o/types/sba/edge_project_xyz_onlypose.h:38` — `BaseUnaryEdge<2, Vector2, VertexSE3Expmap>`, 멤버 `fx,fy,cx,cy`(`:53`) + `Xw`(`:52`) |
| `EdgeStereoSE3ProjectXYZ` | VERIFIED | `g2o/types/sba/edge_project_stereo_xyz.h:38` |
| `EdgeStereoSE3ProjectXYZOnlyPose` | VERIFIED | `g2o/types/sba/edge_project_stereo_xyz_onlypose.h:38` — 멤버 `fx,fy,cx,cy,bf`(`:58`) |
| `VertexSE3Expmap` | VERIFIED | `g2o/types/sba/vertex_se3_expmap.h:40` — `BaseVertex<6, SE3Quat>` |
| 헤더 구조 | **CHANGED — 분할됨** | `types_six_dof_expmap.h`는 존재하나 umbrella 헤더로 축소(`:38-47`에서 `edge_project_*.h` 개별 헤더 include). 직접 include 경로를 개별 헤더로 잡아도 됨 |

## (3) 커스텀 타입이 맞춰야 할 가상함수 시그니처

| 항목 | 판정 | 근거 |
|---|---|---|
| `oplusImpl(const double*)` | VERIFIED (double, number_t 아님) | `g2o/core/optimizable_graph.h:398` — `virtual void oplusImpl(const double* v) = 0;` |
| `setToOriginImpl()` | VERIFIED | `g2o/core/optimizable_graph.h:401` |
| `number_t` | 존재하나 alias | `g2o/core/eigen_types.h:36` — `using number_t = double;` (하위호환용). `VectorX::MapType` 강제 시그니처 아님 |
| vertex/edge `read`/`write` | VERIFIED (순수가상) | `optimizable_graph.h:374,376` (vertex), `:525,527` (edge) — `virtual bool read(std::istream&) = 0; virtual bool write(std::ostream&) const = 0;` 반드시 구현 필요 |
| `computeError()` | VERIFIED | `optimizable_graph.h:429` — 순수가상 |
| `BaseBinaryEdge` 존재 | VERIFIED (thin alias) | `g2o/core/base_binary_edge.h:37` — `class BaseBinaryEdge : public BaseFixedSizedEdge<D, E, VertexXi, VertexXj>`. `BaseUnaryEdge`도 동일 패턴(`base_unary_edge.h:37`). 신규 코드는 `BaseFixedSizedEdge<D, E, VertexTypes...>`(`base_fixed_sized_edge.h:124`, `template <int D, ...>`) 직접 상속 가능하나 기존 이름 유지 가능 |
| `setRobustKernel` | VERIFIED (raw pointer) | `optimizable_graph.h:454` — `void setRobustKernel(RobustKernel* ptr);` shared_ptr 아님. (`robust_kernel.h:73`의 `RobustKernelPtr`는 내부 typedef일 뿐) |

## (4) include 경로 존재 여부

| 헤더 | 판정 |
|---|---|
| `g2o/solvers/eigen/linear_solver_eigen.h` | VERIFIED — `class LinearSolverEigen : public LinearSolverCCS<MatrixType>` (`:51`) |
| `g2o/solvers/dense/linear_solver_dense.h` | VERIFIED — `class LinearSolverDense : public LinearSolver<MatrixType>` (`:46`) |
| sim3 타입 | VERIFIED — `g2o/types/sim3/types_seven_dof_expmap.h`: `VertexSim3Expmap`(`:68`, `BaseVertex<7, Sim3>`), `EdgeSim3`(`:111`), `EdgeSim3ProjectXYZ`(`:148`, **VertexPointXYZ 사용**), `EdgeInverseSim3ProjectXYZ`(`:169`) |

## (5) CMake 계약

| 항목 | 판정 | 근거 (모두 top-level `CMakeLists.txt`) |
|---|---|---|
| export 타깃 | `core`, `stuff`, `solver_eigen`, `solver_dense`, `types_sba`, `types_sim3`, `types_slam3d` 등 (add_subdirectory 시 이 이름 그대로; 설치 시 namespace `g2o::` `:503,:522`, 출력파일명 prefix `g2o_` `:12`) | 각 서브디렉토리 CMakeLists `:1` |
| C++ 표준 | core가 `cxx_std_17` PUBLIC 전파 | `g2o/core/CMakeLists.txt:55` — C++20 프로젝트와 호환 |
| `BUILD_WITH_MARCH_NATIVE` | 기본 **OFF** (`:366`) — 명시 OFF 불요하나 고정 권장. arm/darwin 분기 있음(`:388,:406`) | |
| `G2O_USE_LOGGING` | 기본 **ON** (`:177`) — 단, spdlog 미발견 시 조용히 `G2O_HAVE_LOGGING=0`으로 폴백(`:178-185`, `find_package QUIET`). `logger.h`의 spdlog include는 `#ifdef G2O_HAVE_LOGGING` 가드(`g2o/stuff/logger.h:33`). 결정론적 빌드 위해 **명시 OFF 권장** | |
| `G2O_BUILD_APPS` | 기본 ON (`:265`) — **OFF 필요** | |
| `G2O_BUILD_EXAMPLES` | 기본 ON (`:275`) — **OFF 필요** | |
| `G2O_USE_CHOLMOD`/`G2O_USE_CSPARSE`/`G2O_USE_OPENGL` | 기본 ON (`:97,:130,:159`) — Eigen 솔버만 쓰면 OFF 권장 (외부 의존 차단) | |
| 불필요 타입 라이브러리 | `G2O_BUILD_SLAM2D_TYPES` 등 개별 옵션 존재(`:191-221`) — sba/sim3/slam3d만 ON으로 축소 가능. 단 `types_sba`는 `types_slam3d`에 의존(VertexPointXYZ) | |

## 핵심 마이그레이션 계약 요약

1. `g2o::make_unique` → `std::make_unique` (유일한 solver-API 소스 수정점; unique_ptr 파이프라인 자체는 ORB-SLAM3 원본과 동일하게 유지 가능)
2. `VertexSBAPointXYZ` → `VertexPointXYZ` 전면 치환 (include는 `g2o/types/slam3d/vertex_pointxyz.h` 또는 `types_sba.h` 경유)
3. ORB-SLAM3가 자체 정의하던 `EdgeSE3ProjectXYZ`/`OnlyPose`/Stereo 4종 + `EdgeSim3ProjectXYZ`/`EdgeInverseSim3ProjectXYZ`는 업스트림에 동일 이름·동일 멤버(fx/fy/cx/cy/bf, Xw)로 존재 — 커스텀 재정의 삭제하고 업스트림 사용 가능 (단 IMU 관련 커스텀 타입은 여전히 자체 구현 필요)
4. 커스텀 vertex/edge: `oplusImpl(const double*)`, `read`/`write` 순수가상 구현 필수, `setRobustKernel(RobustKernel*)` raw pointer 유지
5. CMake: `G2O_BUILD_APPS=OFF`, `G2O_BUILD_EXAMPLES=OFF`, `G2O_USE_LOGGING=OFF`(spdlog 의존 차단), `G2O_USE_CHOLMOD/CSPARSE/OPENGL=OFF` 권장; `BUILD_WITH_MARCH_NATIVE`는 이미 기본 OFF; 링크 타깃은 `core stuff solver_eigen solver_dense types_sba types_sim3 types_slam3d`

## B. 등가성 하네스 설계

조사를 마쳤습니다. 실제 코드 근거(파일:라인)를 확인한 설계안을 반환합니다.

---

# P6-1단계: 백엔드 함수별 등가성 테스트 하네스 설계 (ORB_SLAM3_Modern)

## 0. 코드 조사 요약 (설계 근거)

| 확인 항목 | 근거 |
|---|---|
| `PoseOptimization`이 만지는 Frame 필드 | `src/backend/Optimizer.cpp:815-1115` — `GetPose/SetPose`, `N`, `mvpMapPoints`, `mvuRight`, `mvKeysUn`(octave 포함), `mvInvLevelSigma2`, `mvbOutlier`, `mpCamera`, `mpCamera2`(null이면 단안 경로), 스테레오 경로만 `fx/fy/cx/cy/mbf`, 어안 경로만 `mvKeys/mvKeysRight/Nleft/GetRelativePoseTrl` |
| `InertialOptimization(Map*,Rwg,scale)` 접촉면 | `Optimizer.cpp:3393-3500` — `Map::GetMaxKFid/GetAllKeyFrames`; KF별 `mnId`, `isBad()`, `mPrevKF`, `mpImuPreintegrated`; 정점 생성자 경유(`G2oTypes.cpp:25-71,456-488`) `GetImuPosition/GetImuRotation/GetTranslation/GetRotation`, `mImuCalib.mTcb/mTbc`, `mpCamera`, `mbf`, `GetVelocity`, `GetGyroBias/GetAccBias` |
| `EdgeInertialGS` | `G2oTypes.cpp:596-613` — 정보행렬 = `pInt->C.block<9,9>` 역행렬 + 고유값 1e-12 클램프(float C → double 역행렬). `computeError`(617-640)는 `GetDeltaRotation/Velocity/Position(b)`(float 경로) 사용 |
| 출력 정밀도 함정 | `PoseOptimization`은 결과를 **float** `Sophus::SE3f`로 `pFrame->SetPose` (`Optimizer.cpp:1110-1112`) → 출력 비교 하한이 float eps. 반면 `InertialOptimization(Map,Rwg,scale)`의 `scale`(double&)·`Rwg`(Matrix3d&)는 **double 그대로 반환** (3498-3499) → 타이트한 게이트 가능 |
| g2o 결정성 | 구 g2o: `_activeEdges/_activeVertices`를 `internalId`(삽입순)·id로 정렬 (`Thirdparty/g2o/g2o/core/sparse_optimizer.cpp:482-486`, `optimizable_graph.h:115`) → 삽입 순서가 고정이면 단일 바이너리 내 결정적. 신규 g2o(2024-12 master, raw-pointer API 유지) 동일 구조 |
| 픽스처 구성 가능성 | `Frame()`/`KeyFrame()`/`MapPoint()` 기본 생성자 존재. **주의: `KeyFrame::SetPose`는 `mImuCalib.mbIsSet`일 때만 `mOwb` 계산 (`KeyFrame.cpp:116-119`) → 픽스처는 반드시 `mImuCalib` 먼저, `SetPose` 나중** |
| ODR 충돌 | 두 g2o 모두 동일 네임스페이스·동일 심볼 + CMake `project(g2o)` 타깃명까지 충돌 → 한 빌드트리/한 바이너리에 공존 불가 확정 |
| 기존 패턴 | `tests/bit_identity/extract_hash.cpp` (P4) — 직렬화 스트림+해시+2회 실행 자기결정성 검사 패턴을 재사용 |

`src/map/*`, `src/camera/*`, `ImuTypes.cpp`는 g2o 심볼 미사용(단, `Frame.cpp:21`이 `G2oTypes.hpp`를 include하므로 **컴파일**은 g2o 헤더에 의존 → 코어 소스도 변형별로 재컴파일해야 함).

---

## 1. 함수 분류와 최소 합성 입력 (17함수 → 6클래스)

- **A. Frame 단항** — `PoseOptimization` → `FrameFixture`
- **B. Frame 관성 슬라이딩** — `PoseInertialOptimizationLastKeyFrame/LastFrame` → `FrameFixture + ImuLinkFixture` (출력에 `mpcpi`의 15×15 Hessian 포함 — GetHessian 계약)
- **C. KF체인 관성 초기화** — `InertialOptimization` ×3 → `ImuKfChainFixture` (**Remastered 회귀 핵심**)
- **D. 시각 BA** — `BundleAdjustment`, `GlobalBundleAdjustemnt`, `LocalBundleAdjustment` ×2 → `MapFixture` (관측 양방향 배선 + `UpdateConnections` 필요)
- **E. 관성 BA** — `FullInertialBA`, `LocalInertialBA`, `MergeInertialBA` → C+D 합성
- **F. Sim3/포즈그래프** — `OptimizeSim3`, `OptimizeEssentialGraph` ×2, `OptimizeEssentialGraph4DoF` → 최중량(스패닝트리·루프엣지)
- (`Marginalize`는 순수 Eigen이라 g2o 무관 — 단위테스트로 별도 처리, 17개 계산에서 제외)

### 1.1 `FrameFixture` (클래스 A) — 채워야 하는 최소 필드

```
Frame f;                          // 기본 생성자
f.SetPose(Tcw_noisy);             // GT에서 회전 ~2도, 병진 ~5cm 섭동
f.N = 40;                         // 단안: mpCamera2=nullptr, Nleft=-1 명시
f.mpCamera = new Pinhole({fx,fy,cx,cy});
f.mpCamera2 = nullptr;
f.mvpMapPoints = {40개 MapPoint*}; // MapPoint()+SetWorldPos(격자 3D점), 4개는 의도적 아웃라이어
f.mvKeysUn   = GT 투영점(+아웃라이어는 큰 오프셋), octave 0..7 분산
f.mvuRight   = 전부 -1.f (단안) / 스테레오 픽스처는 유효 uR + f.fx/fy/cx/cy/mbf
f.mvbOutlier = N개 false
f.mvInvLevelSigma2 = {1.2^-2k} 8레벨
```
**마진 설계 원칙**: 의도 인라이어는 chi2 < 0.5, 의도 아웃라이어는 chi2 > 30이 되도록 배치 (문턱 5.991/7.815 근처 금지 — 구/신 g2o의 마지막-ULP 차이가 이산 분기(setLevel)로 증폭되는 것을 원천 차단).

### 1.2 `ImuKfChainFixture` (클래스 C) — 합성 KF 체인

핵심 트릭 2개:

1. **등속 궤적으로는 스케일이 비가관측** — `ep = R1ᵀ(s(p2−p1−v1·dt) − ½g·dt²) − ΔP`에서 등속이면 `p2−p1−v1·dt = 0`이라 s 항이 소멸. → **등가속 궤적** 사용 (예: `a_world=(0.3,0,0)`, 자이로 0 또는 소량 상수 회전). IMU 측정치는 `a_meas = Rᵀ(a_world − g_world)`.
2. **GT를 연속시간이 아니라 이산 프리인테그레이션 재귀 자체로 정의**: 측정치 시퀀스(200Hz, KF 간 0.5s)를 `IMU::Preintegrated::IntegrateNewMeasurement`(양 바이너리 공유 float 경로, `ImuTypes.cpp`)에 넣어 ΔR/ΔV/ΔP를 얻고, 그로부터 `p_{k+1} = p_k + v_k·dt + ½g·dt² + R_k·ΔP` 식으로 **정합적인 KF 상태를 역산** → 적분 차수 오차와 무관하게 GT에서 잔차가 정확히 0. 그 후 KF 저장 상태를 `1/s_true` 스케일 + `Rwg_perturb`(중력 방향 ~10도 회전)로 왜곡해서 넣으면 최적해가 해석적으로 `(s_true, Rwg_true)`.

```
Map* m = new Map(0);
KF 4~6개: kf->mImuCalib = Calib(Tbc, 1.7e-4, 2e-3, 1.9e-5, 3e-3);  // ← SetPose보다 먼저!
          kf->mnId = k (0..); kf->SetPose(왜곡 Tcw); kf->SetVelocity(왜곡 v);
          kf->SetNewBias(IMU::Bias());   // GT 바이어스 0
          kf->mPrevKF = 직전 KF; kf->mpImuPreintegrated = 위에서 만든 Preintegrated;
          kf->mpCamera = 공용 Pinhole; m->AddKeyFrame(kf);
```
현실적 노이즈 파라미터(EuRoC 급)를 써야 `C` 블록이 양호조건 → `EdgeInertialGS` 정보행렬 역산(`G2oTypes.cpp:604`)이 안정적.

동일 픽스처가 `InertialOptimization(priorG,priorA)` 두 변형에도 재사용 (bg/ba GT=0, 왜곡 초기값으로 소량 바이어스 주입한 변형 추가).

---

## 2. 비교 메트릭과 톨러런스 (2계층)

**계층 0 — "그래프 구축 등가" (타이트, 상대 1e-12, 사실상 비트동일 기대)**
`optimizer.initializeOptimization(); computeActiveErrors();` 직후, **edge id 정렬 순서로 엣지별 오차 벡터·chi2·정보행렬을 직렬화**. 합계가 아닌 엣지별 값이므로 부동소수 재결합(summation order) 문제 없음. 이 계층은 솔버를 전혀 통과하지 않으므로 우리 커스텀 엣지/정점 코드(computeError, 정보행렬, 로버스트 커널 세팅)의 이식 등가성만 격리 검증 — 여기가 깨지면 이식 버그 확정.

**계층 1 — "수렴 해" (완화)**
구/신 g2o는 LM 감쇠 전략·내부 순서가 달라 반복 경로의 비트동일은 **원리적으로 보장 불가**. 대신 (i) 픽스처가 유일 최소점을 갖고 (ii) 고정 반복수 내에 stationarity ≪ 게이트에 도달하도록 설계하면, 경로 차이는 수렴점에서 소멸. 제안 게이트:

| 출력 | 게이트(구↔신 델타) | GT 대비 | 근거 |
|---|---|---|---|
| Frame 포즈 (float 저장 경로) | 병진 abs 1e-6 m, 회전 1e-6 rad | 1e-5 | 출력이 float SE3로 절단됨(`Optimizer.cpp:1110`) — 1e-9 요구는 무의미 |
| `scale` (double&) | 상대 1e-9 | 상대 1e-7 | 순수 double 반환 + 잔차 0 GT + GN 10회 → 기계정밀 수렴 |
| `Rwg` (Matrix3d&) | geodesic 1e-9 rad | 1e-7 rad | 동상 |
| bg/ba (double) | abs 1e-9 | 1e-7 | 동상 |
| 인라이어 수(반환 int), `mvbOutlier` 플래그 | **완전 일치** | — | 마진 설계로 보장되는 이산 계약 |
| `mpcpi` 15×15 Hessian (클래스 B) | 원소별 상대 1e-6 | — | GetHessian 계약 (PROJECT_PLAN 리스크 4) |

즉 "1e-9 일괄"도 "막연히 완화"도 아니고, **출력 경로의 정밀도(float/double)와 계층(솔버 통과 여부)별로 차등**. 첫 실행에서 실측 델타를 리포트에 기록하고 게이트를 경험적으로 조이는 운영(마진 3자리 이상 확보 시 축소).

**자기결정성 게이트**: 각 바이너리를 동일 픽스처로 2회 실행 → 출력 SHA256 완전 일치 요구 (bit_identity 패턴 승계). g2o가 internalId(삽입순) 정렬이므로 통과해야 정상; 실패 시 픽스처 비결정성 버그.

**입력 해시 게이트**: 두 바이너리가 직렬화한 "픽스처 입력 덤프" 해시가 다르면 비교 자체를 중단(픽스처가 공유 코드이므로 반드시 일치해야 함).

---

## 3. 하네스 아키텍처 — 2-바이너리 + 동결 레퍼런스 + 직렬화 비교

**ODR 결론**: 동일 심볼(같은 `g2o::` 네임스페이스, 헤더 인라인/템플릿 포함)이라 한 바이너리 공존은 불가. objcopy 심볼 리네임·dlopen(RTLD_LOCAL)은 헤더 템플릿이 소비자 TU에 인스턴스화되는 구조상 미봉책 — 기각. **같은 하네스 소스를 두 번 configure**하는 2-바이너리가 정공법.

또한 P6 이전이 `src/backend/*`를 제자리 수정하는 방식이므로, 구현 기준선을 **이전 시작 전에 스냅샷으로 동결**해야 함(그래야 이전이 진행돼도 레퍼런스 바이너리가 계속 빌드됨).

### 생성할 파일 (모두 절대경로)

```
/Users/jhpark/VSLAM/ORB_SLAM3_Modern/tests/backend_equiv/
├── CMakeLists.txt                  # 독립 project. -DEQUIV_BACKEND=vendored|modern
│                                   #  vendored: add_subdirectory(Thirdparty/g2o) + reference_backend/ 소스
│                                   #  modern:   add_subdirectory(third_party/g2o) + 라이브 src/backend/ 소스
│                                   #  공통: src/map/* src/camera/* src/features/* src/recognition/KeyFrameDatabase.cpp
│                                   #        src/io/Converter.cpp src/geometry/* — 변형별 재컴파일
│                                   #        (Frame.cpp가 G2oTypes.hpp를 include하므로 공유 .a 불가)
├── reference_backend/              # ★ 이전 시작 직전 스냅샷(불변, golden):
│   ├── Optimizer.cpp  G2oTypes.cpp  OptimizableTypes.cpp     # src/backend/에서 복사
│   └── include/backend/{Optimizer,G2oTypes,OptimizableTypes}.hpp  # include 우선순위 최상단
│       # ImuTypes.cpp/.hpp는 스냅샷하지 않음 — g2o 무관 공유 float 경로, 양쪽 모두 라이브 사용
├── common/
│   ├── EquivFixtures.hpp / .cpp    # FrameFixture, ImuKfChainFixture, MapFixture, InertialBAFixture
│   │                               #  (map/camera/imu 타입만 사용, g2o API 직접 호출 금지)
│   ├── EquivSerialize.hpp / .cpp   # 정본 직렬화: "%.17g", 쿼터니언 부호 정규화(w>0),
│   │                               #  id 오름차순 고정, 섹션 = INPUT_HASH / STAGE0_EDGES / RESULT
│   └── EquivMain.cpp               # CLI: equiv_runner <function> <fixture> [--stage0] → stdout 레코드
├── compare.py                      # 필드별 톨러런스 테이블 diff (위 표를 코드화), exit code로 게이트
├── run_equiv.sh                    # build_equiv_vendored/ + build_equiv_modern/ 두 빌드트리 configure+build,
│                                   #  함수×픽스처 매트릭스 실행(각 2회 자기결정성), compare.py 일괄, 요약 리포트
└── README.md                       # 계약 문서: 직렬화 스키마, 톨러런스 표, 픽스처 마진 규칙
```

계층 0 훅: 레퍼런스/신규 Optimizer를 수정하지 않고 얻기 위해, `EquivMain`이 함수 본체 호출 전에 **동일 픽스처로 그래프를 직접 구축하는 경량 복제 경로**를 두는 대신 — 그건 이중 유지보수라 기각 — `computeError()`를 공개 API로 가진 엣지를 픽스처+`G2oTypes`로 직접 인스턴스화해 stage0을 뽑는다(엣지 단위 등가는 Optimizer 본체와 독립). 함수 전체는 계층 1로 검증.

빌드 플래그 주의(기존 P1 게이트 교훈 승계): 두 변형 모두 `-O3`, `-march=native` 금지, 신규 g2o 서브모듈의 native/fast-math 옵션 OFF를 CMakeLists에 명시 고정하고 사용 플래그를 리포트에 기록.

---

## 4. 실행 순서 (리스크 = 회귀 가치 × 픽스처 비용 순)

1. **`PoseOptimization`** — 하네스 부트스트랩용. 픽스처 최소·해석적 GT 자명. 2-바이너리 패턴/직렬화/compare.py 전체 배선을 여기서 완성.
2. **`InertialOptimization(Map*,Rwg,scale)`** — Remastered 단안-관성 회귀의 직접 용의 경로(스케일/중력 정합). 클래스 C 픽스처 도입.
3. **`InertialOptimization` 나머지 2변형** — 같은 픽스처 재사용, priorG/priorA·바이어스 정점 활성화만 추가.
4. **`PoseInertialOptimizationLastKeyFrame` / `LastFrame`** — GetHessian(`mpcpi`) 계약 검증 포함.
5. **`FullInertialBA`, `LocalInertialBA`, `MergeInertialBA`** — 관성 모드 잔여(클래스 E).
6. **`BundleAdjustment`/`GlobalBundleAdjustemnt`/`LocalBundleAdjustment` ×2** — 시각 전용(회귀 리스크 낮음, MapFixture 배선 비용 중간).
7. **`OptimizeSim3` → `OptimizeEssentialGraph` ×2 → `OptimizeEssentialGraph4DoF`** — 최중량 픽스처, 루프 시나리오는 P9 게이트(KITTI 00)로도 이중 커버되므로 마지막.

각 함수의 "이전"은 해당 등가성 pass가 초록이 된 뒤에만 다음 함수로 진행 (per-function migrate-then-gate).

---

## 5. 남는 리스크 / 미결

- `Frame.cpp:21`의 `G2oTypes.hpp` include 때문에 코어 소스 전체가 변형별 재컴파일 대상 — 빌드 2회 비용은 수용, 다만 P6 중 `Frame`↔백엔드 결합 축소(mpcpi 전방선언화) 시 하네스도 단순해짐.
- 동결 스냅샷은 P6 기간 중 `src/map` 등 공유 헤더가 크게 바뀌면 컴파일이 깨질 수 있음 — 하네스는 P6 전용 도구로 간주하고, P6 종료 시 최종 등가성 리포트를 `docs/phase_reports/P6.md`에 박제 후 스냅샷 폐기 여부 결정.
- 신규 g2o의 `OptimizationAlgorithmLevenberg` 초기 λ 전략이 다르면 저조건 픽스처에서 계층 1 게이트가 출렁일 수 있음 — 픽스처는 조건수 양호(관측 시차·가속 여기 충분)하게만 설계하고, 실측 델타 기록으로 게이트를 경험 보정.

## C. ISP 인터페이스 설계

모든 입력 사실을 현재 소스에서 재검증했고, 이를 바탕으로 P6 설계를 확정한다.

# P6: ISP 분할 옵티마이저 인터페이스 설계

## 0. 콜사이트 검증 결과 (2026-08-07 grep 기준)

**공개 표면은 17개 선언 중 15개** — `BundleAdjustment`(Optimizer.cpp:56에서 `GlobalBundleAdjustemnt`가 내부 호출)와 `Marginalize`(Optimizer.cpp:5287, `InertialOptimization` 내부 호출)는 외부 콜사이트가 없어 인터페이스에서 제외하고 구현부 내부 헬퍼로 강등한다.

| 모듈 | 콜사이트 (총 27곳) | P5 파라미터 확인 |
|---|---|---|
| **Tracking.cpp** (10곳) | `PoseOptimization` ×7 (1900, 2064, 2126, 2132, 2869, 2885, 2900) · `PoseInertialOptimizationLastFrame` (2140) · `…LastKeyFrame` (2145) · `GlobalBundleAdjustemnt` (1735, 초기화) | GBA 호출은 기본인자 사용 (`GBAResult*=NULL`) |
| **LocalMapping.cpp** (6곳) | `LocalInertialBA` (150) · `LocalBundleAdjustment` KF변형 (155) · `InertialOptimization` full변형 (1272) · `FullInertialBA` ×2 (1317/1319, bInit t/f) · `InertialOptimization` scale변형 (1479) | 150/155/1317/1319 모두 `*mpBAEpochs` 전달, 1317/1319는 `&gbaResult` |
| **LoopClosing.cpp** (11곳) | `OptimizeSim3` ×2 (559, 770) · `OptimizeEssentialGraph4DoF` (1185) · `OptimizeEssentialGraph` loop변형 (1190, `mCorrectedRefs`) · `MergeInertialBA` ×2 (1633, 2061) · 용접 `LocalBundleAdjustment` (1637) · `OptimizeEssentialGraph` merge변형 (1727, `MergeScratch& scratch`) · `InertialOptimization` bias변형 (1873) · `GlobalBundleAdjustemnt` (2298) · `FullInertialBA` (2300) | correctedRefs/MergeScratch/GBAResult/BAEpochs 모두 P5 시그니처대로 확인 |

**주입 선례 확인**: `System.hpp:202`에 `BAEpochs mBAEpochs;` 값 소유, `System.cpp:182–183/197`에서 `&mBAEpochs`를 LocalMapping/LoopClosing 생성자로 주입. `LocalMapping.hpp:40`·`LoopClosing.hpp:43`의 `struct BAEpochs;` 전방선언 패턴도 확인.

**CMake 현황**: `CMakeLists.txt:8–9` 주석이 이미 "backend swap = P6"를 예고. 벤더드 `Thirdparty/g2o`가 활성, 업스트림 `third_party/g2o` 서브모듈 대기 중.

**모듈 간 중복 함수** (인터페이스 설계의 핵심 제약): `GlobalBundleAdjustemnt`(Tracking+LoopClosing), `FullInertialBA`(LocalMapping+LoopClosing), `InertialOptimization`(LocalMapping full/scale + LoopClosing bias), `LocalBundleAdjustment`(모듈별 **다른 오버로드**이므로 실질 중복 아님).

---

## 1. 인터페이스 분할: 소비자 역할 기준 3분할 (권고)

기능 분류(BA/PoseGraph/Inertial)가 아니라 **클라이언트 필요 기준**으로 자른다 — 그것이 ISP의 정의이고, 위 콜사이트 표가 곧 인터페이스 명세다.

```cpp
// include/backend/ITrackingOptimizer.hpp  — Frame, Map 전방선언만 필요. g2o 헤더 0개.
class ITrackingOptimizer {
public:
    virtual ~ITrackingOptimizer() = default;
    virtual int  PoseOptimization(Frame* pFrame) const = 0;
    virtual int  PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false) const = 0;
    virtual int  PoseInertialOptimizationLastFrame(Frame* pFrame, bool bRecInit = false) const = 0;
    virtual void GlobalBundleAdjustment(Map* pMap, int nIterations = 5, bool* pbStopFlag = nullptr,
                                        unsigned long nLoopKF = 0, bool bRobust = true,
                                        GBAResult* pResult = nullptr) const = 0;   // 오탈자 교정
};

// include/backend/IMappingOptimizer.hpp — KeyFrame/Map/BAEpochs/GBAResult 전방선언 + Eigen만.
class IMappingOptimizer {
public:
    virtual ~IMappingOptimizer() = default;
    virtual void LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                       int& nFixedKF, int& nOptKF, int& nMPs, int& nEdges,
                                       BAEpochs& epochs) const = 0;
    virtual void LocalInertialBA(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                 int& nFixedKF, int& nOptKF, int& nMPs, int& nEdges,
                                 bool bLarge, bool bRecInit, BAEpochs& epochs) const = 0;
    virtual void InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale,
                                      Eigen::Vector3d& bg, Eigen::Vector3d& ba, bool bMono,
                                      Eigen::MatrixXd& covInertial, bool bFixedVel = false,
                                      bool bGauss = false, float priorG = 1e2, float priorA = 1e6) const = 0;
    virtual void InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale) const = 0;
    virtual void FullInertialBA(Map* pMap, int its, bool bFixLocal, unsigned long nLoopKF,
                                bool* pbStopFlag, bool bInit, float priorG, float priorA,
                                Eigen::VectorXd* vSingVal, bool* bHess,
                                GBAResult* pResult, BAEpochs& epochs) const = 0;
};

// include/backend/ILoopOptimizer.hpp — 유일하게 LoopClosing.hpp(KeyFrameAndPose)와 g2o::Sim3 의존.
class ILoopOptimizer {
public:
    virtual ~ILoopOptimizer() = default;
    virtual int  OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches1,
                              g2o::Sim3& g2oS12, float th2, bool bFixScale,
                              Eigen::Matrix<double,7,7>& mAcumHessian, bool bAllPoints = false) const = 0;
    virtual void OptimizeEssentialGraph(/* loop 변형, correctedRefs 포함 */) const = 0;
    virtual void OptimizeEssentialGraph(/* merge 변형, MergeScratch& */) const = 0;
    virtual void OptimizeEssentialGraph4DoF(/* … */) const = 0;
    virtual void MergeInertialBA(KeyFrame*, KeyFrame*, bool*, Map*,
                                 LoopClosing::KeyFrameAndPose&, BAEpochs&) const = 0;
    virtual void LocalBundleAdjustment(KeyFrame* pMainKF, std::vector<KeyFrame*> vpAdjustKF,
                                       std::vector<KeyFrame*> vpFixedKF, bool* pbStopFlag,
                                       const BAEpochs& epochs) const = 0;   // 용접 변형
    virtual void InertialOptimization(Map* pMap, Eigen::Vector3d& bg, Eigen::Vector3d& ba,
                                      float priorG = 1e2, float priorA = 1e6) const = 0;  // bias 변형
    virtual void GlobalBundleAdjustment(Map*, int, bool*, unsigned long, bool, GBAResult*) const = 0;
    virtual void FullInertialBA(/* IMappingOptimizer와 동일 시그니처 */) const = 0;
};
```

**설계 규칙 5가지**:
1. **헤더도 3분할** (단일 `OptimizerInterfaces.hpp` 아님). 이유: `Optimizer.hpp`는 현재 g2o 헤더 9개 + `closing/LoopClosing.hpp`를 Tracking/LocalMapping TU에 누출시킨다(23–42행). 분할하면 Tracking·LocalMapping TU에서 g2o와 LoopClosing 순환 의존이 완전히 제거되고, `ILoopOptimizer.hpp`만 그 비용을 진다.
2. **중복 함수는 양쪽 인터페이스에 중복 선언** (`GlobalBundleAdjustment`, `FullInertialBA`). C++에서 다중 베이스의 동일 시그니처 순수가상은 파생 클래스의 오버라이드 하나로 동시에 충족되므로 구현 비용 0. 공통 베이스 인터페이스 추출은 상속 다이아몬드만 만들 뿐 클라이언트에 이득이 없어 기각.
3. **전 메서드 `const`**. P5가 은닉 상태를 BAEpochs/MergeScratch/GBAResult로 완전히 추출했으므로 백엔드는 무상태가 자연스럽고, `const`가 이를 컴파일 타임에 강제한다 → 3스레드(+GBA 분리 스레드) 공유 안전성이 타입으로 보장됨.
4. **기본인자는 인터페이스 선언에만** 두고 오버라이드에서는 절대 재선언 금지(가상함수 기본인자 정적 바인딩 함정 차단). 콜사이트가 전부 인터페이스 포인터 경유이므로 안전.
5. **Phase A에서는 시그니처를 기계적으로 보존**하되 `GlobalBundleAdjustemnt` 오탈자만 교정한다(콜사이트를 어차피 전부 만지므로 무료). `InertialOptimization` 3오버로드의 의미명 개명(Full/Scale/Bias)은 P7 정리 항목으로 이연.

**대안(기각)**: 기능별 세분화(IPoseOptimizer/ILocalBA/IInertialInit/IPoseGraph/IGBA 5+개)는 System 배선과 생성자 파라미터만 늘리고, 실제 소비자가 3모듈뿐인 현실에서 얻는 게 없다.

---

## 2. 인스턴스 전달: System 소유 + 생성자 주입 (BAEpochs 선례 그대로)

```cpp
// include/core/System.hpp — mBAEpochs(202행) 바로 옆
G2oBackend mBackend;   // ITrackingOptimizer + IMappingOptimizer + ILoopOptimizer 전부 구현

// src/core/System.cpp:178/182/197 — 기존 &mBAEpochs 주입과 동일 역학
mpTracker     = new Tracking(..., static_cast<ITrackingOptimizer*>(&mBackend), ...);
mpLocalMapper = new LocalMapping(..., &mBAEpochs, static_cast<IMappingOptimizer*>(&mBackend), ...);
mpLoopCloser  = new LoopClosing(..., &mBAEpochs, static_cast<ILoopOptimizer*>(&mBackend));
```

- `G2oBackend : public ITrackingOptimizer, public IMappingOptimizer, public ILoopOptimizer` — 단일 구체 클래스, 멤버 변수 0개(무상태), 각 모듈은 **자기 좁은 인터페이스 포인터만** 저장 (`ITrackingOptimizer* mpOptimizer;`).
- 모듈 헤더에는 전방선언만: `class ITrackingOptimizer;` (LocalMapping.hpp:40 `struct BAEpochs;` 선례).
- Tracking 생성자에 파라미터 1개 추가가 유일한 신규 배선(현재 BAEpochs를 안 받는 유일한 모듈).
- LoopClosing:2298의 GBA는 분리 스레드에서 같은 `ILoopOptimizer*`로 호출 — 무상태 `const` 계약이므로 추가 동기화 불요(현행 static 호출과 동일한 스레드 의미론).

---

## 3. 마이그레이션 역학

### Phase A — 인터페이스 도입, 행동 변화 0 (smoke-gated)

1. 3개 인터페이스 헤더 + `G2oBackend.hpp/cpp` 추가. Phase A의 `G2oBackend.cpp`는 **전 메서드가 기존 `Optimizer::` static으로 1줄 위임**.
2. 27개 콜사이트를 `Optimizer::X(...)` → `mpOptimizer->X(...)`로 기계 치환 (Tracking 10, LocalMapping 6, LoopClosing 11) + 생성자 3개 + System 배선. LoopClosing.cpp:2315의 주석 콜사이트는 삭제.
3. `Optimizer.hpp` 포함은 `G2oBackend.cpp`와 `Optimizer.cpp`만 남긴다 — g2o 헤더가 트래킹/매핑 TU에서 사라지는 것 자체가 Phase A의 검증 가능한 부산물.
4. **게이트**: 기존 빌드로그 관행(p5f_build.log 계보)대로 클린 빌드 + EuRoC 검증 모드 스모크, ATE가 P5 기준선과 동일(위임뿐이므로 결정론 모드에서 동일 궤적 기대).

### Phase B — 업스트림 g2o 구현, **빌드 타임 스위치** (권고)

**런타임 A/B 기각 근거 3가지**:
1. **ODR/심볼 충돌이 원천 봉쇄**: 벤더드와 업스트림 g2o는 동일한 `g2o` 네임스페이스·동일 심볼(`g2o::Sim3`, block_solver 등)을 차지한다. 한 바이너리에 둘을 링크하는 것은 미정의 동작이고, 회피하려면 벤더드 g2o 전체 리네임이라는 대형 오염 diff가 필요하다. 인터페이스의 `g2o::Sim3&` 파라미터도 빌드 타임 선택이면 "선택된 그 g2o"로 자연 해석된다.
2. **Remastered 교훈 5 (상태 격리)**: 단안-관성 회귀 추적에서 libg2o 빌드 플래그가 용의선상에 올랐던 경험대로, g2o는 전역/정적 상태(솔버 팩토리 등)를 가지며 맵 객체의 epoch 마크(`mnBALocalForKF` 계열)는 공유 가변 상태다. 한 프로세스에서 두 백엔드를 교차 실행하면 오염 경로가 곱으로 늘어 회귀 원인 분리가 불가능해진다. 빌드 타임 분리는 각 바이너리의 수치 특성(컴파일 플래그 포함)을 일관되게 유지한다.
3. **비교 인프라가 이미 빌드 단위**: `results/`·`evaluation/`·`benchmark/` 하네스는 두 빌드 산출물의 궤적 비교에 그대로 쓸 수 있다. 런타임 A/B가 주는 유일한 이득(단일 바이너리 비교)은 이 하네스가 이미 대체한다.

**CMake 스위치**:
```cmake
set(ORB_BACKEND "vendored" CACHE STRING "g2o backend: vendored | upstream")
if(ORB_BACKEND STREQUAL "vendored")
    add_subdirectory(Thirdparty/g2o)
    file(GLOB BACKEND_IMPL src/backend/vendored/*.cpp)
else()
    add_subdirectory(third_party/g2o)
    file(GLOB BACKEND_IMPL src/backend/upstream/*.cpp)
endif()
```
- 클래스명은 양쪽 다 `G2oBackend` 유지(디렉터리로 선택) → System.cpp 무변경.
- **함수 단위 진행**: `src/backend/upstream/`을 vendored 복제로 시작해, 함수별로 업스트림 g2o API(unique_ptr 솔버 생성자 등)로 포팅하고 API 차이는 `upstream/g2o_compat.hpp` 한 곳에 흡수한다. 마일스톤 게이트 = `ORB_BACKEND=upstream` 빌드의 EuRoC ATE가 vendored 기준선 허용오차 내. 함수별 런타임 폴백은 1번 ODR 제약상 불가능하며 필요하지도 않다.
- `ImuTypes.cpp`(Eigen 전용, g2o 무관)는 공통 유지, `G2oTypes.cpp`/`OptimizableTypes.cpp`(커스텀 정점·에지)는 백엔드별 디렉터리로 이동.

---

## 4. 파일 레이아웃

```
include/backend/
  ITrackingOptimizer.hpp     # 신규 (Phase A)
  IMappingOptimizer.hpp      # 신규 (Phase A)
  ILoopOptimizer.hpp         # 신규 (Phase A)
  G2oBackend.hpp             # 신규 (Phase A)
  BAEpochs.hpp GBAResult.hpp ImuTypes.hpp          # 공통, 유지
  Optimizer.hpp G2oTypes.hpp OptimizableTypes.hpp  # Phase A: 유지(내부화) → Phase B: 백엔드별 이동

src/backend/
  ImuTypes.cpp               # 공통, 유지
  G2oBackend.cpp             # Phase A: Optimizer:: 위임 → Phase B: 백엔드별 디렉터리로 이동
  Optimizer.cpp G2oTypes.cpp OptimizableTypes.cpp  # Phase A: 유지
  vendored/                  # Phase B: 위 3+G2oBackend.cpp 이동 (BundleAdjustment/Marginalize는 여기 내부 static화)
  upstream/                  # Phase B: 복제 후 함수별 포팅 + g2o_compat.hpp
```

---

## 5. 리스크 및 후속 항목

- **`g2o::Sim3`가 `ILoopOptimizer` 시그니처에 잔존** (`OptimizeSim3`, `KeyFrameAndPose`). 빌드 타임 스위치 하에서는 무해하나(단일 g2o만 존재), 인터페이스의 백엔드 중립성을 완성하려면 P7에서 `Sophus::Sim3d` 경계 타입으로 교체 권고(Sophus는 이미 벤더드·서브모듈 양쪽 존재).
- Phase A diff 규모: 신규 헤더 4 + cpp 1, 수정 파일 7(콜사이트 3, 모듈 헤더 3, System 2), 콜사이트 27곳 — 1커밋으로 원자적 처리 가능.
- `GlobalBundleAdjustemnt`→`GlobalBundleAdjustment` 교정은 인터페이스 신설 시점이 유일한 무료 기회.
- Phase B 완료 후 P7에서 `Thirdparty/g2o` + `src/backend/vendored/` 삭제로 종결.
