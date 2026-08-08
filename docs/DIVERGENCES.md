# 업스트림 대비 의도적 편차·계승 결함 대장 (중간 점검, 2026-08-07)

체크포인트 적대적 리뷰(15건 검증: CONFIRMED 11 / REFUTED 4)로 확정된 항목.
"게이트가 볼 수 없는 차이"는 여기 기록하는 것이 게이트 통과와 동급의 의무다.

## 의도적 편차 (동작 확장/개선 — 게이트 yaml 무영향, 사용자 yaml에는 관측 가능)

1. **IMU.fastInit이 V1.0 yaml에서 동작함** (Settings.cpp / Tracking.cpp)
   업스트림 V1.0 경로는 이 키를 조용히 무시했다(레거시 파서 전용). 현재는
   흡수되어 스테레오/RGBD-관성 초기화의 가속도 게이트를 실제로 끈다.
   레거시 문서화 기능의 복원으로 유지. (단안-관성에는 해당 게이트 자체가 없음)
2. **Camera.imageScale ≠ 1 → 명시 오류 종료** — 업스트림은 조용히 무시(죽은 키).
   silent-ignore보다 진단 가능성이 높아 유지.
3. **mnFramesToResetIMU = 0 고정** — 업스트림은 신형 경로에서 미초기화(UB),
   레거시 경로에선 mMaxFrames. 골든 기준선의 실측 동작(=0)을 정본화했다.
   레거시 의미(fps 크기의 재추적 IMU 리셋 창)를 원하면 FixLevel로 제공할 것.
4. **KFDB 이웃 누적에서 임계 미달 후보의 점수 기여 = 0** — 업스트림은 스탬프만
   통과하면 **이전 질의의 stale 점수**를 읽을 수 있었다(질의 이력 의존 =
   재현 불가능한 동작). 결정적 0 기여로 정본화.

## 계승 결함 (bug-for-bug 보존, FixLevel 후보)

5. **Frame::mb 미초기화 읽기** — 스테레오 생성자가 `mb = mbf/fx` 대입(167행)
   이전인 140행 ComputeStereoMatches에서 mb를 읽음(minZ). 업스트림 동일.
6. **Rectified calibration2_가 리사이즈 전 내부파라미터로 합성됨** —
   readCamera2(리사이즈 전) 시점에 calibration1_을 복제하므로
   Camera.newWidth/newHeight를 쓰는 Rectified 구성에서 calibration1_(후처리)과
   불일치. 게이트 yaml(KITTI)은 리사이즈 미사용이라 무영향. P1 세그폴트 픽스의
   잔여 이슈로, 골든 빌드도 동일 동작.
7. LocalMapping의 댕글링 위험 delete 2곳, Atlas 무락 메서드군, mGlobalMutex
   광역 잠금 — docs/OWNERSHIP.md 참조.

## 게이트 방법론 이력

- 체크포인트 리뷰가 **풀 게이트 v2.1의 검정력 결함**을 적발: 2라운드에서
  최소 p=1/C(4,2)=0.167이라 형식 FAIL 불가능. phase-3/4의 PASS는 분포·중앙값
  근거로는 타당하나 형식 검정으로는 UNDERPOWERED였다. v2.2부터 기본 4라운드
  (최소 p=1/70), 미달 시 verdict가 UNDERPOWERED로 표기됨.
- 스모크 상한을 N=1 시드(0.084)에서 N=5 중앙값 기준(0.0758)으로 교정,
  신선도 가드 추가. 비트 게이트는 비결정성 감지 exit code를 전파.
- 골든 해시의 환경 전제(dev 이미지 digest, OpenCV 버전)를
  benchmark/golden/bit_hashes_env.txt에 고정.

8. **GBA 전파의 stamped-but-bad refKF 엣지** (P5-4 그룹 D) — 업스트림은 GBA
   스탬프는 찍혔으나 pop되지 않은(bad) 참조 KF의 stale/미기록 mTcwBefGBA로
   맵포인트를 변환할 수 있었다(가비지 읽기). 사이드테이블화 이후 해당 엣지는
   결정적으로 스킵된다(비퇴화 실행에서는 stamped ⟺ popped라 동작 동일).

9. **[발견/보존] EdgeInertialGS 스케일 갱신 발산 결함** (P6-3 실증) — 스케일
   야코비안은 가산(∂r/∂s)인데 VertexScale::oplusImpl은 승산(s·eʷ). GN 고정점
   s←s·exp(s*−s)의 수렴 배율이 −(s*−1)이라 **s*>2에서 발산, s*=2가 한계진동**
   (2.4% 오차 정지 실측), s*≈1 근방만 급수렴. 스케일이 2배 이상 틀어진
   단안-관성 초기화가 InertialOptimization으로 영영 복구되지 않는 메커니즘.
   bug-for-bug 원칙에 따라 양 백엔드에 동일 보존(등가성이 곧 증거), 수정은
   FixLevel 후보 1순위. 재현: EQUIV_INERTIAL_DEBUG=1 + s_true=2.0 픽스처.

## P6 백엔드 이전에서 드러난 g2o 포크 차이 (2026-08-08 감사)

벤더드 `Thirdparty/g2o`는 순정 g2o가 아니라 **Raúl Mur-Artal의 ORB-SLAM 포크**다
(`optimization_algorithm_levenberg.cpp:27` "Modified Raul Mur Artal (2014)").
업스트림 20241228_git로 이전하면서 아래 세 가지가 조용히 바뀌었고, 등가성
테스트가 커버한 5개 함수는 전부 이 차이를 타지 않아(GN 사용 또는 조기 수렴)
감지되지 않았다. 나머지 12개 LM 함수가 영향권이다.

10. **[복원] LM 조기종료 기준 소실** (F1, 13/16 사이트) — 포크는 상대 χ² 개선이
    0.1% 미만인 반복이 3회 연속되면 `Terminate`한다(`_nBad>=3`). 업스트림에는
    `_nBad`가 존재하지 않아, 이전 직후에는 모든 LM이 요청 반복수를 **전부**
    소진했다. 결과는 (a) 추가 스텝만큼 추정치 이동 (b) LocalBA 벽시계 시간 증가
    → LocalMapping 스레드의 KF 삽입률 변화라는 트래킹 수준 동작 변화.
    ORB-SLAM 고유의 의도적 설계이므로 `ORB_SLAM3::OrbLevenberg` 서브클래스로
    **복원 완료**(14/14 사이트, 서브모듈 무수정). 래퍼 구현이 포크의 in-place
    구현과 의미 동일함을 증명해 두었다(`src/backend/OrbLevenberg.cpp` 주석):
    push/pop이 추정치를 정확히 복원하고 chi2는 추정치의 순수 함수이므로
    solve() 이후 재계산한 chi2가 포크가 추적하던 값과 비트 동일하다.
    리셋 지점은 생성자와 `iteration==0`(init()이 아님 — 원본 재확인).
11. **[복원] LinearSolverEigen blockOrdering 기본값 반전** (F2, 12/16 사이트) —
    포크 `false`(스칼라 AMD) → 업스트림 `true`(블록 AMD, LinearSolverCCS 기저).
    코드가 `setBlockOrdering`을 부른 적이 없어 소거 순서가 통째로 바뀌었다.
    수학적으로는 양쪽 다 유효하나 소거 순서는 부동소수 합산 순서를 바꾼다.
    각 생성 지점에서 `setBlockOrdering(false)` 명시로 **복원 완료**(12/12,
    Dense 솔버 4곳은 소거 순서 개념이 없어 해당 없음).
    **직접 증거**: GaussNewton을 쓰는 `inertial_optimization_bias` 등가쌍의
    vendored↔upstream 편차가 복원 전 |dbg|=1.22e-11에서 복원 후 4.34e-19로
    7자릿수 개선됐다. GN에는 #10이 적용되지 않으므로 이 개선분은 오로지
    소거 순서 복원 효과 — 벤더드 순서가 실제로 회복됐다는 실증이다.
12. **[보존/개선] Sim3::exp 소각(小角) 분기의 수학 오류를 업스트림이 수정** (D1) —
    포크(`Thirdparty/g2o/g2o/types/sim3.h:199 외`)는 (i) `θ<eps` 분기에서
    `R = I + Ω + Ω²`로 2차항 계수 ½가 누락됐고 (ii) `B` 분자의 `-1`이 빠져
    σ가 작을 때 B가 `~1/σ³`로 발산한다(정상값 1/6). 업스트림은 둘 다 수정했다.
    **복원하지 않는다** — 수학 버그 복원은 bug-for-bug 정책의 대상이 아니다
    (정책은 "동작 보존"이지 "오류 재현"이 아니며, 이 분기는 Sim3 최적화 수렴
    근방에서 실제로 밟힌다). 루프 클로징/본질 그래프 결과가 골든과 미세하게
    달라질 수 있으므로 풀 게이트(KITTI 00 루프 포함)로 영향을 계측한다.

### 방법론 함의
등가성 테스트는 **테스트한 함수에 대해서만** 등가를 증명한다. 5/17 커버리지에서
"바이트 동일"이 나왔다고 이전 전체가 안전하다고 말할 수 없다는 것을 이 세 건이
보여준다 — 커버되지 않은 함수는 **차이 그 자체를 소스 대조로** 찾아야 한다.

### 추가 발견 (같은 감사, 등가성 미커버 함수 대상)

13. **[주의/계측] SimplicialLDLT → SimplicialLLT** (F3, Eigen 솔버 12사이트,
    MEDIUM-HIGH) — 포크의 `LinearSolverEigen`은 LDLT(준정정 허용), 업스트림은
    LLT(엄격 양정정 요구). 두 `OptimizeEssentialGraph`는 `setUserLambdaInit(1e-16)`
    으로 사실상 감쇠 없이 `BlockSolver_7_3` 포즈그래프를 푼다. 게이지가 덜
    구속된 초기 루프에서 **LDLT는 풀리고 LLT는 실패** → `solve()` false →
    스텝 기각·λ 팽창 → 루프 보정이 크게 축소되거나 `optimize(20)`이 사실상
    아무것도 안 하고 끝난다. 크래시도 로그도 없다(항목 15 참조).
    이것만은 "업스트림이 더 옳다"가 아니라 **강건성 후퇴**다.
14. **[보존] Sim3::log·SE3Quat::exp의 소각 분기도 업스트림이 수정** (D2, D3) —
    D2: `Sim3::log`의 `B`에도 동일한 `-1` 누락(EdgeSim3의 오차함수 자체;
    단안 본질그래프 수렴 상태에서 밟힘). D3: `SE3Quat::exp`의 포크판은
    `V = R = I + Ω + Ω²`이고 소스에 `//TODO: CHECK WHETHER THIS IS CORRECT!!!`
    주석까지 달려 있다. 업스트림은 `V = I + Ω/2 + Ω²/6`(정답) — 1차항 계수가
    100% 다르다. `VertexSE3Expmap::oplusImpl` 경유라 **모든 BA가 수렴 근방에서
    영향**을 받는다. D1과 같은 이유로 복원하지 않되, 이것이 골든 대비 궤적 차이의
    주된 원천일 수 있으므로 풀 게이트로 계측한다.
    (정량: D1 단안 분기 실측 예시에서 적용 스텝이 900배까지 차이.)
15. **[조치 필요] 진단 침묵** (C1) — `G2O_USE_LOGGING=OFF`로 g2o의
    `G2O_ERROR` 34개·`G2O_WARN` 24개가 전부 no-op이 됐다(NaN 검출, CCS 구축
    실패, 알고리즘 init 실패 포함). 그런데 우리 코드의 **~24개 `optimize()`
    호출이 전부 반환값을 버린다**. 즉 최적화가 실패해도(-1 반환) 함수는 변경되지
    않은 추정치를 그대로 맵에 써 넣고 조용히 성공한 척한다. 항목 13과 결합하면
    "루프 클로징이 조용히 무력화"가 관측 불가능해진다. → 호출부 반환값 점검 +
    경고 출력 추가(동작 변경 없는 관측성 추가)로 대응.
16. **[문서 오류 교정] `BUILD_WITH_MARCH_NATIVE`의 arm 가드는 aarch64를 막지
    못한다** (C3) — 가드는 `CMAKE_SYSTEM_PROCESSOR MATCHES "arm"`인데 Jetson/
    arm64 리눅스의 값은 `aarch64`로 부분문자열 `arm`을 포함하지 않는다. 즉
    P6_DESIGN §A(5)의 "arm 분기 있음" 서술은 틀렸고, 루트 CMakeLists의 명시적
    OFF 고정이 **유일한 방어선**이다(장식이 아니라 하중 부재).
17. **[조치 필요] `DO_SSE_AUTODETECT` 미고정** (C4) — 기본 ON이며 빌드 호스트의
    `/proc/cpuinfo`를 읽어 컴파일 플래그를 고른다. arm64에서는 무해하나
    x86 호스트에서 빌드하면 g2o 디렉터리 스코프에만 `-msse*`가 붙어 코어와
    비대칭이 된다 → 고정 필요.
18. **[게이트 결함] 플래그 패리티 증거가 정작 그 플래그를 못 본다** (C2) —
    `container_build.sh`/`run_equiv.sh`는 최상위 `CMakeCache.txt`를 grep하는데,
    g2o는 **디렉터리 스코프**에서 플래그를 덧붙이므로 캐시에 흔적이 없다.
    실측 예: 캐시 `-O3 -DNDEBUG` vs 실제 g2o TU 규칙 `-Wall -O3 -DNDEBUG
    -std=gnu++17 -fPIC -Wall -O3`. → `build.ninja`/`compile_commands.json`을
    보도록 교정.
