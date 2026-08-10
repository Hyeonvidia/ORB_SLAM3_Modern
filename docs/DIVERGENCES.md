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
19. **[P8-3] IMU 초기화 큐 퍼지의 delete를 isBad() 조건부로 강등** —
    `InitializeIMU`/`ScaleRefinement` 말미의 큐 퍼지는 SetBadFlag 후 무조건
    delete였으나, SetBadFlag는 맵 원점 KF(`mnId==GetInitKFid`)와 mbNotErase에서
    **조기 return하여 KF를 맵에 등록된 채로 남긴다** — 그 상태의 delete는
    mspKeyFrames/연결 그래프 댕글링(UAF 대기 상태, #7·OWNERSHIP.md). 이제
    `isBad()`가 참이 된 경우에만 delete한다. 조기 return 조건은 큐 잔류
    KF(미처리 = 맵 미편입, LC 미경유)에서 사실상 도달 불가하므로 정상 실행
    동작은 동일하며, 도달하는 병리적 경우엔 UAF 대신 누수(+맵 잔존)를 택한다.
    mbNotErase 지연 삭제는 이후 LoopClosing::SetErase의 재-SetBadFlag로 완결.

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
21. **[P9 발견/보존] 루프 탐지의 오염 소비 합성 결함 (R-1 형상, FixLevel
    1순위)** — 병합 스케일 게이트 중단(`continue`)이 `mbLoopDetected=true`를
    남긴 채 반복을 탈출하고(루프 상태 소거는 스킵된 병합-소비 경로에 있음),
    루프 reffine 실패 분기는 병합 쪽과 달리 플래그를 소거하지 않는다. 두
    비대칭이 합성되면 다음 KF에서 **이전 KF에 앵커된 `mg2oLoopSlw`가 현재
    KF의 보정 Sim3로 적용**되어 본질그래프가 오염된다. 발현 조건: 다중 맵
    병합 시도 + 스케일 [0.90,1.1] 밖 + 동시 루프 가설 DETECTED + 후속 reffine
    실패 — 게이트 시나리오(단일 맵)에서는 도달 불가. bug-for-bug 보존, 수정은
    FixLevel. 부수 비대칭 3건도 보존: 병합 reffine 성공이 NotFound를 안 지움,
    BoW 시딩이 NotFound를 안 지움, `mvpMergeMPs` 증거는 어떤 소비 경로도 안
    읽음. 래치 누수 L1/L2와 리셋 미소거는 OWNERSHIP.md 참조.
22. **[P9-3] GBA 완주 스레드 객체 join-후-delete** — 업스트림은 정상 완주한
    GBA의 joinable `std::thread` 객체를 다음 스폰이 포인터로 덮어써 누수했다
    (delete는 중단 경로에만 존재). 스폰 직전 `joinable()`이면 join(완주 상태라
    즉시 반환)+delete 후 재스폰한다. 관측 가능 동작 동일, 자원 누수만 해소.
23. **[P9-3] KFDB DetectNBestCandidates의 bad-KF 무한 루프 가드** — 누적 점수
    리스트 순회에서 `if(pKFi->isBad()) continue;`가 반복자/인덱스 전진을
    건너뛰어, bad KF가 리스트에 오르면 **전면 행(무한 루프)**이었다(업스트림
    계승). 전진을 보장하도록 교정 — 정상 실행에서 동작 동일(해당 조건 미발현
    시 동일 순회), 병리 조건에서 행 대신 해당 후보 스킵. #19와 동급의 안전
    강등이며, 후보 집합이 달라질 수 있는 유일한 경우는 기존엔 행이었다.
24. **[P9 발견/보존] OptimizeSim3에 raw mbFixScale 전달** — BoW 탐지 경로가
    IMU_MONOCULAR pre-BA2 완화 규칙으로 `bFixedScale`을 계산해 놓고 호출은
    raw `mbFixScale`을 넘긴다(다른 3개 사이트는 완화값 사용). 단안-관성 초기
    구간의 Sim3 정련이 스케일 고정으로 도는 수치 차이 — bug-for-bug 보존,
    게이트가 판별 영역. P9-1에서 죽은 계산을 삭제하며 호출부에 의도 주석 명시.
25. **[P10-5] GBA 조기 return의 mbRunningGBA 영구 잔류 수정 (scope-exit
    가드)** — `RunGlobalBundleAdjustment`의 정상 꼬리만
    `mbFinishedGBA=true; mbRunningGBA=false`를 기록했고, 에포크 불일치
    return과 imu-도중-초기화 return은 두 플래그를 건드리지 않아
    **mbRunningGBA=true가 영구 잔류**했다(업스트림 계승; 중단자도 안
    되돌림). 이후 isRunningGBA()가 유령 GBA를 보고해 CorrectLoop/MergeLocal
    은 존재하지 않는 GBA를 다시 "중단"하고(에포크만 헛증가), MergeLocal은
    bRelaunchBA=true로 **돌지 않던 GBA를 재스폰**했다. 이제 함수 전체를
    mMutexGBA 하의 scope-exit 가드(지역 RAII 구조체)가 감싸 **모든 return
    경로**에서 플래그를 정리한다. 정상 완주 경로 동작 동일; 병리 조건
    (에포크 중단 경합)에서만 재스폰 결과가 달라진다 — 잔류-true의 유령
    재스폰 대신 정직한 not-running. #22/#23과 동급의 안전 강등이며, P10-5의
    no-detach 전환(스폰 직전 reap-join이 가드의 정리 이후에 오도록 join →
    플래그 기록 → 스폰 순서로 재배열)과 한 몸이다.
    **부수 관측 — 티어다운 크래시 소멸**: Shutdown join 복원 후 P10-5 검증
    전 실행(스모크·kitti07×3·kitti00·SI 페어·TSAN T1/T2)이 **exit 0 클린
    종료**. TSAN 원시 로그가 직접 증거: P10-4 시점 T2는 `SEGV in _XSend →
    ABORTING`(미조인 viewer의 X 활동이 프로세스 종료와 경쟁), T1은 종료 중
    glib 파괴-뮤텍스 레이스에서 로그 절단이었으나, P10-5는 양쪽 다 TSAN
    atexit 에필로그("reported N warnings")까지 완주하고 thread-leak 리포트
    0건(리크 검사가 실제로 돌았는데 0 = LM/LC/Viewer 전부 join됨). 종료
    코드 게이트化는 아직 하지 않음(정책 유지, phase 게이트에서 재확인).

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
    실패, 알고리즘 init 실패 포함). 그런데 우리 코드의 **18개 `optimize()`
    호출이 전부 반환값을 버렸다**(초기 기술의 ~24는 과대 계수). 즉 최적화가 실패해도(-1 반환) 함수는 변경되지
    않은 추정치를 그대로 맵에 써 넣고 조용히 성공한 척한다. 항목 13과 결합하면
    "루프 클로징이 조용히 무력화"가 관측 불가능해진다. → **해결됨**(2026-08-09): 18곳 전부를
    `RunOptimization()` 헬퍼 경유로 바꿔 실패 반환을 stderr로 보고한다
    (Verbose가 아니라 stderr인 이유: System이 VERBOSITY_QUIET을 강제하므로
    Verbose 경로면 그대로 묻힌다). 정지 플래그가 올라간 정상 중단은 경고하지
    않는다.
    **부수 발견 — 반환값만으로는 #13을 잡을 수 없다**: Eigen LLT 거부는
    `optimization_algorithm_levenberg.cpp:107-146`에서 "시도 스텝 기각"으로
    처리되어 `Fail`이 아니라 `Terminate`로 귀결된다. 즉 추정치가 한 번도
    움직이지 않아도 `optimize()`는 **양수**를 반환한다. 그래서 λ=1e-16으로
    도는 두 `OptimizeEssentialGraph` 변형에는 chi2 전후 비교를 따로 넣어
    "포즈 그래프가 움직이지 않음"을 직접 보고하게 했다.
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

## 게이트 병렬화 검증 — 음성 결과 (2026-08-09)

머신이 16코어인데 게이트 실행 하나가 코어 0.5개(46.7%)만 쓰고 3시간 내내
3%만 가동되는 것을 실측하고, 라운드 전체(10실행) 동시 실행을 시도했다.
**결과: 채택하지 않는다.**

- 속도는 실제로 빨랐다 — 라운드당 8.8분(순차 43분 대비 4.9배), 부하 9~11.5/16.
- 그러나 판정의 근거인 **짝지은 델타(new−orig)가 보존되지 않았다**:
  동시 실행의 델타는 순차 대비 모드별 2.6~73.9%p 벗어났다(평균 36%p).
- 잡음으로 설명되지 않는다. 순차 4v4를 2+2로 쪼갠 내부 잡음은 평균 9.6%p인데,
  이는 n=2 중앙값이라 **과대추정**이다. 표본이 더 큰(n=3v4) 동시 비교가
  그보다 4배 큰 차이를 보인다면 방향이 반대다.
- 메커니즘도 있다: 10실행 × 3~4스레드 = 30~40스레드가 16코어에 올라가면
  트래킹이 50ms 프레임 예산을 넘길 수 있고, 두 arm은 **LocalBA 소요 시간이
  다른**(#10 복원분) 빌드라 경합에 차등 반응한다.
- 설령 경합 잡음이 공통 모드라 해도 분산을 키워 **검정력을 갉아먹는다** —
  4라운드 하한을 어렵게 확보한 마당에 그 반대급부는 받아들일 수 없다.

**채택한 것**: 경합이 전혀 없는 절감만. `KITTI_SEQ=07`(1101프레임, 루프 1회
검출 확인)로 루틴 게이트의 최장 실행을 1/4로 줄인다. Phase 경계 판정은
기본값 KITTI 00(루프 4~5회)을 유지한다.

## P8 게이트 사후 분석 — stereo-inertial의 바이너리 레이아웃 민감성 (2026-08-09)

20. **[계측 사실/수용] 의미론적으로 무해한 삭제 커밋이 stereo-inertial ATE를
    페어드 +9~13% 이동시켰다.** P8 풀 게이트는 형식 PASS였으나(4v4, +24.4%
    p=0.057 — FAIL 기준 미달 한 순위 차), 사전등록 확장(보충 4쌍→8v8)이
    +13.5% p=0.0325를 검출했다. bisect-replication(당일, 페어드 인터리브,
    프로브당 4~8쌍):
    - phase-7 대조: **−8.4%, 악화 0/4** (전일 P7 게이트도 −5.7% — 이중 청정)
    - P8-1(사장 코드 삭제): +8.7% p=0.139, 5/8 · P8-3: +9.2% p=0.139, 6/8 ·
      팁(P8-4): +13.5% p=0.033, 7/8 — **P8 범위 합산 24쌍 중 18쌍 악화
      (부호검정 p=0.011), 계단은 P8-1에서 진입, 이후 정체**
    - 배제된 가설: 의미론 변화(삭제 항목 전수 재감사 — 전부 write-only/미호출,
      제어 흐름 불변; 비트 게이트 9해시 동일), InertialOptimization 입력
      bg/ba 미초기화(정점은 KF 바이어스로 초기화, 인자는 순수 out-param),
      KF 비율 변화(전 빌드·전 arm 122~132개 동일), 머신 상태(대조 프로브가
      같은 날 같은 방식으로 청정), ORIG-arm 우연(창 내 페어드 델타 일관).
    - 남는 메커니즘: **바이너리 레이아웃/코드젠 매개 타이밍 변화**. 이 모드는
      KF 삽입률이 가장 높고 큐깊이<3 배압 규칙(Tracking.cpp c3/c4)이 있어
      LocalMapping 반복 타이밍에 가장 민감하다 — P1의 strict-c++17 건
      (p=0.017), #10의 LM 조기종료 타이밍 건과 동일 효과 계급. KF 개수가
      아니라 **어떤 프레임이 KF가 되는가**가 이동한다.
    - 맥락: 절대 NEW 중앙값 0.0425~0.0436(≈4.3cm)은 ORIG의 당일 실행 산포
      0.031~0.051 내부이고, 역대 게이트 이 모드 델타(+6.3/+3.8/−3.7/−5.7/
      −4.7%) 대비 최초 이상치다. 수술 부위인 mono-inertial은 −1.7% 청정.
    - **처분(사용자 확정)**: 수용+문서화+태그. 죽은 코드 복원은 원칙적 해법이
      아니며(코드베이스를 캐시라인 고고학에 인질로 잡는 것), P9/P10 게이트에서
      이 모드를 계속 계측한다. P9 게이트에서 stereo-inertial이 추가 악화하면
      이 항목 재개 — 그때는 레이아웃이 아니라 실제 회귀일 수 있다.
    - **P9 재계측 (2026-08-10)**: 상시 8v8 판정 결과 **Δ=−7.7%, p=0.677 —
      청정**. P8의 +13.5% 상승이 P9 팁(LC 재배치 후)에서는 관측되지 않음 =
      레이아웃/타이밍 효과의 왕복 또는 P8 측정의 잔여 일간 성분. 재개 조건
      미발동. P10 게이트에서 동일 프로토콜로 계속 계측.
    - 방법론 교훈: **경계 p값(0.05~0.10)은 라운드 추가로 선예화할 것** —
      4v4의 최소 p=0.014는 +24% 효과도 놓칠 수 있다. 프로브 재사용 도구:
      세션 스크래치패드 `test_si_at.sh <git-ref>` 패턴(체크아웃+빌드+4쌍
      인터리브+창 내 판정)을 benchmark/scripts에 이식할 가치 있음.
