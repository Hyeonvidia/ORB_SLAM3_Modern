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
