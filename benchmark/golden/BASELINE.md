# Golden Baseline — upstream ORB-SLAM3 V1.0 (4452a3c)

원본(무수정 업스트림, `orb_slam3_docker` 하네스, ubuntu:22.04 arm64, 패치 5종은
빌드 호환용만)으로 산출한 ATE RMSE 기준선. 모든 Phase 게이트는 이 표와 비교한다.

- 산출일: 2026-08-05 (궤적) / 2026-08-06 (ATE)
- 평가: evo 1.31.1 (`benchmark/docker/eval.Dockerfile`), 정렬 정책은
  `benchmark/scripts/evaluate.sh` 헤더 참조 (mono만 `-as`, 나머지 `-a`)
- GT: EuRoC 시각 모드 = `evaluation/Ground_truth/EuRoC_left_cam`(in-repo),
  관성 모드 = 데이터셋 `mav0/state_groundtruth_estimate0`, KITTI = odometry poses
- **N=1 시드 기준선** — Phase 1 게이트 시점에 N=5 중앙값으로 확장 예정
  (멀티스레드 비결정성 대응). 재현 명령:
  `orb_slam3_docker/run.sh <mode> <seq> --headless`

| mode | seq | ATE RMSE [m] | 궤적 행수 | 비고 |
|---|---|---|---|---|
| euroc_mono | MH01 | 0.017151 | 3679 | Sim(3) 정렬(-as) |
| euroc_stereo | MH01 | 0.042182 | 3682 | |
| euroc_mono_inertial | MH01 | 0.067695 | 2740 | VIBA 2 완료 후 |
| euroc_stereo_inertial | MH01 | 0.046888 | 3678 | |
| kitti_stereo | 00 | 1.368228 | 4541 | KITTI 12-col |

게이트 기준(PROJECT_PLAN §4): 신규 구현의 모드별 ATE RMSE 중앙값이
기준선 대비 +10% 또는 절대 여유(EuRoC +5mm / KITTI +0.2m) 중 큰 쪽 이내,
그리고 트래킹 완주(프레임 손실률 동등).

제외: `kitti_mono`(업스트림이 KF 궤적만 저장), KITTI 13–21(업스트림 캘리브 yaml 부재).

## 재베이스라인 노트 (2026-08-20, R1)

- `euroc_stereo_inertial`: 초기 골든 중앙값 0.0356은 2026-08-07 하루(N=5-6)의
  호조건 캡처였음. P8~P10 게이트에서 축적된 ORIG arm 실측 23런(중앙값 0.0394,
  p25-p75 = 0.0372-0.0425)을 CSV에 편입 — **현행 SI 기준 중앙값 = 0.0394**.
  경량 검증(REFACTOR_PLAN)의 회귀 바닥은 이 값 기준.
- 비트 해시는 R1 툴체인(Ubuntu 24.04 / GCC 13 / OpenCV 4.6)에서 재기록
  (bit_hashes_env.txt의 R1 노트 참조).
