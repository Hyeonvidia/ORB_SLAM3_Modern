# 수명 이벤트 분포 — 파일럿 캡처 (P12-L0-b @ e4032eb)

캡처 2026-08-12, /build/lifetime 트리(-DLIFETIME_TRACE=1, 게이트 Release
플래그 동일), 시나리오 L1 euroc_stereo MH01 · L2 euroc_mono_inertial MH01.
프로브 커버리지: 솔버-로컬 2 + 셧다운/직렬화 18 = 파일럿 20사이트
(sites.txt). 파서/집계는 lifetime_report.py.

## 결과 (L1.report / L2.report)

| 클래스 | 발화 | 판정 증거 |
|---|---|---|
| **SysTrajEuRoC.walk** (부모 체인 워크) | L1: 죽은 KF 10개 × 333회, 사망 후 최대 190s · L2(리셋 스톰): **645개 × 2,772회, 중앙값 62s, 최대 146s** | **내력벽 실측 확정** — 셧다운 궤적 저장이 분 단위로 오래된 죽은 KF의 mTcp/mpParent를 읽는다. C-full 시 child→parent strong 필수(P11_RECON 차단 요인 4의 실증) |
| 스킵 가드 10종 (SysKFTraj*.skip 등) | **0건** | GetAllKeyFrames/GetAllMapPoints가 bad를 이미 제외하므로 도달 불능 — 만료-안전 후보 (정적 도달성 논거로 승격 대기) |
| Map::PreSave/PostLoad 8종 | 미발화 | 이 시나리오에 SaveAtlas/LoadAtlas 없음 — **미발화 ≠ 0건**, S 착수 전 save-enabled 시나리오로 별도 캡처 필요 |
| Sim3Solver/MLPnP.match | 미발화 | MH01에 루프/재국소화 미발생 — KITTI 00 시나리오로 별도 캡처 필요 (A3와 동일 시점) |

## 판정에 쓰는 법

- walk 클래스의 긴 꼬리는 "툼스톤이 하중을 받는 실측 증거"로 L2 이행
  대상에서 제외(또는 pin 유지)의 근거가 된다.
- 0건 사이트는 게이트급 N런 반복 + 도달성 논거를 갖춰야 만료-안전으로
  승격된다(1런 0건은 후보일 뿐).
- 남은 L0 절차: 계측 섭동 검증(P7 골든 트레이스 전이 타입 집합 비교)은
  프로브가 핫 클래스(Optimizer 62)에 진입하기 전 필수 — 파일럿 클래스는
  실시간 루프 밖(셧다운/솔버 예외 경로)이라 섭동 면제로 판단.

주의: 진단 전용 트리 — ATE 판정 금지. 게이트 바이너리 불활성은 L0-b1
커밋에서 md5 동일로 증명됨(전 14 타깃).
