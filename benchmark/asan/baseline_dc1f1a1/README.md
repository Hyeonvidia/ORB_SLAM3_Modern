# ASan 기준 대장 (P12-L0-a, 이행 전 캡처 @ dc1f1a1)

캡처 2026-08-12, dev 서비스(별도 seccomp 불요 — TSAN과 달리 ASan은
personality(2) 미사용), 시나리오 정의는 `benchmark/scripts/asan_smoke.sh`,
파서는 `asan_ledger.py`(pc-주소 제거 후 최상위 프레임 쌍 dedup).
A1 euroc_stereo MH01 full(~3.5분) · A2 euroc_mono_inertial MH01 full(~7분).
`ASAN_OPTIONS=detect_leaks=0:halt_on_error=0` — leak 탐지는 툼스톤 계약
(OWNERSHIP rule 1, 의도된 누수)이 신호를 익사시키므로 반드시 OFF.

## 결과: 리포트 0건 (공집합 베이스라인)

| 시나리오 | 리포트 | 판정 |
|---|---|---|
| A1 (stereo MH01, 매핑/culling/툼스톤 핫패스) | **0건** | 설계 예측 일치 |
| A2 (mono-inertial MH01, IMU init + 리셋 스톰) | **0건** | 설계 예측 일치 |

이 공집합은 우연이 아니라 **구조의 산물**이다: 맵 상주 KF/MP는 아무도
해제하지 않으므로(Map::Erase*는 포인터만 제거, delete는 업스트림부터 주석
처리) use-after-free가 원리상 불가능하다. 실재하는 delete 2곳(미편입 큐
KF 전용, #19 가드)은 이 워크로드에서 미도달.

## 이 대장의 용도 (docs/P12_L0_DESIGN.md)

**귀속 앵커**: L2 수명 이행 커밋이 진짜 회수를 도입하는 순간부터, 여기서
0건인 시나리오에 나타나는 모든 리포트는 해당 커밋에 귀속된다. 각 L2 커밋은
`step_<커밋>_<시나리오>.ledger`를 이 디렉터리 옆에 남기고 본 베이스라인과의
델타로 판정한다 (TSAN 대장과 동일 규율).

A3(kitti 00, 루프클로징/GBA 경로)는 L2가 LoopClosing 사이트 클래스에
도달할 때 베이스라인을 추가 캡처한다 (L0-D3).

주의: 진단 전용 트리(/build/asan) — ATE 판정 금지. 원시 run.log는
raw_runlogs.tgz에 보존(리포트 파일은 0건이라 부재 자체가 증거).
