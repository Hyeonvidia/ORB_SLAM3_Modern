# P12-G0-2: backend_equiv `gtsam` 제3 변형 — 통계 봉투 방법론 (v0, 2026-08-12)

목적: GTSAM 부분 백엔드(G1)가 착지하기 **전에** 그 검증 체제를 문서로
선점한다. P6의 g2o→g2o 이전은 비트/수치 패리티로 증명했지만, 다른
최적화기(GTSAM)에는 그 잣대가 원리적으로 불가능하다 — 검증 체제가
분기한다는 사실 자체가 P11에서 GTSAM을 이월시킨 사유 중 하나였으므로,
분기의 정확한 형태를 여기서 고정한다. 근거: docs/P6_DESIGN.md §B(기존
하네스), docs/P11_RECON.md 3부 (2)(메서드 타당성/부분 백엔드 합성),
docs/P12_PLAN.md §3.

## 1. 변형 정의

`-DEQUIV_BACKEND=gtsam` — tests/backend_equiv의 세 번째 빌드 트리.
- 링크: third_party/gtsam(핀은 D8: 4.3 대기) + **third_party/g2o 동시
  링크**(G1의 위임 구조 — GBA/FullInertialBA는 내부 G2oBackend). 심볼
  충돌 없음(네임스페이스 분리; ODR 제약은 두 g2o 간 문제였음).
- 대상 함수: G1이 네이티브 포팅하는 ITrk+IMap 자연군만. 위임 함수는
  gtsam 변형의 검증 대상이 아니다(이미 modern 변형이 커버).
- 하네스 5쌍 중 적용: pose_optimization, inertial_optimization,
  inertial_optimization_full, inertial_optimization_bias,
  pose_inertial_lastkf — **전부 ITrk/IMap 소속이라 5쌍 모두 gtsam 변형
  실행 대상**이다.

## 2. 판정 3층 — 무엇이 정확 판정으로 남고 무엇이 봉투가 되나

| 층 | 판정 | g2o쌍 대비 변화 |
|---|---|---|
| **자기결정성** (변형 내 2회 실행 바이트 동일) | **정확 판정 유지** — SHA256 동일 필수 | 무변화. GTSAM이라도 같은 입력·같은 바이너리는 같은 바이트를 내야 한다 |
| **GT 복구** (compare.py --gt-check, 픽스처의 해석적 정답) | **정확 판정 유지** — g2o 변형과 **동일 임계값** | 무변화가 원칙. 픽스처 GT는 백엔드 무관한 물리적 정답이므로, GTSAM이 이걸 못 맞추면 봉투 이전에 포팅 결함이다 |
| **크로스 백엔드** (vendored↔gtsam 파라미터 비교) | **봉투 판정으로 완화** — §3 | 기존 vendored↔modern은 수치 패리티(예: pose_optimization 바이트 동일). 다른 선형솔버·다른 스텝 정책의 GTSAM에는 불가능 |

핵심 설계 판단: **1차 오라클을 크로스 백엔드 비교에서 GT 복구로 옮긴다.**
g2o쌍에서는 "두 백엔드가 같다"가 증명이었지만, gtsam 변형에서는 "정답을
같은 정확도로 맞춘다"가 증명이고 크로스 비교는 회귀 감지용 2차 지표다.

## 3. 봉투의 정의와 산출

- 비교 대상: 최적화 결과 파라미터(포즈/속도/바이어스/스케일)의 레코드
  필드별 델타 |gtsam − vendored|.
- **봉투 초기값**: 해당 함수의 |modern − vendored| 실측 델타(기존 그린
  기록) × **10**. 근거: g2o쌍 델타는 같은 수학의 부동소수 잡음 바닥이고,
  10×는 "다른 최적화기의 정당한 도착점 차이"를 허용하되 발산·오식별을
  걸러내는 보수적 시작점. GT 게이트가 절대 정확도를 이미 묶고 있으므로
  봉투는 상대 회귀 감지만 담당한다.
- 봉투 값은 compare.py의 함수별 허용오차 표에 `gtsam:` 열로 명문화하고,
  G1 착지 시 실측 분포(픽스처 시드 N=20 섭동, §4)로 재조정한 뒤 고정한다.
  조정 이력은 본 문서에 부기(방법론 변경은 코드가 아니라 문서 커밋).

## 4. 시드 섭동 프로토콜 (봉투의 통계적 근거)

기존 픽스처는 결정적 1구성이다. gtsam 변형 도입 시 EquivFixtures에
시드 파라미터를 추가해(기본 = 현행 시드: 기존 레코드 불변) N=20 섭동
구성을 생성, 세 변형 모두에서 실행한다:
- 분포 비교: 함수별·필드별 |gtsam−vendored| 분포가 |modern−vendored|
  분포의 10× 봉투 내에 드는 비율 ≥ 19/20.
- GT 복구는 20구성 전부 통과(정확 판정이므로 비율 없음).
- 시드 섭동은 G1 검증 시 1회성 캠페인이 아니라 run_equiv.sh의
  `--seeds N` 옵션으로 재현 가능하게 스크립트화한다.

## 5. 하네스 변경 목록 (G1 시점에 코드화)

1. CMakeLists: `gtsam` 분기 — GtsamBackend 소스 + gtsam/g2o 링크,
   섀도 include 없음(모던과 동일하게 라이브 트리가 계약).
2. run_equiv.sh: 변형 루프에 gtsam 추가, 크로스 판정 2종
   (vendored↔modern 기존 유지 + vendored↔gtsam 봉투).
3. compare.py: `--envelope gtsam` 모드 + 함수별 봉투 표.
4. EquivFixtures: 시드 파라미터(기본값 = 현행, 레코드 하위호환).

## 6. 명시적 비목표

- gtsam 변형의 비트 패리티(원리적 불가 — 본 문서의 존재 이유)
- ILoop 함수의 gtsam 검증(위임 구조라 대상 아님; G2 진입 시 별도 문서)
- ATE 게이트 대체(하네스는 함수 단위, 시스템 단위는 기존 페어드 풀 게이트
  가 GtsamBackend 활성 구성으로 별도 판정 — FixLevel과 같은 "동일 바이너리
  구성 스위치" 원칙)

## 7. 선행 의존

- D8: GTSAM 4.3 대기(스테이블 부재 시 프리릴리스 태그 핀) — 서브모듈
  추가는 G1 착수 시.
- G0-1 실기기 재검증(D4)— 컨테이너 대리 판정은 통과(benchmark/times/
  g0_profile_f095359), Orin/iPhone 확인 후 G1 진입.
