# P12-L0 설계 초안 (v0, 2026-08-11) — 수명 오라클 인프라

목적: C-full(진짜 회수) 기각의 유일 근거였던 "152개 만료 결정의 오라클
부재"(P11_RECON 1부)를 **측정 가능한 두 계기**로 해소하거나, 실패 시 영구
툼스톤을 근거 있는 결정으로 확정한다(P12_PLAN §1 L0 Go/No-Go). 설계 원칙은
P10-0 TSAN 대장 방법론의 재사용: 진단 전용 빌드 트리 + 시그니처/분포 대장 +
커밋별 델타 판정.

## L0-a. ASan 대장 (UAF 오라클)

TSAN 인프라의 1:1 미러. 목적: L2 이행 커밋이 회수를 도입하는 순간부터
"이 커밋이 UAF를 만들었는가"를 커밋 단위로 귀속.

- **빌드**: `benchmark/scripts/asan_build.sh` — tsan_build.sh와 동일하게
  커맨드라인 플래그 주입만(CMakeLists 무변경), `/build/asan` 트리를 게이트
  트리 옆에(오염 없음). 플래그:
  `-fsanitize=address -fsanitize-recover=address -fno-omit-frame-pointer -g -O1`
  (recover 필수 — 기본 ASan은 첫 리포트에서 abort하므로 대장이 한 런에서
  전체 시그니처를 못 모은다). DBoW2/g2o 포함 전체 계측(TSAN과 동일 근거:
  비계측 측이 리포트를 숨긴다).
- **런타임**: `ASAN_OPTIONS=detect_leaks=0:halt_on_error=0` 이 **기본**.
  LeakSanitizer는 반드시 꺼야 한다 — 툼스톤은 설계상 누수 계약(OWNERSHIP
  rule 1)이라 leak 리포트가 진짜 신호를 익사시킨다. 컨테이너 제약: TSAN의
  personality(2)/seccomp 문제는 ASan에 없음 → **dev 서비스 그대로 사용,
  dev-asan 별도 서비스 불요** (검증 후 필요 시에만 추가).
- **대장**: `benchmark/scripts/asan_ledger.py` — tsan_ledger.py 포크.
  시그니처 = (리포트 타입, 액세스 스택 top frame, free/alloc 스택 top
  frame), 동일 정규화(모듈 오프셋·컬럼 제거, /workspace/ 접두 제거).
  산출물은 `benchmark/asan/baseline_<sha>/` + `step_*.ledger` — TSAN 대장과
  같은 배치.
- **베이스라인 주장**: 현 트리에서 게이트 워크로드의 ASan 리포트는 **공집합
  예상** — KF/MP는 아무도 해제하지 않으므로 UAF가 원리상 불가능하다(유일한
  실해제인 Release 드레인 창은 게이트 미도달, #19형). **공집합 베이스라인이
  곧 가치다**: L2 커밋 이후의 모든 리포트가 그 커밋에 귀속된다.
- **런 규모**: 스모크급(euroc_stereo MH01 + euroc_mono_inertial MH01) 각
  1회/커밋. ~2x 슬로다운. **진단 전용 — ATE 판정 금지** (dev-tsan과 동일
  정책 문구를 스크립트 헤더에 명기).

## L0-b. 수명 이벤트 분포 계측 (만료 결정 오라클)

핵심 질문을 데이터로 바꾼다: *"SetBadFlag 이후 이 사이트는 그 객체를
얼마나 늦게까지, 몇 번이나 읽는가"* — 사이트별 bad-이후 읽기 창의 실측
분포가 곧 만료 결정의 1차 증거다.

- **사이드테이블**(P5 원칙 — 데이터 클래스에 낙서 필드 금지):
  `LifetimeLedger` 전역 레지스트리.
  - 이벤트 A: `SetBadFlag(kind, mnId)`가 전역 atomic 시퀀스 + 단조 시각을
    스탬프 (KeyFrame.cpp/MapPoint.cpp의 SetBadFlag 말미 각 1줄).
  - 이벤트 B: `LIFETIME_PROBE(site_id, p)` — deref 사이트에서 `p->isBad()`
    일 때만 (site_id, kind, mnId, seq_delta, t_delta) 기록.
  - 스레드별 링버퍼 → 셧다운 시 `/results/<run>/lifetime_trace.csv` 플러시
    (조인 사슬이 P10-5에서 복원됐으므로 플러시 지점은 Shutdown 이후 안전).
- **조건부 컴파일이되 #20 위반이 아니다**: FixLevel의 런타임-플래그 원칙은
  **게이트 바이너리의 동작 픽스**에 적용되는 규율이다. L0-b는 TSAN/ASan과
  같은 **진단 전용 별도 트리**(`/build/lifetime`, `-DLIFETIME_TRACE=1`)라
  게이트 바이너리는 프로브가 컴파일 자체가 안 된 상태로 유지된다. 이 구분을
  스크립트와 본 문서에 명기해 재논의를 차단한다.
- **프로브 전개는 L2 슬라이싱과 동일 순서**: 한 번에 152개를 박지 않는다.
  파일럿 = 클래스 1(솔버-로컬 2사이트) + 클래스 2(직렬화/셧다운 경로
  18사이트)만. 이후 클래스별 L2 착수 직전에 해당 클래스 프로브를 먼저 심어
  "이행 전 분포"를 확보한다.
- **분석기**: `benchmark/scripts/lifetime_report.py` — 사이트별 bad-deref
  건수, seq_delta 히스토그램, 최대 꼬리. 판정 스케치:
  - 게이트급 N런에서 bad-deref **0건** + 정적 도달성 논거 → 만료-안전 후보
    (weak/회수 전환 가능).
  - 꼬리가 길거나 교차 스레드 유입이 있으면 → 그 사이트는 툼스톤 계약의
    하중 부재 — 전환 금지, 근거와 함께 대장 기록.
- **계측 섭동 검증**: P7 골든 트레이스 방법론 재사용 — 계측/비계측 진단
  런의 상태 전이 타입 집합 비교(신규 전이 0 요구) + 프로브 단가 상한
  (링버퍼 append, ns급) 명기.

## Go/No-Go (P12_PLAN §1 L0의 정량화)

| 예산 | 내용 |
|---|---|
| L0-a ≤ 3커밋 | asan_build.sh + asan_ledger.py + 베이스라인 대장 커밋 |
| L0-b ≤ 3커밋 | LifetimeLedger + 파일럿 2클래스 프로브 + lifetime_report.py |

- **Go**: 두 계기 모두 예산 내 착지 + ASan 베이스라인 공집합 확인 + 파일럿
  분포가 판정 가능한 형태(0건이든 꼬리든) → L2 파일럿(솔버-로컬) 진입.
- **No-Go** (하나라도): ① 예산 초과, ② 계측 섭동이 전이 타입 집합을 바꿈,
  ③ 파일럿 분포가 전 클래스 중꼬리(만료-안전 후보 0) → **영구 툼스톤을
  결정으로 확정**하고 L은 L1(커스터디 문서화·카운트 교정)만 수행. 이 경우
  L0 산출물은 폐기가 아니라 계약의 실측 증거로 대장에 남는다.

## 산출물 배치

```
benchmark/scripts/asan_build.sh, asan_smoke.sh, asan_ledger.py, lifetime_report.py
benchmark/asan/baseline_<sha>/ + step_*.ledger        # TSAN 대장과 동형
benchmark/lifetime/pilot_<sha>/                        # 분포 리포트
include/core/LifetimeLedger.hpp (LIFETIME_TRACE 전용)  # 헤더 온리
docs/P12_L0_DESIGN.md (본 문서)
```

## 명시적 비목표

- 계측 빌드의 ATE 게이트 사용 (진단 전용 — TSAN 정책 승계)
- LeakSanitizer 기반 툼스톤 누수 감사 (누수는 계약이지 결함이 아님)
- 완전 결정적 리플레이 (분포 기반 판정으로 충분하게 설계 — 전체 결정성은
  멀티스레드 ATE 비결정성 수용 원칙과 충돌)
- 프로브의 게이트 바이너리 잔류 (컴파일 아웃 확인을 비트 게이트가 담보 —
  프로브 커밋 후 비트 게이트 해시 불변이 곧 증명)

## 열린 설계 결정

| # | 결정 | 기본 권고 |
|---|---|---|
| L0-D1 | seq_delta의 시퀀스 축 — 전역 op 카운터 vs KF 생성 카운터 | 전역 atomic op 카운터 (SetBadFlag/AddKeyFrame/optimize 진입에서 증가 — BAEpochs 에폭과 정렬 가능) |
| L0-D2 | 링버퍼 오버플로 정책 | drop-oldest + 드롭 카운트 기록 (분포 꼬리 보존이 목적이므로 drop-newest 금지) |
| L0-D3 | ASan 스모크의 모드 집합 | stereo + mono-inertial 2종으로 시작, L2가 LC 클래스에 도달하면 KITTI 00 추가 (루프클로징 경로 커버) |
