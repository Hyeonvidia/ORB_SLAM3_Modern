# 수명 이벤트 분포 — 클래스 4 캡처 (교차 스레드 67사이트 @ b56f665)

캡처 2026-08-12, L1 stereo · L2 mono-inertial · L3 kitti 00. 67사이트 전수
계측(제외 2: B3 처분 경로), 발화 ~20사이트. 섭동 게이트: 문서 우주 기준
양 팔 novel 0 — PASS. 라벨의 `OM?.NNN`은 네임스페이스 들여쓰기로 함수
매퍼가 놓친 것: 804/826 = SearchByBoW(KF,KF), 1184 = Fuse.

## 처분 (클래스 4) — 내력벽 제2가족의 발견

| 군 | 대표 사이트 (실측) | 처분 |
|---|---|---|
| **stale forward-reference 순회** (신규 가족) | OM SearchByBoW(KF,KF).826 — L3에서 죽은 MP 415개를 **62,483회, 중앙값 261초**; TrkUpdateLocalPoints.2733 — **64k회, 중앙값 17~77초**, T 스레드 | **내력벽** — KF의 `GetMapPointMatches()` 배열에 남은 죽은 MP 슬롯이 매칭·로컬맵 갱신 빈도로 순회됨. 관측 맵(back-ref)과 대칭인 forward-ref 가족 |
| 공관측/컬링 계열 | KFUpdateBestCo.216, KFTrackedMapPo.350, LMKeyFrameCull.1077, LMSearchInNeig.859, TrkUpdateLocalK.* | **내력벽** — 공관측 리스트·이웃 탐색이 분 단위 죽은 객체를 반복 읽음 |
| 부모 체인/IMU | TrkUpdateFrameI.3318 (중앙값 13초) | **내력벽** — 기지의 walk 가족(T22 문서 경로) |
| 관측 맵 back-ref | MPComputeDisti.354 | **내력벽** — obsErase 가족. 단 가드 뒤라 죽은 KF의 디스크립터는 읽지 않음 |
| **뷰어 드로 루프** | MDDrawMapPoint.78 — 65k회·33k객체(객체당 ~2회, 창 50~130ms), MD.65 (1:1) | **경합 창/한정** — 짧은 사후 창의 스킵. draw-진입부 pin으로 폐쇄 가능 |
| 매처 race-window | OM Fuse.1184 — 0~2ms 창, 6만 회 | **경합 창** — LM 컬링 직후의 fuse. 진입부 pin 가족 |
| 한정 스킵 | LMMapPointCull.442, MD.65 (n_ev == n_obj) | **만료-안전 후보** — 객체당 1회 isBad 판정뿐 |
| 미발화 잔여 | LC/PR/II/KFDB 대부분 (zero-event 목록 참조) | 유보 — 병합/멀티맵/재국소화 시나리오 필요 (LC 병합 계열은 multimap_smoke 확장으로 승격 예정) |

## L6 승격 캡처 (2026-08-13 추가 — 멀티맵 병합 시나리오, L6.report)

stereo MH01 ×2 단일 세션(P11-F3 레시피), 교차 맵 병합 1회 실행 확인
([loopclosing] merge 채널 트레이스).

| 군 | 실측 | 처분 갱신 |
|---|---|---|
| **PRDetectCommon.515 / PRFindMatchesB.669** | 발화 — 죽은 객체를 사망 후 **198~219초**에 읽음 (저빈도: 탐지 이벤트 시에만) | **내력벽 성향** — KFDB 후보군이 오래 죽은 KF/MP를 참조하는 후보/공관측 가족 |
| LCMergeLocal.* 17곳 + LCMergeLocal2.* 4곳 | **병합이 실행됐는데도 0건** | "미발화" → **"실행됨·1런 0건"으로 승격** — 단 포크~병합 간격이 짧아 젊은 맵에 bad가 희소한 시나리오 특성 병기; 강한 승격에는 포크 후 컬링이 쌓인 장기 멀티맵 런 필요 |
| II.*/KFDB.228 | 여전히 미발화 | 유보 유지 — II는 관성 초기화 창의 희귀 경합, KFDB는 #23 가드 창 |

부수: ASan A3(kitti 00, 루프/GBA 경로) 베이스라인 확장 — 0건
(benchmark/asan/baseline_f684146/).

## D3 결정 근거의 완성 (152사이트 대장 폐합)

- 내력벽은 최종적으로 **두 가족 + 골격 읽기**로 수렴한다: ① 관측 맵
  back-reference, ② KF 매치 배열 forward-reference, ③ 부모 체인/mTcp 골격.
- **전 내력벽 사이트가 읽는 것은 골격(포인터 키·isBad·mTcp·부모 링크)뿐**
  — 죽은 객체의 페이로드(디스크립터·키포인트·그리드)를 읽는 사이트는
  152곳 중 0곳 (페이로드 접근은 전부 isBad 가드 뒤). KeyFrame.cpp:682의
  사망 시점 mTcp 준비(원저자의 의도된 골격 유언장)와 정합.
- 따라서 **"골격 영구 툼스톤 + 페이로드 회수"** 절충안이 실측·원저자 의도
  양쪽에 정합하는 D3 1순위 후보로 확정된다. 전면 C-full(객체 소멸)은 세
  가족 전부의 구조 변경을 요구하므로 후순위.

주의: 진단 전용, 1런 캡처. 유보 사이트 승격과 D3 확정 전 반복 런 필요.
