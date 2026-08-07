# 데이터 계층 소유권·수명 계약 (P5-3, 2026-08-07)

정찰 전수조사 기반(3-agent, journal: wf_b582e188-52e). 이 문서는 **현행 계약의
기술**이며, 스마트 포인터 이행은 P6+ 등가성 인프라 이후의 백로그다.

## 할당·해제 지도

| 객체 | 생성자(유일) | 정상 경로 해제 |
|---|---|---|
| KeyFrame | Tracking (3곳: 초기화 2, CreateNewKeyFrame) | **없음 — tombstone** |
| MapPoint | LocalMapping 삼각측량 + Tracking (초기화/신규KF/임시) | **없음 — tombstone** (임시 포인트만 Tracking이 delete) |
| Map | Atlas::CreateNewMap | Atlas 소멸자에서만 (bad 맵은 의도적 누수 — RemoveBadMaps의 delete 주석화) |
| Atlas | System | 없음 (프로세스 수명) |

특수 경로 delete 4곳(맵 편입 전 큐 KF 정리 등)은 LocalMapping/Tracking에 있으며,
그중 **2곳은 업스트림 그대로의 댕글링 위험**으로 문서화한다:
- `LocalMapping::Release()`: 큐 KF를 SetBadFlag 없이 delete — Tracking의
  mpLastKeyFrame이 가리키면 댕글링 (업스트림 계승, FixLevel 후보)
- `LocalMapping::ScaleRefinement()/InitializeIMU()`: SetBadFlag가 초기 KF /
  mbNotErase에서 조기 return해도 delete는 실행 — mspKeyFrames 댕글링 가능

## Tombstone은 버그가 아니라 동시성 계약이다

`SetBadFlag()`는 그래프 연결·관측·DB 색인만 해제하고 메모리는 남긴다(mbBad).
근거: 4개 스레드가 락 없이 보관한 포인터로 `isBad()`를 역참조하는 지점이
**src/ 전체 154곳(13파일)** 존재하고, mpReplaced 체인·스패닝 트리 부모·
Optimizer 로컬 벡터가 bad 객체를 계속 가리킨다. raw delete 도입은 이 154곳
전부의 수명 증명 없이는 UAF다. 따라서:

- **규칙 1**: KeyFrame/MapPoint를 delete하지 말 것. 제거는 SetBadFlag 경유.
- **규칙 2**: 보관한 포인터는 사용 전 isBad() 확인이 관례(락 아님 — 계약).
- **규칙 3**: MapPoint 대체는 Replace()/GetReplaced() 체인을 따라갈 것.

## 직렬화 제약 (소유권 변경 시 필수 고려)

- Map/Atlas가 raw 포인터 vector(mvpBackup*)를 boost로 직렬화 — 로드 시
  boost가 new로 재생성(소유권이 아카이브 경로에서 발생). 정적 ID 카운터
  (Map::nNextId 등)도 직렬화됨.
- 교차 참조는 PreSave에서 ID로 평탄화, PostLoad에서 재연결. 필드 삭제/이동은
  맵 파일 포맷 변경이므로 별도 버전 처리 필요 (예: mfScale).

## 뮤텍스 요약 (정찰 (2)절)

- 정준 잠금 순서: MapUpdate → Connections → Features → Pose / Global → Pos
  (역방향 위반 관측 0건 — 신규 코드도 이 순서를 지킬 것)
- `MapPoint::mGlobalMutex`: pose-only 최적화 동안 전 포인트 위치의 세대 일관
  스냅샷 제공 — per-instance로 축소 시 혼합-세대 좌표 최적화가 되므로 **현상
  유지** (성능 이슈로만 기록)
- Atlas 무락 메서드군(AddKeyFrame/AddMapPoint/AddCamera 등)의 레이스는
  업스트림 계승 — FixLevel 후보 목록에 기재
