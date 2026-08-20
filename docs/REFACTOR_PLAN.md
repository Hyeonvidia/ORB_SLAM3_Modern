# 방향 재정렬: 리팩토링 중심 계획 (R-시리즈, 2026-08-11)

사용자 재지시에 따른 코스 교정. **이 문서가 이후 작업의 정본이며, PROJECT_PLAN.md의
P-시리즈(P12 포함)와 bug-for-bug/페어드 게이트 체제는 이 시점부로 동결한다.**

## 진단 (왜 교정하는가)

P1~P12-L2까지의 작업에서 검증 장치(페어드 통계 게이트, TSAN 시그니처 대장,
FixLevel 스위치, 수명 프로브)가 비대해져 제품이 되어버렸고, 정작 목표인
**코드 자체의 현대화**(스마트 포인터, 최신 표준, 가독성)는 "bug-for-bug 보존"
원칙 아래 보류돼 왔다. 두 방침은 양립 불가 — 원칙을 교체한다.

## 새 방침 (사용자 확정, 2026-08-11)

- **합리적 수정은 그냥 적용**하고 간단한 영문 주석을 남긴다. 업스트림 결함의
  보존 의무 없음. (기존 FixLevel/DIVERGENCES/TSAN 자산은 삭제하지 않고 동결 보존)
- **원본은 목표가 아니라 바닥이다** (사용자 정정 2026-08-11): 원본 ORB-SLAM3의
  동작 자체가 덜 최적화된 것이므로 **원본 성능을 "준수"하지 않는다 — 개선한다**.
  정확도(ATE)든 효율(지연·CPU·메모리)이든 원본을 넘어서는 변경은 정당화가 필요
  없고, 원본보다 나빠지는 변경만 근거를 요구한다.
- **검증 경량화**: 커밋당 빌드+스모크 게이트(기존 smoke_gate.sh), R-스텝 완료당
  6모드 실행 — N=3 중앙값 ATE를 골든 원본 중앙값과 비교하되 **원본−15%를
  회귀 바닥으로만** 쓴다(원본 초과 개선은 환영이며 이상치가 아님). 페어드
  인터리브/순위합 검정/TSAN 대장/8v8 확장은 수행하지 않는다.
- **FixLevel 처분 전환**: OFF-기본 뒤에 숨겨둔 픽스 8종(#9 스케일 체인룰,
  #13 LDLT, #21 상태 위생, L1/L2, D5, #5, #24, #6, #3)은 개선이 맞으므로
  R4 스위프에서 **무조건 적용으로 승격**하고 플래그 간접층을 은퇴시킨다
  (원 코드 흔적은 git 이력과 DIVERGENCES가 보존).
- 주석 처리된 코드는 삭제가 아니라 **구현을 우선 검토**한다.

## R-시리즈

| 스텝 | 내용 | 검증 |
|---|---|---|
| **R1** | 툴체인 상향: Ubuntu 24.04 컨테이너, GCC 13+, 코어 **C++20** (Thirdparty는 빌드 가능한 최고 수준). 컴파일 낙진 수리 | 빌드+스모크+6모드 대역 |
| **R2** | Eigen3·Pangolin **최신 태그 서브모듈**로 전환 (OpenCV는 apt 유지 — 사용자 결정: 이미지 빌드 30-60분 비용 회피) — **완료 2026-08-20**: eigen 5.0.1(신규 서브모듈, cmake/FindEigen3.cmake 단일핀·apt libeigen3-dev 제거) + Pangolin v0.9.4→v0.9.6(API 낙진 0), smoke 0.0343 / kitti07 0.486 / bit·유닛 전부 PASS | 빌드+스모크 |
| **R3** | DBoW2 **업스트림(dorian3d) 서브모듈** 기준으로 전환 + 포크 수정분을 diff로 추출해 **래퍼 층으로 분리** (g2o의 OrbLevenberg 전례). 바이너리 어휘 로드도 래퍼로 이관 — **완료 2026-08-20**: third_party/DBoW2(master 3924753=최신) 4소스 정적 타깃 + OrbVocabulary 래퍼(include/src/recognition/, 텍스트 로더+P11-V 바이너리 캐시 v1 무변경, extern template 단일 TU) + BowTypes.hpp(비침습 Boost 직렬화) + DUtils::Random→src/util/ 이전, 벤더드 Thirdparty/DBoW2 삭제. 포크 TemplatedVocabulary의 전역 `using namespace std` 낙진 수리: 헤더 ~19개 std:: 정규화, cpp 16개 TU별 지시어 복원(R4 스위프에서 은퇴 예정), backend_equiv Eigen 모듈경로 R2 낙진도 수리. bit 9/9·유닛 7/7·smoke 0.0597·kitti07 0.448(루프 발화)·mono-inertial MH01 0.0680(골든 중앙값 0.0677) 전부 PASS | 빌드+스모크+어휘 라운드트립 |
| **R4** | **modern C++ 전면 스위프** (모듈별 커밋): 스마트 포인터 이행(P11 정찰의 순환 참조 지도 활용 — KF↔MP는 weak 결정, 자식→부모 strong 등), NULL→nullptr, typedef→using, enum class, range-for/알고리즘, rule-of-five, const 정확성, P12-L2 계측 잔재(LifetimeLedger) 제거 — **R4a 완료 2026-08-20**: FixLevel 픽스 8종 무조건 승격(+플래그 기반시설 은퇴 — FixFlags.hpp/.cpp·Settings 파스·System 배너·tests/fixflags/ 삭제; 예외 #3은 플래그만 은퇴, 값은 정본 0 유지 — docs/DIVERGENCES.md 레지스트리 참조) + LifetimeLedger 전체 제거(헤더·LT_PROBE 169줄·lifetime_* 스크립트 4종·benchmark/lifetime/ 아티팩트; 동결 P12 문서는 보존). fl9/fl13/fl5/detection_machine 유닛은 무조건 동작 계약으로 재작성. **R4b 슬라이스 1 완료 2026-08-20**: MapPoint를 shared_ptr(MapPointPtr, include/map/MapTypes.hpp) 이행 — C-lite(전 보유자 strong, 툼스톤 수명 보존, SetBadFlag가 제거 프로토콜 유지), KeyFrame은 슬라이스 2. 소유자 Map::mspMapPoints; 임시 MP delete 루프 은퇴(refcount 소멸); mpReplaced strong; BAEpochs/GBAResult/MergeScratch 키 strong 핀(주소 재사용 방지); .osa 형식 무변경(raw 백업 벡터 유지, PostLoad 1회 래핑 — System.cpp SaveAtlas 주석); 옵티마이저는 핀-셋 관례(Optimizer.cpp 상단). reference_backend 트윈은 타입-온리 변환(양 variant 빌드 검증). 소유권 표는 docs/OWNERSHIP.md "R4b 슬라이스 1". 검증: 빌드+플래그스캔 OK, bit 9/9, 유닛 6/6, smoke 0.0547, kitti07 0.439(루프 발화, R3 0.448), MI MH01 0.0805(골든 [0.068,0.124]), ASan A1 전체 시퀀스 리포트 0건 | 모듈별 빌드+스모크, 스위프 완료 시 6모드 대역 |
| **R5** | **g2o 사용부 재구성**: Optimizer.cpp(~5,000줄)를 기능군별 파일로 분해, 그래프 구축을 빌더/헬퍼로 추출 — "정점→엣지→풀기→회수" 골격이 한눈에 보이게 | 빌드+스모크+kitti07 |
| **R6** | **주석 처리 코드 전수 심사**: 업스트림 원본 기준 목록화 → 구현 가치 판정 → 구현(+영문 주석) 또는 근거 명기 후 제거 | 빌드+스모크 |

## 유지되는 자산

- 모듈 디렉터리 구조(P2), 캡슐화/추출물(ImuInitializer·PlaceRecognition·DetectionChannel),
  ISP 백엔드 인터페이스(P6), g2o 업스트림 서브모듈+OrbLevenberg 래퍼(P6 — R3의 모범),
  concurrency 현대화(P10: CV/atomic/join 사슬), 바이너리 어휘 캐시(P11-V),
  IMU 메시지 패싱(P11-A)
- benchmark/golden의 원본 중앙값 (경량 대역 검증의 기준)
- smoke_gate.sh / bit_gate.sh (bit는 특징 계층 회귀의 값싼 조기 경보로 계속 유용)
