# MAVLink GCS Python

Python 기반 MAVLink Ground Control Station(GCS) 프로토타입입니다.

ArduPilot SITL 또는 Mission Planner SITL 환경과 MAVLink로 연결하여 드론의 텔레메트리 수신, 지도 기반 미션 계획, 비행금지구역 회피 경로계획, 실시간 임무 안전도 모니터링 기능을 실험하는 프로젝트입니다.

본 프로젝트는 학습, 연구, 포트폴리오, 시뮬레이션 검증 목적의 GCS 프로토타입입니다.

---

## 주요 기능

### 1. MAVLink 기반 드론 연결

- MAVLink HEARTBEAT 수신
- 기체 연결 상태 확인
- 비행 모드 표시
- ARM / DISARM 명령
- TAKEOFF 명령
- GUIDED 이동 명령
- RTL / LOITER / LAND 모드 전환
- ArduPilot SITL 기반 테스트 지원

### 2. 실시간 텔레메트리 표시

- 위도 / 경도
- 고도
- 속도
- 비행 모드
- ARM 상태
- GPS 위성 수
- 배터리 전압 및 잔량
- 드론 위치 지도 표시

### 3. 지도 기반 미션 계획

- 지도 클릭 기반 웨이포인트 추가
- 웨이포인트 번호 표시
- 웨이포인트 간 경로선 표시
- AUTO 미션 업로드
- 미션 실행
- 미션 저장 및 불러오기(JSON)

### 4. 비행금지구역 및 위협 반경 회피

- 지도에서 다각형 비행금지구역 생성
- 원형 위협 반경 생성
- 위협 반경을 다각형으로 변환하여 경로계획에 반영
- A* 기반 우회 경로 생성
- 경로 평활화 적용
- 금지구역을 관통하지 않는 우회 경로 표시

### 5. 임무 안전도 모니터링

- 통신 상태 감시
- GPS 상태 감시
- 배터리 상태 감시
- 안전도 점수 계산
- 위험 단계 분류

위험 단계는 다음과 같이 구분합니다.

- NORMAL
- WARNING
- CRITICAL
- LOST

### 6. 귀환 가능성 예측

- 현재 위치와 홈 위치 간 거리 계산
- 배터리 소모율 기반 귀환 가능성 추정
- 귀환 후 예상 잔여 배터리 계산
- 귀환 가능 / 주의 / 불가 상태 표시

### 7. 위험 추세 감지

- GPS 위성 수 변화 추세
- 통신 상태 변화 추세
- 배터리 소모 추세
- 악화 방향 감지 시 선제 경고 표시

### 8. 상황별 권장 대응

상황에 따라 GCS가 운용자에게 권장 행동을 표시합니다.

- CONTINUE
- HOVER
- RTL
- LAND

권장 행동은 통신, GPS, 배터리, 귀환 가능성 상태를 종합하여 결정됩니다.

### 9. 동적 경로 재계획

임무 수행 중 위험 상태가 감지되면 현재 위치에서 홈 위치까지의 안전 경로를 다시 계산합니다.

- 현재 위치 기준 경로 재계획
- 비행금지구역 회피
- 경로 재계획 실패 시 안전 대응
- 이벤트 로그 기록

### 10. 관심지점 및 좌표 표시

- 관심지점(POI) 마킹
- POI 목록 관리
- MGRS 좌표 표시 지원
- 위경도 / MGRS 좌표 활용

---

## 프로젝트 구조

```text
mavlink-gcs-python/
├─ MAVLink_GCS_Submission.py
├─ path_planner.py
├─ test_path_planner.py
├─ README.md
├─ USER_MANUAL.md
├─ requirements.txt
├─ drone.png
├─ target.png
└─ .gitignore
