# SITL 실행 및 연결 가이드

## 1. 개요

본 프로젝트는 MAVLink 기반 GCS 프로토타입입니다.  
실시간 텔레메트리 수신, ARM, TAKEOFF, GUIDED 이동, AUTO 미션 실행, RTL, LAND 등의 기능을 사용하려면 ArduPilot SITL 또는 Mission Planner SITL 환경이 필요합니다.

본 저장소에는 SITL 실행 파일을 포함하지 않습니다.  
SITL은 사용자 환경에 따라 설치 방식과 실행 경로가 달라질 수 있으므로 별도로 준비한 뒤 GCS와 MAVLink로 연결합니다.

---

## 2. 권장 테스트 환경

- Windows 10 또는 Windows 11
- Python 3.10 이상
- Mission Planner SITL 또는 ArduPilot SITL
- MAVLink TCP 연결
- 기본 연결 주소: `tcp:127.0.0.1:5760`

---

## 3. Mission Planner SITL 사용 절차

1. Mission Planner를 실행합니다.
2. Simulation 또는 SITL 메뉴에서 ArduCopter를 실행합니다.
3. SITL이 정상적으로 부팅될 때까지 기다립니다.
4. MAVLink 연결 포트가 `127.0.0.1:5760` 또는 Mission Planner 기본 포트로 열려 있는지 확인합니다.
5. 본 GCS 프로그램을 실행합니다.

```bash
python MAVLink_GCS_Submission.py