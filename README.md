# MAVLink GCS Python

Python 기반 MAVLink Ground Control Station(GCS) 프로토타입입니다.  
ArduPilot SITL 또는 Mission Planner SITL 환경과 연결하여 텔레메트리 수신, 상태 표시, 지도 기반 UI, 웨이포인트 관리 기능을 실험하는 것을 목표로 합니다.

## 주요 기능

- MAVLink HEARTBEAT 수신 및 연결 상태 확인
- 드론 상태 메시지 수신
- 위치, 고도, 속도 등 텔레메트리 표시
- 지도 기반 위치 표시
- 웨이포인트 및 경로 계획 기능 실험
- ArduPilot SITL 기반 테스트

## 개발 환경

- Python
- pymavlink
- Tkinter
- ArduPilot SITL / Mission Planner SITL

## 실행 방법

```bash
pip install -r requirements.txt
python src/main.py