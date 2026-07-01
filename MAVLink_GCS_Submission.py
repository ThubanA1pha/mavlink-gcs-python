"""
MAVLink_GCS_Submission.py

프로그램명 : MAVLink 기반 드론 GCS 시뮬레이션 및 임무 안전도 모니터링 프로그램
설명       : ArduCopter SITL과 MAVLink 프로토콜을 기반으로 동작하는 지상관제소(GCS)
             프로그램이다. 지도 기반 미션 계획(웨이포인트·비행금지구역·위협 반경)과
             경로계획(A* 회피 + 평활화), 실시간 텔레메트리 감시(통신·GPS·배터리)를
             통합하여 임무 안전도를 점수화하고, 위험 단계(NORMAL/WARNING/CRITICAL/LOST)에
             따라 권장 대응과 회피 기동을 제공한다.
주요 기능  : - 지도 기반 미션 플래너 (웨이포인트 계획·저장·불러오기, AUTO 비행 업로드/자동 이륙)
             - 비행금지구역·위협 반경 회피 경로계획 (격자 A* + 경로 평활화)
             - 운용자 선택형 회피 정책 5종 (경고만/제자리정지/우회 후 복귀/대피지점 귀환/착륙)
               및 비상 대피지점(Rally Point) 등록
             - 위협구역/비행금지구역 "내부 진입" 실시간 감시 → 안전도 점수와 무관하게
               진입 즉시 경고하고 자동으로 구역 탈출 경로 재계획
             - 조이스틱 수동 비행 중 위협구역 근접 시 위치 제어를 일시 자동 회피로 전환
               (고도·기수 방향은 조종자 유지)
             - 실시간 안전도 점수화 및 위험 단계 분류, 이벤트 자동 로그(CSV)
             - 배터리 소모율 기반 귀환 가능성 예측, 단계별 알림(50%/30%/20%),
               배터리 부족 시 제자리 착륙보다 RTL(귀환) 우선 권장
             - 상황별 권장 대응 제시 및 위험 시 동적 경로 재계획
             - 관심지점(POI) 기록, MGRS 좌표 표시, 임무 결과 보고서 자동 생성
실행 환경  : Python 3.10+, Tkinter, ArduCopter SITL(TCP 5760), pymavlink, tkintermapview
의존 모듈  : path_planner.py (격자 A* 회피 경로계획 — 동일 폴더에 위치해야 함)
"""

import os
import sys
import json
import time
import math
import queue
import atexit
import sqlite3
import threading
import traceback
import subprocess
import tkinter as tk
import tkinter.ttk as ttk
import tkinter.messagebox
import tkinter.simpledialog
from tkinter import filedialog

import requests
import tkintermapview
from PIL import Image, ImageTk
from pymavlink import mavutil

import path_planner  # 비행금지구역 회피 경로계획 모듈

# 회피 정책 5종 (ArduPilot FENCE_ACTION 모델을 본뜸)
AVOIDANCE_ACTION_REPORT_ONLY = "REPORT_ONLY"
AVOIDANCE_ACTION_BRAKE       = "BRAKE"
AVOIDANCE_ACTION_REROUTE     = "REROUTE"
AVOIDANCE_ACTION_RTL_RALLY   = "RTL_VIA_RALLY"
AVOIDANCE_ACTION_LAND        = "LAND"

AVOIDANCE_ACTION_LABELS = {
    AVOIDANCE_ACTION_REPORT_ONLY: "경고만 (경로 변경 없음)",
    AVOIDANCE_ACTION_BRAKE:       "제자리 정지 (HOVER)",
    AVOIDANCE_ACTION_REROUTE:     "우회 후 원래 경로 복귀 (기본값)",
    AVOIDANCE_ACTION_RTL_RALLY:   "Rally Point(또는 홈)로 귀환",
    AVOIDANCE_ACTION_LAND:        "즉시 착륙",
}

# MGRS 좌표계 (선택적 의존성 — 없어도 GCS는 정상 기동)
try:
    import mgrs as _mgrs_lib
    _MGRS_AVAILABLE = True
except ImportError:
    _MGRS_AVAILABLE = False
    print("[MGRS] mgrs 라이브러리가 없어 MGRS 좌표 표시가 비활성화됩니다. "
          "(pip install mgrs)")


# ============================================================
# 1. 지형 고도 계산
# ============================================================

DEFAULT_ELEVATION = 30


def get_elevation(lat, lon):
    """
    Open-Elevation API를 사용하여 입력 좌표의 지형 고도를 가져온다.
    API 호출 실패 시 기본 고도(DEFAULT_ELEVATION)를 반환한다.
    """
    url = f"https://api.open-elevation.com/api/v1/lookup?locations={lat},{lon}"

    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        data = response.json()
        return data["results"][0]["elevation"]

    except Exception as error:
        print("[ELEVATION ERROR]", error)
        print(f"[ELEVATION] 기본 고도 {DEFAULT_ELEVATION}m 사용")
        return DEFAULT_ELEVATION


# ============================================================
# 2. SITL 관리 클래스
# ============================================================

class SITLManager:
    """
    ArduCopter SITL 프로세스를 실행하고 종료하는 클래스
    """

    def __init__(self):
        self.proc = None

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.sitl_path = os.path.join(current_dir, "sitl")

    def read_output(self):
        """
        SITL 콘솔 출력을 Shell에 출력한다.
        """
        if self.proc is None or self.proc.stdout is None:
            return

        try:
            for line in self.proc.stdout:
                print("[SITL]", line.strip())

        except Exception as error:
            print("[SITL OUTPUT ERROR]", error)

    def start(self, lat, lon):
        """
        지정된 위도/경도를 HOME 위치로 하여 SITL을 실행한다.
        poll()로 실제 프로세스 생존 여부를 확인하므로 Stop 후 재시작이 가능하다.
        """
        if self.proc is not None and self.proc.poll() is None:
            print("[WARN] SITL is already running.")
            return True

        exe_path = os.path.join(self.sitl_path, "ArduCopter.exe")
        default_param_path = os.path.join(self.sitl_path, "default_params", "copter.parm")

        if not os.path.exists(self.sitl_path):
            print("[ERROR] sitl 폴더를 찾을 수 없습니다.")
            print("[CHECK]", self.sitl_path)
            return False

        if not os.path.exists(exe_path):
            print("[ERROR] ArduCopter.exe 파일을 찾을 수 없습니다.")
            print("[CHECK]", exe_path)
            return False

        if not os.path.exists(default_param_path):
            print("[WARN] default_params/copter.parm 파일을 찾을 수 없습니다.")
            print("[WARN] 기본 파라미터 없이 SITL을 실행합니다.")

        terrain = get_elevation(lat, lon)

        cmd = [
            exe_path,
            "-S",
            "-I0",
            "--model", "quad",
            "--home", f"{lat},{lon},{terrain},0"
        ]

        if os.path.exists(default_param_path):
            cmd.extend(["--defaults", "default_params/copter.parm"])

        print("[SITL CMD]", " ".join(cmd))

        try:
            self.proc = subprocess.Popen(
                cmd,
                cwd=self.sitl_path,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace"
            )

            threading.Thread(target=self.read_output, daemon=True).start()
            return True

        except Exception as error:
            print("[SITL START ERROR]", error)
            self.proc = None
            return False

    def stop(self):
        """
        실행 중인 SITL 프로세스를 안전하게 종료한다.
        terminate() 후 응답이 없으면 kill()로 강제 종료한다.
        """
        if self.proc is None:
            return

        print("[SITL] STOP")

        try:
            self.proc.terminate()

            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=2)

        except Exception as error:
            print("[SITL STOP ERROR]", error)

        finally:
            self.proc = None


# ============================================================
# 3. 지도 선택 팝업 클래스
# ============================================================

class MapSelectionDialog(tk.Toplevel):
    """
    프로그램 시작 시 사용할 지도(타일 서버)를 선택하는 팝업 창.
    선택 결과를 self.result = (타일 서버 URL, DB 파일명, 최대 줌) 으로 저장한다.
    선택 없이 창을 닫으면 self.result = None 으로 둔다.
    """

    def __init__(self, parent):
        super().__init__(parent)

        self.title("지도 선택")
        self.geometry("400x200")
        self.resizable(False, False)
        self.attributes("-topmost", True)

        self.result = None

        self.protocol("WM_DELETE_WINDOW", self.on_cancel)

        # 지도 종류: 표시이름 -> (타일 서버 URL, DB 파일명, 최대 줌)
        self.map_options = {
            "1. OpenStreetMap": (
                "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
                "osm.db", 19),
            "2. CartoDB Light": (
                "https://a.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png",
                "carto_light.db", 19),
            "3. CartoDB Dark": (
                "https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png",
                "carto_dark.db", 19),
            "4. OpenTopoMap": (
                "https://a.tile.opentopomap.org/{z}/{x}/{y}.png",
                "opentopo.db", 19),
            "5. Esri Satellite": (
                "https://server.arcgisonline.com/ArcGIS/rest/services/"
                "World_Imagery/MapServer/tile/{z}/{y}/{x}",
                "esri_sat.db", 19),
            "6. Google Hybrid": (
                "https://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}&s=Ga",
                "google_hybrid.db", 19),
        }

        tk.Label(
            self,
            text="사용할 지도를 선택해 주세요.",
            font=("맑은 고딕", 12, "bold")
        ).pack(pady=20)

        self.combo = ttk.Combobox(
            self,
            values=list(self.map_options.keys()),
            state="readonly",
            width=35
        )
        self.combo.current(0)
        self.combo.pack(pady=10)

        tk.Button(
            self,
            text="확인 및 실행",
            width=20,
            bg="#1ecc71",
            fg="white",
            font=("맑은 고딕", 10, "bold"),
            command=self.on_confirm
        ).pack(pady=15)

        self.update_idletasks()
        pos_x = (self.winfo_screenwidth() // 2) - (self.winfo_width() // 2)
        pos_y = (self.winfo_screenheight() // 2) - (self.winfo_height() // 2)
        self.geometry(f"+{pos_x}+{pos_y}")

    def on_confirm(self):
        selected = self.combo.get()
        self.result = self.map_options[selected]
        self.destroy()

    def on_cancel(self):
        self.result = None
        self.destroy()


# ============================================================
# 4. HUD 클래스 (인공 수평선 + 상태 표시) - 신규
# ============================================================

class GCSHUD(tk.Canvas):
    """
    드론 상태를 표시하는 HUD(Heads-Up Display) 캔버스.
      - 인공 수평선(롤/피치)
      - ARM/DISARM, 비행 모드
      - 배터리 전압/잔량, 위성 수
      - 지면 속도, 상대 고도
    update_hud()로 받은 값으로 화면을 갱신한다.
    """

    R = 70  # 인공 수평선 반지름

    # 모드 표시 매핑 (내부 값은 그대로 두고 화면 표시만 변환)
    MODE_DISPLAY_MAP = {
        "DISCONN": "연결 대기",
    }

    def __init__(self, parent, width=300, height=220):
        super().__init__(
            parent,
            width=width,
            height=height,
            bg="#0e1828",
            highlightthickness=1,
            highlightbackground="#2a3f5a"
        )

        self.w = width
        self.h = height
        self.cx = width / 2    # 중심 x
        self.cy = height / 2   # 중심 y

        r = self.R

        # 인공 수평선 배경: 하늘(원) + 지면(반원) + 수평선
        # (하늘색/갈색은 유지, 테두리만 팔레트 색으로)
        self.create_oval(
            self.cx - r, self.cy - r, self.cx + r, self.cy + r,
            fill="#1e90ff", outline="#2a3f5a", tags="horizon_bg")
        self.create_arc(
            self.cx - r, self.cy - r, self.cx + r, self.cy + r,
            start=0, extent=-180, fill="#8B4513", outline="#2a3f5a",
            style=tk.CHORD, tags="horizon_ground")
        self.create_line(
            self.cx - r, self.cy, self.cx + r, self.cy,
            fill="#34d399", width=2, tags="horizon_line")

        # 중앙 고정 기준선 (기체 표식)
        self.create_line(self.cx - 25, self.cy, self.cx - 10, self.cy, fill="#cadcfc", width=3)
        self.create_line(self.cx + 10, self.cy, self.cx + 25, self.cy, fill="#cadcfc", width=3)
        self.create_line(self.cx, self.cy - 10, self.cx, self.cy, fill="#cadcfc", width=3)

        # 좌상단: ARM / DISARM
        self.arm_text = self.create_text(
            10, 10, text="DISARM", fill="#f47174",
            font=("맑은 고딕", 10, "bold"), anchor="nw")

        # 우상단: 위성 수 / 배터리
        self.sat_text = self.create_text(
            self.w - 10, 10, text="GPS 0", fill="#cadcfc",
            font=("맑은 고딕", 10, "bold"), anchor="ne")
        self.bat_text = self.create_text(
            self.w - 10, 30, text="0.0V (0%)", fill="#34d399",
            font=("맑은 고딕", 10, "bold"), anchor="ne")

        # 우하단: 비행 모드
        self.create_text(
            self.w - 80, self.h - 10, text="모드:", fill="#9db2c8",
            font=("맑은 고딕", 10, "bold"), anchor="se")
        self.mode_text = self.create_text(
            self.w - 10, self.h - 10, text="NONE", fill="#34d399",
            font=("맑은 고딕", 10, "bold"), anchor="se")

        # 좌측: 속도
        self.create_text(
            35, self.cy - 22, text="속도(m/s)", fill="#9db2c8",
            font=("맑은 고딕", 8, "bold"))
        self.speed_val = self.create_text(
            35, self.cy, text="0.0", fill="#cadcfc",
            font=("Consolas", 14, "bold"))

        # 우측: 고도
        self.create_text(
            self.w - 35, self.cy - 22, text="고도(m)", fill="#9db2c8",
            font=("맑은 고딕", 8, "bold"))
        self.alt_val = self.create_text(
            self.w - 35, self.cy, text="0", fill="#cadcfc",
            font=("Consolas", 14, "bold"))
        self.create_text(
            self.w - 35, self.cy + 20, text="(AGL)", fill="#9db2c8",
            font=("맑은 고딕", 8, "bold"))

    def update_hud(self, roll, pitch, alt, speed, mode, sats, bat_v, bat_p, is_armed):
        """전달받은 드론 상태값으로 HUD 그래픽/텍스트를 갱신한다."""
        # 인공 수평선: 롤(회전) + 피치(상하 이동)
        rad_roll = math.radians(roll)
        p_off = pitch * 1.2  # 피치 1도당 화면 이동량
        x1 = self.cx + self.R * math.cos(rad_roll)
        y1 = self.cy + p_off + self.R * math.sin(rad_roll)
        x2 = self.cx - self.R * math.cos(rad_roll)
        y2 = self.cy + p_off - self.R * math.sin(rad_roll)
        self.coords("horizon_line", x1, y1, x2, y2)

        # ARM 상태
        if is_armed:
            self.itemconfig(self.arm_text, text="ARM", fill="#34d399")
        else:
            self.itemconfig(self.arm_text, text="DISARM", fill="#f47174")

        # 텍스트 갱신 (모드는 내부 값 유지, 표시 직전에만 매핑)
        display_mode = self.MODE_DISPLAY_MAP.get(mode, mode)
        self.itemconfig(self.mode_text, text=display_mode)
        self.itemconfig(self.sat_text, text=f"GPS {sats}")
        self.itemconfig(self.bat_text, text=f"{bat_v:.1f}V ({bat_p}%)")
        self.itemconfig(self.speed_val, text=f"{speed:.1f}")
        self.itemconfig(self.alt_val, text=f"{int(alt)}")


# ============================================================
# 5. 이륙 고도 입력 팝업 클래스
# ============================================================

class TakeoffDialog(tk.Toplevel):
    """이륙 고도를 입력받아 콜백으로 전달하는 팝업."""

    def __init__(self, parent, callback):
        super().__init__(parent)
        self.title("이륙 고도 설정")
        self.resizable(False, False)
        self.callback = callback
        self.transient(parent)
        self.grab_set()
        self.update_idletasks()

        popup_width = 250
        popup_height = 120
        parent_x = parent.winfo_rootx()
        parent_y = parent.winfo_rooty()
        parent_w = parent.winfo_width()
        parent_h = parent.winfo_height()
        pos_x = parent_x + (parent_w // 2) - (popup_width // 2)
        pos_y = parent_y + (parent_h // 2) - (popup_height // 2)
        self.geometry(f"{popup_width}x{popup_height}+{pos_x}+{pos_y}")

        tk.Label(self, text="이륙 고도를 입력하세요 (m):", font=("맑은 고딕", 10)).pack(pady=10)
        self.alt_entry = tk.Entry(self, justify="center")
        self.alt_entry.insert(0, "10")
        self.alt_entry.pack(pady=5)
        self.alt_entry.focus_set()

        btn_frame = tk.Frame(self)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="이륙 실행", width=8, bg="#2ecc71", fg="white",
                  command=self.on_ok).pack(side="left", padx=5)
        tk.Button(btn_frame, text="취소", width=8, command=self.destroy).pack(side="left", padx=5)

    def on_ok(self):
        try:
            altitude = float(self.alt_entry.get())
            if altitude <= 0:
                raise ValueError
            self.callback(altitude)
            self.destroy()
        except ValueError:
            tk.messagebox.showwarning("입력 오류", "올바른 고도(숫자)를 입력하세요.")


# ============================================================
# 5-2. 목적지 이동 고도 입력 팝업
# ============================================================

class GuidedMoveDialog(tk.Toplevel):
    """지도 클릭 위치로 이동할 고도를 입력받아 콜백(lat, lon, alt)으로 전달."""

    def __init__(self, parent, callback, lat, lon):
        super().__init__(parent)
        self.title("목적지 고도 설정")
        self.resizable(False, False)
        self.callback = callback
        self.lat = lat
        self.lon = lon
        self.transient(parent)
        self.grab_set()

        tk.Label(self, text="이동할 고도 (m):", font=("맑은 고딕", 10)).pack(pady=(12, 5))
        self.alt_entry = tk.Entry(self, justify="center")
        self.alt_entry.insert(0, "20")
        self.alt_entry.pack(pady=5)
        self.alt_entry.focus_set()

        btn_frame = tk.Frame(self)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="이동", width=10, bg="#3498db", fg="white",
                  command=self.on_ok).pack(side="left", padx=5)
        tk.Button(btn_frame, text="취소", width=10, command=self.destroy).pack(side="left", padx=5)

        self.update_idletasks()
        popup_w, popup_h = 250, 130
        pos_x = parent.winfo_rootx() + (parent.winfo_width() // 2) - (popup_w // 2)
        pos_y = parent.winfo_rooty() + (parent.winfo_height() // 2) - (popup_h // 2)
        self.geometry(f"{popup_w}x{popup_h}+{pos_x}+{pos_y}")

    def on_ok(self):
        try:
            alt = float(self.alt_entry.get())
            if alt <= 0:
                raise ValueError
            self.callback(self.lat, self.lon, alt)
            self.destroy()
        except ValueError:
            tk.messagebox.showwarning("입력 오류", "올바른 고도(숫자)를 입력하세요.")


# ============================================================
# 5-3. 토스트 메시지 (자동으로 사라지는 안내 팝업)
# ============================================================

class ToastMessage(tk.Toplevel):
    """화면 중앙에 잠깐 떴다가 일정 시간 후 자동으로 사라지는 안내 메시지."""

    def __init__(self, parent, message, duration=2500):
        super().__init__(parent)
        self.overrideredirect(True)            # 창 테두리 제거
        self.attributes("-topmost", True)
        try:
            self.configure(bg="#333333")
        except Exception:
            pass

        label = tk.Label(
            self, text=message, fg="white", bg="#333333",
            font=("맑은 고딕", 11, "bold"), padx=20, pady=12, justify="center")
        label.pack()

        self.update_idletasks()
        x = parent.winfo_rootx() + (parent.winfo_width() // 2) - (self.winfo_width() // 2)
        y = parent.winfo_rooty() + (parent.winfo_height() // 2) - (self.winfo_height() // 2)
        self.geometry(f"+{x}+{y}")

        # duration(ms) 후 자동으로 닫는다.
        self.after(duration, self.destroy)


# ============================================================
# 6. GCS + MAP 메인 클래스
# ============================================================

class GCSMap(tk.Tk):
    """
    Tkinter GUI, 지도 표시, MAVLink 수신, 드론 마커 갱신, HUD 표시를 담당하는 메인 클래스

    STEP 1 대비 추가된 기능:
      - 시작 시 지도 종류 선택 팝업 (MapSelectionDialog)
      - 선택한 지도별 SQLite DB(오프라인 타일 캐시) 저장/사용
      - 지도 좌상단 줌 레벨 텍스트 라벨
      - 드론 상태 HUD (인공 수평선 + 배터리/자세/고도/속도/위성/모드)
      - MAVLink 데이터 스트림 세분화 요청 + 스레드 Lock
    """

    USE_DATABASE_ONLY = False

    # 경로계획/실행 파라미터
    PLAN_CELL_M = 5.0          # 격자 해상도 r (m/cell)
    PLAN_MARGIN_M = 10.0       # 안전여유 (m)
    WAYPOINT_REACH_M = 3.0     # 웨이포인트 도달 판정 임계 거리 (m)

    # 배터리 규정 임계값: 80% 소모(잔량 20% 이하) 시 즉시 귀환(RTL) 권장
    #   (제자리 강제 착륙보다 RTL 우선 — 귀환 불가능할 때만 최후수단 착륙)
    # (일반적인 드론 운용 규정 — 30% 경고 후 20%에서 착륙 권고)
    BATTERY_LAND_THRESHOLD_PCT = 20

    # 사전(정지) 배터리 검증용 기본 소모 모델 — 통상 멀티콥터 평균 기준
    # 특정 제품이 아닌 일반 소형 멀티콥터의 통상 비행시간(약 25분)을 근거로 한다.
    #   100% ÷ (25분 × 60초) ≈ 0.067 %/초
    # 순항 속도는 ArduCopter WPNAV_SPEED 기본값(약 5 m/s)을 참고한다.
    BATTERY_DRAIN_DEFAULT_PCT_S = 0.067   # %/초 (≈ 25분 비행)
    CRUISE_SPEED_DEFAULT_MPS = 5.0        # m/s

    # 작전 상황 기반 대응 매트릭스
    # (link, gps, battery, return_feasibility) → (action, reason),  "*" = 무관, 위에서부터 우선
    ACTION_MATRIX = [
        ("LOST",     "*",    "*",        "*",          "LAND",     "통신 두절 — 제자리 착륙"),
        ("*",        "*",    "*",        "IMPOSSIBLE", "LAND",     "귀환 불가 — 즉시 착륙"),
        ("CRITICAL", "*",    "CRITICAL", "*",          "LAND",     "통신+배터리 동시 위험"),
        ("*",        "BAD",  "CRITICAL", "*",          "LAND",     "GPS 불량+배터리 위험"),
        ("*",        "*",    "CRITICAL", "MARGINAL",   "RTL",      "배터리 위험 — 즉시 귀환"),
        ("WARNING",  "BAD",  "*",        "*",          "HOVER",    "통신+GPS 동시 저하 — 링크 회복 대기"),
        ("*",        "*",    "CRITICAL", "OK",         "RTL",      "배터리 위험 — 귀환 권장"),
        ("*",        "*",    "LOW",      "MARGINAL",   "RTL",      "배터리 부족+귀환 여유 감소"),
        ("*",        "*",    "LOW",      "*",          "CONTINUE", "배터리 부족 — 임무 단축 검토"),
        ("*",        "WEAK", "*",        "*",          "CONTINUE", "GPS 약화 감시 중"),
        ("NORMAL",   "GOOD", "GOOD",     "OK",         "CONTINUE", "정상"),
        ("*",        "*",    "*",        "*",          "CONTINUE", "정상"),  # 기본값
    ]

    # 임무 단계 라벨 / 체크리스트
    PHASE_LABELS = {
        "PRE_FLIGHT": "사전 계획", "TAKEOFF": "이륙", "TRANSIT": "이동",
        "PATROL": "정찰", "RETURN": "귀환", "LANDING": "착륙", "COMPLETE": "완료",
    }
    PHASE_CHECKLIST = {
        "TAKEOFF": [
            ("배터리 30% 이상",   lambda s: s.d_bat_p >= 30),
            ("GPS 위성 8개 이상", lambda s: s.d_sats >= 8),
            ("링크 정상",         lambda s: s.link_state == "NORMAL"),
            ("ARM 상태",          lambda s: s.d_is_armed),
        ],
        "TRANSIT": [
            ("웨이포인트 설정됨",  lambda s: len(s.planned_waypoints) > 0 or s.mission_active),
            ("귀환 여유 20% 이상", lambda s: (s.return_margin_pct or 0) >= 20
                                             or s.return_feasibility == "OK"),
        ],
        "RETURN": [
            ("홈 위치 설정됨", lambda s: s.home_lat is not None),
        ],
    }

    # 주간/야간 테마 색상
    DAY_COLORS = {"bg": "#d9d9d9", "panel_bg": "#d9d9d9", "btn_bg": "#f0f0f0",
                  "btn_fg": "#000000", "label_fg": "#000000"}
    # 중립 다크 테마 (안전도 모니터링 패널 색과 통일)
    NIGHT_COLORS = {"bg": "#0a0f18", "panel_bg": "#0e1828", "btn_bg": "#1b2a3f",
                    "btn_fg": "#d8e2f0", "label_fg": "#c8d4e4"}

    # 오버레이 패널 배치 상수 (탭 스트립 폭 / 패널 폭 / 탭-패널 여백)
    OVERLAY_TAB_W = 44     # 탭 스트립 폭
    OVERLAY_PANEL_W = 220  # 오버레이 패널 폭
    OVERLAY_MARGIN = 8     # 탭-패널 간 여백

    def __init__(self):
        super().__init__()

        self.base_dir = os.path.dirname(os.path.abspath(__file__))

        # 1) 지도 선택 팝업
        self.withdraw()

        selector = MapSelectionDialog(self)
        self.wait_window(selector)

        if not selector.result:
            print("[INFO] 지도 선택이 취소되어 프로그램을 종료합니다.")
            try:
                self.destroy()
            except Exception:
                pass
            sys.exit(0)

        self.selected_map_url, self.selected_db_name, self.selected_max_zoom = selector.result
        print("[MAP] 선택한 지도 DB:", self.selected_db_name)

        self.deiconify()

        self.title("MAVLink GCS — 임무 안전도 모니터링")
        self.geometry("1400x1000")

        self.sitl = SITLManager()
        self.drone = None

        # MAVLink 송수신 동시 접근을 막기 위한 Lock
        self.mav_lock = threading.Lock()

        self.running = True
        self.closing = False
        self.cleaned_up = False
        self.stop_requested = False

        self.mav_queue = queue.Queue()

        self.home_marker = None
        self.drone_marker = None

        self.home_icon = None
        self.drone_icon = None

        self.init_msg_sent = False
        self.gps_enable_time = None
        self.countdown_popup = None

        self.update_after_id = None

        # HUD 표시용 상태 데이터 (MAVLink 수신값 저장)
        self.d_roll = 0.0
        self.d_pitch = 0.0
        self.d_alt = 0.0
        self.d_speed = 0.0
        self.d_mode = "DISCONN"
        self.d_sats = 0
        self.d_bat_v = 0.0
        self.d_bat_p = 0
        self.d_is_armed = False
        self.d_lat = 0.0
        self.d_lon = 0.0

        # 시동 가능 여부 / 모드 변경 알림용 (제어 명령)
        self.is_ready_to_arm = False
        self.last_mode = None

        # 마우스로 드론 이동(GUIDED GoTo)
        self.target_marker = None   # 목적지(TARGET) 마커
        self.path_line = None       # 드론 -> 목적지 경로선
        self.d_yaw = 0.0            # 드론 헤딩(방향)
        self.last_draw_yaw = -1.0   # 마지막으로 그린 yaw (회전 갱신 판단용)

        # 회전을 위해 드론 이미지를 PIL 원본(RGBA)으로 보관
        self.drone_raw_img = None
        try:
            drone_img_path = os.path.join(self.base_dir, "drone.png")
            if os.path.exists(drone_img_path):
                self.drone_raw_img = Image.open(drone_img_path).convert("RGBA").resize((70, 70))
        except Exception as error:
            print("[DRONE IMG LOAD ERROR]", error)

        # 목적지(TARGET) 마커 아이콘
        self.target_icon = None
        try:
            target_img_path = os.path.join(self.base_dir, "target.png")
            if os.path.exists(target_img_path):
                self.target_icon = ImageTk.PhotoImage(
                    Image.open(target_img_path).resize((45, 45)))
        except Exception as error:
            print("[TARGET IMG LOAD ERROR]", error)

        # 가상 조이스틱 (RC 오버라이드, Mode 2 매핑) PWM 기본값(중립 1500)
        self.js_roll = 1500       # Ch1 (Aileron / Roll)
        self.js_pitch = 1500      # Ch2 (Elevator / Pitch)
        self.js_throttle = 1500   # Ch3 (Throttle)
        self.js_yaw = 1500        # Ch4 (Rudder / Yaw)

        # 수동(조이스틱) 비행 중 회피 개입 상태
        self._manual_avoidance_active = False
        self.MANUAL_AVOID_SPEED_MPS = 3.0     # 의도위치 추정용 가정 속도
        self.MANUAL_AVOID_LOOKAHEAD_S = 2.0   # 몇 초 후 위치를 미리 보고 회피 판단할지
        self.joystick_visible = tk.BooleanVar(value=False)

        # 지도 중심 자동 이동 모드 (0=없음, 1=홈 중심, 2=드론 추적)
        self.map_center_mode = tk.IntVar(value=0)
        self.last_map_center_update_time = 0

        # 착륙(시동 해제) 후 마커/이동선 자동 정리 타이머
        self.disarm_time = None

        # 비행금지구역 회피 경로계획
        self.zone_drawing_mode = False        # 금지구역 그리기 모드 on/off
        self.current_zone_vertices = []       # 그리는 중인 폴리곤 정점 [(lat,lon)]
        self.current_zone_markers = []        # 정점 표시 마커들
        self.current_zone_polygon = None      # 미리보기 폴리곤 객체
        self.no_fly_zones = []                # 확정된 금지구역들: list[list[(lat,lon)]]
        self.zone_polygons = []               # 확정 폴리곤 객체들

        # 순차 GUIDED 실행 상태머신
        self.mission_active = False
        self.mission_waypoints = []           # [(lat,lon), ...]
        self.mission_index = 0
        self.mission_alt = None

        # AUTO(MISSION 프로토콜) 비행 상태 — 동적 재계획/ESC 중단 대상에 포함
        self.auto_mission_active = False
        self._auto_seen = False               # AUTO 모드 실제 진입 확인 플래그

        # 회피 정책 상태머신 (우선순위1: REROUTE — 우회 후 원래 목표로 자동 복귀)
        self.avoidance_state = "NONE"         # "NONE" | "REROUTING" | "RESUMING"
        self.avoidance_resume_target = None   # (lat, lon, alt) 우회 끝나고 돌아갈 원래 목표
        self.avoidance_resume_kind = None     # "GUIDED" | "MISSION" (이번엔 GUIDED만 사용)
        self.in_threat_zone = False           # 현재 위협/금지구역 내부 여부(디바운스 적용)
        self.last_zone_breach_replan_time = 0.0  # 구역 진입 회피 재계획 쿨다운
        self._zone_raw_prev = False           # 직전 raw 구역판정(경계 떨림 디바운스용)
        self._zone_confirm = 0                # 같은 판정 연속 횟수
        self.last_takeoff_time = 0.0          # 이륙 명령 시각(이륙 직후 회피 보류용)
        self.TAKEOFF_GRACE_S = 10.0           # 이륙 후 이 시간 동안은 구역회피 기동 보류

        self.mission_path_line = None         # 계획 경로 폴리라인
        self._mission_guided_seen = False     # 미션 중 GUIDED 진입 확인(모드이탈 감지용)

        # ============================================================
        # 웨이포인트 미션 플래너
        # ============================================================
        self.wp_planning_mode = False         # 웨이포인트 찍기 모드
        self.planned_waypoints = []           # [(lat, lon, alt), ...]
        self.wp_markers = []                  # 지도 마커들
        self.wp_lines = []                    # 웨이포인트 간 연결선
        self.wp_default_alt = 10.0            # 기본 고도(m)

        # MAVLink MISSION 업로드 핸드셰이크 상태 (mavlink_loop에서 응답 처리)
        self._mission_upload_active = False
        self._mission_upload_items = []       # [(lat, lon, alt, cmd), ...] (seq0=HOME, seq1=이륙)

        # ============================================================
        # 위협 반경(원형 장애물)
        # ============================================================
        self.threat_radii = []                # [(lat, lon, radius_m), ...]
        self.threat_circles = []              # 지도 표시 객체(폴리곤)
        self.threat_drawing_mode = False
        self.threat_pending_center = None     # 중심 찍은 후 반경 입력 대기
        self._threat_zone_refs = []           # no_fly_zones에 넣은 위협 다각형 참조(초기화 동기화용)

        # ============================================================
        # 안전 감시 고도화 (Day 2)
        # ============================================================

        # B1: 홈 위치 (귀환 가능성 계산 기준)
        self.home_lat = None
        self.home_lon = None

        # B1: 배터리 소모율 추정 (슬라이딩 윈도우)
        self.bat_history = []            # [(timestamp, bat_p), ...]  최근 60초
        self.bat_drain_rate = 0.0        # %/초 소모율

        # B1: 귀환 가능성
        self.return_feasibility = None   # "OK" / "MARGINAL" / "IMPOSSIBLE"
        self.return_margin_pct = None    # 귀환 후 남을 배터리 %
        self.return_time_limit_s = None  # 귀환 불가까지 남은 초 (None=여유있음)

        # B2: 위험 추세 감지
        self.sats_history = []           # [(timestamp, sats), ...]
        self.link_delay_history = []     # [(timestamp, delay_s), ...]
        self.trend_gps = "→"             # "↑" / "→" / "↓"
        self.trend_link = "→"
        self.trend_battery = "→"

        # B3: 대응 매트릭스 결과
        self.recommended_action = "CONTINUE"
        self.recommended_reason = "정상"

        # B4: 동적 재계획 쿨다운 (연속 트리거 방지)
        self.last_replan_time = 0.0
        self.REPLAN_COOLDOWN_S = 30.0

        # 보고서용 최솟값/최댓값 기록
        self.min_risk_score_recorded = 100
        self.min_risk_level_recorded = "NORMAL"
        self.max_alt_recorded = 0.0
        self.max_dist_recorded = 0.0
        self.mission_start_time = None
        self.mission_id = None

        # ============================================================
        # 관심지점(POI) - Day 3
        # ============================================================
        self.poi_list = []               # [{"id","lat","lon","priority","note","time"}, ...]
        self.poi_markers = {}            # {id: marker_object}
        self.poi_counter = 0

        # 비상 대피지점(Rally Point) + 회피 정책 설정
        self.rally_points = []           # [{"id","lat","lon","alt","time"}, ...]
        self.rally_counter = 0
        self.rally_markers = {}          # {id: marker}
        self.avoidance_action_setting = AVOIDANCE_ACTION_REROUTE   # 기본값(기존 동작과 동일)

        # ============================================================
        # 전술 지도 레이어 (비행 트레일)
        # ============================================================
        self.flight_trail = []           # [(lat, lon), ...] 최근 500개
        self.trail_line = None
        self._last_trail_lat = None
        self._last_trail_lon = None

        # ============================================================
        # 임무 단계 관리
        # ============================================================
        self.mission_phase = "PRE_FLIGHT"
        self.mission_name = ""           # mission_id는 위 B 블록에서 이미 선언됨

        # ============================================================
        # 통신·GPS 동시 이상 감지
        # ============================================================
        self.anomaly_suspected = False
        self.last_anomaly_check_time = 0.0

        # ============================================================
        # 야간 모드
        # ============================================================
        self.night_mode = False
        self._orig_colors = {}      # 야간 진입 전 위젯별 원래 색 저장(복귀 시 복원용)

        # ============================================================
        # MGRS 좌표계
        # ============================================================
        self.coord_display_mode = "LATLON"   # "LATLON" / "MGRS" 토글
        self._mgrs_converter = _mgrs_lib.MGRS() if _MGRS_AVAILABLE else None

        # ============================================================
        # Safety Monitor 상태 변수 - 1차 구현
        # ============================================================
        self.last_heartbeat_time = None
        self.last_position_time = None
        self.last_gps_time = None
        self.last_sys_status_time = None

        self.link_state = "DISCONNECTED"      # DISCONNECTED / NORMAL / WARNING / CRITICAL / LOST
        self.gps_state = "UNKNOWN"            # UNKNOWN / GOOD / WEAK / BAD
        self.battery_state = "UNKNOWN"        # UNKNOWN / GOOD / LOW / CRITICAL
        self.battery_return_required = False    # 잔량 20% 이하(80% 소모) → 즉시 귀환(RTL) 규정 도달
        self._battery_return_warned = False     # 착륙 권장 팝업 1회만 띄우기 위한 플래그
        self._battery_notify_50 = False       # 50% 권고 알림 1회 표시 플래그
        self._battery_notify_30 = False       # 30% 경고 알림 1회 표시 플래그

        self.flight_risk_score = 100
        self.risk_level = "NORMAL"            # NORMAL / WARNING / CRITICAL / LOST
        self.risk_reasons = []

        self.last_safety_eval_time = 0.0
        self.SAFETY_EVAL_INTERVAL_S = 1.0

        self.event_log_enabled = True
        self.safety_events = []
        self.last_event_times = {}
        self.EVENT_COOLDOWN_S = 5.0

        self.safety_log_file_path = None

        self.create_widgets()
        self.register_handlers()

        self.mav_thread = threading.Thread(target=self.mavlink_loop, daemon=True)
        self.mav_thread.start()

        self.js_loop()            # 조이스틱 RC 오버라이드 송신 루프(100ms) 시작
        self.update_drone_data()

    # ------------------------------------------------------------
    # SQLite DB 초기화
    # ------------------------------------------------------------

    def init_sqlite_db(self, path):
        """
        선택된 지도용 SQLite DB 파일을 초기화한다.
        tkintermapview가 타일을 저장할 tile_data 테이블이 없으면 생성한다.
        """
        try:
            conn = sqlite3.connect(path)
            conn.execute(
                "CREATE TABLE IF NOT EXISTS tile_data ("
                "server TEXT, zoom INTEGER, x INTEGER, y INTEGER, "
                "image BLOB, PRIMARY KEY (server, zoom, x, y))"
            )
            conn.commit()
            conn.close()
        except Exception as error:
            print(f"[DB INIT ERROR] {error}")

    # ------------------------------------------------------------
    # Tkinter 안전 호출 보조 함수
    # ------------------------------------------------------------

    def run_on_ui(self, callback, allow_when_closing=False):
        """Worker Thread에서 Tkinter UI를 직접 건드리지 않도록 after()로 예약한다."""
        try:
            if self.winfo_exists() and (allow_when_closing or not self.closing):
                self.after(0, callback)
        except Exception:
            pass

    def set_status(self, text):
        """하단 상태바 메시지를 안전하게 갱신한다."""
        if self.closing:
            return

        self.run_on_ui(lambda: self.status_var.set(text))

    # ------------------------------------------------------------
    # GUI 구성
    # ------------------------------------------------------------

    def create_widgets(self):
        """GUI 구성 요소 생성"""
        # 긴급 대응 패널을 화면 최상단에 가장 먼저 고정(전체 폭)
        self.create_emergency_panel()

        # UI 마감: SITL 제어바를 긴급 패널 바로 아래 별도 줄로 배치(겹침 해소).
        # 보조 도구 모음(secondary toolbar) 톤으로 낮춰 표시한다.
        sitl_bar = tk.Frame(self, bg="#1e2d3d", height=40)
        sitl_bar.pack(side="top", fill="x")
        sitl_bar.pack_propagate(False)

        bar_font = ("맑은 고딕", 10)
        bar_font_b = ("맑은 고딕", 10, "bold")

        tk.Label(sitl_bar, text="시작 위치", bg="#1e2d3d", fg="#9db2c8",
                 font=bar_font).pack(side="left", padx=(12, 6))

        tk.Label(sitl_bar, text="LAT", bg="#1e2d3d", fg="#cadcfc",
                 font=bar_font).pack(side="left", padx=(4, 2))
        self.lat_entry = tk.Entry(sitl_bar, width=12, font=bar_font,
                                  bg="#0e1828", fg="#cadcfc",
                                  insertbackground="#cadcfc",
                                  relief="flat", bd=4)
        self.lat_entry.insert(0, "37.565575")
        self.lat_entry.pack(side="left")

        tk.Label(sitl_bar, text="LON", bg="#1e2d3d", fg="#cadcfc",
                 font=bar_font).pack(side="left", padx=(10, 2))
        self.lon_entry = tk.Entry(sitl_bar, width=12, font=bar_font,
                                  bg="#0e1828", fg="#cadcfc",
                                  insertbackground="#cadcfc",
                                  relief="flat", bd=4)
        self.lon_entry.insert(0, "126.977987")
        self.lon_entry.pack(side="left")

        self.toggle_btn = tk.Button(
            sitl_bar, text="▶ SITL 시작",
            font=bar_font_b,
            bg="#2ecc71", fg="#0e1828",
            activebackground="#27ae60",
            relief="flat", bd=0, padx=14,
            cursor="hand2",
            command=self.toggle_sitl)
        self.toggle_btn.pack(side="left", padx=(12, 0))

        # 우측: 현재 좌표 표시 (읽기 전용)
        self.sitl_bar_coord_label = tk.Label(
            sitl_bar, text="", bg="#1e2d3d", fg="#9db2c8",
            font=("Consolas", 9))
        self.sitl_bar_coord_label.pack(side="right", padx=12)

        self.status_var = tk.StringVar(value="SITL 실행 대기 중...")

        self.status_bar = tk.Label(
            self,
            textvariable=self.status_var,
            bd=1,
            relief="sunken",
            anchor="w",
            font=("맑은 고딕", 11)
        )
        self.status_bar.pack(side="bottom", fill="x")

        # MAP DB 경로 준비 (선택한 지도별로 다른 DB 파일 사용)
        map_db_dir = os.path.join(self.base_dir, "map")
        if not os.path.exists(map_db_dir):
            os.makedirs(map_db_dir)
            print(f"[MAP DB] 폴더가 없어서 생성했습니다: {map_db_dir}")

        database_path = os.path.join(map_db_dir, self.selected_db_name)

        self.init_sqlite_db(database_path)

        print("[MAP DB] database_path:", database_path)
        print("[MAP DB] use_database_only:", self.USE_DATABASE_ONLY)

        self.map_widget = tkintermapview.TkinterMapView(
            self,
            width=800,
            height=600,
            corner_radius=0,
            database_path=database_path,
            use_database_only=self.USE_DATABASE_ONLY
        )
        self.map_widget.pack(fill="both", expand=True)

        self.map_widget.set_tile_server(
            self.selected_map_url,
            max_zoom=self.selected_max_zoom
        )

        self.map_widget.set_position(37.565575, 126.977987)
        self.map_widget.set_zoom(18)

        # 지도 왼쪽 클릭 시 드론 이동(GUIDED GoTo) 명령 연결
        try:
            self.map_widget.add_left_click_map_command(self.on_map_left_click)
        except Exception as error:
            print("[MAP CLICK BIND ERROR]", error)

        # 지도 우클릭 메뉴 → 우선순위별 POI 추가
        # tkintermapview API는 항목을 하나씩 등록하며, pass_coords=True면
        # 콜백이 클릭 좌표 (lat, lon) 튜플을 인자 하나로 받는다.
        try:
            self.map_widget.add_right_click_menu_command(
                label="POI 추가 (긴급)",
                command=lambda coord: self.add_poi(coord[0], coord[1], priority="HIGH"),
                pass_coords=True)
            self.map_widget.add_right_click_menu_command(
                label="POI 추가 (일반)",
                command=lambda coord: self.add_poi(coord[0], coord[1], priority="NORMAL"),
                pass_coords=True)
            self.map_widget.add_right_click_menu_command(
                label="POI 추가 (참고)",
                command=lambda coord: self.add_poi(coord[0], coord[1], priority="LOW"),
                pass_coords=True)
            self.map_widget.add_right_click_menu_command(
                label="Rally Point 추가",
                command=lambda coord: self.add_rally_point(coord[0], coord[1]),
                pass_coords=True)
        except Exception as error:
            print("[RIGHT CLICK MENU ERROR]", error)

        # 줌 레벨 표시 라벨 (좌상단)
        self.zoom_label = tk.Label(
            self.map_widget,
            text="Zoom: --",
            font=("맑은 고딕", 11, "bold"),
            fg="blue",
            bg="white",
            relief="solid",
            bd=1
        )
        self.zoom_label.place(relx=0.0, y=100, x=10, anchor="nw")

        # HUD 위젯 (지도 위 오른쪽 상단) - 추가
        # UI 마감: HUD 위에 미니 타이틀을 붙여 오버레이 패널과 톤을 맞춘다.
        self.hud_title = tk.Label(
            self.map_widget, text="비행 상태",
            bg="#0e1828", fg="#9db2c8",
            font=("맑은 고딕", 8, "bold"),
            padx=6, pady=2)
        # HUD/미니뱃지는 우측 탭 스트립과 겹치지 않게 탭 폭만큼 왼쪽으로 비켜 배치.
        # 타이틀은 지도 안쪽(y>0)에 둬서 상단 경계에 잘리지 않게 한다.
        self.hud_title.place(relx=1.0, rely=0.0, x=-(self.OVERLAY_TAB_W + 8), y=4, anchor="ne")

        self.hud = GCSHUD(self.map_widget)
        self.hud.place(relx=1.0, rely=0.0, x=-(self.OVERLAY_TAB_W + 8), y=30, anchor="ne")

        # 가상 조이스틱 패널 (지도 위 하단 중앙, 기본은 숨김)
        self.js_panel_frame = tk.Frame(self.map_widget, bg="#2c3e50", bd=2, relief="raised")
        self.create_virtual_joystick()

        # (SITL 제어바는 위에서 별도 줄로 이미 배치됨 — lat/lon/toggle 위젯 생성 완료)

        # 오버레이 탭/패널은 map_widget 생성 이후 마지막에 배치한다.
        self.create_control_panel()

        # 탭 스트립이 렌더된 뒤 실제 폭을 측정해 HUD/미니뱃지가 겹치지 않게 재배치
        self.after(250, self._fit_right_overlays)

    def _fit_right_overlays(self):
        """우측 탭 스트립의 실제 폭을 측정해 HUD/미니뱃지를 탭과 안 겹치게 옮긴다."""
        try:
            if not self.winfo_exists():
                return
            tab_w = self.tab_strip.winfo_width()
            if tab_w <= 1:           # 아직 렌더 전이면 잠시 후 재시도
                self.after(150, self._fit_right_overlays)
                return
            clear = tab_w + 8        # 탭 폭 + 여백만큼 왼쪽으로
            self.hud_title.place_configure(x=-clear)
            self.hud.place_configure(x=-clear)
            self.mini_badge.place_configure(x=-clear)
        except Exception:
            pass

    # ------------------------------------------------------------
    # 지도 오버레이 탭/제어 패널
    # ------------------------------------------------------------

    def create_control_panel(self):
        """
        [UI 개편] 기존 오른쪽 스크롤 패널을 제거하고
        지도 위 오버레이 탭 패널로 교체한다.
        지도는 전체 폭을 사용하며, 탭 클릭으로 패널을 열고 닫는다.
        """
        self._active_panel_key = None
        self._overlay_panels = {}
        self._tab_buttons = {}
        self._overlay_panel_pos = {}   # key -> (x, y) 드래그로 옮긴 위치 기억

        badge_font_big = ("맑은 고딕", 13, "bold")
        badge_font_sm = ("맑은 고딕", 9)
        badge_bg = "#0e1828"
        badge_border = "#2a3f5a"

        # 미니 안전 뱃지: 전체 배경을 위험색으로 칠하지 않고
        # 좌측 4px 컬러 스트립으로만 위험 단계를 표시 (텍스트 잘림 방지 위해 폭 확장)
        self.mini_badge = tk.Frame(
            self.map_widget, bg=badge_bg,
            bd=0, highlightbackground=badge_border, highlightthickness=1)
        self.mini_badge.place(relx=1.0, rely=0.0, x=-(self.OVERLAY_TAB_W + 8), y=262, anchor="ne")

        self.mini_badge_strip = tk.Frame(self.mini_badge, bg="#34d399", width=4)
        self.mini_badge_strip.pack(side="left", fill="y")

        mini_inner = tk.Frame(self.mini_badge, bg=badge_bg)
        mini_inner.pack(side="left", fill="both", expand=True)

        self.mini_score_label = tk.Label(
            mini_inner, text="-- / 100",
            bg=badge_bg, fg="#34d399",
            font=badge_font_big)
        self.mini_score_label.pack(padx=10, pady=(6, 0), anchor="w")

        self.mini_level_label = tk.Label(
            mini_inner, text="대기 중",
            bg=badge_bg, fg="#9db2c8",
            font=badge_font_sm)
        self.mini_level_label.pack(padx=10, anchor="w")

        self.mini_return_label = tk.Label(
            mini_inner, text="귀환여유: 계산 중",
            bg=badge_bg, fg="#9db2c8",
            font=badge_font_sm,
            wraplength=150, justify="left")
        self.mini_return_label.pack(padx=10, pady=(0, 6), anchor="w")

        tab_bg = "#1e2d3d"
        tab_active = "#34d399"
        tab_idle = "#9db2c8"

        self.tab_strip = tk.Frame(
            self.map_widget, bg=tab_bg,
            bd=1, relief="solid",
            highlightbackground="#2a3f5a", highlightthickness=1)
        self.tab_strip.place(relx=1.0, rely=0.5, anchor="e", x=-1)

        # 아이콘은 한글 글자 대신 기호 문자(▲■●≡)로 — 라벨 글자와 중복 노출 방지
        tab_defs = [
            ("safe", "▲", "안전"),
            ("mission", "■", "미션"),
            ("poi", "●", "POI"),
            ("setting", "≡", "설정"),
        ]

        for key, icon, label in tab_defs:
            btn = tk.Button(
                self.tab_strip,
                text=f"{icon}\n{label}",
                font=("맑은 고딕", 9, "bold"),
                fg=tab_idle,
                bg=tab_bg,
                relief="flat",
                bd=0,
                width=5,
                pady=8,
                cursor="hand2",
                activebackground="#243448",
                command=lambda k=key: self.toggle_overlay_panel(k))
            btn.pack(fill="x", pady=1)
            self._tab_buttons[key] = btn

        panel_w = self.OVERLAY_PANEL_W
        panel_bg = "#0e1828"
        panel_fg = "#cadcfc"
        sec_bg = "#11202f"
        border = "#2a3f5a"
        font_sm = ("맑은 고딕", 9)
        font_bold = ("맑은 고딕", 9, "bold")

        def make_panel(key):
            """공통 오버레이 Frame을 숨김 상태로 생성한다."""
            frame = tk.Frame(
                self.map_widget,
                bg=panel_bg, bd=1, relief="solid",
                highlightbackground=border, highlightthickness=1,
                width=panel_w)
            frame.place_configure(
                relx=1.0,
                x=-(self.OVERLAY_TAB_W + self.OVERLAY_PANEL_W + self.OVERLAY_MARGIN),
                rely=0.05, anchor="ne")
            frame.place_forget()
            self._overlay_panels[key] = frame
            return frame

        def add_header(parent, title, key):
            """오버레이 패널 헤더를 만든다. (헤더를 드래그하면 패널을 옮길 수 있다)"""
            hdr = tk.Frame(parent, bg=sec_bg, cursor="fleur")
            hdr.pack(fill="x")
            title_lbl = tk.Label(hdr, text=title, bg=sec_bg, fg=panel_fg,
                                 font=font_bold, anchor="w", cursor="fleur")
            title_lbl.pack(side="left", padx=8, pady=6)
            tk.Button(hdr, text="×", bg=sec_bg, fg="#9db2c8",
                      relief="flat", bd=0, font=font_sm,
                      cursor="hand2",
                      command=lambda k=key: self.toggle_overlay_panel(k)
                      ).pack(side="right", padx=6)
            tk.Frame(parent, bg=border, height=1).pack(fill="x")
            # 헤더(빈 영역 + 제목 라벨)를 잡고 드래그하면 패널 이동
            self._bind_panel_drag(parent, [hdr, title_lbl], key)

        def section_label(parent, text):
            tk.Label(parent, text=text,
                     bg=panel_bg, fg="#9db2c8",
                     font=("맑은 고딕", 8), anchor="w").pack(
                         fill="x", pady=(0, 2))

        # 패널 A: 안전 대시보드
        pa = make_panel("safe")
        add_header(pa, "임무 안전도", "safe")

        body_a = tk.Frame(pa, bg=panel_bg)
        body_a.pack(fill="both", expand=True, padx=10, pady=8)

        phase_bar_frame = tk.Frame(body_a, bg=panel_bg)
        phase_bar_frame.pack(fill="x", pady=(0, 6))
        self.phase_pips = []
        for _ in range(5):
            pip = tk.Frame(phase_bar_frame, bg=border, height=4, width=28)
            pip.pack(side="left", padx=1)
            self.phase_pips.append(pip)
        self.phase_mini_label = tk.Label(
            phase_bar_frame, text="사전 계획",
            bg=panel_bg, fg="#9db2c8", font=("맑은 고딕", 8))
        self.phase_mini_label.pack(side="left", padx=(4, 0))

        score_row = tk.Frame(body_a, bg=panel_bg)
        score_row.pack(fill="x", pady=(0, 6))
        self.risk_score_label = tk.Label(
            score_row, text="-- / 100",
            bg=panel_bg, fg="#34d399",
            font=("맑은 고딕", 22, "bold"))
        self.risk_score_label.pack(side="left")
        self.risk_level_label = tk.Label(
            score_row, text="UNKNOWN",
            bg="#1a2a1a", fg="#34d399",
            font=("맑은 고딕", 9, "bold"),
            padx=6, pady=2, relief="solid", bd=1)
        self.risk_level_label.pack(side="right", padx=(0, 4))

        def stat_row(parent, label_text, attr_name):
            row = tk.Frame(parent, bg=panel_bg)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=label_text, bg=panel_bg,
                     fg="#9db2c8", font=font_sm,
                     width=5, anchor="w").pack(side="left")
            lbl = tk.Label(row, text="--", bg=panel_bg,
                           fg=panel_fg, font=font_sm, anchor="w")
            lbl.pack(side="left")
            setattr(self, attr_name, lbl)

        stat_row(body_a, "링크", "link_state_label")
        stat_row(body_a, "GPS", "gps_state_label")
        stat_row(body_a, "배터리", "battery_state_label")

        tk.Frame(body_a, bg=border, height=1).pack(fill="x", pady=4)

        self.return_label = tk.Label(
            body_a, text="귀환여유: 계산 중",
            bg=panel_bg, fg="#9db2c8", font=font_sm, anchor="w")
        self.return_label.pack(fill="x")

        self.trend_label = tk.Label(
            body_a, text="링크→  GPS→  배터리→",
            bg=panel_bg, fg="#9db2c8", font=font_sm, anchor="w")
        self.trend_label.pack(fill="x", pady=(2, 0))

        tk.Frame(body_a, bg=border, height=1).pack(fill="x", pady=4)

        self.action_label = tk.Label(
            body_a, text="권장: CONTINUE — 정상",
            bg="#0d2d1a", fg="#34d399",
            font=font_bold, anchor="w",
            wraplength=190, justify="left",
            padx=6, pady=4)
        self.action_label.pack(fill="x")

        self.action_btn = tk.Button(
            body_a, text="권장 실행",
            bg="#243448", fg=panel_fg,
            font=font_sm, relief="flat", bd=1,
            cursor="hand2",
            command=self._execute_recommended_action)
        self.action_btn.pack(fill="x", pady=(4, 0))

        tk.Frame(body_a, bg=border, height=1).pack(fill="x", pady=4)

        self.risk_reason_label = tk.Label(
            body_a, text="최근 경고: 없음",
            bg=panel_bg, fg="#9db2c8",
            font=("맑은 고딕", 8), anchor="w",
            wraplength=190, justify="left")
        self.risk_reason_label.pack(fill="x")

        # 패널 B: 미션 플래너
        pb = make_panel("mission")
        add_header(pb, "미션 플래너", "mission")

        body_b = tk.Frame(pb, bg=panel_bg)
        body_b.pack(fill="both", expand=True, padx=10, pady=8)

        def btn_row(*btns):
            """여러 버튼을 한 줄에 가로 배치한다."""
            row = tk.Frame(body_b, bg=panel_bg)
            row.pack(fill="x", pady=2)
            for text, color, cmd in btns:
                tk.Button(
                    row, text=text,
                    bg=color, fg="#fff" if color != "#243448" else panel_fg,
                    font=font_sm, relief="flat", bd=1, cursor="hand2",
                    command=cmd
                ).pack(side="left", expand=True, fill="x", padx=1)

        section_label(body_b, "기체 제어")
        btn_row(
            ("ARM", "#e67e22", self.cmd_arm),
            ("DISARM", "#243448", self.cmd_disarm),
        )
        btn_row(
            ("TAKEOFF", "#2980b9", self.cmd_takeoff_popup),
            ("RTL", "#e74c3c", lambda: self.set_drone_mode("RTL")),
        )

        self.arm_status_label = tk.Label(
            body_b, text="상태: 연결 대기중",
            bg="#243448", fg="#cadcfc",
            font=font_sm, anchor="w", padx=4, pady=2)
        self.arm_status_label.pack(fill="x", pady=2)

        tk.Frame(body_b, bg=border, height=1).pack(fill="x", pady=4)

        section_label(body_b, "비행 모드")
        mode_row1 = tk.Frame(body_b, bg=panel_bg)
        mode_row1.pack(fill="x", pady=1)
        mode_row2 = tk.Frame(body_b, bg=panel_bg)
        mode_row2.pack(fill="x", pady=1)
        for name, row in [
            ("GUIDED", mode_row1), ("LOITER", mode_row1), ("AUTO", mode_row1),
            ("STABILIZE", mode_row2), ("RTL", mode_row2), ("LAND", mode_row2),
        ]:
            tk.Button(row, text=name, font=("맑은 고딕", 8),
                      bg="#243448", fg=panel_fg,
                      relief="flat", bd=1, cursor="hand2",
                      command=lambda n=name: self.set_drone_mode(n)
                      ).pack(side="left", expand=True, fill="x", padx=1)

        tk.Frame(body_b, bg=border, height=1).pack(fill="x", pady=4)

        section_label(body_b, "미션 계획")
        self.wp_plan_btn = tk.Button(
            body_b, text="미션 계획 (클릭으로 WP 추가)",
            bg="#243448", fg=panel_fg,
            font=font_sm, relief="flat", bd=1, cursor="hand2",
            command=self.toggle_wp_planning)
        self.wp_plan_btn.pack(fill="x", pady=1)

        btn_row(
            ("AUTO 실행", "#2980b9", self.upload_and_run_mission),
            ("WP 초기화", "#243448", self.clear_planned_waypoints),
        )
        btn_row(
            ("저장", "#243448", self.save_mission_dialog),
            ("불러오기", "#243448", self.load_mission_dialog),
            ("마지막 삭제", "#243448", self.remove_last_waypoint),
        )

        tk.Frame(body_b, bg=border, height=1).pack(fill="x", pady=4)

        section_label(body_b, "전술 구역")
        self.zone_draw_btn = tk.Button(
            body_b, text="금지구역 그리기",
            bg="#243448", fg=panel_fg,
            font=font_sm, relief="flat", bd=1, cursor="hand2",
            command=self.toggle_zone_drawing)
        self.zone_draw_btn.pack(fill="x", pady=1)

        btn_row(
            ("구역 확정", "#e74c3c", self.finalize_zone),
            ("구역 초기화", "#243448", self.clear_zones),
        )

        self.threat_btn = tk.Button(
            body_b, text="위협 반경 추가",
            bg="#243448", fg=panel_fg,
            font=font_sm, relief="flat", bd=1, cursor="hand2",
            command=self.toggle_threat_drawing)
        self.threat_btn.pack(fill="x", pady=1)

        tk.Button(body_b, text="위협 전체 삭제",
                  bg="#243448", fg=panel_fg,
                  font=font_sm, relief="flat", bd=1, cursor="hand2",
                  command=self.clear_threat_radii
                  ).pack(fill="x", pady=1)

        # 패널 C: POI
        pc = make_panel("poi")
        add_header(pc, "관심지점(POI)", "poi")

        body_c = tk.Frame(pc, bg=panel_bg)
        body_c.pack(fill="both", expand=True, padx=8, pady=6)

        tk.Label(body_c, text="지도 우클릭으로 POI 추가",
                 bg=panel_bg, fg="#9db2c8",
                 font=("맑은 고딕", 8), anchor="w").pack(fill="x", pady=(0, 4))

        tk.Button(body_c, text="전체 POI 삭제",
                  bg="#3d0d0d", fg="#f47174",
                  font=font_sm, relief="flat", bd=1, cursor="hand2",
                  command=lambda: [self.remove_poi(p["id"]) for p in list(self.poi_list)]
                  ).pack(fill="x", pady=(0, 4))

        poi_canvas = tk.Canvas(body_c, height=180, bg="#0e1828",
                               highlightthickness=0)
        poi_sb = tk.Scrollbar(body_c, orient="vertical", command=poi_canvas.yview)
        poi_canvas.configure(yscrollcommand=poi_sb.set)
        poi_sb.pack(side="right", fill="y")
        poi_canvas.pack(side="left", fill="both", expand=True)

        self.poi_panel_inner = tk.Frame(poi_canvas, bg="#0e1828")
        poi_canvas.create_window((0, 0), window=self.poi_panel_inner, anchor="nw")
        self.poi_panel_inner.bind(
            "<Configure>",
            lambda e: poi_canvas.configure(
                scrollregion=poi_canvas.bbox("all")))

        tk.Label(self.poi_panel_inner, text="POI 없음",
                 fg="#9db2c8", bg="#0e1828",
                 font=("맑은 고딕", 9)).pack(pady=8)

        # 패널 D: 설정
        pd_ = make_panel("setting")
        add_header(pd_, "설정", "setting")

        body_d = tk.Frame(pd_, bg=panel_bg)
        body_d.pack(fill="both", expand=True, padx=10, pady=8)

        # 좌표 표시 모드 (위경도 ↔ MGRS)
        section_label(body_d, "좌표 표시")
        self.coord_display_label = tk.Label(
            body_d, text="-- (위경도)",
            bg=panel_bg, fg="#cadcfc",
            font=("Consolas", 9), anchor="w")
        self.coord_display_label.pack(fill="x", pady=(0, 2))
        tk.Button(
            body_d, text="좌표 형식 전환 (위경도 ↔ MGRS)",
            bg="#243448", fg=panel_fg,
            font=font_sm, relief="flat", bd=1, cursor="hand2",
            command=self.toggle_coord_display
        ).pack(fill="x", pady=(0, 6))
        tk.Frame(body_d, bg=border, height=1).pack(fill="x", pady=6)

        section_label(body_d, "지도 중심")
        for text, value in [("이동 없음", 0), ("홈 중심", 1), ("드론 추적", 2)]:
            tk.Radiobutton(
                body_d, text=text,
                value=value, variable=self.map_center_mode,
                bg=panel_bg, fg=panel_fg,
                selectcolor="#17293f",
                activebackground=panel_bg,
                font=font_sm, anchor="w",
                command=self.on_center_mode_change
            ).pack(fill="x")

        tk.Frame(body_d, bg=border, height=1).pack(fill="x", pady=6)

        section_label(body_d, "회피 정책")
        self.avoidance_action_var = tk.StringVar(value=self.avoidance_action_setting)
        for action_key, label_text in AVOIDANCE_ACTION_LABELS.items():
            tk.Radiobutton(
                body_d, text=label_text,
                value=action_key, variable=self.avoidance_action_var,
                bg=panel_bg, fg=panel_fg,
                selectcolor="#17293f",
                activebackground=panel_bg,
                font=font_sm, anchor="w",
                command=self.on_avoidance_action_change
            ).pack(fill="x")

        tk.Frame(body_d, bg=border, height=1).pack(fill="x", pady=6)

        section_label(body_d, "가상 조이스틱")
        tk.Checkbutton(
            body_d, text="조이스틱 표시 / 조종",
            variable=self.joystick_visible,
            bg=panel_bg, fg=panel_fg,
            selectcolor="#17293f",
            activebackground=panel_bg,
            font=font_sm, anchor="w",
            command=self.toggle_joystick_view
        ).pack(fill="x")

        tk.Frame(body_d, bg=border, height=1).pack(fill="x", pady=6)

        tk.Button(
            body_d, text="야간 모드 전환 (N)",
            bg="#2d0000", fg="#ff6666",
            font=font_sm, relief="flat", bd=1, cursor="hand2",
            command=self.toggle_night_mode
        ).pack(fill="x", pady=2)

        tk.Button(
            body_d, text="비행 트레일 지우기",
            bg="#243448", fg=panel_fg,
            font=font_sm, relief="flat", bd=1, cursor="hand2",
            command=self.clear_flight_trail
        ).pack(fill="x", pady=2)

        tk.Frame(body_d, bg=border, height=1).pack(fill="x", pady=6)

        section_label(body_d, "기타")
        tk.Button(
            body_d, text="배터리 초기화",
            bg="#243448", fg=panel_fg,
            font=font_sm, relief="flat", bd=1, cursor="hand2",
            command=self.cmd_battery_reset
        ).pack(fill="x", pady=2)

        self.bottom_info_bar = tk.Frame(
            self.map_widget, bg="#11202f",
            bd=1, relief="solid",
            highlightbackground=border, highlightthickness=1)
        self.bottom_info_bar.place(relx=0.0, rely=1.0, anchor="sw", y=-1)

        self.bottom_msn_label = tk.Label(
            self.bottom_info_bar, text="MSN: --",
            bg="#11202f", fg="#9db2c8",
            font=("맑은 고딕", 8))
        self.bottom_msn_label.pack(side="left", padx=8, pady=3)

        self.bottom_phase_label = tk.Label(
            self.bottom_info_bar, text="단계: 사전 계획",
            bg="#11202f", fg="#9db2c8",
            font=("맑은 고딕", 8))
        self.bottom_phase_label.pack(side="left", padx=8, pady=3)

        self.bottom_wp_label = tk.Label(
            self.bottom_info_bar, text="WP: --",
            bg="#11202f", fg="#9db2c8",
            font=("맑은 고딕", 8))
        self.bottom_wp_label.pack(side="left", padx=8, pady=3)

        tk.Label(
            self.bottom_info_bar,
            text="F1=RTL  F2=HOVER  F3=LAND  N=야간  M=미션계획",
            bg="#11202f", fg="#2a3f5a",
            font=("맑은 고딕", 8)
        ).pack(side="left", padx=8, pady=3)

    def toggle_overlay_panel(self, key):
        """
        탭에 해당하는 오버레이 패널을 열고 닫는다.
        같은 탭을 다시 클릭하면 닫고, 다른 탭을 클릭하면 기존 패널을 닫는다.
        """
        tab_bg = "#1e2d3d"
        tab_active = "#34d399"
        tab_idle = "#9db2c8"

        if self._active_panel_key == key:
            self._overlay_panels[key].place_forget()
            self._tab_buttons[key].config(fg=tab_idle, bg=tab_bg)
            self._active_panel_key = None
            return

        if self._active_panel_key is not None:
            self._overlay_panels[self._active_panel_key].place_forget()
            self._tab_buttons[self._active_panel_key].config(
                fg=tab_idle, bg=tab_bg)

        panel = self._overlay_panels[key]
        if key in self._overlay_panel_pos:
            # 사용자가 드래그로 옮겨둔 위치가 있으면 그 위치로 복원
            nx, ny = self._overlay_panel_pos[key]
            panel.place(relx=0.0, rely=0.0, x=nx, y=ny, anchor="nw")
        else:
            # 기본 위치: 좌상단(줌 라벨 아래) — 우측 HUD/미니뱃지/탭 클러스터와 분리
            panel.place(relx=0.0, rely=0.0, x=12, y=130, anchor="nw")
        panel.lift()
        self._tab_buttons[key].config(fg=tab_active, bg="#17293f")
        self._active_panel_key = key

    def _bind_panel_drag(self, panel, handles, key):
        """패널 헤더(handles)를 드래그하면 패널을 지도 안에서 이동시킨다."""
        state = {}

        def on_press(event):
            state["mx"] = event.x_root
            state["my"] = event.y_root
            state["px"] = panel.winfo_x()
            state["py"] = panel.winfo_y()
            panel.lift()

        def on_drag(event):
            if "mx" not in state:
                return
            nx = state["px"] + (event.x_root - state["mx"])
            ny = state["py"] + (event.y_root - state["my"])
            # 지도 영역 안으로 클램프
            mw = self.map_widget.winfo_width()
            mh = self.map_widget.winfo_height()
            pw = panel.winfo_width()
            ph = panel.winfo_height()
            nx = max(0, min(nx, max(0, mw - pw)))
            ny = max(0, min(ny, max(0, mh - ph)))
            panel.place_configure(relx=0.0, rely=0.0, x=nx, y=ny, anchor="nw")
            self._overlay_panel_pos[key] = (nx, ny)

        for handle in handles:
            handle.bind("<Button-1>", on_press)
            handle.bind("<B1-Motion>", on_drag)

    # ------------------------------------------------------------
    # 드론 제어 명령
    # ------------------------------------------------------------

    def set_drone_mode(self, mode_name):
        """비행 모드를 변경한다."""
        if self.drone is None:
            self.set_status("드론이 연결되지 않았습니다.")
            return
        # 미션 비행 중 사용자가 GUIDED 외 모드를 고르면 takeover로 보고 경로 비행 중단
        if self.mission_active and mode_name.strip().upper() != "GUIDED":
            self.stop_mission()
            self.set_status("모드 변경으로 경로 비행을 중단했습니다.")
        try:
            with self.mav_lock:
                self.drone.set_mode(mode_name.upper())
            self.set_status(f"모드 변경 명령 전송: {mode_name}")
            print(f"[명령] 비행 모드 변경: {mode_name}")
        except Exception as error:
            print(f"[MODE ERROR] 모드 변경 실패: {error}")
            self.set_status(f"모드 변경 실패: {error}")

    def cmd_arm(self):
        """시동(ARM) 명령. 허용 모드에서만 전송한다."""
        if self.drone is None:
            self.set_status("드론이 연결되지 않았습니다.")
            return
        allowed_modes = ["STABILIZE", "LOITER", "GUIDED", "ALT_HOLD"]
        current_mode = self.d_mode.strip().upper()
        if current_mode in allowed_modes:
            try:
                with self.mav_lock:
                    self.drone.mav.command_long_send(
                        self.drone.target_system, self.drone.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                        1, 0, 0, 0, 0, 0, 0)
                print(f"[명령] {current_mode} 모드에서 시동(ARM) 명령 전송")
                self.set_status("시동(ARM) 명령 전송")
            except Exception as error:
                print("[ARM ERROR]", error)
        else:
            tk.messagebox.showwarning(
                "시동 불가",
                f"현재 [{current_mode}] 모드에서는 시동을 걸 수 없습니다.\n"
                f"STABILIZE, LOITER, GUIDED 등으로 모드를 변경해 주세요.")

    def cmd_disarm(self):
        """시동 해제(DISARM) 명령."""
        if self.drone is None:
            self.set_status("드론이 연결되지 않았습니다.")
            return
        try:
            with self.mav_lock:
                self.drone.mav.command_long_send(
                    self.drone.target_system, self.drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    0, 0, 0, 0, 0, 0, 0)
            print("[명령] 시동 해제(DISARM) 명령 전송")
            self.set_status("시동 해제(DISARM) 명령 전송")
        except Exception as error:
            print("[DISARM ERROR]", error)

    def cmd_takeoff_popup(self):
        """이륙 고도 입력 팝업을 띄운다."""
        if self.drone is None:
            tk.messagebox.showwarning("연결 오류", "드론이 연결되지 않았습니다.")
            return
        TakeoffDialog(self, self.send_takeoff_cmd)

    def send_takeoff_cmd(self, altitude):
        """
        이륙 명령을 전송한다. GUIDED 모드 전환 → 시동 보장 → 이륙 순서로
        진행하며, UI를 막지 않도록 워커 스레드에서 처리한다.
        (모달 팝업 도중 messagebox/메인스레드 sleep로 인해 이륙이 막히던 문제 해결)
        """
        if self.drone is None:
            return
        self.set_status(f"이륙 준비 중... (목표 고도 {altitude:.0f}m)")

        def _worker():
            try:
                # 1) GUIDED 모드로 전환 (이륙은 GUIDED에서만 가능)
                with self.mav_lock:
                    try:
                        self.drone.set_mode(self.drone.mode_mapping()["GUIDED"])
                    except Exception:
                        self.drone.set_mode("GUIDED")
                time.sleep(0.4)

                # 2) 시동 보장 (이미 ARM 상태면 무시됨)
                with self.mav_lock:
                    self.drone.mav.command_long_send(
                        self.drone.target_system, self.drone.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                        1, 0, 0, 0, 0, 0, 0)
                time.sleep(0.6)

                # 3) 이륙
                with self.mav_lock:
                    self.drone.mav.command_long_send(
                        self.drone.target_system, self.drone.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                        0, 0, 0, 0, 0, 0, altitude)
                # 이륙 직후 구역회피가 상승을 가로채지 않도록 유예 시작점 기록
                self.last_takeoff_time = time.time()
                self.run_on_ui(lambda: self.set_status(f"이륙 명령 전송: 고도 {altitude:.0f}m"))
                print(f"[명령] GUIDED+시동+이륙 전송 (고도: {altitude}m)")
            except Exception as error:
                print("[TAKEOFF ERROR]", error)
                self.run_on_ui(lambda: self.set_status(f"이륙 실패: {error}"))

        threading.Thread(target=_worker, daemon=True).start()

    def cmd_battery_reset(self):
        """배터리 잔량을 100%로 초기화한다."""
        if self.drone is None:
            self.set_status("드론이 연결되지 않았습니다.")
            return
        if self.d_is_armed:
            confirm = tk.messagebox.askyesno(
                "배터리 초기화 경고",
                "현재 드론이 시동(ARM) 상태입니다.\n"
                "비행 중에 배터리를 100%로 강제 초기화하겠습니까?")
        else:
            confirm = tk.messagebox.askyesno(
                "배터리 초기화",
                "배터리 소모량을 리셋하고 100%로 다시 채우겠습니까?")
        if confirm:
            try:
                with self.mav_lock:
                    self.drone.mav.command_long_send(
                        self.drone.target_system, self.drone.target_component,
                        mavutil.mavlink.MAV_CMD_BATTERY_RESET, 0,
                        1, 100, 0, 0, 0, 0, 0)
                # 소모율 이력 초기화 — 잔량 급상승으로 인한 소모율 오산정 방지
                self.bat_history = []
                self.bat_drain_rate = 0.0
                self.set_status("배터리 잔량 100% 초기화 완료")
                print("[명령] 배터리 100% 상태로 초기화했습니다.")
            except Exception as error:
                print("[BATTERY RESET ERROR]", error)

    # ------------------------------------------------------------
    # 마우스로 드론 이동 (GUIDED GoTo)
    # ------------------------------------------------------------

    def on_map_left_click(self, coordinate):
        """
        지도 왼쪽 클릭 처리.
          - 금지구역 그리기 모드: 클릭 좌표를 현재 폴리곤 정점으로 추가
          - 일반 모드: 목적지 설정 → (금지구역 있으면) 회피 경로계획 후 순차 GUIDED
        """
        # 좌상단 줌 버튼(+/-) 영역 클릭이면 무시 (공통)
        try:
            x_rel = self.map_widget.canvas.winfo_pointerx() - self.map_widget.canvas.winfo_rootx()
            y_rel = self.map_widget.canvas.winfo_pointery() - self.map_widget.canvas.winfo_rooty()
            if x_rel < 80 and y_rel < 120:
                return
        except Exception:
            pass

        # 금지구역 그리기 모드면 정점 추가 후 종료 (드론 연결과 무관)
        if self.zone_drawing_mode:
            self.add_zone_vertex(coordinate)
            return

        # 웨이포인트 미션 계획 모드 (드론 연결과 무관하게 사전 계획 가능)
        if self.wp_planning_mode:
            self.add_mission_waypoint(*coordinate)
            return

        # 위협 반경 그리기 모드 (클릭 = 중심 → 반경 입력)
        if self.threat_drawing_mode:
            self.add_threat_radius(*coordinate)
            return

        # 일반 모드: SITL 실행 및 드론 연결 확인
        if self.drone is None or self.sitl.proc is None:
            return

        # 시동 상태에서만 이동 가능
        if not self.d_is_armed:
            tk.messagebox.showinfo("안내", "드론 시동(ARM) 상태에서만 이동이 가능합니다.")
            return

        lat, lon = coordinate

        # 목적지(TARGET) 마커 표시
        if self.target_marker is not None:
            try:
                self.target_marker.delete()
            except Exception:
                pass
        try:
            if self.target_icon is not None:
                self.target_marker = self.map_widget.set_marker(
                    lat, lon, text="TARGET", icon=self.target_icon)
            else:
                self.target_marker = self.map_widget.set_marker(lat, lon, text="TARGET")
        except Exception as error:
            print("[TARGET MARKER ERROR]", error)

        # 이동 고도 입력 팝업 → 확인 시 회피 경로계획 + 실행
        GuidedMoveDialog(self, self.plan_and_execute, lat, lon)

    def send_guided_move_cmd(self, lat, lon, alt):
        """GUIDED 모드로 지정 좌표/고도로 이동 명령을 전송한다."""
        if self.drone is None:
            return

        # 안전: 지상에 있으면(이륙 전) 이동 불가
        if self.d_alt < 0.5:
            tk.messagebox.showwarning("이동 불가", "드론이 지상에 있습니다. 이륙 후 이동하세요.")
            return

        # GUIDED 모드 강제
        if self.d_mode != "GUIDED":
            tk.messagebox.showinfo("안내", "GUIDED 모드에서만 이동이 가능합니다. GUIDED 모드로 변경합니다.")
            self.set_drone_mode("GUIDED")
            time.sleep(0.3)

        try:
            with self.mav_lock:
                self.drone.mav.set_position_target_global_int_send(
                    0,
                    self.drone.target_system, self.drone.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b110111111000,   # 위치만 사용 (속도/가속도/yaw 무시)
                    int(lat * 1e7), int(lon * 1e7), alt,
                    0, 0, 0, 0, 0, 0, 0, 0)
            self.set_status(f"목표 지점 이동 명령 전송 완료 (고도 {alt}m)")
            print(f"[명령] GUIDED 이동: LAT={lat:.7f}, LON={lon:.7f}, ALT={alt}m")
        except Exception as error:
            self.set_status("드론 이동 명령 전송 에러")
            print("[GUIDED MOVE ERROR]", error)

    def _create_rotated_drone_marker(self, lat, lon):
        """드론 헤딩(yaw)에 맞춰 회전한 드론 마커를 생성한다."""
        try:
            if self.drone_raw_img is not None:
                rotated = self.drone_raw_img.rotate(-self.d_yaw, resample=Image.BICUBIC)
                self.drone_icon = ImageTk.PhotoImage(rotated)
                self.drone_marker = self.map_widget.set_marker(lat, lon, icon=self.drone_icon)
            else:
                self.drone_marker = self.map_widget.set_marker(lat, lon, text="Drone")
        except Exception as error:
            print("[DRONE IMAGE ERROR]", error)
            self.drone_marker = self.map_widget.set_marker(lat, lon, text="Drone")

    def _update_path_line(self, lat, lon):
        """드론과 목적지(TARGET)를 잇는 경로선을 갱신한다."""
        try:
            if self.target_marker is not None:
                t_pos = self.target_marker.position
                if self.path_line is not None:
                    try:
                        self.path_line.delete()
                    except Exception:
                        pass
                self.path_line = self.map_widget.set_path(
                    [(lat, lon), t_pos], color="red", width=2)
            elif self.path_line is not None:
                self.path_line.delete()
                self.path_line = None
        except Exception as error:
            print("[PATH LINE ERROR]", error)

    def delete_target_and_path(self):
        """목적지 마커와 경로선을 제거한다."""
        if self.target_marker is not None:
            try:
                self.target_marker.delete()
            except Exception:
                pass
            self.target_marker = None
        if self.path_line is not None:
            try:
                self.path_line.delete()
            except Exception:
                pass
            self.path_line = None

    # ------------------------------------------------------------
    # 비행금지구역 그리기
    # ------------------------------------------------------------

    def toggle_zone_drawing(self):
        """금지구역 그리기 모드를 켜고 끈다."""
        self.zone_drawing_mode = not self.zone_drawing_mode
        if self.zone_drawing_mode:
            self.zone_draw_btn.config(text="그리기 ON (클릭)", bg="#e74c3c", fg="white")
            self.set_status("금지구역 그리기 ON: 지도를 클릭해 정점을 추가하고 '구역 확정'을 누르세요.")
        else:
            self.zone_draw_btn.config(text="금지구역 그리기", bg="#f0f0f0", fg="black")
            self.set_status("금지구역 그리기 OFF")

    def add_zone_vertex(self, coordinate):
        """그리는 중인 폴리곤에 정점을 추가하고 미리보기를 갱신한다."""
        lat, lon = coordinate
        self.current_zone_vertices.append((lat, lon))
        try:
            mk = self.map_widget.set_marker(
                lat, lon, text=str(len(self.current_zone_vertices)))
            self.current_zone_markers.append(mk)
        except Exception as error:
            print("[ZONE VERTEX MARKER ERROR]", error)
        self._redraw_current_zone()
        self.set_status(f"금지구역 정점 {len(self.current_zone_vertices)}개 (3개 이상에서 '구역 확정')")

    def _redraw_current_zone(self):
        """미리보기 폴리곤(주황)을 다시 그린다. (정점 3개 이상일 때)"""
        if self.current_zone_polygon is not None:
            try:
                self.current_zone_polygon.delete()
            except Exception:
                pass
            self.current_zone_polygon = None
        if len(self.current_zone_vertices) >= 3:
            try:
                self.current_zone_polygon = self.map_widget.set_polygon(
                    list(self.current_zone_vertices),
                    fill_color="orange", outline_color="#e67e22", border_width=2)
            except Exception as error:
                print("[ZONE POLYGON ERROR]", error)

    def finalize_zone(self):
        """현재 그리던 폴리곤을 금지구역으로 확정한다. (정점 3개 이상)"""
        if len(self.current_zone_vertices) < 3:
            ToastMessage(self, "정점을 3개 이상 찍어야 구역이 됩니다.", duration=2500)
            return

        self.no_fly_zones.append(list(self.current_zone_vertices))

        # 정점 마커 제거
        for mk in self.current_zone_markers:
            try:
                mk.delete()
            except Exception:
                pass
        self.current_zone_markers = []

        # 미리보기 폴리곤 제거 후 확정(빨강) 폴리곤으로 다시 표시
        if self.current_zone_polygon is not None:
            try:
                self.current_zone_polygon.delete()
            except Exception:
                pass
            self.current_zone_polygon = None
        try:
            poly = self.map_widget.set_polygon(
                self.no_fly_zones[-1],
                fill_color="red", outline_color="#8B0000", border_width=2)
            self.zone_polygons.append(poly)
        except Exception as error:
            print("[ZONE FINALIZE ERROR]", error)

        self.current_zone_vertices = []
        self._draw_wp_markers()   # 새 구역을 반영해 회피 경로 다시 그림
        self.set_status(f"금지구역 확정: 총 {len(self.no_fly_zones)}개")

    def clear_zones(self):
        """확정/미리보기 금지구역과 계획 경로를 모두 지운다."""
        # 안정성: 진행 중인 미션이 있으면 먼저 중단
        if self.mission_active:
            self.stop_mission()
            self.set_status("금지구역 초기화로 진행 중인 미션을 중단했습니다.")

        for poly in self.zone_polygons:
            try:
                poly.delete()
            except Exception:
                pass
        self.zone_polygons = []

        if self.current_zone_polygon is not None:
            try:
                self.current_zone_polygon.delete()
            except Exception:
                pass
            self.current_zone_polygon = None

        for mk in self.current_zone_markers:
            try:
                mk.delete()
            except Exception:
                pass
        self.current_zone_markers = []

        self.no_fly_zones = []
        self.current_zone_vertices = []
        self.clear_mission_path()
        self._draw_wp_markers()   # 구역 삭제 반영(직선으로 복귀)
        self.set_status("모든 금지구역을 삭제했습니다.")

    # ------------------------------------------------------------
    # 회피 경로계획 + 순차 GUIDED 실행
    # ------------------------------------------------------------

    def _point_in_any_zone(self, lat, lon):
        """
        현재 위치가 금지구역/위협반경 다각형 중 하나의 내부인지 검사한다.
        내부라면 해당 다각형을 반환하고, 아니면 None을 반환한다.
        """
        if not self.no_fly_zones:
            return None

        for zone in self.no_fly_zones:
            if len(zone) < 3:
                continue
            ref_lat, ref_lon = zone[0]
            poly_local = [
                path_planner.latlon_to_local(zlat, zlon, ref_lat, ref_lon)
                for zlat, zlon in zone
            ]
            px, py = path_planner.latlon_to_local(lat, lon, ref_lat, ref_lon)
            if path_planner.point_in_polygon(px, py, poly_local):
                return zone
        return None

    def _evaluate_avoidance(self, trigger_source, current_pos, intended_target, alt):
        """
        회피 판단 단일 진입점. self.avoidance_action_setting에 따라 5종 정책으로 분기한다.

        trigger_source : "GUIDED_CLICK" | "RISK_REPLAN" | "MANUAL_RC" | "AUTO_MISSION_EXPAND"
        current_pos    : (lat, lon)
        intended_target: (lat, lon) — 회피가 없었다면 원래 가려던 목표
        alt            : 비행 고도

        반환:
          {"action": "DIRECT"}
          {"action": "REROUTE", "waypoints": [...], "metrics": {...}, "resume_target": (lat,lon,alt)}
          {"action": "RTL", "waypoints": [...], "metrics": {...}}   # resume 없음(대피 목적)
          {"action": "BRAKE"}
          {"action": "LAND"}
          {"action": "FAILED", "reason": str}
        """
        if not self.no_fly_zones:
            return {"action": "DIRECT"}

        # 실제로 구역과 교차할 때만 정책 발동 (REPORT_ONLY/BRAKE/LAND도 교차 시에만)
        crosses = False
        try:
            crosses = path_planner.path_intersects_zones(
                [current_pos, intended_target], self.no_fly_zones)
        except Exception:
            crosses = False

        if not crosses:
            return {"action": "DIRECT"}

        policy = self.avoidance_action_setting

        if policy == AVOIDANCE_ACTION_REPORT_ONLY:
            self.log_safety_event("ZONE_APPROACH", "WARNING",
                f"{trigger_source}: 위협구역 교차 감지, 정책=경고만")
            return {"action": "DIRECT"}

        if policy == AVOIDANCE_ACTION_BRAKE:
            self.log_safety_event("AVOIDANCE_BRAKE", "WARNING",
                f"{trigger_source}: 위협구역 교차, 제자리 정지")
            return {"action": "BRAKE"}

        if policy == AVOIDANCE_ACTION_LAND:
            self.log_safety_event("AVOIDANCE_LAND", "CRITICAL",
                f"{trigger_source}: 위협구역 교차, 즉시 착륙")
            return {"action": "LAND"}

        if policy == AVOIDANCE_ACTION_RTL_RALLY:
            goal = self._nearest_rally_point(current_pos)
            if goal is None and self.home_lat is not None:
                goal = (self.home_lat, self.home_lon, alt)
            if goal is None:
                return {"action": "FAILED", "reason": "Rally Point도 홈 위치도 설정되지 않음"}
            try:
                result = path_planner.plan_path(
                    current_pos, (goal[0], goal[1]), self.no_fly_zones,
                    cell_size_m=self.PLAN_CELL_M, safety_margin_m=self.PLAN_MARGIN_M)
            except Exception as error:
                print(f"[AVOID ERROR][{trigger_source}]", error)
                return {"action": "FAILED", "reason": str(error)}
            if not result["success"]:
                return {"action": "FAILED", "reason": result["reason"]}
            return {"action": "RTL", "waypoints": result["waypoints"],
                    "metrics": result["metrics"]}

        # 기본값: REROUTE
        try:
            result = path_planner.plan_path(
                current_pos, intended_target, self.no_fly_zones,
                cell_size_m=self.PLAN_CELL_M, safety_margin_m=self.PLAN_MARGIN_M)
        except Exception as error:
            print(f"[AVOID ERROR][{trigger_source}]", error)
            return {"action": "FAILED", "reason": str(error)}
        if not result["success"]:
            return {"action": "FAILED", "reason": result["reason"]}
        return {
            "action": "REROUTE",
            "waypoints": result["waypoints"],
            "metrics": result["metrics"],
            "resume_target": (intended_target[0], intended_target[1], alt),
        }

    def plan_and_execute(self, lat, lon, alt):
        """
        목표(lat, lon, alt)로 이동. 금지구역이 있으면 A* 회피 경로를 계획해
        순차 GUIDED로 비행하고, 없으면 기존 단일 GUIDED로 이동한다.
        회피(REROUTE) 시 원래 목표를 기억했다가 우회 완료 후 자동 복귀한다.
        """
        if self.drone is None:
            return

        # 안전: 지상(이륙 전)이면 이동 불가 (기존 규칙)
        if self.d_alt < 0.5:
            tk.messagebox.showwarning("이동 불가", "드론이 지상에 있습니다. 이륙 후 이동하세요.")
            return

        start = (self.d_lat, self.d_lon)
        decision = self._evaluate_avoidance("GUIDED_CLICK", start, (lat, lon), alt)

        if decision["action"] == "DIRECT":
            # 장애물 없음 → 직행. (복귀 상태가 남아있지 않게 리셋)
            self.avoidance_state = "NONE"
            self.send_guided_move_cmd(lat, lon, alt)
            return

        if decision["action"] == "FAILED":
            self.avoidance_state = "NONE"
            self.avoidance_resume_target = None
            ToastMessage(self, f"경로 계획 실패\n{decision['reason']}", duration=4000)
            self.set_status(f"경로 계획 실패: {decision['reason']}")
            return

        if decision["action"] == "BRAKE":
            self.avoidance_state = "NONE"
            self.avoidance_resume_target = None
            self.cmd_hover()
            ToastMessage(self, "⚠ 위협구역 — 정책상 제자리 정지", duration=4000)
            return

        if decision["action"] == "LAND":
            self.avoidance_state = "NONE"
            self.avoidance_resume_target = None
            self.set_drone_mode("LAND")
            ToastMessage(self, "⚠ 위협구역 — 정책상 즉시 착륙", duration=4000)
            return

        if decision["action"] == "RTL":
            wps = decision["waypoints"]
            self.draw_mission_path(wps)
            self.avoidance_state = "NONE"          # RTL은 복귀 개념 없음(대피 목적)
            self.avoidance_resume_target = None
            self.avoidance_resume_kind = None
            self.start_mission(wps, alt)
            return

        # REROUTE
        wps = decision["waypoints"]
        m = decision["metrics"]
        if m.get("start_corrected"):
            ToastMessage(self, "출발점이 금지구역 안전여유 안이라\n가장 가까운 자유 지점에서 경로를 시작합니다.",
                         duration=4000)
        self.set_status(
            f"회피 경로 생성: WP {m['waypoints_smoothed']}개 | "
            f"길이 {m['path_length_smoothed_m']:.0f}m | {m['planning_time_s']*1000:.0f}ms")
        print(f"[PLAN] raw {m['waypoints_raw']} -> smoothed {m['waypoints_smoothed']} WP, "
              f"len {m['path_length_smoothed_m']:.1f}m, {m['planning_time_s']*1000:.1f}ms")

        self.draw_mission_path(wps)
        self.avoidance_state = "REROUTING"
        self.avoidance_resume_target = decision["resume_target"]   # 우회 후 복귀할 원래 목표
        self.avoidance_resume_kind = "GUIDED"
        self.start_mission(wps, alt)

    def draw_mission_path(self, waypoints):
        """계획된 경로를 지도에 초록색 폴리라인으로 표시한다."""
        self.clear_mission_path()
        if len(waypoints) < 2:
            return
        try:
            self.mission_path_line = self.map_widget.set_path(
                list(waypoints), color="#2ecc71", width=3)
        except Exception as error:
            print("[MISSION PATH ERROR]", error)

    def clear_mission_path(self):
        """계획 경로 폴리라인을 제거한다."""
        if self.mission_path_line is not None:
            try:
                self.mission_path_line.delete()
            except Exception:
                pass
            self.mission_path_line = None

    def start_mission(self, waypoints, alt):
        """순차 GUIDED 실행을 시작한다. (waypoints[0]=출발점이므로 1번부터 전송)"""
        self.mission_waypoints = list(waypoints)
        self.mission_alt = alt
        self.mission_index = 1 if len(waypoints) > 1 else 0
        self.mission_active = True
        self._mission_guided_seen = False

        # GUIDED 모드 보장
        if self.d_mode != "GUIDED":
            self.set_drone_mode("GUIDED")

        self._send_current_waypoint()

    def _send_current_waypoint(self):
        """현재 인덱스의 웨이포인트로 GUIDED 이동 명령을 전송한다."""
        if not self.mission_active:
            return
        if self.mission_index >= len(self.mission_waypoints):
            return
        wlat, wlon = self.mission_waypoints[self.mission_index]
        if self._send_goto(wlat, wlon, self.mission_alt):
            total = len(self.mission_waypoints) - 1
            self.set_status(f"웨이포인트 {self.mission_index}/{total} 이동 중...")

    def _send_goto(self, lat, lon, alt):
        """GUIDED 위치 명령(set_position_target_global_int)을 전송한다."""
        if self.drone is None:
            return False
        try:
            with self.mav_lock:
                self.drone.mav.set_position_target_global_int_send(
                    0,
                    self.drone.target_system, self.drone.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b110111111000,
                    int(lat * 1e7), int(lon * 1e7), alt,
                    0, 0, 0, 0, 0, 0, 0, 0)
            return True
        except Exception as error:
            print("[GOTO SEND ERROR]", error)
            return False

    def _mission_step(self):
        """현재 위치와 목표 웨이포인트 간 거리를 보고 도달 시 다음으로 진행한다."""
        if not self.mission_active:
            return
        if self.mission_index >= len(self.mission_waypoints):
            self.mission_active = False
            return

        # 안전 가드 1: 시동(ARM)이 풀리면 즉시 중단 (착륙/추락/사용자 DISARM)
        if not self.d_is_armed:
            self.stop_mission()
            self.set_status("미션 중단: 시동(ARM)이 해제되었습니다.")
            return

        # 안전 가드 2: GUIDED 진입을 한 번 확인한 뒤 GUIDED를 벗어나면 takeover로 보고 중단
        mode = self.d_mode.strip().upper()
        if mode == "GUIDED":
            self._mission_guided_seen = True
        elif self._mission_guided_seen:
            self.stop_mission()
            self.set_status("미션 중단: 비행 모드가 GUIDED에서 변경되었습니다.")
            return

        wlat, wlon = self.mission_waypoints[self.mission_index]
        # 현재 위치와 웨이포인트 간 거리(m) 계산
        dx, dy = path_planner.latlon_to_local(self.d_lat, self.d_lon, wlat, wlon)
        dist = math.hypot(dx, dy)

        if dist < self.WAYPOINT_REACH_M:
            self.mission_index += 1
            if self.mission_index >= len(self.mission_waypoints):
                self.mission_active = False

                # 회피 우회(REROUTING)가 끝났으면 원래 목표로 자동 복귀
                if self.avoidance_state == "REROUTING" and \
                        self.avoidance_resume_target is not None:
                    rlat, rlon, ralt = self.avoidance_resume_target
                    self.avoidance_state = "RESUMING"
                    self.avoidance_resume_target = None
                    self.set_status("회피 완료 — 원래 목표로 복귀합니다.")
                    ToastMessage(self, "✓ 회피 완료, 원래 목표로 이동합니다", duration=3000)
                    # 복귀 이동도 다시 _evaluate_avoidance를 거친다(복귀 경로 위에 또 다른
                    # 장애물이 있을 수 있으므로 직행 단정 금지). after(0)로 재진입을 피한다.
                    self.run_on_ui(lambda: self.plan_and_execute(rlat, rlon, ralt))
                else:
                    self.avoidance_state = "NONE"
                    self.set_status("미션 완료: 모든 웨이포인트에 도달했습니다.")
                    ToastMessage(self, "회피 경로 비행 완료", duration=3000)
            else:
                self._send_current_waypoint()

    def stop_mission(self):
        """순차 실행을 중단하고 계획 경로 표시를 지운다."""
        self.mission_active = False
        self.mission_waypoints = []
        self.mission_index = 0
        self._mission_guided_seen = False
        # 회피 복귀 상태도 함께 리셋(중단 후 엉뚱한 곳으로 복귀하지 않도록)
        self.avoidance_state = "NONE"
        self.avoidance_resume_target = None
        self.avoidance_resume_kind = None
        self.clear_mission_path()

    def abort_mission(self):
        """
        진행 중인 미션을 중단한다 (단축키 ESC).
        GUIDED 순차 미션은 stop_mission으로, AUTO 비행은 제자리 정지(호버)로
        실제로 멈춘다. (AUTO는 모드를 바꿔야 멈추므로 stop_mission만으론 부족)
        """
        was_auto = self.auto_mission_active
        self.stop_mission()
        if was_auto:
            self.auto_mission_active = False
            self._auto_seen = False
            if self.drone is not None and self.d_is_armed:
                self.cmd_hover()      # AUTO 비행 → 제자리 정지로 중단
        self.set_status("미션 중단")

    # ------------------------------------------------------------
    # 긴급 대응 패널 (화면 최상단 고정)
    # ------------------------------------------------------------

    def create_emergency_panel(self):
        """화면 최상단에 RTL/HOVER/LAND 긴급 버튼 3개를 항상 크게 고정 표시한다."""
        em_frame = tk.Frame(self, bg="#1a1a1a", height=64)
        em_frame.pack(side="top", fill="x")
        em_frame.pack_propagate(False)

        em_font = ("맑은 고딕", 14, "bold")

        tk.Button(
            em_frame, text="RTL (귀환)", bg="#e74c3c", fg="white", font=em_font,
            activebackground="#c0392b",
            command=lambda: self.set_drone_mode("RTL")
        ).pack(side="left", fill="both", expand=True, padx=2, pady=4)

        tk.Button(
            em_frame, text="HOVER (제자리)", bg="#e67e22", fg="white", font=em_font,
            activebackground="#d35400",
            command=self.cmd_hover
        ).pack(side="left", fill="both", expand=True, padx=2, pady=4)

        tk.Button(
            em_frame, text="LAND (착륙)", bg="#c0392b", fg="white", font=em_font,
            activebackground="#922b21",
            command=lambda: self.set_drone_mode("LAND")
        ).pack(side="left", fill="both", expand=True, padx=2, pady=4)

    def cmd_hover(self):
        """
        현재 위치에서 제자리 정지(호버)한다.
        ArduCopter SITL은 실제 조종기(RC)가 없어 LOITER로 바꾸면 스로틀이
        최소값으로 인식되어 하강·착륙·시동해제가 발생한다. 이를 피하기 위해
        RC 입력에 의존하지 않는 GUIDED 모드로 현재 위치를 목표로 지정해
        그 자리에 머무르게 한다.
        """
        if self.drone is None:
            self.set_status("드론이 연결되지 않았습니다.")
            return
        if self.mission_active:
            self.stop_mission()
        self.set_status("제자리 정지(호버) 준비 중...")

        def _worker():
            try:
                with self.mav_lock:
                    try:
                        self.drone.set_mode(self.drone.mode_mapping()["GUIDED"])
                    except Exception:
                        self.drone.set_mode("GUIDED")
                time.sleep(0.3)
                lat, lon = self.d_lat, self.d_lon
                # 현재 고도 유지(센서 오차로 0 근처면 최소 2m로 보정해 하강 방지)
                alt = self.d_alt if self.d_alt and self.d_alt > 2.0 else 2.0
                with self.mav_lock:
                    self.drone.mav.set_position_target_global_int_send(
                        0, self.drone.target_system, self.drone.target_component,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        0b0000111111111000,  # 위치만 사용
                        int(lat * 1e7), int(lon * 1e7), alt,
                        0, 0, 0, 0, 0, 0, 0, 0)
                self.run_on_ui(lambda: self.set_status("제자리 정지(호버) — 현재 위치 유지"))
                print("[명령] HOVER — GUIDED 현재 위치 유지")
            except Exception as error:
                print("[HOVER ERROR]", error)
                self.run_on_ui(lambda: self.set_status(f"호버 실패: {error}"))

        threading.Thread(target=_worker, daemon=True).start()

    # ------------------------------------------------------------
    # 웨이포인트 미션 플래너 (지도 클릭 → AUTO 비행)
    # ------------------------------------------------------------

    def toggle_wp_planning(self):
        """웨이포인트 찍기 모드를 켜고 끈다."""
        self.wp_planning_mode = not self.wp_planning_mode
        if self.wp_planning_mode:
            self.wp_plan_btn.config(text="미션 계획 ON (클릭)", bg="#e74c3c", fg="white")
            self.set_status("미션 계획 ON: 지도를 클릭해 웨이포인트를 추가하세요.")
        else:
            self.wp_plan_btn.config(text="미션 계획", bg="#f0f0f0", fg="black")
            self.set_status("미션 계획 OFF")

    def add_mission_waypoint(self, lat, lon):
        """클릭 좌표를 계획 웨이포인트(기본 고도)로 추가하고 지도에 표시한다."""
        alt = self.wp_default_alt
        self.planned_waypoints.append((lat, lon, alt))
        self._draw_wp_markers()
        self.set_status(f"웨이포인트 {len(self.planned_waypoints)}개 (고도 {alt:.0f}m)")

    def remove_last_waypoint(self):
        """마지막 계획 웨이포인트를 제거한다."""
        if self.planned_waypoints:
            self.planned_waypoints.pop()
            self._draw_wp_markers()
            self.set_status(f"웨이포인트 {len(self.planned_waypoints)}개")

    def clear_planned_waypoints(self):
        """계획 웨이포인트를 모두 제거한다."""
        self.planned_waypoints = []
        self._draw_wp_markers()
        self.set_status("웨이포인트 전체 삭제")

    def _draw_wp_markers(self):
        """계획 웨이포인트의 번호 마커와 연결선을 다시 그린다."""
        for mk in self.wp_markers:
            try:
                mk.delete()
            except Exception:
                pass
        self.wp_markers = []
        for ln in self.wp_lines:
            try:
                ln.delete()
            except Exception:
                pass
        self.wp_lines = []

        for i, (la, lo, al) in enumerate(self.planned_waypoints, start=1):
            try:
                self.wp_markers.append(self.map_widget.set_marker(la, lo, text=f"WP{i}"))
            except Exception as error:
                print("[WP MARKER ERROR]", error)

        # 연결선: 금지구역이 있고 드론 위치를 알면 회피 경로(초록)로,
        #         아니면 직선(파랑)으로 표시한다.
        start = None
        if self.drone is not None and (self.d_lat != 0.0 or self.d_lon != 0.0):
            start = (self.d_lat, self.d_lon)

        if self.planned_waypoints:
            color = "#2980b9"
            if start is not None and self.no_fly_zones:
                try:
                    expanded = self._expand_waypoints_avoiding_zones(self.planned_waypoints)
                    pts = [start] + [(la, lo) for (la, lo, al) in expanded]
                    color = "#27ae60"   # 회피 경로
                except Exception as error:
                    print("[WP AVOID DRAW ERROR]", error)
                    pts = [start] + [(la, lo) for (la, lo, al) in self.planned_waypoints]
            else:
                pts = ([start] if start is not None else []) + \
                      [(la, lo) for (la, lo, al) in self.planned_waypoints]
            if len(pts) >= 2:
                try:
                    self.wp_lines.append(self.map_widget.set_path(pts, color=color, width=2))
                except Exception as error:
                    print("[WP LINE ERROR]", error)

    def _expand_waypoints_avoiding_zones(self, wps):
        """
        각 구간(이전 위치→다음 WP)이 금지구역/위협 반경을 통과하면
        _evaluate_avoidance로 우회 중간 웨이포인트를 삽입한다.
        금지구역이 없으면 입력을 그대로 돌려준다.
        반환: [(lat, lon, alt), ...]

        업로드 시점의 '사전 정적 가공'이므로 운용자 정책(BRAKE/LAND/RTL 등)과
        무관하게 항상 REROUTE로 강제한다. (실시간 정지·착륙·귀환은 비행 중
        동적재계획 _trigger_dynamic_replan이 담당)
        """
        if not self.no_fly_zones:
            return [(la, lo, al) for (la, lo, al) in wps]

        saved_policy = self.avoidance_action_setting
        self.avoidance_action_setting = AVOIDANCE_ACTION_REROUTE

        out = []
        prev = (self.d_lat, self.d_lon)
        for (la, lo, al) in wps:
            decision = self._evaluate_avoidance("AUTO_MISSION_EXPAND", prev, (la, lo), al)
            if decision["action"] == "REROUTE":
                for (wla, wlo) in decision["waypoints"][1:]:
                    out.append((wla, wlo, al))
                prev = (la, lo)
                continue
            # DIRECT 또는 FAILED(우회 실패) → 원래 WP 유지 (기존 폴백과 동일)
            out.append((la, lo, al))
            prev = (la, lo)

        self.avoidance_action_setting = saved_policy
        return out

    def upload_and_run_mission(self):
        """
        계획 웨이포인트를 MAVLink MISSION 프로토콜로 업로드하고 AUTO로 전환한다.
        업로드 핸드셰이크(MISSION_REQUEST 응답)는 mavlink_loop에서 처리한다.
        """
        if self.drone is None:
            self.set_status("드론이 연결되지 않았습니다.")
            return
        if not self.planned_waypoints:
            ToastMessage(self, "계획된 웨이포인트가 없습니다.", duration=2500)
            return

        # 사전 위험 검증 (치명 문제 시 업로드 중단)
        if not self._preflight_risk_check():
            return

        # 금지구역/위협 반경을 통과하는 구간은 A*로 우회 삽입
        expanded = self._expand_waypoints_avoiding_zones(self.planned_waypoints)

        # 미션 항목: (lat, lon, alt, command)
        #   seq0 = HOME(현재 위치), seq1 = 이륙(NAV_TAKEOFF), seq2~ = 웨이포인트
        # 이륙 항목을 넣어 AUTO 진입 시 (지상 ARM 상태라도) 스스로 이륙하게 한다.
        NAV_WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        NAV_TO = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        takeoff_alt = expanded[0][2] if expanded else 10.0
        items = [(self.d_lat, self.d_lon, 0.0, NAV_WP)]            # HOME
        items.append((self.d_lat, self.d_lon, takeoff_alt, NAV_TO))  # 이륙
        items += [(la, lo, al, NAV_WP) for (la, lo, al) in expanded]
        self._mission_upload_items = items
        self._mission_upload_active = True
        n = len(self._mission_upload_items)

        try:
            with self.mav_lock:
                self.drone.mav.mission_count_send(
                    self.drone.target_system, self.drone.target_component,
                    n, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.set_status(f"미션 업로드 시작: {n}개 항목 (HOME 포함)")
            print(f"[MISSION] upload start, count={n}")
        except Exception as error:
            self._mission_upload_active = False
            print("[MISSION COUNT ERROR]", error)
            self.set_status(f"미션 업로드 실패: {error}")

    def _send_mission_item(self, seq):
        """업로드 핸드셰이크: 요청된 seq의 미션 아이템(MISSION_ITEM_INT)을 전송한다."""
        if seq < 0 or seq >= len(self._mission_upload_items):
            return
        lat, lon, alt, cmd = self._mission_upload_items[seq]
        try:
            with self.mav_lock:
                self.drone.mav.mission_item_int_send(
                    self.drone.target_system, self.drone.target_component,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    cmd,              # NAV_WAYPOINT 또는 NAV_TAKEOFF
                    0,                # current
                    1,                # autocontinue
                    0, 0, 0, 0,       # param 1~4
                    int(lat * 1e7), int(lon * 1e7), float(alt),
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            print(f"[MISSION] item {seq} sent ({lat:.6f},{lon:.6f},{alt}m) cmd={cmd}")
        except Exception as error:
            print("[MISSION ITEM ERROR]", error)

    # ------------------------------------------------------------
    # 위협 반경(원형 장애물) — A* 자동 회피에 통합
    # ------------------------------------------------------------

    def toggle_threat_drawing(self):
        """위협 반경 그리기 모드를 켜고 끈다."""
        self.threat_drawing_mode = not self.threat_drawing_mode
        if self.threat_drawing_mode:
            self.threat_btn.config(text="위협 반경 ON (중심 클릭)", bg="#e74c3c", fg="white")
            self.set_status("위협 반경 ON: 지도에서 중심을 클릭한 뒤 반경을 입력하세요.")
        else:
            self.threat_btn.config(text="위협 반경 추가", bg="#f0f0f0", fg="black")
            self.set_status("위협 반경 OFF")

    def circle_to_polygon(self, lat, lon, radius_m, n_points=36):
        """원형 위협 반경을 n각형 다각형으로 근사하여 [(lat, lon), ...]로 반환한다."""
        points = []
        for i in range(n_points):
            angle = 2 * math.pi * i / n_points
            dlat = (radius_m * math.cos(angle)) / 111320.0
            dlon = (radius_m * math.sin(angle)) / (111320.0 * math.cos(math.radians(lat)))
            points.append((lat + dlat, lon + dlon))
        return points

    def add_threat_radius(self, lat, lon):
        """클릭한 중심에 대해 반경(m)을 입력받아 위협 원을 추가하고 A* 회피에 통합한다."""
        self.threat_pending_center = (lat, lon)
        radius = tk.simpledialog.askfloat(
            "위협 반경", "반경(m)을 입력하세요:",
            initialvalue=100.0, minvalue=1.0, parent=self)
        self.threat_pending_center = None
        if radius is None or radius <= 0:
            self.set_status("위협 반경 추가 취소")
            return

        self.threat_radii.append((lat, lon, radius))

        # 원 → 다각형 근사 후 no_fly_zones에 추가 (A*가 자동으로 회피)
        poly = self.circle_to_polygon(lat, lon, radius)
        self.no_fly_zones.append(poly)
        self._threat_zone_refs.append(poly)

        # 지도에 빨간 외곽선 원(다각형 근사)으로 표시
        try:
            circle = self.map_widget.set_polygon(
                poly, fill_color=None, outline_color="#e74c3c", border_width=2)
            self.threat_circles.append(circle)
        except Exception as error:
            print("[THREAT CIRCLE ERROR]", error)

        self._draw_wp_markers()   # 새 위협 반경을 반영해 회피 경로 다시 그림
        self.set_status(f"위협 반경 추가: 반경 {radius:.0f}m (총 {len(self.threat_radii)}개)")

    def clear_threat_radii(self):
        """모든 위협 반경과 그에 대응하는 회피용 다각형을 제거한다."""
        for circle in self.threat_circles:
            try:
                circle.delete()
            except Exception:
                pass
        self.threat_circles = []

        # no_fly_zones에서 위협 다각형만 골라 제거 (그린 금지구역은 유지)
        for ref in self._threat_zone_refs:
            try:
                self.no_fly_zones.remove(ref)
            except ValueError:
                pass
        self._threat_zone_refs = []
        self.threat_radii = []
        self._draw_wp_markers()   # 위협 삭제 반영
        self.set_status("위협 반경 전체 삭제")

    # ------------------------------------------------------------
    # Safety Monitor (통신/GPS/배터리 위험도 감시) - 1차 구현
    # ------------------------------------------------------------

    def evaluate_flight_safety(self):
        """
        MAVLink 수신 상태, GPS 위성 수, 배터리 잔량을 기반으로
        기본 임무 안전도 점수와 위험 단계를 계산한다.
        1차 구현에서는 위치 튐/고도 급변 분석이나 자동 대응은 수행하지 않는다.
        """
        score = 100
        reasons = []
        now = time.time()

        # 5-1. HEARTBEAT 기준 (링크 상태)
        if self.last_heartbeat_time is None:
            score -= 40
            self.link_state = "DISCONNECTED"
            reasons.append("HEARTBEAT 미수신")
        else:
            heartbeat_delay = now - self.last_heartbeat_time
            if heartbeat_delay >= 8.0:
                score -= 70
                self.link_state = "LOST"
                reasons.append(f"HEARTBEAT 지연 {heartbeat_delay:.1f}초")
            elif heartbeat_delay >= 5.0:
                score -= 40
                self.link_state = "CRITICAL"
                reasons.append(f"HEARTBEAT 지연 {heartbeat_delay:.1f}초")
            elif heartbeat_delay >= 2.0:
                score -= 20
                self.link_state = "WARNING"
                reasons.append(f"HEARTBEAT 지연 {heartbeat_delay:.1f}초")
            else:
                self.link_state = "NORMAL"

        # 5-2. 위치 메시지 기준
        if self.last_position_time is None:
            score -= 20
            reasons.append("위치 메시지 미수신")
        else:
            position_delay = now - self.last_position_time
            if position_delay >= 5.0:
                score -= 30
                reasons.append(f"위치 메시지 지연 {position_delay:.1f}초")
            elif position_delay >= 2.0:
                score -= 15
                reasons.append(f"위치 메시지 지연 {position_delay:.1f}초")

        # 5-3. GPS 위성 수 기준
        if self.last_gps_time is None:
            score -= 20
            self.gps_state = "UNKNOWN"
            reasons.append("GPS 메시지 미수신")
        else:
            if self.d_sats >= 8:
                self.gps_state = "GOOD"
            elif self.d_sats >= 5:
                score -= 15
                self.gps_state = "WEAK"
                reasons.append(f"GPS 위성 수 부족: {self.d_sats}개")
            elif self.d_sats >= 1:
                score -= 30
                self.gps_state = "BAD"
                reasons.append(f"GPS 위성 수 매우 부족: {self.d_sats}개")
            else:
                score -= 40
                self.gps_state = "BAD"
                reasons.append("GPS 위성 없음")

        # 5-4. 배터리 기준 (SYS_STATUS 미수신이면 정보 없음으로 처리)
        if self.last_sys_status_time is None or self.d_bat_p is None or self.d_bat_p < 0:
            score -= 10
            self.battery_state = "UNKNOWN"
            self.battery_return_required = False   # 정보 없으면 규정 판정 보류
            reasons.append("배터리 정보 미수신")
        else:
            # 규정: 잔량 20% 이하(80% 소모) → 즉시 귀환(RTL) 권장
            self.battery_return_required = (self.d_bat_p <= self.BATTERY_LAND_THRESHOLD_PCT)
            if self.d_bat_p >= 30:
                self.battery_state = "GOOD"
            elif self.d_bat_p > self.BATTERY_LAND_THRESHOLD_PCT:
                score -= 15
                self.battery_state = "LOW"
                reasons.append(f"배터리 부족: {self.d_bat_p}%")
            elif self.d_bat_p >= 10:
                score -= 30
                self.battery_state = "CRITICAL"
                reasons.append(
                    f"배터리 {self.d_bat_p}% (80% 소모) — 규정상 즉시 귀환(RTL)")
            else:
                score -= 50
                self.battery_state = "CRITICAL"
                reasons.append(f"배터리 매우 위험: {self.d_bat_p}%")

        # 5-5. 최종 위험 단계
        score = max(0, min(100, score))
        self.flight_risk_score = score
        self.risk_reasons = reasons

        if self.link_state == "LOST":
            self.risk_level = "LOST"
        elif score >= 80:
            self.risk_level = "NORMAL"
        elif score >= 60:
            self.risk_level = "WARNING"
        elif score >= 30:
            self.risk_level = "CRITICAL"
        else:
            self.risk_level = "LOST"

        # B1~B4: 고도화 감시 (귀환 예측·추세·대응 매트릭스·동적 재계획)
        self._update_bat_history()
        self._update_bat_drain_rate()
        self._calc_return_feasibility()
        self._calc_trends()
        self.recommended_action, self.recommended_reason = self._get_recommended_action()

        # AUTO(MISSION 프로토콜) 비행 상태 동기화:
        #   AUTO를 실제로 한 번 본(_auto_seen) 뒤에만 종료 판정한다.
        #   착륙(시동 해제) 또는 모드 변경(takeover) 시 추적 종료.
        #   시동 전 지상 대기 중에는 끄지 않아 'AUTO 실행→ARM→이륙' 흐름을 놓치지 않는다.
        if self.auto_mission_active:
            mode = self.d_mode.strip().upper()
            if mode == "AUTO":
                self._auto_seen = True
            if self._auto_seen and (not self.d_is_armed or mode != "AUTO"):
                self.auto_mission_active = False
                self._auto_seen = False

        # B4: 실제 비행 중(시동+고도>1m) 미션(GUIDED 순차 또는 AUTO)에서
        #     CRITICAL/LOST → 동적 재계획 트리거. (지상/시동 전 오발동 방지)
        airborne = self.d_is_armed and self.d_alt > 1.0
        if airborne and (self.mission_active or self.auto_mission_active) \
                and self.risk_level in ("CRITICAL", "LOST"):
            self._trigger_dynamic_replan(
                self.risk_reasons[0] if self.risk_reasons else "위험 감지")

        # B5: 안전도 점수와 무관하게 위협구역/금지구역 내부 진입을 항상 감시한다.
        breached_zone = self._point_in_any_zone(self.d_lat, self.d_lon)
        raw_in_zone = breached_zone is not None

        # 경계 GPS 떨림 억제: 같은 raw 판정이 2회 연속일 때만 상태를 전환한다.
        if raw_in_zone == self._zone_raw_prev:
            self._zone_confirm = min(self._zone_confirm + 1, 2)
        else:
            self._zone_confirm = 1
        self._zone_raw_prev = raw_in_zone
        was_in_zone = self.in_threat_zone
        if self._zone_confirm >= 2:
            self.in_threat_zone = raw_in_zone

        if self.in_threat_zone and not was_in_zone:
            self.log_safety_event(
                "ZONE_BREACH", "CRITICAL", "위협구역/금지구역 내부 진입 감지")
            self.run_on_ui(lambda: ToastMessage(
                self, "🚨 위협구역 진입! 즉시 회피 기동을 시작합니다", duration=4000))

        if self.in_threat_zone:
            now_z = time.time()
            in_takeoff_grace = (now_z - self.last_takeoff_time) < self.TAKEOFF_GRACE_S
            # 이미 우회 비행(REROUTE/RESUMING) 중이면 그 기동이 처리하도록 둔다.
            avoidance_busy = self.avoidance_state in ("REROUTING", "RESUMING")
            if airborne and not in_takeoff_grace and not avoidance_busy \
                    and (now_z - self.last_zone_breach_replan_time
                         >= self.REPLAN_COOLDOWN_S):
                self.last_zone_breach_replan_time = now_z
                self._trigger_zone_breach_avoidance(breached_zone)
        elif was_in_zone:
            self.log_safety_event(
                "ZONE_CLEAR", "INFO", "위협구역/금지구역 이탈")
            self.run_on_ui(lambda: ToastMessage(
                self, "✓ 위협구역을 벗어났습니다", duration=3000))

        # 통신·GPS 동시 이상 감지
        self._detect_link_gps_anomaly()

        # 7. 위험 상태 이벤트 기록 (쿨다운은 log_safety_event 내부에서 처리)
        if self.link_state == "WARNING":
            self.log_safety_event("LINK_WARNING", "WARNING", "HEARTBEAT 지연 감지")
        elif self.link_state == "CRITICAL":
            self.log_safety_event("LINK_CRITICAL", "CRITICAL", "HEARTBEAT 심각 지연")
        elif self.link_state == "LOST":
            self.log_safety_event("LINK_LOST", "CRITICAL", "HEARTBEAT 손실")

        if self.gps_state == "WEAK":
            self.log_safety_event("GPS_WEAK", "WARNING", f"GPS 위성 수 부족: {self.d_sats}개")
        elif self.gps_state == "BAD":
            self.log_safety_event("GPS_BAD", "CRITICAL", f"GPS 상태 불량: {self.d_sats}개")

        if self.battery_state == "LOW":
            self.log_safety_event("BATTERY_LOW", "WARNING", f"배터리 부족: {self.d_bat_p}%")
        elif self.battery_state == "CRITICAL":
            self.log_safety_event("BATTERY_CRITICAL", "CRITICAL", f"배터리 위험: {self.d_bat_p}%")

        # 규정 도달(잔량 20% 이하 = 80% 소모): 즉시 귀환(RTL) 강한 경고 — 팝업은 1회만
        if self.battery_return_required:
            self.log_safety_event(
                "BATTERY_RETURN_REQUIRED", "CRITICAL",
                f"배터리 {self.d_bat_p}% (80% 소모) — 규정상 즉시 귀환(RTL) 권장")
            if not self._battery_return_warned:
                self._battery_return_warned = True
                self.run_on_ui(lambda: ToastMessage(
                    self,
                    f"⚠ 배터리 {self.d_bat_p}% — 80% 소모, 규정상 즉시 귀환(RTL) 권장",
                    duration=5000))
        else:
            # 배터리가 20% 위로 회복되면 다음 도달 시 다시 경고할 수 있게 초기화
            self._battery_return_warned = False

        # 배터리 단계별 1회 알림 (50% 권고 / 30% 경고)
        if self.last_sys_status_time is not None and self.d_bat_p is not None \
                and self.d_bat_p >= 0:
            # 배터리가 임계 위로 회복되면 다음 하강 시 다시 알릴 수 있게 초기화
            if self.d_bat_p > 50:
                self._battery_notify_50 = False
            if self.d_bat_p > 30:
                self._battery_notify_30 = False

            # 30% 이하 경고를 먼저 판정 — 빠른 방전으로 한 틱에 50→30 구간을
            # 건너뛰어도 경고가 누락되지 않도록 밴드가 아닌 임계값(<=30)으로 판단한다.
            if self.d_bat_p <= 30 and not self._battery_notify_30:
                self._battery_notify_30 = True
                self._battery_notify_50 = True   # 권고 단계는 건너뛴 것으로 표시
                # 20% 이하면 착륙 권장 경고가 대신 뜨므로 중복 토스트는 생략
                if not self.battery_return_required:
                    self.log_safety_event(
                        "BATTERY_WARN_30", "WARNING",
                        f"배터리 {self.d_bat_p}% — 경고: 귀환/착륙 준비")
                    self.run_on_ui(lambda: ToastMessage(
                        self, f"⚠ 배터리 {self.d_bat_p}% — 경고: 귀환·착륙을 준비하세요",
                        duration=5000))
            elif 30 < self.d_bat_p <= 50 and not self._battery_notify_50:
                # 50% 이하: 권고 (가벼운 안내)
                self._battery_notify_50 = True
                self.log_safety_event(
                    "BATTERY_ADVISORY_50", "WARNING",
                    f"배터리 {self.d_bat_p}% — 권고: 잔량 확인")
                self.run_on_ui(lambda: ToastMessage(
                    self, f"🔋 배터리 {self.d_bat_p}% — 권고: 잔량을 확인하세요",
                    duration=3500))

    # ------------------------------------------------------------
    # 귀환 가능성 예측
    # ------------------------------------------------------------

    def _update_bat_history(self):
        """배터리 잔량 이력을 60초 슬라이딩 윈도우로 유지한다."""
        if self.d_bat_p is None or self.d_bat_p < 0:
            return
        now = time.time()
        self.bat_history.append((now, self.d_bat_p))
        cutoff = now - 60.0
        self.bat_history = [(t, p) for t, p in self.bat_history if t >= cutoff]

    def _update_bat_drain_rate(self):
        """슬라이딩 윈도우 내 배터리 소모율(%/초)을 추정한다."""
        if len(self.bat_history) < 2:
            return
        dt = self.bat_history[-1][0] - self.bat_history[0][0]
        dp = self.bat_history[0][1] - self.bat_history[-1][1]  # 소모량 (양수)
        # 측정 구간이 너무 짧으면(연결·리셋 직후 글리치) 갱신하지 않는다.
        if dt < 5.0 or dp < 0:
            return
        rate = dp / dt
        # 비현실적 급변(센서 글리치)은 현실적 상한(0.5%/s ≈ 200초 만에 방전)으로 제한
        self.bat_drain_rate = min(rate, 0.5)

    def _calc_return_feasibility(self):
        """
        현재 배터리 소모율과 홈까지 거리를 기반으로 귀환 가능성을 계산한다.
        return_feasibility: "OK" / "MARGINAL" / "IMPOSSIBLE"
        return_margin_pct: 귀환 후 남을 배터리 %
        return_time_limit_s: MARGINAL일 때 귀환 불가까지 남은 초
        """
        if self.home_lat is None or self.home_lon is None:
            return
        if self.bat_drain_rate <= 0 or self.d_bat_p is None or self.d_bat_p < 0:
            return
        if self.d_lat == 0.0 and self.d_lon == 0.0:
            return

        R = 6371000.0
        lat1 = math.radians(self.d_lat)
        lon1 = math.radians(self.d_lon)
        lat2 = math.radians(self.home_lat)
        lon2 = math.radians(self.home_lon)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        dist_m = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        speed_mps = max(self.d_speed, 3.0)          # 최소 3m/s 가정
        return_time_s = dist_m / speed_mps
        needed_pct = self.bat_drain_rate * return_time_s + 10.0  # 10% 예비
        self.return_margin_pct = self.d_bat_p - needed_pct

        if self.return_margin_pct > 20:
            self.return_feasibility = "OK"
            self.return_time_limit_s = None
        elif self.return_margin_pct > 0:
            self.return_feasibility = "MARGINAL"
            self.return_time_limit_s = self.return_margin_pct / self.bat_drain_rate
        else:
            self.return_feasibility = "IMPOSSIBLE"
            self.return_time_limit_s = 0
            self.log_safety_event(
                "RETURN_IMPOSSIBLE", "CRITICAL",
                f"귀환 가능성 한계 도달 (여유: {self.return_margin_pct:.1f}%)")

    # ------------------------------------------------------------
    # 위험 추세 감지 (↑ → ↓)
    # ------------------------------------------------------------

    def _calc_trends(self):
        """
        최근 30초 윈도우에서 GPS 위성 수, 링크 지연, 배터리 소모율의
        추세 방향을 계산한다. ↑(개선) / →(안정) / ↓(악화)
        """
        now = time.time()
        window = 30.0

        # GPS 추세
        recent = [(t, s) for t, s in self.sats_history if now - t <= window]
        if len(recent) >= 4:
            half = len(recent) // 2
            first_avg = sum(s for t, s in recent[:half]) / half
            last_avg = sum(s for t, s in recent[half:]) / (len(recent) - half)
            if last_avg > first_avg + 1.0:
                self.trend_gps = "↑"
            elif last_avg < first_avg - 1.0:
                self.trend_gps = "↓"
            else:
                self.trend_gps = "→"

        # 링크 추세: HEARTBEAT 수신 간격으로 추정
        if self.last_heartbeat_time is not None:
            hb_delay = now - self.last_heartbeat_time
            if hb_delay >= 2.0:
                self.trend_link = "↓"
            elif hb_delay < 1.0:
                self.trend_link = "→"

        # 선제 경보: GPS가 아직 GOOD이어도 악화 추세면 이벤트
        if self.trend_gps == "↓" and self.d_sats <= 9:
            self.log_safety_event(
                "GPS_DEGRADING", "WARNING",
                f"GPS 위성 수 감소 추세 ({self.d_sats}개)")

        # 배터리 추세: 소모율로 단순 판단
        if self.bat_drain_rate > 0.05:      # 분당 3% 이상 소모
            self.trend_battery = "↓"
        elif self.bat_drain_rate > 0.02:
            self.trend_battery = "→"
        else:
            self.trend_battery = "↑"        # 소모 거의 없음 (호버링 등)

    # ------------------------------------------------------------
    # 작전 상황 기반 대응 매트릭스
    # ------------------------------------------------------------

    def _get_recommended_action(self):
        """
        현재 링크/GPS/배터리/귀환가능성 조합으로 대응 매트릭스를 조회해
        (action, reason) 튜플을 반환한다.
        """
        # 규정 우선: 잔량 20% 이하(80% 소모) 도달 시 — 제자리 강제 착륙보다 RTL(귀환)을
        # 우선한다. 귀환이 불가능할 때(잔량 부족/홈 미설정 등)에만 최후수단으로 착륙.
        if self.battery_return_required:
            if self.return_feasibility == "IMPOSSIBLE" or self.home_lat is None:
                return "LAND", f"배터리 {self.d_bat_p}% — 귀환 불가, 최후수단 착륙"
            return "RTL", f"배터리 {self.d_bat_p}% — 규정상 즉시 귀환(RTL)"

        for row in self.ACTION_MATRIX:
            link_c, gps_c, bat_c, ret_c, action, reason = row
            if (link_c == "*" or link_c == self.link_state) and \
               (gps_c == "*" or gps_c == self.gps_state) and \
               (bat_c == "*" or bat_c == self.battery_state) and \
               (ret_c == "*" or ret_c == self.return_feasibility):
                return action, reason
        return "CONTINUE", "정상"

    def _execute_recommended_action(self):
        """'권장 실행' 버튼 핸들러 — 현재 권장 행동을 실제로 수행한다."""
        action = self.recommended_action
        if action == "RTL":
            self.set_drone_mode("RTL")
            self.log_safety_event("ACTION_RTL", "WARNING", "운용자 권장 RTL 실행")
        elif action == "LAND":
            self.set_drone_mode("LAND")
            self.log_safety_event("ACTION_LAND", "CRITICAL", "운용자 권장 LAND 실행")
        elif action == "HOVER":
            self.cmd_hover()
            self.log_safety_event("ACTION_HOVER", "WARNING", "운용자 권장 HOVER 실행")
        else:
            ToastMessage(self, "현재 권장 행동: 임무 계속", duration=2000)

    # ------------------------------------------------------------
    # 동적 경로 재계획 (In-flight Replanning)
    # ------------------------------------------------------------

    def _trigger_dynamic_replan(self, reason="위험 감지"):
        """
        비행 중 CRITICAL/LOST 상태 시 남은 미션을 중단하고
        현재 위치→홈 A* 안전 경로로 즉시 대체한다.
        REPLAN_COOLDOWN_S 이내 중복 트리거를 방지한다.
        UI/모드 조작(start_mission 포함)은 반드시 run_on_ui로 메인 스레드에서 실행한다.
        """
        now = time.time()
        if now - self.last_replan_time < self.REPLAN_COOLDOWN_S:
            return
        if not (self.mission_active or self.auto_mission_active):
            return
        if self.home_lat is None or self.home_lon is None:
            return
        if self.d_lat == 0.0 and self.d_lon == 0.0:
            return

        self.last_replan_time = now
        # GUIDED·AUTO 미션 모두 takeover 대상 → 재진입 방지 위해 AUTO 플래그 즉시 해제
        self.auto_mission_active = False
        self._auto_seen = False
        start = (self.d_lat, self.d_lon)
        goal = (self.home_lat, self.home_lon)
        # 현재 미션 고도 또는 현재 고도로 귀환
        alt = self.mission_alt if self.mission_alt else max(self.d_alt, 10.0)

        # A* 경로계획은 수십 ms 걸릴 수 있어 UI를 막지 않도록 워커 스레드에서 수행한다.
        # 위험상황은 운용자 회피정책과 무관하게 항상 Rally/홈 귀환을 시도한다.
        def _worker():
            saved_policy = self.avoidance_action_setting
            self.avoidance_action_setting = AVOIDANCE_ACTION_RTL_RALLY
            decision = self._evaluate_avoidance("RISK_REPLAN", start, goal, alt)
            self.avoidance_action_setting = saved_policy

            # 기존 미션 중단(회피 복귀 상태도 함께 리셋됨)
            self.run_on_ui(self.stop_mission)

            if decision["action"] in ("REROUTE", "DIRECT", "RTL"):
                # DIRECT(장애물 없음)=직선 귀환, REROUTE/RTL=우회 귀환 경로
                wps = decision.get("waypoints") or [start, goal]
                self.log_safety_event(
                    "DYNAMIC_REPLAN", "CRITICAL",
                    f"{reason} — 귀환 경로 재계획 ({len(wps)}개 WP)")
                self.run_on_ui(
                    lambda: ToastMessage(self, f"⚠ 동적 재계획: {reason}", duration=4000))
                self.run_on_ui(lambda: self.start_mission(wps, alt))
            else:
                # 경로 없음(FAILED) → 제자리 착륙
                self.log_safety_event(
                    "FORCED_LAND", "CRITICAL",
                    f"{reason} — 재계획 실패, 제자리 착륙")
                self.run_on_ui(lambda: self.set_drone_mode("LAND"))
                self.run_on_ui(
                    lambda: ToastMessage(self, "⚠ 재계획 실패 — 제자리 착륙", duration=4000))

        threading.Thread(target=_worker, daemon=True).start()

    def _trigger_zone_breach_avoidance(self, breached_zone):
        """
        위협구역/금지구역 내부 진입 시 운용자 정책과 무관하게 Rally/홈
        방향의 안전 경로를 계획해 현재 구역에서 탈출한다.
        """
        if self.home_lat is None or self.home_lon is None:
            self.run_on_ui(self.cmd_hover)
            self.log_safety_event(
                "ZONE_BREACH_HOVER", "CRITICAL",
                "구역 진입 — 홈 위치 미설정으로 제자리 정지")
            return

        if self.d_lat == 0.0 and self.d_lon == 0.0:
            return

        # AUTO 미션도 즉시 takeover 대상으로 표시한다.
        self.auto_mission_active = False
        self._auto_seen = False
        start = (self.d_lat, self.d_lon)
        goal = (self.home_lat, self.home_lon)
        alt = self.mission_alt if self.mission_alt else max(self.d_alt, 10.0)

        def _worker():
            # 1) 우선 Rally/홈 귀환 경로를 시도(정책 무관 강제)
            saved_policy = self.avoidance_action_setting
            try:
                self.avoidance_action_setting = AVOIDANCE_ACTION_RTL_RALLY
                decision = self._evaluate_avoidance(
                    "ZONE_BREACH", start, goal, alt)
            except Exception as error:
                print("[ZONE BREACH AVOID ERROR]", error)
                decision = {"action": "FAILED", "reason": str(error)}
            finally:
                self.avoidance_action_setting = saved_policy

            self.run_on_ui(self.stop_mission)

            if decision["action"] in ("REROUTE", "DIRECT", "RTL"):
                wps = decision.get("waypoints") or [start, goal]
                self.log_safety_event(
                    "ZONE_BREACH_REPLAN", "CRITICAL",
                    f"위협구역 탈출 경로 재계획 ({len(wps)}개 WP)")
                self.run_on_ui(lambda: ToastMessage(
                    self, "🚨 위협구역 탈출 경로로 이동합니다", duration=4000))
                self.run_on_ui(lambda: self.start_mission(wps, alt))
                return

            # 2) 홈/Rally가 막힘(구역 안 등) → 가장 가까운 안전지점으로 직접 탈출
            try:
                safe = path_planner.nearest_safe_point(
                    start, self.no_fly_zones,
                    cell_size_m=self.PLAN_CELL_M, safety_margin_m=self.PLAN_MARGIN_M)
            except Exception as error:
                print("[ZONE BREACH SAFE-POINT ERROR]", error)
                safe = None

            if safe is not None and safe != start:
                escape = [start, (safe[0], safe[1])]
                self.log_safety_event(
                    "ZONE_BREACH_ESCAPE", "CRITICAL",
                    f"가장 가까운 안전지점으로 탈출 ({safe[0]:.5f},{safe[1]:.5f})")
                self.run_on_ui(lambda: ToastMessage(
                    self, "🚨 가장 가까운 안전지점으로 탈출합니다", duration=4000))
                self.run_on_ui(lambda: self.start_mission(escape, alt))
            else:
                # 3) 탈출 지점도 못 찾음 → 제자리 정지(호버). 절대 LAND/시동해제 안 함.
                self.log_safety_event(
                    "ZONE_BREACH_HOVER", "CRITICAL",
                    "탈출 지점 계산 실패 — 제자리 정지(호버)")
                self.run_on_ui(self.cmd_hover)
                self.run_on_ui(lambda: ToastMessage(
                    self, "⚠ 탈출 지점을 찾지 못해 제자리 정지합니다", duration=4000))

        threading.Thread(target=_worker, daemon=True).start()

    # ------------------------------------------------------------
    # 미션 저장 / 불러오기 (JSON)
    # ------------------------------------------------------------

    def save_mission_dialog(self):
        """파일 저장 다이얼로그 → save_mission."""
        if not self.planned_waypoints:
            ToastMessage(self, "저장할 웨이포인트가 없습니다.", duration=2000)
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON 미션 파일", "*.json"), ("모든 파일", "*.*")],
            title="미션 저장")
        if path:
            self.save_mission(path)

    def save_mission(self, filepath):
        """
        계획 웨이포인트, 금지구역, 위협 반경을 JSON으로 저장한다.
        (위협 반경은 no_fly_zones에 통합되어 있으므로 별도 키로 저장)
        """
        data = {
            "version": 1,
            "waypoints": [
                {"lat": la, "lon": lo, "alt": al}
                for la, lo, al in self.planned_waypoints
            ],
            "no_fly_zones": [
                [{"lat": lat, "lon": lon} for lat, lon in zone]
                for zone in self.no_fly_zones
                if zone not in self._threat_zone_refs  # 위협 반경 제외 (별도 저장)
            ],
            "threat_radii": [
                {"lat": la, "lon": lo, "radius_m": r}
                for la, lo, r in self.threat_radii
            ]
        }
        try:
            os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            ToastMessage(self, f"미션 저장 완료: {os.path.basename(filepath)}", duration=2500)
            self.set_status(f"미션 저장: {filepath}")
        except Exception as error:
            print(f"[SAVE MISSION ERROR] {error}")
            ToastMessage(self, f"저장 실패: {error}", duration=3000)

    def load_mission_dialog(self):
        """파일 열기 다이얼로그 → load_mission."""
        path = filedialog.askopenfilename(
            filetypes=[("JSON 미션 파일", "*.json"), ("모든 파일", "*.*")],
            title="미션 불러오기")
        if path:
            self.load_mission(path)

    def load_mission(self, filepath):
        """JSON 미션 파일을 불러와 웨이포인트·금지구역·위협 반경을 복원한다."""
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)

            # 1) 기존 상태 초기화 (지도 표시 포함)
            self.clear_planned_waypoints()
            self.clear_threat_radii()
            self.clear_zones()

            # 2) 웨이포인트 복원
            for wp in data.get("waypoints", []):
                self.planned_waypoints.append(
                    (wp["lat"], wp["lon"], wp.get("alt", self.wp_default_alt)))
            self._draw_wp_markers()

            # 3) 금지구역 복원 (지도 표시 포함)
            for zone_data in data.get("no_fly_zones", []):
                zone = [(p["lat"], p["lon"]) for p in zone_data]
                if len(zone) >= 3:
                    self.no_fly_zones.append(zone)
                    try:
                        poly = self.map_widget.set_polygon(
                            zone, fill_color="red",
                            outline_color="#8B0000", border_width=2)
                        self.zone_polygons.append(poly)
                    except Exception:
                        pass

            # 4) 위협 반경 복원 (팝업 없이 직접 처리)
            for tr in data.get("threat_radii", []):
                la, lo, r = tr["lat"], tr["lon"], tr["radius_m"]
                self.threat_radii.append((la, lo, r))
                poly = self.circle_to_polygon(la, lo, r)
                self.no_fly_zones.append(poly)
                self._threat_zone_refs.append(poly)
                try:
                    circle = self.map_widget.set_polygon(
                        poly, fill_color=None,
                        outline_color="#e74c3c", border_width=2)
                    self.threat_circles.append(circle)
                except Exception:
                    pass

            n_wp = len(self.planned_waypoints)
            n_zone = len(self.no_fly_zones)
            n_threat = len(self.threat_radii)
            ToastMessage(
                self,
                f"미션 불러오기 완료: WP {n_wp}개 / 금지구역 {n_zone}개 / 위협 {n_threat}개",
                duration=3000)
            self.set_status(f"미션 불러오기: {filepath}")
        except Exception as error:
            print(f"[LOAD MISSION ERROR] {error}")
            ToastMessage(self, f"불러오기 실패: {error}", duration=3000)

    # ------------------------------------------------------------
    # 관심지점(POI) 마킹 + 목록
    # ------------------------------------------------------------

    def add_poi(self, lat, lon, priority="NORMAL", note=""):
        """지도 우클릭으로 관심지점(POI)을 추가한다."""
        self.poi_counter += 1
        poi_id = self.poi_counter
        mgrs_str = self.latlon_to_mgrs(lat, lon)   # A3
        poi = {
            "id": poi_id, "lat": lat, "lon": lon,
            "mgrs": mgrs_str,                       # A3
            "priority": priority, "note": note,
            "time": time.strftime("%H:%M:%S"),
        }
        self.poi_list.append(poi)
        self._draw_poi_marker(poi)
        self._refresh_poi_panel()
        self.log_safety_event(
            "POI_ADDED", "INFO",
            f"POI #{poi_id} [{priority}] {mgrs_str or f'({lat:.5f},{lon:.5f})'}")
        self.set_status(f"POI #{poi_id} 추가 [{priority}]")

    def remove_poi(self, poi_id):
        """POI를 목록과 지도에서 제거한다."""
        self.poi_list = [p for p in self.poi_list if p["id"] != poi_id]
        marker = self.poi_markers.pop(poi_id, None)
        if marker:
            try:
                marker.delete()
            except Exception:
                pass
        self._refresh_poi_panel()

    def goto_poi(self, poi_id):
        """선택한 POI 위치로 드론을 이동시킨다."""
        poi = next((p for p in self.poi_list if p["id"] == poi_id), None)
        if poi is None:
            return
        if self.drone is None or not self.d_is_armed:
            ToastMessage(self, "드론 ARM 상태에서만 이동 가능합니다.", duration=2000)
            return
        alt = max(self.d_alt, 5.0)
        self.plan_and_execute(poi["lat"], poi["lon"], alt)
        self.set_status(f"POI #{poi_id}로 이동 중")

    def _draw_poi_marker(self, poi):
        """POI를 지도에 마커로 표시한다."""
        marks = {"HIGH": "[긴급]", "NORMAL": "[일반]", "LOW": "[참고]"}
        label = f"{marks.get(poi['priority'], 'POI')} #{poi['id']}"
        try:
            marker = self.map_widget.set_marker(poi["lat"], poi["lon"], text=label)
            self.poi_markers[poi["id"]] = marker
        except Exception as error:
            print(f"[POI MARKER ERROR] {error}")

    def _refresh_poi_panel(self):
        """POI 목록 패널을 최신 poi_list로 다시 그린다."""
        try:
            for widget in self.poi_panel_inner.winfo_children():
                widget.destroy()
        except Exception:
            return
        priority_order = {"HIGH": 0, "NORMAL": 1, "LOW": 2}
        sorted_pois = sorted(self.poi_list,
                             key=lambda p: priority_order.get(p["priority"], 9))
        if not sorted_pois:
            tk.Label(self.poi_panel_inner, text="POI 없음",
                     fg="#95a5a6", bg="#0e1828",
                     font=("맑은 고딕", 9)).pack(pady=4)
        color = {"HIGH": "#e74c3c", "NORMAL": "#f1c40f", "LOW": "#3498db"}
        for poi in sorted_pois:
            row = tk.Frame(self.poi_panel_inner, bg=color.get(poi["priority"], "#ecf0f1"))
            row.pack(fill="x", pady=1, padx=2)
            tk.Label(row,
                     text=f"#{poi['id']} [{poi['priority']}] {poi.get('mgrs','') or poi['time']}",
                     font=("맑은 고딕", 8, "bold"), bg=row["bg"]).pack(anchor="w", padx=4)
            btn_row = tk.Frame(row, bg=row["bg"])
            btn_row.pack(fill="x")
            _id = poi["id"]
            tk.Button(btn_row, text="이동", font=("맑은 고딕", 8),
                      command=lambda i=_id: self.goto_poi(i)).pack(side="left", padx=2, pady=1)
            tk.Button(btn_row, text="삭제", font=("맑은 고딕", 8),
                      command=lambda i=_id: self.remove_poi(i)).pack(side="left", padx=2, pady=1)

        # POI 목록 아래에 Rally Point 구분 섹션을 함께 그린다.
        self._render_rally_section()

    # ------------------------------------------------------------
    # 비상 대피지점 (Rally Point) + 회피 정책
    # ------------------------------------------------------------

    def add_rally_point(self, lat, lon, alt=None):
        """비상 귀환 시 사용할 대피지점(Rally Point)을 추가한다."""
        self.rally_counter += 1
        rp_id = self.rally_counter
        rp = {
            "id": rp_id, "lat": lat, "lon": lon,
            "alt": alt if alt is not None else (self.mission_alt or 10.0),
            "time": time.strftime("%H:%M:%S"),
        }
        self.rally_points.append(rp)
        self._draw_rally_marker(rp)
        self._refresh_rally_panel()
        self.log_safety_event("RALLY_ADDED", "INFO",
                              f"Rally Point #{rp_id} ({lat:.5f},{lon:.5f})")
        self.set_status(f"Rally Point #{rp_id} 추가")

    def remove_rally_point(self, rp_id):
        """Rally Point를 목록과 지도에서 제거한다."""
        self.rally_points = [r for r in self.rally_points if r["id"] != rp_id]
        marker = self.rally_markers.pop(rp_id, None)
        if marker:
            try:
                marker.delete()
            except Exception:
                pass
        self._refresh_rally_panel()

    def _draw_rally_marker(self, rp):
        """지도에 Rally Point 마커를 그린다. (POI와 구분: 주황색 계열)"""
        try:
            marker = self.map_widget.set_marker(
                rp["lat"], rp["lon"], text=f"RP{rp['id']}",
                marker_color_circle="#e67e22", marker_color_outside="#d35400")
            self.rally_markers[rp["id"]] = marker
        except Exception as error:
            print("[RALLY MARKER ERROR]", error)

    def _nearest_rally_point(self, current_pos):
        """현재 위치에서 가장 가까운 Rally Point를 (lat, lon, alt)로 반환. 없으면 None."""
        if not self.rally_points:
            return None
        clat, clon = current_pos
        best, best_dist = None, float("inf")
        for rp in self.rally_points:
            dx, dy = path_planner.latlon_to_local(clat, clon, rp["lat"], rp["lon"])
            dist = math.hypot(dx, dy)
            if dist < best_dist:
                best, best_dist = rp, dist
        return (best["lat"], best["lon"], best["alt"]) if best else None

    def _refresh_rally_panel(self):
        """Rally 목록 갱신 — POI 패널을 다시 그리면 그 안에 Rally 섹션이 함께 갱신된다."""
        self._refresh_poi_panel()

    def _render_rally_section(self):
        """POI 패널 하단에 Rally Point 목록을 그린다. (_refresh_poi_panel에서 호출)"""
        try:
            parent = self.poi_panel_inner
        except Exception:
            return
        tk.Label(parent, text="— Rally Points —",
                 fg="#e67e22", bg="#0e1828",
                 font=("맑은 고딕", 8, "bold")).pack(pady=(6, 1))
        if not self.rally_points:
            tk.Label(parent, text="Rally 없음", fg="#95a5a6", bg="#0e1828",
                     font=("맑은 고딕", 8)).pack(pady=1)
            return
        for rp in self.rally_points:
            row = tk.Frame(parent, bg="#e67e22")
            row.pack(fill="x", pady=1, padx=2)
            tk.Label(row, text=f"RP#{rp['id']} ({rp['lat']:.4f},{rp['lon']:.4f})",
                     font=("맑은 고딕", 8, "bold"), bg=row["bg"]).pack(anchor="w", padx=4)
            _id = rp["id"]
            tk.Button(row, text="삭제", font=("맑은 고딕", 8),
                      command=lambda i=_id: self.remove_rally_point(i)).pack(anchor="e", padx=2, pady=1)

    def on_avoidance_action_change(self):
        """회피 정책 라디오버튼 변경 핸들러."""
        self.avoidance_action_setting = self.avoidance_action_var.get()
        self.set_status(f"회피 정책 변경: {AVOIDANCE_ACTION_LABELS[self.avoidance_action_setting]}")
        self.log_safety_event("AVOIDANCE_POLICY_CHANGED", "INFO", self.avoidance_action_setting)

    # ------------------------------------------------------------
    # 임무 단계 관리
    # ------------------------------------------------------------

    def set_mission_phase(self, phase):
        """임무 단계를 변경하고 체크리스트를 검증한다."""
        checklist = self.PHASE_CHECKLIST.get(phase, [])
        failed = [label for label, check in checklist if not check(self)]
        if failed:
            msg = f"[{self.PHASE_LABELS.get(phase, phase)}] 체크리스트 미충족:\n"
            msg += "\n".join(f"  • {f}" for f in failed)
            msg += "\n\n그래도 진행하시겠습니까?"
            if not tk.messagebox.askyesno("체크리스트 경고", msg):
                return
        if phase == "TAKEOFF" and self.mission_start_time is None:
            self.mission_start_time = time.time()
            self.mission_id = time.strftime("MSN-%Y%m%d-") + \
                str(self.poi_counter % 1000).zfill(3)
        self.mission_phase = phase
        self._update_phase_indicator()
        self.log_safety_event(
            f"PHASE_{phase}", "INFO",
            f"임무 단계: {self.PHASE_LABELS.get(phase, phase)}")

    def _update_phase_indicator(self):
        """임무 단계 진행바와 라벨을 갱신한다."""
        phases = ["PRE_FLIGHT", "TAKEOFF", "TRANSIT", "PATROL", "RETURN"]
        phase_colors = {
            "PRE_FLIGHT": "#9db2c8", "TAKEOFF": "#3498db",
            "TRANSIT": "#2980b9", "PATROL": "#27ae60",
            "RETURN": "#e67e22", "LANDING": "#e74c3c",
            "COMPLETE": "#2ecc71",
        }
        try:
            cur_idx = phases.index(self.mission_phase) \
                if self.mission_phase in phases else -1
            for i, pip in enumerate(self.phase_pips):
                if i < cur_idx:
                    pip.config(bg="#34d399")
                elif i == cur_idx:
                    pip.config(bg="#f5b14c")
                else:
                    pip.config(bg="#2a3f5a")
            label = self.PHASE_LABELS.get(self.mission_phase, self.mission_phase)
            color = phase_colors.get(self.mission_phase, "#9db2c8")
            self.phase_mini_label.config(text=label, fg=color)
        except Exception:
            pass

        try:
            label = self.PHASE_LABELS.get(self.mission_phase, self.mission_phase)
            self.bottom_phase_label.config(text=f"단계: {label}")
        except Exception:
            pass

    def _auto_detect_phase(self):
        """텔레메트리로 임무 단계를 자동 감지한다. update_drone_data 1초 주기 호출."""
        if not self.d_is_armed:
            if self.mission_phase not in ("PRE_FLIGHT", "COMPLETE"):
                self.set_mission_phase("COMPLETE")
            return
        mode = self.d_mode.strip().upper()
        if self.mission_phase == "PRE_FLIGHT" and self.d_alt > 1.0:
            self.set_mission_phase("TAKEOFF")
        elif self.mission_phase == "TAKEOFF" and self.d_alt > 3.0:
            in_mission = self.mission_active or self.auto_mission_active
            self.set_mission_phase("TRANSIT" if in_mission else "PATROL")
        elif mode == "RTL" and self.mission_phase not in ("RETURN", "LANDING"):
            self.set_mission_phase("RETURN")
        elif mode == "LAND" and self.mission_phase != "LANDING":
            self.set_mission_phase("LANDING")

    # ------------------------------------------------------------
    # 단축키 + 야간 모드
    # ------------------------------------------------------------

    def _bind_shortcuts(self):
        """GCS 단축키를 바인딩한다."""
        shortcuts = {
            "<F1>":     lambda e: self.set_drone_mode("RTL"),
            "<F2>":     lambda e: self.cmd_hover(),
            "<F3>":     lambda e: self.set_drone_mode("LAND"),
            "<F4>":     lambda e: self.cmd_arm(),
            "<F5>":     lambda e: self.cmd_disarm(),
            "<Escape>": lambda e: self.abort_mission(),
            "n":        lambda e: self.toggle_night_mode(),
            "N":        lambda e: self.toggle_night_mode(),
            "m":        lambda e: self.toggle_wp_planning(),
            "M":        lambda e: self.toggle_wp_planning(),
        }
        for key, handler in shortcuts.items():
            try:
                self.bind(key, handler)
            except Exception as error:
                print(f"[SHORTCUT BIND ERROR] {key}: {error}")
        print("[단축키] F1=RTL F2=HOVER F3=LAND F4=ARM F5=DISARM ESC=미션중단 N=야간 M=미션계획")

    def toggle_night_mode(self):
        """
        야간/주간 모드를 전환한다 (단축키 N).
        야간 진입 시 위젯별 원래 색을 저장하고, 주간 복귀 시 그 색을 그대로
        복원한다. (긴급 버튼·안전도 색 등 위젯 고유 색이 망가지지 않도록)
        """
        self.night_mode = not self.night_mode
        try:
            if self.night_mode:
                self._orig_colors = {}
                self._night_root_bg = self.cget("bg")
                self.configure(bg=self.NIGHT_COLORS["bg"])
                self._apply_night_recursive(self, self.NIGHT_COLORS)
            else:
                self._restore_day_colors()
            self.set_status(f"{'야간' if self.night_mode else '주간'} 모드 전환")
        except Exception as error:
            print(f"[NIGHT MODE ERROR] {error}")

    def _apply_night_recursive(self, widget, theme):
        """위젯 트리를 순회하며 원래 색을 저장한 뒤 야간 테마 색을 적용한다."""
        try:
            wclass = widget.winfo_class()
            opts = {}
            if wclass in ("Frame", "LabelFrame"):
                opts = {"bg": theme["panel_bg"]}
            elif wclass == "Label":
                opts = {"bg": theme["panel_bg"], "fg": theme["label_fg"]}
            elif wclass == "Button":
                opts = {"bg": theme["btn_bg"], "fg": theme["btn_fg"]}
            elif wclass == "Canvas":
                opts = {"bg": theme["bg"]}
            if opts:
                # 원래 색 저장 후 야간 색 적용
                self._orig_colors[widget] = {k: widget.cget(k) for k in opts}
                widget.configure(**opts)
        except Exception:
            pass
        for child in widget.winfo_children():
            self._apply_night_recursive(child, theme)

    def _restore_day_colors(self):
        """야간 진입 전 저장해둔 위젯별 원래(주간) 색을 그대로 되돌린다."""
        try:
            self.configure(bg=getattr(self, "_night_root_bg", self.DAY_COLORS["bg"]))
        except Exception:
            pass
        for widget, saved in list(self._orig_colors.items()):
            try:
                if widget.winfo_exists():
                    widget.configure(**saved)
            except Exception:
                pass
        self._orig_colors = {}

    # ------------------------------------------------------------
    # 통신·GPS 동시 이상 감지
    # ------------------------------------------------------------

    def _detect_link_gps_anomaly(self):
        """GPS와 통신 링크가 동시에 저하되는 패턴을 감지해 복합 이상 상황으로 분류한다."""
        now = time.time()
        if now - self.last_anomaly_check_time < 30.0:
            return
        score = 0
        if self.gps_state in ("WEAK", "BAD"):
            score += 2
        if self.link_state in ("WARNING", "CRITICAL"):
            score += 2
        if self.trend_gps == "↓":
            score += 1
        if self.trend_link == "↓":
            score += 1
        if score >= 4 and not self.anomaly_suspected:
            self.anomaly_suspected = True
            self.last_anomaly_check_time = now
            self.log_safety_event(
                "LINK_GPS_ANOMALY", "CRITICAL",
                f"통신·GPS 동시 이상 감지 — GPS {self.gps_state} + 링크 {self.link_state}")
            if self.d_lat != 0.0 or self.d_lon != 0.0:
                _lat, _lon = self.d_lat, self.d_lon  # 람다 캡처용 값 복사
                self.run_on_ui(lambda: self._auto_mark_threat_zone(
                    _lat, _lon, radius_m=200, label="복합 이상 감지"))
            self.run_on_ui(lambda: ToastMessage(
                self, "⚠ 통신·GPS 동시 이상 감지 — 위험 구역 자동 표시", duration=5000))
        elif score < 2:
            self.anomaly_suspected = False

    def _auto_mark_threat_zone(self, lat, lon, radius_m=200, label="자동 위협"):
        """팝업 없이 위협 반경을 자동 추가한다. (반드시 run_on_ui로 호출)"""
        self.threat_radii.append((lat, lon, radius_m))
        poly = self.circle_to_polygon(lat, lon, radius_m)
        self.no_fly_zones.append(poly)
        self._threat_zone_refs.append(poly)
        try:
            circle = self.map_widget.set_polygon(
                poly, fill_color=None, outline_color="#c0392b", border_width=2)
            self.threat_circles.append(circle)
        except Exception as error:
            print(f"[AUTO THREAT ERROR] {error}")
        self.set_status(f"자동 위협 구역: {label} 반경 {radius_m}m")

    # ------------------------------------------------------------
    # 임무 결과 자동 보고서
    # ------------------------------------------------------------

    def generate_mission_report(self):
        """임무 종료(DISARM 감지) 시 자동으로 텍스트 보고서를 생성한다."""
        if self.mission_start_time is None:
            return
        duration_s = time.time() - self.mission_start_time
        mins, secs = int(duration_s // 60), int(duration_s % 60)
        event_counts = {}
        for ev in self.safety_events:
            t = ev.get("event_type", "UNKNOWN")
            event_counts[t] = event_counts.get(t, 0) + 1
        # POI 위치를 MGRS로(없으면 위경도로) 표시
        poi_lines = ""
        for p in self.poi_list:
            loc = p.get("mgrs", "") or f"({p['lat']:.5f},{p['lon']:.5f})"
            poi_lines += f"  #{p['id']} [{p['priority']}] {loc} {p['time']} {p['note']}\n"
        poi_lines = poi_lines or "  없음\n"
        event_lines = "".join(
            f"  {k}: {v}회\n"
            for k, v in sorted(event_counts.items())) or "  없음\n"
        report = (
            f"\n{'='*55}\n[임무 결과 보고서]\n{'='*55}\n"
            f"임무 ID    : {self.mission_id or 'N/A'}\n"
            f"임무 이름  : {self.mission_name or '미지정'}\n"
            f"생성 시각  : {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"임무 시간  : {mins}분 {secs}초\n\n"
            f"[비행 기록]\n"
            f"최대 고도  : {self.max_alt_recorded:.1f} m\n"
            f"최대 거리  : {self.max_dist_recorded:.0f} m\n\n"
            f"[안전도 기록]\n"
            f"최저 안전도: {self.min_risk_score_recorded}점 ({self.min_risk_level_recorded})\n\n"
            f"[이상 이벤트]\n{event_lines}\n"
            f"[관심지점(POI)]\n{poi_lines}\n{'='*55}\n"
        )
        try:
            report_dir = os.path.join(self.base_dir, "reports")
            os.makedirs(report_dir, exist_ok=True)
            filename = f"mission_report_{time.strftime('%Y%m%d_%H%M%S')}.txt"
            filepath = os.path.join(report_dir, filename)
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(report)
            self.set_status(f"임무 보고서 저장: reports/{filename}")
            self.run_on_ui(lambda: ToastMessage(self, "임무 보고서 저장 완료", duration=3000))
            print(f"[REPORT] {filepath}")
        except Exception as error:
            print(f"[REPORT ERROR] {error}")
        # 다음 임무를 위해 초기화
        self.mission_start_time = None
        self.mission_id = None
        self.min_risk_score_recorded = 100
        self.min_risk_level_recorded = "NORMAL"
        self.max_alt_recorded = 0.0
        self.max_dist_recorded = 0.0

    # ------------------------------------------------------------
    # 사전 위험 검증
    # ------------------------------------------------------------

    def _preflight_risk_check(self):
        """미션 업로드 전 사전 위험을 분석한다. 치명 문제 시 False 반환."""
        issues, warnings = [], []
        # 1. 배터리 (전체 경로 거리 기반 예상 소모율)
        if self.planned_waypoints:
            # 시작 위치가 유효(GPS 수신)할 때만 드론→첫 WP 구간을 포함한다.
            # 위치가 (0,0)이면 거리가 수천 km로 폭주하므로 WP 간 거리만 사용.
            pos_valid = (self.d_lat != 0.0 or self.d_lon != 0.0)
            pts = ([(self.d_lat, self.d_lon)] if pos_valid else []) + \
                  [(la, lo) for la, lo, _ in self.planned_waypoints]
            total_dist = sum(
                math.hypot((pts[i+1][0]-pts[i][0])*111320,
                           (pts[i+1][1]-pts[i][1])*111320*math.cos(math.radians(pts[i][0])))
                for i in range(len(pts)-1))
            # 실제 비행 중(시동+이동)이면 측정 소모율을, 정지(사전 계획) 상태이면
            # 보수적 기본 모델을 사용한다. 정지 상태의 측정값/속도는 비현실적이라 쓰지 않는다.
            if self.d_is_armed and self.d_speed > 1.0 and 0 < self.bat_drain_rate <= 0.5:
                drain = self.bat_drain_rate              # %/s (비행 중 실측, 상한 0.5)
                cruise = self.d_speed                    # m/s
            else:
                drain = self.BATTERY_DRAIN_DEFAULT_PCT_S  # 통상 멀티콥터 평균(25분)
                cruise = self.CRUISE_SPEED_DEFAULT_MPS    # m/s 순항 가정
            needed = min(drain * (total_dist / cruise) + 15, 999)
            # 잔량이 유효(0 초과)할 때만 배터리로 차단한다. 초기화 직후 등
            # 잔량이 불명(-1)·0으로 잠깐 보고되는 경우의 오차단을 방지한다.
            if self.d_bat_p is not None and self.d_bat_p > 0:
                if needed > self.d_bat_p:
                    issues.append(f"배터리 부족: 필요 {needed:.0f}% > 현재 {self.d_bat_p}%")
                elif needed > self.d_bat_p * 0.8:
                    warnings.append(f"배터리 여유 부족: 예상 소모 {needed:.0f}%")
        # 2. GPS
        if self.gps_state == "BAD":
            issues.append(f"GPS 상태 불량: 위성 {self.d_sats}개")
        elif self.gps_state == "WEAK":
            warnings.append(f"GPS 약화: 위성 {self.d_sats}개")
        # 3. 링크
        if self.link_state in ("CRITICAL", "LOST"):
            issues.append(f"통신 위험: {self.link_state}")
        elif self.link_state == "WARNING":
            warnings.append("통신 지연 감지")
        # 4. 위협 근접
        for i, (la, lo, _) in enumerate(self.planned_waypoints):
            for tla, tlo, tr in self.threat_radii:
                dist = math.hypot((la-tla)*111320,
                                  (lo-tlo)*111320*math.cos(math.radians(tla)))
                if dist < tr * 1.2:
                    warnings.append(f"WP{i+1} — 위협 반경 {dist:.0f}m 근접")
        # 결과
        if issues:
            msg = "치명적 문제:\n\n" + "\n".join(f"• {x}" for x in issues)
            if warnings:
                msg += "\n\n추가 경고:\n" + "\n".join(f"• {w}" for w in warnings)
            tk.messagebox.showerror("사전 검증 실패", msg)
            return False
        if warnings:
            msg = "경고:\n\n" + "\n".join(f"• {w}" for w in warnings) + \
                  "\n\n그래도 미션을 시작하시겠습니까?"
            return tk.messagebox.askyesno("사전 검증 경고", msg)
        return True

    # ------------------------------------------------------------
    # 비행 이력 트레일
    # ------------------------------------------------------------

    def _update_trail_line(self):
        """비행 이력 트레일을 지도에 표시한다."""
        if len(self.flight_trail) < 2:
            return
        try:
            if self.trail_line is not None:
                self.trail_line.delete()
            self.trail_line = self.map_widget.set_path(
                self.flight_trail, color="#27ae60", width=2)
        except Exception as error:
            print(f"[TRAIL ERROR] {error}")

    def clear_flight_trail(self):
        """비행 이력 트레일을 지운다."""
        self.flight_trail = []
        self._last_trail_lat = self._last_trail_lon = None
        if self.trail_line is not None:
            try:
                self.trail_line.delete()
            except Exception:
                pass
            self.trail_line = None

    # ------------------------------------------------------------
    # MGRS 좌표계 변환/표시
    # ------------------------------------------------------------

    def latlon_to_mgrs(self, lat, lon):
        """위경도를 MGRS 문자열로 변환한다. 실패 시 빈 문자열을 반환한다."""
        if not _MGRS_AVAILABLE or self._mgrs_converter is None:
            return ""
        try:
            return self._mgrs_converter.toMGRS(lat, lon)
        except Exception as error:
            print(f"[MGRS CONVERT ERROR] {error}")
            return ""

    def mgrs_to_latlon(self, mgrs_str):
        """MGRS 문자열을 (lat, lon) 튜플로 변환한다. 실패 시 None을 반환한다."""
        if not _MGRS_AVAILABLE or self._mgrs_converter is None:
            return None
        try:
            lat, lon = self._mgrs_converter.toLatLon(mgrs_str.strip())
            return (lat, lon)
        except Exception as error:
            print(f"[MGRS PARSE ERROR] {error}")
            return None

    def toggle_coord_display(self):
        """좌표 표시 모드를 위경도 ↔ MGRS로 전환한다."""
        if not _MGRS_AVAILABLE:
            ToastMessage(self, "MGRS 라이브러리가 설치되지 않았습니다.", duration=2500)
            return
        self.coord_display_mode = (
            "MGRS" if self.coord_display_mode == "LATLON" else "LATLON")
        self._update_coord_display_label()
        self.set_status(f"좌표 표시: {self.coord_display_mode}")

    def _update_coord_display_label(self):
        """현재 드론 위치를 선택된 좌표 형식으로 표시한다."""
        try:
            if self.coord_display_mode == "MGRS" and _MGRS_AVAILABLE:
                mgrs_str = self.latlon_to_mgrs(self.d_lat, self.d_lon)
                text = mgrs_str if mgrs_str else "MGRS 변환 실패"
            else:
                text = f"{self.d_lat:.6f}, {self.d_lon:.6f}"
            self.coord_display_label.config(text=text)
        except Exception:
            pass

    def log_safety_event(self, event_type, severity, message):
        """
        안전 이벤트를 메모리 리스트(self.safety_events)와 CSV 파일에 기록한다.
        동일 이벤트 타입은 EVENT_COOLDOWN_S 이내 반복 저장하지 않는다.
        """
        if not self.event_log_enabled:
            return

        now = time.time()
        last_time = self.last_event_times.get(event_type, 0)
        if now - last_time < self.EVENT_COOLDOWN_S:
            return
        self.last_event_times[event_type] = now

        try:
            log_dir = os.path.join(self.base_dir, "logs")
            os.makedirs(log_dir, exist_ok=True)

            if self.safety_log_file_path is None:
                stamp = time.strftime("%Y%m%d_%H%M%S")
                self.safety_log_file_path = os.path.join(
                    log_dir, f"safety_events_{stamp}.csv")
                with open(self.safety_log_file_path, "w", encoding="utf-8-sig") as f:
                    f.write(
                        "timestamp,event_type,severity,mode,armed,lat,lon,alt,"
                        "speed,battery_percent,sats,message\n")

            event_time = time.strftime("%Y-%m-%d %H:%M:%S")
            self.safety_events.append({
                "timestamp": event_time, "event_type": event_type,
                "severity": severity, "mode": self.d_mode,
                "armed": self.d_is_armed, "lat": self.d_lat, "lon": self.d_lon,
                "alt": self.d_alt, "speed": self.d_speed,
                "battery_percent": self.d_bat_p, "sats": self.d_sats,
                "message": message,
            })

            safe_message = str(message).replace(",", " ")
            with open(self.safety_log_file_path, "a", encoding="utf-8-sig") as f:
                f.write(
                    f"{event_time},{event_type},{severity},{self.d_mode},"
                    f"{self.d_is_armed},{self.d_lat:.7f},{self.d_lon:.7f},"
                    f"{self.d_alt:.2f},{self.d_speed:.2f},{self.d_bat_p},"
                    f"{self.d_sats},{safe_message}\n")
        except Exception as error:
            if not self.closing:
                print("[SAFETY LOG ERROR]", error)

    def update_safety_panel(self):
        """evaluate_flight_safety() 결과를 오버레이 '임무 안전도' 패널에 표시한다."""
        try:
            self.risk_score_label.config(text=f"안전도: {self.flight_risk_score} / 100")
            self.risk_level_label.config(text=f"위험 단계: {self.risk_level}")
            self.link_state_label.config(text=f"링크: {self.link_state}")
            self.gps_state_label.config(text=f"GPS: {self.gps_state} ({self.d_sats}개)")
            self.battery_state_label.config(text=f"배터리: {self.battery_state} ({self.d_bat_p}%)")

            if self.risk_reasons:
                reason_text = "\n".join(self.risk_reasons[:3])
                self.risk_reason_label.config(text=f"최근 경고:\n{reason_text}")
            else:
                self.risk_reason_label.config(text="최근 경고: 없음")

            # 위험 단계별 색: 큰 안전도 숫자는 어두운 배경 + 색 글씨(가독성↑),
            # 작은 단계 배지는 색 배경 + 어두운 글씨(가독성↑)
            level_color = {
                "NORMAL": "#34d399", "WARNING": "#f5b14c",
                "CRITICAL": "#fb923c", "LOST": "#f47174",
            }.get(self.risk_level, "#9db2c8")
            self.risk_score_label.config(bg="#0e1828", fg=level_color)
            self.risk_level_label.config(bg=level_color, fg="#0e1828")

            # 귀환 가능성 표시
            if self.return_feasibility == "OK":
                ret_text = f"귀환여유: +{self.return_margin_pct:.0f}%  ✓"
                ret_color = "#2ecc71"
            elif self.return_feasibility == "MARGINAL":
                t = int(self.return_time_limit_s or 0)
                ret_text = f"귀환여유: +{self.return_margin_pct:.0f}%  ⚠ ({t//60}분{t%60}초)"
                ret_color = "#f1c40f"
            elif self.return_feasibility == "IMPOSSIBLE":
                ret_text = "귀환 불가  🔴 즉시 RTL"
                ret_color = "#e74c3c"
            else:
                ret_text = "귀환 예측: 계산 중"
                ret_color = "#95a5a6"
            self.return_label.config(text=ret_text, fg=ret_color)

            # 추세 표시
            self.trend_label.config(
                text=f"링크{self.trend_link}  GPS{self.trend_gps}  배터리{self.trend_battery}")

            # 권장 행동 표시
            action_colors = {
                "CONTINUE": "#2ecc71", "RTL": "#e74c3c",
                "LAND": "#c0392b", "HOVER": "#e67e22"
            }
            self.action_label.config(
                text=f"권장: {self.recommended_action} — {self.recommended_reason}",
                fg=action_colors.get(self.recommended_action, "#ecf0f1"))

            # 미니 뱃지 갱신: 전체 배경 대신 좌측 컬러 스트립만 위험색으로 표시
            try:
                self.mini_score_label.config(text=f"{self.flight_risk_score} / 100")
                strip_colors = {
                    "NORMAL": "#34d399", "WARNING": "#f5b14c",
                    "CRITICAL": "#fb923c", "LOST": "#f47174",
                }
                strip_color = strip_colors.get(self.risk_level, "#2a3f5a")
                self.mini_badge_strip.config(bg=strip_color)
                self.mini_score_label.config(fg=strip_color)
                self.mini_level_label.config(text=self.risk_level)
                self.mini_return_label.config(text=self.return_label.cget("text"))
                # 오버레이 패널 내부 위험단계 라벨은 기존처럼 색 강조 유지
                panel_bg_colors = {
                    "NORMAL": "#0d3d2a", "WARNING": "#3d2d00",
                    "CRITICAL": "#3d1500", "LOST": "#3d0d0d",
                }
                self.risk_level_label.config(
                    fg=strip_color,
                    bg=panel_bg_colors.get(self.risk_level, "#1a1a2e"))
            except Exception:
                pass

            # 하단 정보 바 갱신
            try:
                if self.mission_id:
                    self.bottom_msn_label.config(text=f"MSN: {self.mission_id}")
                else:
                    self.bottom_msn_label.config(text="MSN: --")

                phase_label = self.PHASE_LABELS.get(
                    self.mission_phase, self.mission_phase)
                self.bottom_phase_label.config(text=f"단계: {phase_label}")

                if self.mission_active and self.mission_waypoints:
                    wp_text = (
                        f"WP: {self.mission_index + 1} / "
                        f"{len(self.mission_waypoints)}")
                else:
                    wp_text = f"WP: {len(self.planned_waypoints)}개 계획"
                self.bottom_wp_label.config(text=wp_text)
            except Exception:
                pass

            # 좌표 표시 갱신
            try:
                self._update_coord_display_label()
            except Exception:
                pass
        except Exception as error:
            if not self.closing:
                print("[SAFETY PANEL UPDATE ERROR]", error)

    def reset_safety_state(self):
        """SITL 종료/재시작 시 안전 감시 상태를 초기화한다. (로그 파일은 유지)"""
        self.last_heartbeat_time = None
        self.last_position_time = None
        self.last_gps_time = None
        self.last_sys_status_time = None

        self.link_state = "DISCONNECTED"
        self.gps_state = "UNKNOWN"
        self.battery_state = "UNKNOWN"
        self.battery_return_required = False
        self._battery_return_warned = False
        self._battery_notify_50 = False
        self._battery_notify_30 = False

        # AUTO 미션 추적 상태 초기화
        self.auto_mission_active = False
        self._auto_seen = False
        self.in_threat_zone = False
        self.last_zone_breach_replan_time = 0.0
        self._zone_raw_prev = False
        self._zone_confirm = 0
        self.last_takeoff_time = 0.0

        self.flight_risk_score = 100
        self.risk_level = "NORMAL"
        self.risk_reasons = []

        self.last_safety_eval_time = 0.0

    # ------------------------------------------------------------
    # 가상 조이스틱 (RC 오버라이드, Mode 2)
    # ------------------------------------------------------------

    def create_virtual_joystick(self):
        """지도 위 하단에 표시할 가상 조이스틱(두 개의 스틱)을 만든다."""
        tk.Label(
            self.js_panel_frame, text="가상 조이스틱 (Mode 2)",
            fg="white", bg="#2c3e50", font=("맑은 고딕", 10, "bold")
        ).pack(pady=(5, 0))

        self.js_canvas = tk.Canvas(
            self.js_panel_frame, width=280, height=130,
            bg="#1c1c1c", highlightthickness=1, highlightbackground="#34495e")
        self.js_canvas.pack(padx=5, pady=5)

        # 스틱 중심 좌표 / 반경
        self.lx, self.ly = 65, 65     # 왼쪽 스틱 (Throttle / Yaw)
        self.rx, self.ry = 215, 65    # 오른쪽 스틱 (Pitch / Roll)
        self.js_radius = 45

        # 패드 배경 원 + 중심 십자선
        for cx, cy in [(self.lx, self.ly), (self.rx, self.ry)]:
            self.js_canvas.create_oval(
                cx - self.js_radius, cy - self.js_radius,
                cx + self.js_radius, cy + self.js_radius,
                outline="#34495e", width=2)
            self.js_canvas.create_line(cx - 10, cy, cx + 10, cy, fill="#34495e")
            self.js_canvas.create_line(cx, cy - 10, cx, cy + 10, fill="#34495e")

        # 조종 노브(Knob)
        self.left_knob = self.js_canvas.create_oval(
            self.lx - 12, self.ly - 12, self.lx + 12, self.ly + 12,
            fill="#e67e22", outline="white")
        self.right_knob = self.js_canvas.create_oval(
            self.rx - 12, self.ry - 12, self.rx + 12, self.ry + 12,
            fill="#3498db", outline="white")

        # 가이드 텍스트
        self.js_canvas.create_text(self.lx, self.ly + 52, text="스로틀 / 요",
                                   fill="#95a5a6", font=("Consolas", 8, "bold"))
        self.js_canvas.create_text(self.rx, self.ry + 52, text="피치 / 롤",
                                   fill="#95a5a6", font=("Consolas", 8, "bold"))

        # 드래그 / 해제 이벤트
        self.js_canvas.bind("<B1-Motion>", self.on_joystick_drag)
        self.js_canvas.bind("<ButtonRelease-1>", self.on_joystick_release)

    def on_joystick_drag(self, event):
        """스틱을 드래그하면 PWM(1000~2000, 중립 1500)으로 환산한다."""
        x, y = event.x, event.y

        if x < 140:
            # 왼쪽 스틱: 위/아래=스로틀, 좌/우=요
            dx, dy = x - self.lx, y - self.ly
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > self.js_radius:
                dx, dy = (dx / dist) * self.js_radius, (dy / dist) * self.js_radius
            self.js_canvas.coords(
                self.left_knob, self.lx + dx - 12, self.ly + dy - 12,
                self.lx + dx + 12, self.ly + dy + 12)
            self.js_throttle = int(1500 - (dy / self.js_radius) * 500)  # 위로 올리면 상승(+)
            self.js_yaw = int(1500 + (dx / self.js_radius) * 500)
        else:
            # 오른쪽 스틱: 위/아래=피치, 좌/우=롤
            dx, dy = x - self.rx, y - self.ry
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > self.js_radius:
                dx, dy = (dx / dist) * self.js_radius, (dy / dist) * self.js_radius
            self.js_canvas.coords(
                self.right_knob, self.rx + dx - 12, self.ry + dy - 12,
                self.rx + dx + 12, self.ry + dy + 12)
            self.js_pitch = int(1500 + (dy / self.js_radius) * 500)
            self.js_roll = int(1500 + (dx / self.js_radius) * 500)

        # 위협구역 근접 시 pitch/roll을 회피 방향으로 보정(throttle/yaw는 유지)
        self._apply_manual_avoidance_override()

    def on_joystick_release(self, event):
        """손을 떼면 스프링처럼 모든 스틱을 중립(1500)으로 되돌린다."""
        self.js_canvas.coords(
            self.left_knob, self.lx - 12, self.ly - 12, self.lx + 12, self.ly + 12)
        self.js_canvas.coords(
            self.right_knob, self.rx - 12, self.ry - 12, self.rx + 12, self.ry + 12)
        self.js_roll = 1500
        self.js_pitch = 1500
        self.js_throttle = 1500
        self.js_yaw = 1500

    def _estimate_manual_target(self):
        """
        현재 조이스틱 입력(js_pitch, js_roll)과 드론 헤딩(d_yaw)으로
        MANUAL_AVOID_LOOKAHEAD_S초 후 도달할 것으로 예상되는 위치를 추정한다.
        스틱이 중립이면 None(이동 의도 없음).
        """
        pitch_in = (self.js_pitch - 1500) / 500.0   # -1(후진) ~ +1(전진)
        roll_in = (self.js_roll - 1500) / 500.0     # -1(좌) ~ +1(우)

        if abs(pitch_in) < 0.1 and abs(roll_in) < 0.1:
            return None

        yaw_rad = math.radians(self.d_yaw)
        forward_m = pitch_in * self.MANUAL_AVOID_SPEED_MPS * self.MANUAL_AVOID_LOOKAHEAD_S
        right_m = roll_in * self.MANUAL_AVOID_SPEED_MPS * self.MANUAL_AVOID_LOOKAHEAD_S

        # 기체좌표(전진/우측) → 지리좌표(dx=동서, dy=남북)
        dx = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)
        dy = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)

        # local_to_latlon(x, y, ref_lat, ref_lon) — 인자 순서 주의
        return path_planner.local_to_latlon(dx, dy, self.d_lat, self.d_lon)

    def _apply_manual_avoidance_override(self):
        """
        조이스틱 입력으로 추정한 목표가 위협구역과 교차하면 pitch/roll PWM만
        회피 방향으로 일시 대체한다. (throttle/yaw=고도·기수는 절대 변경 안 함, 모드 전환도 안 함)
        1부의 _evaluate_avoidance를 그대로 사용하되, 조이스틱 컨텍스트에선
        BRAKE/LAND/RTL/REROUTE 모두 "스틱만 덮어쓰기"로 통일 처리한다.
        """
        if not self.no_fly_zones or self.drone is None:
            return
        if self.mission_active or self.auto_mission_active:
            return   # 자동비행 중에는 조이스틱 비활성 — 이중 개입 방지

        intended = self._estimate_manual_target()
        if intended is None:
            self._manual_avoidance_active = False
            return

        current_pos = (self.d_lat, self.d_lon)
        decision = self._evaluate_avoidance(
            "MANUAL_RC", current_pos, intended, max(self.d_alt, 1.0))

        if decision["action"] == "DIRECT":
            if self._manual_avoidance_active:
                self.set_status("위협구역 이탈 — 조종 입력이 다시 적용됩니다.")
            self._manual_avoidance_active = False
            return   # PWM은 조종자 입력 그대로 둠

        # 회피 발동: pitch/roll만 회피 경로 다음 지점 방향으로 덮어쓴다.
        waypoints = decision.get("waypoints")
        if decision["action"] == "BRAKE" or not waypoints or len(waypoints) < 2:
            # 경로 못 찾음 또는 BRAKE 정책 → 안전하게 정지(중립)
            self.js_roll, self.js_pitch = 1500, 1500
        else:
            next_wp = waypoints[1]
            dx, dy = path_planner.latlon_to_local(
                self.d_lat, self.d_lon, next_wp[0], next_wp[1])
            yaw_rad = math.radians(self.d_yaw)
            forward = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
            right = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
            norm = math.hypot(forward, right) or 1.0
            self.js_pitch = int(1500 + (forward / norm) * 300)   # 편향 제한 ±300
            self.js_roll = int(1500 + (right / norm) * 300)

        if not self._manual_avoidance_active:
            self.log_safety_event("MANUAL_AVOIDANCE_OVERRIDE", "WARNING",
                f"수동 조작 중 위협구역 근접({decision['action']}) — 위치 입력이 일시 회피로 대체됨")
            ToastMessage(self, "⚠ 위협구역 근접: 조종 입력이 일시적으로 회피 경로로 전환됩니다",
                         duration=4000)
        self._manual_avoidance_active = True

    def toggle_joystick_view(self):
        """조이스틱 표시/조종을 켜고 끈다. (ARM + LOITER/ALT_HOLD 에서만 켜짐)"""
        if self.joystick_visible.get():
            # 켜기: 연결/시동/모드 조건 확인
            if self.drone is None or not self.d_is_armed:
                ToastMessage(self, "드론 연결 및 시동(ARM) 상태에서만\n조이스틱 조종이 가능합니다.", duration=4000)
                self.joystick_visible.set(False)
                return

            current_m = self.d_mode.strip().upper()
            if current_m not in ["LOITER", "ALT_HOLD"]:
                ToastMessage(
                    self,
                    f"현재 비행 모드 [{current_m}]에서는 사용할 수 없습니다.\n"
                    "LOITER 또는 ALT_HOLD 모드로 변경 후 사용하세요.",
                    duration=4000)
                self.joystick_visible.set(False)
                return

            # 스틱 중립으로 초기화 후 패널 표시
            self.js_roll = self.js_pitch = self.js_throttle = self.js_yaw = 1500
            self.js_panel_frame.place(relx=0.5, rely=1.0, y=-10, anchor="s")
            print("[INFO] 가상 조이스틱 활성화 (지도 하단 중앙)")
        else:
            # 끄기: 패널 숨김 + RC 오버라이드 해제(0 전송) → 기체 자체 제어권 복귀
            try:
                self.js_panel_frame.place_forget()
            except Exception:
                pass
            if self.drone is not None:
                try:
                    with self.mav_lock:
                        self.drone.mav.rc_channels_override_send(
                            self.drone.target_system, self.drone.target_component,
                            0, 0, 0, 0, 0, 0, 0, 0)
                except Exception:
                    print("[MAVLINK] 조이스틱 비활성화: RC 제어권을 기체로 안전 이관")

    def js_loop(self):
        """조이스틱이 켜져 있으면 100ms 주기로 RC 채널 오버라이드를 전송한다."""
        if self.running:
            if self.joystick_visible.get() and self.drone is not None:
                # RC 송신 직전에 위협구역 회피 보정(스틱 정지 상태도 대응)
                self._apply_manual_avoidance_override()
                try:
                    with self.mav_lock:
                        self.drone.mav.rc_channels_override_send(
                            self.drone.target_system, self.drone.target_component,
                            self.js_roll,      # Ch1: Roll
                            self.js_pitch,     # Ch2: Pitch
                            self.js_throttle,  # Ch3: Throttle
                            self.js_yaw,       # Ch4: Yaw
                            0, 0, 0, 0)        # Ch5~8: 미사용
                except Exception as error:
                    print("[JOYSTICK SEND ERROR]", error)

            try:
                if self.winfo_exists():
                    self.after(100, self.js_loop)
            except Exception:
                pass

    def on_center_mode_change(self):
        """지도 중심 자동 이동 모드 변경(없음/홈/드론)."""
        mode = self.map_center_mode.get()
        if mode == 0:
            self.set_status("지도 자동 이동: 없음")
        elif mode == 1:
            # 홈 중심: 입력한 LAT/LON으로 1회 이동
            try:
                lat = float(self.lat_entry.get())
                lon = float(self.lon_entry.get())
                self.map_widget.set_position(lat, lon)
                self.set_status("지도 중심: 홈 위치로 이동")
            except Exception:
                pass
        elif mode == 2:
            self.set_status("지도 중심: 드론 추적")

    def register_handlers(self):
        """종료 및 예외 처리 핸들러 등록"""
        atexit.register(self.cleanup)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        sys.excepthook = self.handle_crash
        # GCS 단축키 바인딩 (F1~F5, ESC, N, M)
        self._bind_shortcuts()

    # ------------------------------------------------------------
    # SITL 시작
    # ------------------------------------------------------------

    # ------------------------------------------------------------
    # Start/Stop 통합 토글 버튼 제어
    # ------------------------------------------------------------

    def _set_toggle_idle(self):
        """버튼을 'SITL 시작'(그린, 활성) 상태로 만든다."""
        try:
            self.toggle_btn.config(
                text="▶ SITL 시작", bg="#2ecc71", activebackground="#27ae60",
                fg="#0e1828", state="normal")
        except Exception:
            pass

    def _set_toggle_running(self):
        """버튼을 'SITL 중지'(레드, 활성) 상태로 만든다."""
        try:
            self.toggle_btn.config(
                text="■ SITL 중지", bg="#e74c3c", activebackground="#c0392b",
                fg="white", state="normal")
        except Exception:
            pass

    def _set_toggle_busy(self):
        """전환 중에는 버튼을 비활성화하고 '연결 중...'으로 표시한다."""
        try:
            self.toggle_btn.config(
                text="연결 중...", bg="#7f8c8d", fg="white", state="disabled")
        except Exception:
            pass

    def _refresh_toggle(self):
        """실제 SITL 실행 상태에 맞춰 버튼 모양을 동기화한다."""
        if self.sitl.proc is not None and self.sitl.proc.poll() is None:
            self._set_toggle_running()
        else:
            self._set_toggle_idle()

    def toggle_sitl(self):
        """
        하나의 버튼으로 SITL 시작/정지를 토글한다.
        SITL이 실행 중이면 정지, 아니면 시작한다.
        """
        if self.closing:
            return

        if self.sitl.proc is not None and self.sitl.proc.poll() is None:
            self.stop_sitl_thread()
        else:
            self.start_sitl_thread()

    def start_sitl_thread(self):
        """버튼 클릭 시 Worker Thread에서 SITL 시작 로직 실행"""
        if self.closing:
            return

        threading.Thread(target=self.start_sitl, daemon=True).start()

    def start_sitl(self):
        """SITL 프로세스 실행 및 MAVLink 연결 과정"""
        if self.closing:
            return

        if self.sitl.proc is not None and self.sitl.proc.poll() is None:
            self.set_status("SITL이 이미 실행 중입니다.")
            return

        self.stop_requested = False

        self.run_on_ui(lambda: self.config(cursor="watch"))
        self.run_on_ui(self._set_toggle_busy)

        try:
            try:
                lat = float(self.lat_entry.get())
                lon = float(self.lon_entry.get())

            except ValueError:
                self.set_status("위도/경도는 숫자로 입력해야 합니다.")
                self.run_on_ui(self._set_toggle_idle)
                self.run_on_ui(lambda: self.config(cursor=""))
                return

            self.delete_existing_markers()

            self.set_status("SITL 프로세스 시작 중...")

            if not self.sitl.start(lat, lon):
                self.set_status("SITL 실행 실패")
                self.run_on_ui(self._set_toggle_idle)
                self.run_on_ui(lambda: self.config(cursor=""))
                return

            # SITL 프로세스가 떴으니 버튼을 'Stop SITL'로 전환
            self.run_on_ui(self._set_toggle_running)

            if self.closing or self.stop_requested:
                self.run_on_ui(self._refresh_toggle)
                return

            self.set_status("SITL 부팅 중... 약 10초 대기")
            print("[INFO] Waiting SITL boot...")
            # 10초 대기를 잘게 쪼개 STOP에 빠르게 반응한다.
            for _ in range(100):
                if self.closing or self.stop_requested:
                    self.run_on_ui(self._refresh_toggle)
                    return
                time.sleep(0.1)

            self.set_status("MAVLink 연결 및 HEARTBEAT 대기 중...")

            connection = self.connect_mavlink()

            if self.closing or self.stop_requested:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
                self.run_on_ui(self._refresh_toggle)
                return

            if connection is None:
                self.set_status("MAVLink 연결 실패")
                # SITL 프로세스는 떠 있으므로 버튼은 'Stop SITL'로 유지
                self.run_on_ui(self._refresh_toggle)
                self.run_on_ui(lambda: self.config(cursor=""))
                return

            self.drone = connection

            print("[MAVLink] Connected")
            print("[MAVLink] Target System:", self.drone.target_system)
            print("[MAVLink] Target Component:", self.drone.target_component)

            self.set_status("드론 연결 성공! HOME 설정 중...")

            self.run_on_ui(lambda: self.create_home_marker(lat, lon))

            self.request_mavlink_stream()

            self.gps_enable_time = time.time() + 10

            self.run_on_ui(
                lambda: self.show_countdown_popup(
                    10,
                    self.gps_enable_time,
                    "GPS 초기화..."
                )
            )

            self.set_status("모든 시스템 준비 완료. GPS 활성화 대기 중...")
            self.run_on_ui(lambda: self.config(cursor=""))
            self.run_on_ui(self._set_toggle_running)

        except Exception as error:
            print("[START SITL ERROR]", error)
            traceback.print_exc()
            self.set_status(f"오류 발생: {error}")

            # 실제 SITL 실행 상태에 맞춰 버튼 동기화
            self.run_on_ui(self._refresh_toggle)
            self.run_on_ui(lambda: self.config(cursor=""))

    # ------------------------------------------------------------
    # SITL 정지
    # ------------------------------------------------------------

    def stop_sitl_thread(self):
        """Stop SITL 버튼 클릭 시 Worker Thread에서 SITL 정리 로직 실행"""
        if self.closing:
            return

        threading.Thread(target=self.stop_sitl, daemon=True).start()

    def stop_sitl(self):
        """
        MAVLink 연결과 SITL 프로세스를 정리하되,
        Tkinter 창은 닫지 않고 유지한다. (재시작 가능)
        """
        if self.closing:
            return

        self.stop_requested = True
        self.set_status("SITL 종료 중...")

        self.run_on_ui(self._set_toggle_busy)

        try:
            self.run_on_ui(self.destroy_countdown_popup)

            if self.drone is not None:
                try:
                    with self.mav_lock:
                        self.drone.close()
                except Exception:
                    pass
                self.drone = None

            self.clear_mav_queue()

            self.sitl.stop()

            self.run_on_ui(self.delete_drone_marker)
            self.run_on_ui(self.delete_target_and_path)   # 목적지 마커/경로선 제거
            self.run_on_ui(self.stop_mission)             # 순차 미션 중단 + 계획 경로 제거
            self.run_on_ui(self._hide_joystick)           # 조이스틱 패널 숨김

            self.init_msg_sent = False
            self.gps_enable_time = None

            # HUD 상태 초기화
            self.reset_hud_state()

            # 안전 감시 상태 초기화 + 패널 갱신
            self.reset_safety_state()
            self.run_on_ui(self.update_safety_panel)

            self.set_status("SITL 종료 완료. 다시 시작할 수 있습니다.")

        except Exception as error:
            print("[STOP SITL ERROR]", error)
            self.set_status(f"SITL 종료 오류: {error}")

        finally:
            self.run_on_ui(self._set_toggle_idle)
            self.run_on_ui(lambda: self.config(cursor=""))

    def reset_hud_state(self):
        """HUD 표시용 상태 데이터를 기본값으로 되돌린다."""
        self.d_roll = 0.0
        self.d_pitch = 0.0
        self.d_alt = 0.0
        self.d_speed = 0.0
        self.d_mode = "DISCONN"
        self.d_sats = 0
        self.d_bat_v = 0.0
        self.d_bat_p = 0
        self.d_is_armed = False
        self.d_yaw = 0.0
        self.last_draw_yaw = -1.0
        # 조이스틱/지도중심/disarm 타이머 초기화
        self.js_roll = self.js_pitch = self.js_throttle = self.js_yaw = 1500
        self.disarm_time = None

    def _hide_joystick(self):
        """조이스틱 표시를 끄고 패널을 숨긴다."""
        try:
            self.joystick_visible.set(False)
            self.js_panel_frame.place_forget()
        except Exception:
            pass

    def destroy_countdown_popup(self):
        """GPS 초기화 팝업을 제거한다."""
        try:
            if self.countdown_popup is not None and self.countdown_popup.winfo_exists():
                self.countdown_popup.destroy()
        except Exception:
            pass

        self.countdown_popup = None

    # ------------------------------------------------------------
    # 마커 / 큐 정리
    # ------------------------------------------------------------

    def delete_existing_markers(self):
        """기존 HOME / Drone 마커 삭제"""
        if self.home_marker is not None:
            try:
                self.home_marker.delete()
            except Exception:
                pass
            self.home_marker = None

        if self.drone_marker is not None:
            try:
                self.drone_marker.delete()
            except Exception:
                pass
            self.drone_marker = None

    def delete_drone_marker(self):
        """지도 위 드론 마커만 삭제한다. HOME 마커는 유지한다."""
        if self.drone_marker is not None:
            try:
                self.drone_marker.delete()
            except Exception:
                pass

            self.drone_marker = None

    def clear_mav_queue(self):
        """MAVLink 위치 데이터 큐를 비운다."""
        try:
            while not self.mav_queue.empty():
                self.mav_queue.get_nowait()
        except Exception:
            pass

    # ------------------------------------------------------------
    # MAVLink 연결 및 스트림 요청
    # ------------------------------------------------------------

    def connect_mavlink(self):
        """
        MAVLink 연결을 시도한다. (5760 우선, 없으면 5762 — 포트 fallback)
        wait_heartbeat 대신 하트비트를 1초 간격으로 폴링하며 stop_requested를
        확인하므로, 긴 블로킹 없이 STOP에 빠르게 반응한다.
        (별도 probe 소켓은 SITL 단일 연결 슬롯을 깨뜨리므로 사용하지 않는다.)
        """
        candidates = [
            "tcp:127.0.0.1:5760",
            "tcp:127.0.0.1:5762",
        ]

        for connection_address in candidates:
            if self.closing or self.stop_requested:
                return None

            vehicle = None
            try:
                print("[MAVLink] Trying:", connection_address)
                vehicle = mavutil.mavlink_connection(connection_address)

                # 하트비트 폴링 — 최대 15초, 매 1초 stop_requested 확인(중단 가능)
                deadline = time.time() + 15.0
                while time.time() < deadline:
                    if self.closing or self.stop_requested:
                        vehicle.close()
                        return None
                    heartbeat = vehicle.recv_match(
                        type="HEARTBEAT", blocking=True, timeout=1)
                    if heartbeat is not None:
                        print("[MAVLink] HEARTBEAT received:", connection_address)
                        return vehicle

                print("[MAVLink] HEARTBEAT timeout:", connection_address)
                vehicle.close()

            except Exception as error:
                print("[MAVLink CONNECT ERROR]", connection_address, error)
                if vehicle is not None:
                    try:
                        vehicle.close()
                    except Exception:
                        pass

        return None

    def request_mavlink_stream(self):
        """
        드론에게 텔레메트리 데이터 스트림을 요청한다. (HUD용으로 세분화)
          - EXTRA1        : ATTITUDE (자세) 빠르게
          - EXTRA2        : VFR_HUD (속도/고도/상승률)  ← 속도 표시에 필요
          - POSITION      : GLOBAL_POSITION_INT (위치/고도)
          - EXTENDED_STATUS : SYS_STATUS, GPS_RAW_INT (배터리/위성)
        """
        if self.drone is None:
            return

        try:
            with self.mav_lock:
                self.drone.mav.request_data_stream_send(
                    self.drone.target_system,
                    self.drone.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50, 1)

                # VFR_HUD(속도)는 EXTRA2 스트림에 속한다. 누락 시 속도가 0으로 고정된다.
                self.drone.mav.request_data_stream_send(
                    self.drone.target_system,
                    self.drone.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10, 1)

                self.drone.mav.request_data_stream_send(
                    self.drone.target_system,
                    self.drone.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 20, 1)

                self.drone.mav.request_data_stream_send(
                    self.drone.target_system,
                    self.drone.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5, 1)

            print("[MAVLink] Data streams requested (HUD)")

        except Exception as error:
            print("[MAVLink STREAM ERROR]", error)

    def mavlink_loop(self):
        """
        MAVLink 메시지를 지속적으로 수신하여
          - 위치 정보는 큐에 저장 (지도 마커용)
          - 자세/배터리/모드/위성/속도 등은 HUD 상태 변수에 저장
        """
        while self.running:
            if self.closing:
                break

            if self.drone is None:
                time.sleep(0.1)
                continue

            try:
                with self.mav_lock:
                    msg = self.drone.recv_match(blocking=False)

                if msg is None:
                    time.sleep(0.005)
                    continue

                msg_type = msg.get_type()

                # Safety Monitor: 메시지 타입별 마지막 수신 시간 기록
                # (기존 처리 로직은 그대로 두고 시간 기록만 추가)
                _now = time.time()
                if msg_type == "HEARTBEAT":
                    self.last_heartbeat_time = _now
                    # 링크 지연 추세용 이력 누적 (수신 성공 = 지연 0)
                    self.link_delay_history.append((_now, 0.0))
                    if len(self.link_delay_history) > 200:
                        self.link_delay_history.pop(0)
                elif msg_type == "GLOBAL_POSITION_INT":
                    self.last_position_time = _now
                elif msg_type == "GPS_RAW_INT":
                    self.last_gps_time = _now
                elif msg_type == "SYS_STATUS":
                    self.last_sys_status_time = _now

                if msg_type == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    self.d_lat = lat
                    self.d_lon = lon
                    self.d_alt = msg.relative_alt / 1000.0  # 상대 고도(m)
                    self.mav_queue.put((lat, lon, self.d_alt))

                elif msg_type == "ATTITUDE":
                    self.d_roll = math.degrees(msg.roll)
                    self.d_pitch = math.degrees(msg.pitch)
                    self.d_yaw = math.degrees(msg.yaw)  # 드론 헤딩(방향)

                elif msg_type == "VFR_HUD":
                    self.d_speed = msg.groundspeed

                elif msg_type == "HEARTBEAT":
                    new_mode = mavutil.mode_string_v10(msg)
                    # 모드가 바뀌면 알림
                    if self.last_mode is not None and self.last_mode != new_mode:
                        print(f"[알림] 비행 모드가 [{self.last_mode}] -> [{new_mode}](으)로 변경되었습니다.")
                        self.set_status(f"모드 변경 완료: {new_mode}")
                    self.last_mode = new_mode
                    self.d_mode = new_mode
                    self.d_is_armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                elif msg_type == "GPS_RAW_INT":
                    self.d_sats = msg.satellites_visible
                    # GPS 추세 계산용 이력 누적
                    self.sats_history.append((time.time(), self.d_sats))
                    if len(self.sats_history) > 200:
                        self.sats_history.pop(0)

                elif msg_type == "SYS_STATUS":
                    self.d_bat_v = msg.voltage_battery / 1000.0  # mV -> V
                    self.d_bat_p = msg.battery_remaining         # %
                    # 활성화된 센서가 모두 정상이면 시동 준비 완료로 판단 (PreArm)
                    if (msg.onboard_control_sensors_health
                            & msg.onboard_control_sensors_enabled) == msg.onboard_control_sensors_enabled:
                        self.is_ready_to_arm = True
                    else:
                        self.is_ready_to_arm = False

                elif msg_type == "STATUSTEXT":
                    if "Arming checks passed" in msg.text:
                        self.is_ready_to_arm = True
                        self.set_status("시동 준비 완료 (PreArm Passed)")

                # 미션 업로드 핸드셰이크 응답
                elif msg_type in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                    if self._mission_upload_active:
                        self._send_mission_item(msg.seq)

                elif msg_type == "MISSION_ACK":
                    if self._mission_upload_active:
                        self._mission_upload_active = False
                        self.auto_mission_active = True   # AUTO 비행 추적 시작
                        self._auto_seen = False
                        print("[MISSION] ACK 수신 → AUTO 전환")
                        # UI/모드 전환은 메인 스레드에서
                        self.run_on_ui(lambda: self.set_drone_mode("AUTO"))
                        self.run_on_ui(
                            lambda: self.set_status("미션 업로드 완료 → AUTO 모드 전환"))

            except Exception as error:
                if not self.closing and not self.stop_requested:
                    print("[MAVLink LOOP ERROR]", error)
                time.sleep(0.5)

            time.sleep(0.001)  # CPU 과점유 방지

    # ------------------------------------------------------------
    # 지도 마커 생성 및 갱신
    # ------------------------------------------------------------

    def create_home_marker(self, lat, lon):
        """지도 위치 이동 및 HOME 마커 생성"""
        # 홈 위치 저장 (귀환 가능성 계산 기준)
        self.home_lat = lat
        self.home_lon = lon

        if self.closing:
            return

        self.map_widget.set_position(lat, lon)

        home_image_path = os.path.join(self.base_dir, "home.png")

        try:
            if os.path.exists(home_image_path):
                home_raw = Image.open(home_image_path)
                home_resized = home_raw.resize((60, 60))
                self.home_icon = ImageTk.PhotoImage(home_resized)

                self.home_marker = self.map_widget.set_marker(
                    lat, lon, text="HOME", icon=self.home_icon)
            else:
                self.home_marker = self.map_widget.set_marker(lat, lon, text="HOME")

        except Exception as error:
            print("[HOME IMAGE ERROR]", error)
            self.home_marker = self.map_widget.set_marker(lat, lon, text="HOME")

    def create_drone_marker(self, lat, lon):
        """지도 위 드론 마커 생성"""
        if self.closing:
            return

        drone_image_path = os.path.join(self.base_dir, "drone.png")

        try:
            if os.path.exists(drone_image_path):
                drone_raw = Image.open(drone_image_path)
                drone_resized = drone_raw.resize((70, 70))
                self.drone_icon = ImageTk.PhotoImage(drone_resized)

                self.drone_marker = self.map_widget.set_marker(
                    lat, lon, icon=self.drone_icon)
            else:
                self.drone_marker = self.map_widget.set_marker(lat, lon, text="Drone")

        except Exception as error:
            print("[DRONE IMAGE ERROR]", error)
            self.drone_marker = self.map_widget.set_marker(lat, lon, text="Drone")

    def update_drone_data(self):
        """
        매 주기마다:
          - 줌 레벨 라벨 갱신
          - HUD 갱신 (자세/배터리/모드/속도/고도/위성)
          - 큐의 최신 위치로 드론 마커 갱신
        """
        if not self.running or self.closing:
            return

        # 줌 레벨 라벨
        try:
            current_zoom = round(self.map_widget.zoom)
            self.zoom_label.config(text=f"Zoom: {current_zoom}")
            self.zoom_label.lift()   # 항상 지도 위에 보이도록
        except Exception:
            pass

        # HUD 갱신 (매 주기 - 위치와 무관하게 항상 갱신)
        try:
            self.hud.update_hud(
                self.d_roll, self.d_pitch, self.d_alt,
                self.d_speed, self.d_mode, self.d_sats,
                self.d_bat_v, self.d_bat_p, self.d_is_armed
            )
        except Exception as error:
            if not self.closing:
                print("[HUD UPDATE ERROR]", error)

        # 임무 안전도 평가 + 패널 갱신 (1초 주기로만)
        try:
            _safe_now = time.time()
            if _safe_now - self.last_safety_eval_time >= self.SAFETY_EVAL_INTERVAL_S:
                self.evaluate_flight_safety()
                self.update_safety_panel()
                self.last_safety_eval_time = _safe_now
        except Exception as error:
            if not self.closing:
                print("[SAFETY EVAL ERROR]", error)

        # 임무 단계 자동 감지 (안전도 1초 주기에 맞춰)
        try:
            if time.time() - self.last_safety_eval_time < 1.5:
                self._auto_detect_phase()
                self._update_phase_indicator()
        except Exception:
            pass

        # 보고서용 최댓값/최저 안전도 기록
        try:
            if self.d_is_armed:
                self.max_alt_recorded = max(self.max_alt_recorded, self.d_alt)
                if self.home_lat is not None and (self.d_lat != 0.0 or self.d_lon != 0.0):
                    _dlat = (self.d_lat - self.home_lat) * 111320
                    _dlon = (self.d_lon - self.home_lon) * \
                        111320 * math.cos(math.radians(self.home_lat))
                    self.max_dist_recorded = max(self.max_dist_recorded,
                                                 math.hypot(_dlat, _dlon))
            if self.flight_risk_score < self.min_risk_score_recorded:
                self.min_risk_score_recorded = self.flight_risk_score
                self.min_risk_level_recorded = self.risk_level
        except Exception:
            pass

        # DISARM 감지 → 임무 보고서 자동 생성
        try:
            if not self.d_is_armed and self.mission_start_time is not None:
                self.generate_mission_report()
                self.set_mission_phase("COMPLETE")
        except Exception:
            pass

        # 시동 가능 상태 라벨 갱신
        try:
            armable_modes = ["STABILIZE", "LOITER", "GUIDED", "ALT_HOLD", "POSHOLD"]
            current_mode = self.d_mode.strip().upper()
            if self.drone is None:
                arm_text = "상태: 연결 대기중"; arm_color = "#95a5a6"
            elif self.d_is_armed:
                arm_text = "상태: ARMED (비행 중)"; arm_color = "#2ecc71"
            elif current_mode not in armable_modes:
                arm_text = f"상태: 시동 불가 ({current_mode})"; arm_color = "#95a5a6"
            elif not self.is_ready_to_arm:
                arm_text = "상태: PreArm 점검 중..."; arm_color = "#e67e22"
            else:
                arm_text = "상태: READY (시동 가능)"; arm_color = "#3498db"
            self.arm_status_label.config(text=arm_text, fg=arm_color)
        except Exception:
            pass

        # 조이스틱 안전 자동 차단: 시동이 풀리거나 LOITER/ALT_HOLD가 아니면 끈다.
        try:
            if self.joystick_visible.get():
                current_m = self.d_mode.strip().upper()
                if (self.drone is None or not self.d_is_armed
                        or current_m not in ["LOITER", "ALT_HOLD"]):
                    self.joystick_visible.set(False)
                    self.toggle_joystick_view()
                    self.set_status("비행 상태가 충족되지 않아 조이스틱 조종이 자동 차단되었습니다.")
        except Exception:
            pass

        latest = None

        try:
            while not self.mav_queue.empty():
                latest = self.mav_queue.get()

            if latest is not None:
                lat, lon, relative_alt = latest

                if self.gps_enable_time is not None and time.time() < self.gps_enable_time:
                    return

                if self.stop_requested:
                    return

                if not self.init_msg_sent:
                    self.init_msg_sent = True
                    self.set_status("실시간 데이터 수신 중")

                # 드론 마커 (헤딩 yaw 방향으로 회전)
                yaw_diff = abs(self.d_yaw - self.last_draw_yaw)
                if self.drone_marker is None:
                    self._create_rotated_drone_marker(lat, lon)
                    self.last_draw_yaw = self.d_yaw
                else:
                    try:
                        self.drone_marker.set_position(lat, lon)
                        # 1도 이상 방향이 바뀐 경우에만 회전 이미지를 새로 생성(자원 절약)
                        if yaw_diff > 1.0 and self.drone_raw_img is not None:
                            self.drone_marker.delete()
                            self._create_rotated_drone_marker(lat, lon)
                            self.last_draw_yaw = self.d_yaw
                    except Exception as error:
                        print("[DRONE MARKER UPDATE ERROR]", error)

                # 목적지 직선 경로선 갱신 (미션 비행 중에는 회피 경로를 보여주므로 생략)
                if not self.mission_active:
                    self._update_path_line(lat, lon)

                # 순차 GUIDED 미션 도달 판정 / 다음 웨이포인트 진행
                self._mission_step()

                # 비행 이력 트레일 (3m 이상 이동 시 점 추가)
                try:
                    if self.d_is_armed and self.d_lat != 0.0:
                        _add = False
                        if self._last_trail_lat is None:
                            _add = True
                        else:
                            _dd = math.hypot(
                                (self.d_lat - self._last_trail_lat) * 111320,
                                (self.d_lon - self._last_trail_lon) * 111320 *
                                math.cos(math.radians(self.d_lat)))
                            if _dd >= 3.0:
                                _add = True
                        if _add:
                            self.flight_trail.append((self.d_lat, self.d_lon))
                            self._last_trail_lat = self.d_lat
                            self._last_trail_lon = self.d_lon
                            if len(self.flight_trail) > 500:
                                self.flight_trail.pop(0)
                            self._update_trail_line()
                except Exception:
                    pass

                # 지도 중심 자동 이동: 드론 추적 모드(2)면 1초마다 드론 중심으로
                if self.map_center_mode.get() == 2:
                    now = time.time()
                    if now - self.last_map_center_update_time > 1.0:
                        try:
                            self.map_widget.set_position(lat, lon)
                        except Exception:
                            pass
                        self.last_map_center_update_time = now

                # 착륙(시동 해제) 후 목적지 마커/이동선 자동 정리
                if (not self.d_is_armed) and (self.target_marker is not None or self.path_line is not None):
                    if self.disarm_time is None:
                        self.disarm_time = time.time()
                    elif time.time() - self.disarm_time > 2.0:
                        self.delete_target_and_path()
                        self.disarm_time = None
                        self.set_status("착륙 후 시동 해제가 감지되어 목적지 마커와 이동선을 정리했습니다.")
                else:
                    self.disarm_time = None

                self.set_status(
                    f"실시간 데이터 수신 중 | LAT={lat:.7f}, LON={lon:.7f}, ALT={relative_alt:.2f}m"
                )

        except Exception as error:
            if not self.closing:
                print("[UPDATE DRONE DATA ERROR]", error)

        finally:
            if self.running and not self.closing:
                try:
                    if self.winfo_exists():
                        self.update_after_id = self.after(50, self.update_drone_data)
                except Exception:
                    pass

    # ------------------------------------------------------------
    # 카운트다운 팝업
    # ------------------------------------------------------------

    def show_countdown_popup(self, seconds, target_time, title_text):
        """GPS 초기화 대기 팝업을 표시한다."""
        if self.closing:
            return

        try:
            self.countdown_popup = CountdownProgressBarPopup(
                self, seconds, target_time, title_text)
        except Exception as error:
            print("[COUNTDOWN POPUP ERROR]", error)

    # ------------------------------------------------------------
    # 종료 처리
    # ------------------------------------------------------------

    def cancel_update_loop(self):
        """예약된 update_drone_data after 콜백을 명시적으로 취소한다."""
        try:
            if self.update_after_id is not None:
                self.after_cancel(self.update_after_id)
                self.update_after_id = None
        except Exception as error:
            print("[AFTER CANCEL ERROR]", error)

    def cleanup(self):
        """프로그램 종료 시 MAVLink 연결과 SITL 프로세스를 정리한다."""
        if self.cleaned_up:
            return

        self.cleaned_up = True
        print("[EXIT] cleanup SITL")

        self.running = False

        try:
            if self.drone is not None:
                try:
                    with self.mav_lock:
                        self.drone.close()
                except Exception:
                    pass

                self.drone = None

            self.clear_mav_queue()
            self.sitl.stop()

        except Exception:
            pass

    def on_close(self):
        """창 닫기 이벤트 처리"""
        if self.closing:
            return

        self.closing = True
        self.running = False
        self.stop_requested = True

        self.cancel_update_loop()

        try:
            self.status_var.set("프로그램 종료 중...")
        except Exception:
            pass

        try:
            self.destroy_countdown_popup()
        except Exception:
            pass

        try:
            self.withdraw()
        except Exception:
            pass

        shutdown_done = threading.Event()

        def shutdown_worker():
            try:
                self.cleanup()
            except Exception as error:
                print("[SHUTDOWN ERROR]", error)
            finally:
                shutdown_done.set()

        def finish_when_done():
            if shutdown_done.is_set():
                try:
                    self.quit()
                except Exception:
                    pass
                return

            try:
                self.after(100, finish_when_done)
            except Exception:
                pass

        threading.Thread(target=shutdown_worker, daemon=True).start()

        try:
            self.after(100, finish_when_done)
        except Exception:
            pass

    def handle_crash(self, exc_type, exc_value, exc_traceback):
        """예외 발생 시 SITL 종료 후 프로그램 종료"""
        print("[FATAL ERROR]")
        traceback.print_exception(exc_type, exc_value, exc_traceback)

        self.closing = True
        self.running = False
        self.stop_requested = True

        self.cancel_update_loop()

        try:
            self.cleanup()
        except Exception:
            pass

        try:
            self.quit()
        except Exception:
            pass

        sys.exit(1)


# ============================================================
# 6. 카운트다운 팝업 클래스
# ============================================================

class CountdownProgressBarPopup(tk.Toplevel):
    """GPS 초기화 대기 시간을 표시하는 팝업 클래스"""

    def __init__(self, parent, seconds, target_time, title_text="대기 중..."):
        super().__init__(parent)

        self.parent = parent
        self.seconds = seconds
        self.target_time = target_time

        self.title(title_text)

        width = 300
        height = 60

        parent_x = parent.winfo_rootx()
        parent_y = parent.winfo_rooty()

        pos_x = parent_x + (parent.winfo_width() // 2) - (width // 2)
        pos_y = parent_y + (parent.winfo_height() // 2) - (height * 4)

        self.geometry(f"{width}x{height}+{pos_x}+{pos_y}")
        self.resizable(False, False)
        self.attributes("-topmost", True)

        self.progress = ttk.Progressbar(
            self,
            orient="horizontal",
            length=250,
            mode="determinate"
        )
        self.progress.pack(pady=(10, 2))
        self.progress["maximum"] = seconds

        self.time_label = tk.Label(
            self,
            text=f"{seconds}초",
            font=("맑은 고딕", 10, "bold")
        )
        self.time_label.pack(pady=(0, 5))

        self.update_countdown()

    def update_countdown(self):
        """0.1초마다 남은 시간을 체크하여 UI와 상태바를 갱신한다."""
        if self.parent.closing or self.parent.stop_requested:
            try:
                self.destroy()
            except Exception:
                pass
            return

        now = time.time()
        remaining_float = self.target_time - now
        remaining_int = int(max(0, remaining_float))

        if remaining_float > 0:
            try:
                self.progress["value"] = self.seconds - remaining_float

                self.time_label.config(
                    text=f"{remaining_int}초 후 완료",
                    fg="blue"
                )

                if remaining_float < 20:
                    self.parent.status_var.set(
                        f"시스템 안정화 대기 중... {remaining_int}초 남음"
                    )

                self.after(100, self.update_countdown)

            except Exception:
                pass

        else:
            try:
                self.parent.status_var.set("준비 완료!")
                self.destroy()
            except Exception:
                pass


# ============================================================
# 7. 실행부
# ============================================================

if __name__ == "__main__":
    app = GCSMap()
    app.mainloop()
