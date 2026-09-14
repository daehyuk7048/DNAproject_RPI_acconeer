#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import collections
import time
import csv
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple
from enum import Enum

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QGroupBox, QDoubleSpinBox,
    QTextEdit, QFileDialog, QGridLayout, QMessageBox, QSpinBox,
    QComboBox, QTableWidget, QTableWidgetItem, QHeaderView
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot, QThread
from PySide6.QtGui import QFont, QColor

import pyqtgraph as pg

from acconeer.exptool import a121
from acconeer.exptool.a121.algo.distance import (
    Detector, DetectorConfig, ThresholdMethod, ReflectorShape
)

import os
import rgpio
import socket
import threading

RGPIO_HOST = os.environ.get("RGPIO_HOST", "192.168.28.227")
RGPIO_PORT = int(os.environ.get("RGPIO_PORT", "8889"))
_sbc = None
_chip = None
_pwm_freq = {}

RPI_BRIDGE_HOST = "192.168.28.227"
RPI_BRIDGE_PORT = 9999
_sock = None
_sock_lock = threading.Lock()

SOLENOID_PIN_BCM = 14
REGULATOR_ARDUINO_PIN = 11

def _init_rgpio_backend():
    global _sbc, _chip
    if _sbc is not None:
        return
    try:
        _sbc = rgpio.sbc(RGPIO_HOST, RGPIO_PORT)
        _chip = _sbc.gpiochip_open(0)
        print("rgpio backend initialized.")
    except Exception as e:
        print(f"Failed to initialize rgpio backend: {e}")
        _sbc, _chip = None, None

def _init_socket_backend():
    global _sock
    if _sock is not None:
        return
    try:
        _sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        _sock.connect((RPI_BRIDGE_HOST, RPI_BRIDGE_PORT))
        print("Socket backend initialized.")
    except socket.error as e:
        _sock = None
        print(f"Failed to connect to RPi Bridge: {e}")

def _send_arduino_command(command: str):
    global _sock
    try:
        if _sock is None:
            _init_socket_backend()
        if _sock:
            with _sock_lock:
                _sock.sendall((command + "\n").encode("utf-8"))
    except (socket.error, BrokenPipeError):
        if _sock:
            try: _sock.close()
            except: pass
        _sock = None
        _init_socket_backend()
        if _sock:
            with _sock_lock:
                _sock.sendall((command + "\n").encode("utf-8"))

def gpio_setup_out(pin: int, initial_low: bool = True):
    if pin == SOLENOID_PIN_BCM:
        _init_rgpio_backend()
        if _sbc:
            _sbc.gpio_claim_output(_chip, pin)
            _sbc.gpio_write(_chip, pin, 0 if initial_low else 1)

def gpio_write(pin: int, value_high: bool):
    if pin == SOLENOID_PIN_BCM:
        _init_rgpio_backend()
        if _sbc:
            _sbc.gpio_write(_chip, pin, 1 if value_high else 0)

def gpio_pwm_start(pin: int, freq_hz: int = 1000):
    if pin == REGULATOR_ARDUINO_PIN:
        _send_arduino_command(f"P{pin},0.0")
        return pin

def gpio_pwm_change_duty(pwm_handle, duty_percent: float):
    pin = int(pwm_handle)
    if pin == REGULATOR_ARDUINO_PIN:
        p = float(np.clip(duty_percent, 0.0, 100.0))
        _send_arduino_command(f"P{pin},{p:.2f}")

def gpio_pwm_stop(pwm_handle):
    pin = int(pwm_handle)
    if pin == REGULATOR_ARDUINO_PIN:
        _send_arduino_command(f"P{pin},0.0")

def gpio_cleanup():
    global _sbc, _chip, _sock
    try:
        _send_arduino_command(f"P{REGULATOR_ARDUINO_PIN},0.0")
        if _sock: _sock.close()
    except Exception:
        pass
    finally:
        _sock = None
    try:
        if _chip is not None and _sbc is not None:
            _sbc.gpiochip_close(_chip)
        if _sbc is not None:
            _sbc.stop()
    except Exception:
        pass
    finally:
        _sbc, _chip = None, None
    print("All backends cleaned up.")

def update_backend_pins(solenoid_bcm, regulator_arduino):
    global SOLENOID_PIN_BCM, REGULATOR_ARDUINO_PIN
    SOLENOID_PIN_BCM = solenoid_bcm
    REGULATOR_ARDUINO_PIN = regulator_arduino

class PressureMode(Enum):
    FIXED = 0
    MANUAL = 1
    ADAPTIVE = 2

@dataclass
class SystemConfig:
    tank_height_mm: float = 100.0
    tank_diameter_mm: float = 35.0
    sensor_offset_mm: float = 50.0
    float_submersion_mm: float = 5.0

    solenoid_pin: int = 14
    regulator_pin: int = 11

    min_pressure_pa: float = 0
    max_pressure_pa: float = 200000

    flow_coefficient: float = 0.000042

    fixed_pressure_kpa: float = 50.0
    manual_pressure_kpa: float = 60.0
    precharge_ms: int = 100
    valve_lead_ms: int = 60
    valve_trail_ms: int = 30
    min_pulse_ms: int = 80
    height_time_gain: float = 0.25

    # ★ NEW: 시간 보정계수 (현장 오차율 반영) – 실측 0.63s → 5.6s 기준 ≈ 8.9
    time_correction_gain: float = 8.9
    target_mapping_ratio: float = 0.60

    @property
    def sensor_height_mm(self) -> float:
        return self.tank_height_mm + self.sensor_offset_mm

    @property
    def tank_area_mm2(self) -> float:
        return np.pi * (self.tank_diameter_mm/2)**2

    def height_to_volume_ml(self, height_mm: float) -> float:
        return self.tank_area_mm2 * height_mm / 1000.0

    def height_to_pressure_pa(self, height_mm: float) -> float:
        if height_mm <= 0:
            return self.max_pressure_pa
        base_pressure = 100000
        height_ratio = 1 - (height_mm / self.tank_height_mm)
        compensation_pressure = height_ratio * 50000
        target_pressure = base_pressure + compensation_pressure
        return np.clip(target_pressure, self.min_pressure_pa, self.max_pressure_pa)

    def time_scale_by_height(self, height_mm: float) -> float:
        if self.tank_height_mm <= 1e-9:
            return 1.0
        r = float(np.clip(height_mm / self.tank_height_mm, 0.0, 1.0))
        return 1.0 + self.height_time_gain * (1.0 - r)

class PressureRegulator:
    def __init__(self, pin: int, min_pa: float, max_pa: float, freq_hz: int = 1000,
                 invert_pwm: bool = True):
        self.pin = int(pin)
        self.min_pa = float(min_pa)
        self.max_pa = float(max_pa)
        self.freq_hz = int(freq_hz)
        self.invert_pwm = bool(invert_pwm)
        self.pwm = gpio_pwm_start(self.pin, self.freq_hz)
        self.current_pressure = 0.0
        self.current_duty = 0.0
        self._apply_duty(0.0)

    def _apply_duty(self, duty_percent: float):
        d = float(np.clip(duty_percent, 0.0, 100.0))
        d_tx = 100.0 - d if self.invert_pwm else d
        try:
            gpio_pwm_change_duty(self.pwm, d_tx)
        except Exception:
            try: gpio_cleanup()
            except Exception: pass
            self.pwm = gpio_pwm_start(self.pin, self.freq_hz)
            gpio_pwm_change_duty(self.pwm, d_tx)
        self.current_duty = d

    def set_pressure(self, pressure_pa: float):
        p = float(np.clip(pressure_pa, self.min_pa, self.max_pa))
        duty = (p - self.min_pa) / (self.max_pa - self.min_pa) * 100.0 if self.max_pa > self.min_pa else 0.0
        self._apply_duty(duty)
        self.current_pressure = p
        d_tx = 100.0 - self.current_duty if self.invert_pwm else self.current_duty
        print(f"[PWM] PIN{self.pin} @ {self.freq_hz} Hz | target={p/1000:.1f} kPa  (duty={self.current_duty:.1f}%, tx={d_tx:.1f}%)")
        return p

    def stop(self): self._apply_duty(0.0)
    def cleanup(self):
        try: self.stop()
        finally: gpio_pwm_stop(self.pwm)

class AdaptivePressureController:
    def __init__(self, config: SystemConfig, invert_pwm: bool = True):
        self.config = config
        self.regulator = PressureRegulator(
            config.regulator_pin,
            config.min_pressure_pa,
            config.max_pressure_pa,
            freq_hz=1000,
            invert_pwm=invert_pwm,
        )
        self.is_active = False
        self.current_height_mm = 0.0
        self.flow_rate_cache: Dict[int, float] = {}
        self.mode = PressureMode.FIXED
        self.fixed_pressure_pa = float(self.config.fixed_pressure_kpa) * 1000.0
        self.manual_pressure_pa = float(self.config.manual_pressure_kpa) * 1000.0
        self.regulator.set_pressure(self.fixed_pressure_pa)
        gpio_setup_out(config.solenoid_pin, initial_low=True)

    def set_mode(self, mode: PressureMode):
        self.mode = mode
        if mode == PressureMode.FIXED:
            self.regulator.set_pressure(self.fixed_pressure_pa)
        elif mode == PressureMode.MANUAL:
            self.regulator.set_pressure(self.manual_pressure_pa)

    def set_manual_pressure_kpa(self, kpa: float):
        self.config.manual_pressure_kpa = float(kpa)
        self.manual_pressure_pa = float(kpa) * 1000.0
        if self.mode == PressureMode.MANUAL:
            self.regulator.set_pressure(self.manual_pressure_pa)

    def set_fixed_pressure_kpa(self, kpa: float):
        self.config.fixed_pressure_kpa = float(kpa)
        self.fixed_pressure_pa = float(kpa) * 1000.0
        if self.mode == PressureMode.FIXED:
            self.regulator.set_pressure(self.fixed_pressure_pa)

    def update_height(self, height_mm: float):
        self.current_height_mm = float(height_mm)

    def _compute_target_pressure_pa(self) -> float:
        if self.mode == PressureMode.FIXED:
            return self.fixed_pressure_pa
        elif self.mode == PressureMode.MANUAL:
            return self.manual_pressure_pa
        else:
            return float(self.config.height_to_pressure_pa(self.current_height_mm))

    def calculate_flow_rate(self, pressure_pa: float) -> float:
        key = int(pressure_pa / 1000.0) * 1000
        if key in self.flow_rate_cache:
            return self.flow_rate_cache[key]
        flow_rate = self.config.flow_coefficient * pressure_pa  # ml/s
        self.flow_rate_cache[key] = flow_rate
        return flow_rate

    def calculate_dispense_parameters(self, target_ml: float) -> Dict:
        p = self._compute_target_pressure_pa()
        flow = self.calculate_flow_rate(p)  # ml/s
        base_time = (target_ml / flow) if flow > 0 else 0.0

        h = self.current_height_mm
        h_factor = self.config.time_scale_by_height(h)

        deadtime = (self.config.valve_lead_ms + self.config.valve_trail_ms) / 1000.0
        min_pulse = self.config.min_pulse_ms / 1000.0

        # ★ 핵심: 현장 오차율(진공X 누설 등) 반영
        open_time = (max(base_time * h_factor, min_pulse) + deadtime) * self.config.time_correction_gain

        return {
            "pressure_pa": float(p),
            "flow_rate_ml_s": float(flow),
            "time_s": float(open_time),
            "base_time_s": float(base_time),
            "height_mm": float(h),
            "height_factor": float(h_factor),
        }

    def open_valve(self):
        gpio_write(self.config.solenoid_pin, True)
        self.is_active = True

    def close_valve(self):
        gpio_write(self.config.solenoid_pin, False)
        self.is_active = False

    def dispense(self, target_ml: float) -> Dict:
        params = self.calculate_dispense_parameters(target_ml)
        if params["time_s"] <= 0:
            return {"success": False, "error": "Invalid parameters"}
        self.regulator.set_pressure(params["pressure_pa"])
        time.sleep(max(self.config.precharge_ms, 0) / 1000.0)
        start_time = time.time()
        self.open_valve()
        time.sleep(params["time_s"])
        self.close_valve()
        actual_time = time.time() - start_time
        used_flow = self.calculate_flow_rate(params["pressure_pa"])
        estimated_ml = used_flow * actual_time / max(self.config.time_correction_gain, 1e-9)  # 추정량은 보정 전 물리유량 기준
        return {
            "success": True,
            "target_ml": float(target_ml),
            "estimated_ml": float(estimated_ml),
            "time_s": float(actual_time),
            "pressure_pa": float(params["pressure_pa"]),
            "flow_rate": float(used_flow),
            "height_mm": float(self.current_height_mm),
            "height_factor": float(params["height_factor"]),
        }

    def cleanup(self):
        try: self.close_valve()
        finally: self.regulator.cleanup()

class DispensingThread(QThread):
    dispense_complete = Signal(dict)
    status_update = Signal(str)
    def __init__(self, controller: AdaptivePressureController):
        super().__init__()
        self.controller = controller
        self.target_ml = 0
        self.should_dispense = False
    def set_parameters(self, target_ml: float):
        self.target_ml = target_ml
        self.should_dispense = True
    def run(self):
        if self.should_dispense:
            self.status_update.emit(f"토출 시작: {self.target_ml}ml")
            result = self.controller.dispense(self.target_ml)
            self.dispense_complete.emit(result)
            self.should_dispense = False

class SensorThread(QThread):
    data_received = Signal(dict)
    error_occurred = Signal(str)
    log_message = Signal(str)
    def __init__(self, config: SystemConfig):
        super().__init__()
        self.config = config
        self.client = None
        self.detector = None
        self.running = False
        self.current_volume_ml = 0
        self.current_height_mm = 0
        self.median_window_size = 7
        self.ema_alpha = 0.2

    def setup_connection(self, ip: str, port: int) -> bool:
        max_retries = 3
        for attempt in range(max_retries):
            try:
                if self.client:
                    try: self.client.close()
                    except: pass
                    self.client = None
                    time.sleep(1)
                self.client = a121.Client.open(ip_address=ip, tcp_port=port)
                self.log_message.emit(f"연결 성공: {ip}:{port}")
                return True
            except Exception as e:
                self.error_occurred.emit(f"연결 시도 {attempt+1}/{max_retries}: {str(e)}")
                if attempt < max_retries - 1: time.sleep(2)
                else: return False

    def setup_detector(self) -> bool:
        try:
            start_distance_m = self.config.sensor_offset_mm / 1000.0
            end_distance_m = (self.config.sensor_offset_mm + self.config.tank_height_mm) / 1000.0
            detector_config = DetectorConfig(
                start_m=start_distance_m,
                end_m=end_distance_m,
                max_step_length=2,
                max_profile=a121.Profile.PROFILE_1,
                threshold_method=ThresholdMethod.CFAR,
                threshold_sensitivity=0.5,
                reflector_shape=ReflectorShape.PLANAR,
                close_range_leakage_cancellation=False,
                update_rate=20
            )
            self.detector = Detector(client=self.client, sensor_ids=[1], detector_config=detector_config)
            time.sleep(0.5)
            self.detector.calibrate_detector()
            self.log_message.emit("캘리브레이션 완료")
            return True
        except Exception as e:
            self.error_occurred.emit(f"설정 실패: {str(e)}")
            return False

    def run(self):
        if not self.detector: return
        distance_buffer = collections.deque(maxlen=self.median_window_size)
        ema_value = None
        try:
            self.detector.start()
            self.running = True
            while self.running:
                result = self.detector.get_next()
                res = result[1] if isinstance(result, dict) and 1 in result else result
                distance_m = 0; strength = 0
                if hasattr(res,'distances') and res.distances is not None and len(res.distances)>0:
                    distance_m = res.distances[0]
                if hasattr(res,'strengths') and res.strengths is not None and len(res.strengths)>0:
                    strength = res.strengths[0]
                distance_mm = distance_m * 1000.0
                if distance_mm < self.config.sensor_offset_mm - 10 or distance_mm > self.config.sensor_offset_mm + self.config.tank_height_mm + 10:
                    continue
                distance_buffer.append(distance_mm)
                if len(distance_buffer) >= 5:
                    median_distance = np.median(list(distance_buffer))
                    q1 = np.percentile(distance_buffer, 25); q3 = np.percentile(distance_buffer, 75); iqr = q3 - q1
                    if iqr > 0:
                        lb = q1 - 1.5*iqr; ub = q3 + 1.5*iqr
                        if median_distance < lb or median_distance > ub:
                            if ema_value is not None: median_distance = ema_value
                    ema_value = median_distance if ema_value is None else self.ema_alpha*median_distance + (1-self.ema_alpha)*ema_value
                    filtered_distance = ema_value
                else:
                    filtered_distance = distance_mm
                    if ema_value is None: ema_value = distance_mm
                height_mm = self.config.sensor_height_mm - float(filtered_distance) - self.config.float_submersion_mm
                height_mm = float(np.clip(height_mm, 0.0, self.config.tank_height_mm))
                self.current_height_mm = height_mm
                volume_ml = self.config.height_to_volume_ml(height_mm)
                self.current_volume_ml = volume_ml
                percentage = (height_mm / self.config.tank_height_mm) * 100
                data = {
                    'timestamp': time.time(),
                    'distance_mm': filtered_distance,
                    'height_mm': height_mm,
                    'volume_ml': round(volume_ml, 2),
                    'strength': abs(strength),
                    'percentage': percentage,
                    'raw_distance_mm': distance_mm
                }
                self.data_received.emit(data)
                time.sleep(0.05)
        except Exception as e:
            self.error_occurred.emit(f"센서 오류: {str(e)}")
        finally:
            self.running = False
            if self.detector:
                try: self.detector.stop()
                except: pass

    def stop(self):
        self.running = False
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.config = SystemConfig()
        update_backend_pins(self.config.solenoid_pin, self.config.regulator_pin)
        self.pressure_controller = AdaptivePressureController(self.config, invert_pwm=False)
        self.sensor_thread = SensorThread(self.config)
        self.dispense_thread = DispensingThread(self.pressure_controller)
        self.data_log = []; self.dispense_log = []
        self.init_ui(); self.setup_connections()

    def init_ui(self):
        self.setWindowTitle("적응형 정밀 토출 제어 시스템")
        self.setGeometry(100, 100, 1200, 800)
        central = QWidget(); self.setCentralWidget(central)
        layout = QHBoxLayout(central)

        left_panel = QWidget(); left_panel.setMaximumWidth(440); left_layout = QVBoxLayout(left_panel)

        conn_group = QGroupBox("센서 연결"); conn_layout = QGridLayout()
        conn_layout.addWidget(QLabel("IP:"), 0, 0); self.ip_input = QLineEdit("192.168.28.227"); conn_layout.addWidget(self.ip_input, 0, 1)
        conn_layout.addWidget(QLabel("Port:"), 1, 0); self.port_input = QLineEdit("6110"); conn_layout.addWidget(self.port_input, 1, 1)
        self.connect_btn = QPushButton("연결"); conn_layout.addWidget(self.connect_btn, 2, 0, 1, 2)
        conn_group.setLayout(conn_layout); left_layout.addWidget(conn_group)

        dispense_group = QGroupBox("토출 제어"); dispense_layout = QGridLayout()

        dispense_layout.addWidget(QLabel("목표량 (ml):"), 0, 0)
        self.target_volume = QDoubleSpinBox(); self.target_volume.setRange(0.05, 5.0)
        self.target_volume.setValue(0.60); self.target_volume.setSingleStep(0.05)
        dispense_layout.addWidget(self.target_volume, 0, 1)

        dispense_layout.addWidget(QLabel("허용 오차 (±ml):"), 1, 0)
        self.tol_volume = QDoubleSpinBox(); self.tol_volume.setRange(0.01, 2.0)
        self.tol_volume.setValue(0.10); self.tol_volume.setSingleStep(0.01)
        dispense_layout.addWidget(self.tol_volume, 1, 1)

        dispense_layout.addWidget(QLabel("압력 모드:"), 2, 0)
        self.pressure_mode = QComboBox(); self.pressure_mode.addItems(["고정(50kPa)", "수동", "적응"])
        dispense_layout.addWidget(self.pressure_mode, 2, 1)

        dispense_layout.addWidget(QLabel("압력 (kPa):"), 3, 0)
        self.pressure_input = QDoubleSpinBox(); self.pressure_input.setRange(0, 200)
        self.pressure_input.setValue(self.config.fixed_pressure_kpa); self.pressure_input.setEnabled(False)
        dispense_layout.addWidget(self.pressure_input, 3, 1)

        dispense_layout.addWidget(QLabel("예상 유량:"), 4, 0)
        self.flow_rate_label = QLabel("-- ml/s"); dispense_layout.addWidget(self.flow_rate_label, 4, 1)

        dispense_layout.addWidget(QLabel("예상 시간:"), 5, 0)
        self.time_estimate = QLabel("-- s"); dispense_layout.addWidget(self.time_estimate, 5, 1)

        # ★ NEW: 시간 보정(×)
        dispense_layout.addWidget(QLabel("시간 보정 (×):"), 6, 0)
        self.time_gain = QDoubleSpinBox()
        self.time_gain.setRange(0.1, 50.0); self.time_gain.setDecimals(2)
        self.time_gain.setSingleStep(0.1); self.time_gain.setValue(self.config.time_correction_gain)
        dispense_layout.addWidget(self.time_gain, 6, 1)

        self.spec_label = QLabel("목표: 0.60 ± 0.10 ml"); self.spec_label.setStyleSheet("color:#333;")
        dispense_layout.addWidget(self.spec_label, 7, 0, 1, 2)

        self.dispense_btn = QPushButton("토출 시작")
        self.dispense_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        dispense_layout.addWidget(self.dispense_btn, 8, 0, 1, 2)

        dispense_group.setLayout(dispense_layout); left_layout.addWidget(dispense_group)

        stats_group = QGroupBox("실시간 측정"); stats_layout = QGridLayout()
        self.volume_label = QLabel("부피: -- ml"); self.volume_label.setStyleSheet("font-size: 16pt; font-weight: bold;")
        stats_layout.addWidget(self.volume_label, 0, 0, 1, 2)
        self.height_label = QLabel("수위: -- mm"); stats_layout.addWidget(self.height_label, 1, 0)
        self.percentage_label = QLabel("충전율: --%"); stats_layout.addWidget(self.percentage_label, 1, 1)
        self.auto_pressure_label = QLabel("권장압력: -- kPa"); self.auto_pressure_label.setStyleSheet("color: blue;")
        stats_layout.addWidget(self.auto_pressure_label, 2, 0, 1, 2)
        stats_group.setLayout(stats_layout); left_layout.addWidget(stats_group)

        log_group = QGroupBox("시스템 로그"); log_layout = QVBoxLayout()
        self.log_text = QTextEdit(); self.log_text.setReadOnly(True); self.log_text.setMaximumHeight(120)
        log_layout.addWidget(self.log_text); log_group.setLayout(log_layout); left_layout.addWidget(log_group)

        left_layout.addStretch(); layout.addWidget(left_panel)

        right_panel = QWidget(); right_layout = QVBoxLayout(right_panel)
        self.height_plot = self.create_plot("유체 높이 (mm)", "Height", '#2E86AB'); right_layout.addWidget(self.height_plot)

        history_group = QGroupBox("토출 이력"); history_layout = QVBoxLayout()
        self.history_table = QTableWidget(); self.history_table.setColumnCount(8)
        self.history_table.setHorizontalHeaderLabels(["시간","목표(ml)","허용오차(±ml)","추정량(ml)","합/불","압력(kPa)","유량(ml/s)","수위(mm)"])
        self.history_table.horizontalHeader().setStretchLastSection(True); self.history_table.setMaximumHeight(220)
        history_layout.addWidget(self.history_table); history_group.setLayout(history_layout); right_layout.addWidget(history_group)
        layout.addWidget(right_panel)

        self.update_timer = QTimer(); self.update_timer.timeout.connect(self.update_estimate); self.update_timer.start(500)

        self.pressure_mode.currentIndexChanged.connect(self.on_pressure_mode_changed)
        self.pressure_input.valueChanged.connect(self.on_pressure_input_changed)
        # ★ NEW: 시간 보정계수 변경 시 config에 반영
        self.time_gain.valueChanged.connect(lambda v: setattr(self.config, "time_correction_gain", float(v)))

    def create_plot(self, title, ylabel, color):
        w = pg.GraphicsLayoutWidget(); w.setBackground('#F5F5F5')
        p = w.addPlot(title=title); p.setLabel('left', ylabel); p.setLabel('bottom', 'Time', units='s'); p.showGrid(x=True, y=True, alpha=0.3)
        w.plot_item = p; w.x_data = collections.deque(maxlen=2000); w.y_data = collections.deque(maxlen=2000)
        w.start_time = time.time(); w.window_sec = 30.0; w.curve = p.plot([], [], pen=pg.mkPen(color, width=2))
        p.setXRange(0, w.window_sec); p.setYRange(0, self.config.tank_height_mm)
        return w

    def setup_connections(self):
        self.connect_btn.clicked.connect(self.connect_sensor)
        self.dispense_btn.clicked.connect(self.start_dispense)
        self.sensor_thread.data_received.connect(self.handle_sensor_data)
        self.sensor_thread.error_occurred.connect(self.handle_error)
        self.sensor_thread.log_message.connect(self.log)
        self.dispense_thread.dispense_complete.connect(self.handle_dispense_complete)
        self.dispense_thread.status_update.connect(self.log)

    def on_pressure_mode_changed(self, idx: int):
        mode = [PressureMode.FIXED, PressureMode.MANUAL, PressureMode.ADAPTIVE][idx]
        self.pressure_controller.set_mode(mode)
        self.pressure_input.setEnabled(mode == PressureMode.MANUAL)
        if mode == PressureMode.FIXED:
            self.pressure_input.setValue(self.config.fixed_pressure_kpa)
            self.auto_pressure_label.setText("권장압력: 고정 50.0 kPa")
        elif mode == PressureMode.MANUAL:
            self.pressure_input.setValue(self.config.manual_pressure_kpa)
            self.auto_pressure_label.setText("권장압력: 수동 설정 사용")
        else:
            self.auto_pressure_label.setText("권장압력: 수위에 따라 자동 계산")

    def on_pressure_input_changed(self, kpa: float):
        idx = self.pressure_mode.currentIndex()
        if idx == 0: self.pressure_controller.set_fixed_pressure_kpa(float(kpa))
        elif idx == 1: self.pressure_controller.set_manual_pressure_kpa(float(kpa))

    def connect_sensor(self):
        ip = self.ip_input.text(); port = int(self.port_input.text())
        self.log("연결 시도중...")
        if self.sensor_thread.setup_connection(ip, port):
            if self.sensor_thread.setup_detector():
                self.sensor_thread.start(); self.connect_btn.setEnabled(False); self.log("시스템 준비 완료")

    def start_dispense(self):
        target = self.target_volume.value()
        # ★ 내부 목표로 환산
        target_internal = target / max(self.config.target_mapping_ratio, 1e-9)

        self._last_target_ml = float(target)               # 사용자 기준 목표(표시/판정용)
        self._last_tol_ml = float(self.tol_volume.value())

        if self.pressure_mode.currentText() == "수동":
            pressure_kpa = self.pressure_input.value()
            self.pressure_controller.regulator.set_pressure(pressure_kpa * 1000)

        # ★ 변경: 내부 목표로 토출
        self.dispense_thread.set_parameters(target_internal)

        self.dispense_thread.start()
        self.dispense_btn.setEnabled(False)
        # (선택) 로그로 내부 목표도 알려주기
        self.log(f"내부 목표 적용: {target:.2f} ml / r={self.config.target_mapping_ratio:.2f} → {target_internal:.2f} ml")

    def update_estimate(self):
        target = self.target_volume.value()
        # ★ 추가: 내부 계산에 사용할 목표량 (맵핑 보정)
        target_internal = target / max(self.config.target_mapping_ratio, 1e-9)

        if hasattr(self.sensor_thread, 'current_height_mm'):
            self.pressure_controller.update_height(self.sensor_thread.current_height_mm)

        # ★ 변경: 내부 목표로 예상치 계산
        params = self.pressure_controller.calculate_dispense_parameters(target_internal)

        self.flow_rate_label.setText(f"{params['flow_rate_ml_s']:.2f} ml/s")
        self.time_estimate.setText(f"{params['time_s']:.2f} s")
        mode_txt = ["고정", "수동", "적응"][self.pressure_mode.currentIndex()]
        kpa_txt = f"{params['pressure_pa']/1000:.1f} kPa"
        self.auto_pressure_label.setText(
            f"모드: {mode_txt} | 목표압력: {kpa_txt} | 수위보정×{params.get('height_factor',1.0):.2f} | 시간보정×{self.config.time_correction_gain:.2f}"
        )
        # ★ 표시는 원래(사용자 입력) 목표 기준으로
        self.spec_label.setText(f"목표: {target:.2f} ± {self.tol_volume.value():.2f} ml")

    @Slot(dict)
    def handle_sensor_data(self, data):
        self.volume_label.setText(f"부피: {data['volume_ml']:.2f} ml")
        self.height_label.setText(f"수위: {data['height_mm']:.1f} mm")
        self.percentage_label.setText(f"충전율: {data['percentage']:.1f}%")
        self.pressure_controller.update_height(data['height_mm'])
        t = time.time() - self.height_plot.start_time; h = float(data['height_mm'])
        self.height_plot.x_data.append(t); self.height_plot.y_data.append(h)
        x_arr = np.array(self.height_plot.x_data, dtype=float)
        y_arr = np.array(self.height_plot.y_data, dtype=float)
        if x_arr.size > 0:
            self.height_plot.curve.setData(x_arr, y_arr)
            W = self.height_plot.window_sec; x0 = max(0.0, t - W)
            self.height_plot.plot_item.setXRange(x0, x0 + W, padding=0)
            if y_arr.size >= 5:
                y_min = max(0.0, np.min(y_arr[-200:]) - 2.0)
                y_max = min(self.config.tank_height_mm, np.max(y_arr[-200:]) + 2.0)
                if y_max - y_min < 5.0:
                    y_mid = 0.5*(y_max + y_min); y_min, y_max = y_mid-2.5, y_mid+2.5
                    y_min = max(0.0, y_min); y_max = min(self.config.tank_height_mm, y_max)
                self.height_plot.plot_item.setYRange(y_min, y_max, padding=0)
            QApplication.processEvents()
        self.data_log.append(data)

    @Slot(dict)
    def handle_dispense_complete(self, result):
        self.dispense_btn.setEnabled(True)
        if result['success']:
            target_ml = getattr(self, "_last_target_ml", float(self.target_volume.value()))
            tol_ml = getattr(self, "_last_tol_ml", float(self.tol_volume.value()))
            est_ml = float(result['estimated_ml']); kpa = result['pressure_pa'] / 1000.0
            flow = float(result['flow_rate']); h_mm = float(result['height_mm'])
            pass_ok = (abs(est_ml - target_ml) <= tol_ml); pass_text = "합격" if pass_ok else "불합격"
            row = self.history_table.rowCount(); self.history_table.insertRow(row)
            time_str = datetime.now().strftime("%H:%M:%S")
            items = [time_str,f"{target_ml:.2f}",f"{tol_ml:.2f}",f"{est_ml:.2f}",pass_text,f"{kpa:.1f}",f"{flow:.2f}",f"{h_mm:.1f}"]
            for col, text in enumerate(items):
                item = QTableWidgetItem(text); item.setTextAlignment(Qt.AlignCenter)
                if col == 4:
                    item.setBackground(QColor(210,245,210) if pass_ok else QColor(255,200,200))
                self.history_table.setItem(row, col, item)
            self.dispense_log.append({**result,"target_ml":target_ml,"tolerance_ml":tol_ml,"pass":pass_ok})
            self.log(f"토출 완료: {target_ml:.2f}ml (±{tol_ml:.2f}) @ {kpa:.0f}kPa → 추정 {est_ml:.2f}ml ({'합격' if pass_ok else '불합격'})")
        else:
            self.log(f"토출 실패: {result.get('error','Unknown')}")

    @Slot(str)
    def handle_error(self, msg):
        self.log(f"오류: {msg}")
        QMessageBox.critical(self, "오류", msg)

    def log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")

    def closeEvent(self, event):
        self.sensor_thread.stop(); self.pressure_controller.cleanup()
        if self.dispense_log:
            filename = f"dispense_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            try:
                with open(filename,'w',newline='') as f:
                    fieldnames = ['timestamp','target_ml','tolerance_ml','estimated_ml','pressure_pa','flow_rate','height_mm','time_s','pass']
                    writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore'); writer.writeheader()
                    for e in self.dispense_log:
                        writer.writerow({'timestamp': datetime.now().isoformat(), **e})
                self.log(f"CSV 저장: {filename}")
            except Exception as e:
                self.log(f"CSV 저장 실패: {e}")
        event.accept()

def main():
    pg.setConfigOptions(antialias=True)
    app = QApplication(sys.argv); window = MainWindow(); window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
