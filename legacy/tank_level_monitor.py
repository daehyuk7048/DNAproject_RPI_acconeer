#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import collections
import time
import threading
import queue
import csv
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

# PySide6 imports
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QLineEdit, QGroupBox,
    QSlider, QSpinBox, QCheckBox, QTextEdit, QFileDialog,
    QStatusBar, QTabWidget, QGridLayout, QFrame, QProgressBar,
    QMessageBox, QSplitter
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot, QThread, QObject
from PySide6.QtGui import QFont, QPalette, QColor, QAction, QIcon

# PyQtGraph for real-time plotting
import pyqtgraph as pg

# Acconeer imports
from acconeer.exptool import a121
from acconeer.exptool.a121.algo.distance import (
    Detector,
    DetectorConfig,
    DetectorResult,
    ThresholdMethod,
    ReflectorShape,
)

# 색상 스키마 정의
@dataclass
class ColorScheme:
    primary_blue: str = '#2E86AB'
    accent_orange: str = '#F24236'
    success_green: str = '#4CAF50'
    warning_yellow: str = '#FFC107'
    danger_red: str = '#F44336'
    background: str = '#F5F5F5'
    surface: str = '#FFFFFF'
    text_primary: str = '#212121'
    text_secondary: str = '#757575'
    grid_color: str = '#E0E0E0'

COLORS = ColorScheme()

# 주사기 탱크 설정 (지름 5cm, 길이 15cm, 센서 거리 5cm)
SYRINGE_CONFIG = {
    'diameter_cm': 5.0,
    'length_cm': 15.0,
    'sensor_distance_cm': 5.0,
    'volume_ml': np.pi * (2.5**2) * 15.0  # πr²h ≈ 294.5 ml
}

class SensorThread(QThread):
    """센서 데이터 수집 스레드"""
    data_received = Signal(dict)
    error_occurred = Signal(str)
    
    def __init__(self):
        super().__init__()
        self.client = None
        self.detector = None
        self.running = False
        self.config = None
        
    def setup_sensor(self, connection_type, ip=None, port=None):
        """센서 연결 설정"""
        try:
            if connection_type == "spi":
                self.client = a121.Client.open(interface="spi")
            else:
                self.client = a121.Client.open(ip_address=ip, tcp_port=int(port))
            return True
        except Exception as e:
            self.error_occurred.emit(f"Connection failed: {str(e)}")
            return False
    
    def setup_detector(self, config):
        """디텍터 설정"""
        try:
            self.config = config
            detector_config = DetectorConfig(
                start_m=(SYRINGE_CONFIG['sensor_distance_cm'] / 100.0),
                end_m=((SYRINGE_CONFIG['sensor_distance_cm'] + SYRINGE_CONFIG['length_cm']) / 100.0),
                max_step_length=1,
                max_profile=a121.Profile.PROFILE_1,  # 근거리 측정용
                threshold_method=ThresholdMethod.CFAR,
                threshold_sensitivity=0.5,
                reflector_shape=ReflectorShape.GENERIC,
                close_range_leakage_cancellation=True,
                update_rate=50  # 50Hz
            )
            
            self.detector = Detector(
                client=self.client,
                sensor_ids=[1],
                detector_config=detector_config
            )
            
            # Calibration
            self.detector.calibrate_detector()
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"Detector setup failed: {str(e)}")
            return False
    
    def run(self):
        """데이터 수집 루프"""
        if not self.detector:
            return
            
        try:
            self.detector.start()
            self.running = True
            
            while self.running:
                result = self.detector.get_next()
                
                # 결과 파싱
                if isinstance(result, dict) and 1 in result:
                    res = result[1]
                else:
                    res = result
                
                # 거리 추출 (mm 단위로)
                distance_m = 0
                if hasattr(res, 'distances') and res.distances:
                    distance_m = res.distances[0]
                
                distance_mm = distance_m * 1000.0
                distance_cm = distance_mm / 10.0
                
                # 유체 레벨 계산 (센서로부터의 거리 - 센서와 탱크 사이 거리)
                fluid_level_cm = distance_cm - SYRINGE_CONFIG['sensor_distance_cm']
                
                # 0 ~ 15cm 범위로 제한
                fluid_level_cm = max(0, min(SYRINGE_CONFIG['length_cm'], 
                                           SYRINGE_CONFIG['length_cm'] - fluid_level_cm))
                
                # 신호 강도
                strength = 0
                if hasattr(res, 'strengths') and res.strengths:
                    strength = res.strengths[0]
                
                # 온도
                temperature = 25.0
                if hasattr(res, 'temperature'):
                    temperature = res.temperature
                
                # 부피 계산 (ml)
                volume_ml = np.pi * (SYRINGE_CONFIG['diameter_cm']/2)**2 * fluid_level_cm
                
                # 데이터 전송
                data = {
                    'timestamp': time.time(),
                    'distance_cm': distance_cm,
                    'fluid_level_cm': fluid_level_cm,
                    'volume_ml': volume_ml,
                    'strength': abs(strength),
                    'temperature': temperature,
                    'percentage': (fluid_level_cm / SYRINGE_CONFIG['length_cm']) * 100
                }
                
                self.data_received.emit(data)
                
                # 50Hz 유지
                time.sleep(0.02)
                
        except Exception as e:
            self.error_occurred.emit(f"Sensor error: {str(e)}")
        finally:
            self.running = False
            if self.detector:
                self.detector.stop()

    def stop(self):
        """센서 정지"""
        self.running = False
        self.wait()

class CircularGauge(pg.GraphicsLayoutWidget):
    """원형 게이지 위젯"""
    def __init__(self):
        super().__init__()
        self.setBackground(COLORS.background)
        
        # 플롯 설정
        self.plot = self.addPlot()
        self.plot.setAspectLocked()
        self.plot.setXRange(-1.5, 1.5)
        self.plot.setYRange(-1.5, 1.5)
        self.plot.hideAxis('left')
        self.plot.hideAxis('bottom')
        
        # 원형 배경
        theta = np.linspace(0, 2*np.pi, 100)
        x_circle = np.cos(theta)
        y_circle = np.sin(theta)
        self.plot.plot(x_circle, y_circle, pen=pg.mkPen(COLORS.text_secondary, width=2))
        
        # 게이지 섹터 (0-100%)
        self.gauge_sectors = []
        for i in range(5):
            start_angle = -np.pi/2 + (i * np.pi * 1.5 / 5)
            end_angle = -np.pi/2 + ((i+1) * np.pi * 1.5 / 5)
            
            if i < 1:  # 0-20%: 빨강
                color = COLORS.danger_red
            elif i < 2:  # 20-40%: 주황
                color = COLORS.accent_orange
            elif i < 3:  # 40-60%: 노랑
                color = COLORS.warning_yellow
            else:  # 60-100%: 초록
                color = COLORS.success_green
            
            theta_sector = np.linspace(start_angle, end_angle, 20)
            x_sector = np.concatenate([[0], 0.9 * np.cos(theta_sector), [0]])
            y_sector = np.concatenate([[0], 0.9 * np.sin(theta_sector), [0]])
            
            sector = self.plot.plot(x_sector, y_sector, 
                                  fillLevel=0, 
                                  brush=pg.mkBrush(color + '40'))
            self.gauge_sectors.append(sector)
        
        # 바늘
        self.needle = self.plot.plot([0, 0], [0, 0.8], 
                                   pen=pg.mkPen(COLORS.text_primary, width=3))
        
        # 중심점
        self.plot.plot([0], [0], 
                      symbol='o', 
                      symbolSize=10, 
                      symbolBrush=COLORS.text_primary)
        
        # 텍스트 레이블
        self.text_item = pg.TextItem('0%', 
                                   color=COLORS.text_primary, 
                                   anchor=(0.5, 0.5))
        self.text_item.setPos(0, -0.3)
        self.text_item.setFont(QFont('Arial', 16, QFont.Bold))
        self.plot.addItem(self.text_item)
        
        # 부피 텍스트
        self.volume_text = pg.TextItem('0 ml', 
                                     color=COLORS.text_secondary, 
                                     anchor=(0.5, 0.5))
        self.volume_text.setPos(0, -0.6)
        self.volume_text.setFont(QFont('Arial', 12))
        self.plot.addItem(self.volume_text)
        
    def update_value(self, percentage, volume_ml):
        """게이지 값 업데이트"""
        # 바늘 각도 계산 (0% = -90도, 100% = 180도)
        angle = -np.pi/2 + (percentage / 100.0) * (np.pi * 1.5)
        x_needle = 0.8 * np.cos(angle)
        y_needle = 0.8 * np.sin(angle)
        self.needle.setData([0, x_needle], [0, y_needle])
        
        # 텍스트 업데이트
        self.text_item.setText(f'{percentage:.1f}%')
        self.volume_text.setText(f'{volume_ml:.1f} ml')

class TankVisualization(pg.GraphicsLayoutWidget):
    """주사기 탱크 시각화"""
    def __init__(self):
        super().__init__()
        self.setBackground(COLORS.background)
        
        self.plot = self.addPlot()
        self.plot.setLabel('left', 'Height', units='cm')
        self.plot.setLabel('bottom', 'Width', units='cm')
        self.plot.setXRange(-4, 4)
        self.plot.setYRange(-2, 22)
        
        # 센서 위치 표시
        sensor_y = -SYRINGE_CONFIG['sensor_distance_cm']
        self.plot.plot([-1, 1], [sensor_y, sensor_y], 
                      pen=pg.mkPen(COLORS.primary_blue, width=3))
        self.sensor_text = pg.TextItem('Sensor', 
                                      color=COLORS.primary_blue, 
                                      anchor=(0.5, 0.5))
        self.sensor_text.setPos(0, sensor_y - 1)
        self.plot.addItem(self.sensor_text)
        
        # 주사기 실린더 (반지름 2.5cm)
        radius = SYRINGE_CONFIG['diameter_cm'] / 2
        x_cylinder = [-radius, radius, radius, -radius, -radius]
        y_cylinder = [0, 0, SYRINGE_CONFIG['length_cm'], 
                     SYRINGE_CONFIG['length_cm'], 0]
        
        self.cylinder = self.plot.plot(x_cylinder, y_cylinder, 
                                     pen=pg.mkPen(COLORS.text_primary, width=2))
        
        # 유체 레벨
        self.fluid_fill = self.plot.plot([], [], 
                                       fillLevel=0, 
                                       brush=pg.mkBrush(COLORS.primary_blue + '60'))
        
        # 레벨 라인
        self.level_line = self.plot.plot([], [], 
                                       pen=pg.mkPen(COLORS.accent_orange, width=3))
        
        # 경고 레벨들
        warning_levels = [3, 5, 10]  # cm
        warning_colors = [COLORS.danger_red, COLORS.accent_orange, COLORS.warning_yellow]
        for level, color in zip(warning_levels, warning_colors):
            self.plot.plot([-radius-0.5, radius+0.5], [level, level], 
                         pen=pg.mkPen(color, width=1, style=Qt.DashLine))
        
        # 거리 표시 화살표
        self.distance_arrow = pg.ArrowItem(angle=90, tipAngle=30, 
                                         brush=COLORS.text_secondary)
        self.distance_arrow.setPos(radius + 1, 0)
        self.plot.addItem(self.distance_arrow)
        
        self.distance_text = pg.TextItem('', 
                                       color=COLORS.text_secondary, 
                                       anchor=(0, 0.5))
        self.plot.addItem(self.distance_text)
        
    def update_level(self, fluid_level_cm, distance_cm):
        """유체 레벨 업데이트"""
        radius = SYRINGE_CONFIG['diameter_cm'] / 2
        
        # 유체 채우기
        x_fluid = [-radius, radius, radius, -radius, -radius]
        y_fluid = [0, 0, fluid_level_cm, fluid_level_cm, 0]
        self.fluid_fill.setData(x_fluid, y_fluid)
        
        # 레벨 라인
        self.level_line.setData([-radius-0.5, radius+0.5], 
                              [fluid_level_cm, fluid_level_cm])
        
        # 거리 화살표와 텍스트
        sensor_y = -SYRINGE_CONFIG['sensor_distance_cm']
        arrow_length = distance_cm
        self.distance_arrow.setPos(radius + 1, sensor_y)
        self.distance_arrow.setStyle(headLen=min(arrow_length * 0.3, 1))
        
        self.distance_text.setPos(radius + 2, sensor_y + arrow_length/2)
        self.distance_text.setText(f'{distance_cm:.1f} cm')

class RealTimePlot(pg.GraphicsLayoutWidget):
    """실시간 데이터 플롯"""
    def __init__(self, title, y_label, y_unit=''):
        super().__init__()
        self.setBackground(COLORS.background)
        
        self.plot = self.addPlot(title=title)
        self.plot.setLabel('left', y_label, units=y_unit)
        self.plot.setLabel('bottom', 'Time', units='s')
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        
        # 데이터 버퍼
        self.max_points = 500
        self.x_data = collections.deque(maxlen=self.max_points)
        self.y_data = collections.deque(maxlen=self.max_points)
        self.start_time = time.time()
        
        # 플롯 라인
        self.curve = self.plot.plot(pen=pg.mkPen(COLORS.primary_blue, width=2))
        
        # 현재 값 텍스트
        self.value_text = pg.TextItem('', 
                                    color=COLORS.text_primary, 
                                    anchor=(0, 1))
        self.value_text.setFont(QFont('Arial', 12, QFont.Bold))
        self.plot.addItem(self.value_text)
        
    def update_data(self, value):
        """데이터 업데이트"""
        current_time = time.time() - self.start_time
        self.x_data.append(current_time)
        self.y_data.append(value)
        
        self.curve.setData(list(self.x_data), list(self.y_data))
        
        # Y축 자동 스케일
        if len(self.y_data) > 0:
            y_min = min(self.y_data) - 1
            y_max = max(self.y_data) + 1
            self.plot.setYRange(y_min, y_max)
            
            # X축 범위 (최근 30초)
            if current_time > 30:
                self.plot.setXRange(current_time - 30, current_time)
            else:
                self.plot.setXRange(0, 30)
        
        # 현재 값 표시
        self.value_text.setPos(current_time - 1, value)
        self.value_text.setText(f'{value:.1f}')

class ConfigurationPanel(QWidget):
    """설정 패널"""
    config_changed = Signal(dict)
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # 연결 설정
        conn_group = QGroupBox("Connection Settings")
        conn_layout = QGridLayout()
        
        conn_layout.addWidget(QLabel("Type:"), 0, 0)
        self.conn_type = QComboBox()
        self.conn_type.addItems(["socket", "spi"])
        conn_layout.addWidget(self.conn_type, 0, 1)
        
        conn_layout.addWidget(QLabel("IP:"), 1, 0)
        self.ip_input = QLineEdit("192.168.10.227")
        conn_layout.addWidget(self.ip_input, 1, 1)
        
        conn_layout.addWidget(QLabel("Port:"), 2, 0)
        self.port_input = QLineEdit("6110")
        conn_layout.addWidget(self.port_input, 2, 1)
        
        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)
        
        # 측정 설정
        measure_group = QGroupBox("Measurement Settings")
        measure_layout = QGridLayout()
        
        measure_layout.addWidget(QLabel("Update Rate (Hz):"), 0, 0)
        self.update_rate = QSpinBox()
        self.update_rate.setRange(1, 100)
        self.update_rate.setValue(50)
        measure_layout.addWidget(self.update_rate, 0, 1)
        
        measure_layout.addWidget(QLabel("Median Filter:"), 1, 0)
        self.median_filter = QSpinBox()
        self.median_filter.setRange(1, 10)
        self.median_filter.setValue(5)
        measure_layout.addWidget(self.median_filter, 1, 1)
        
        measure_group.setLayout(measure_layout)
        layout.addWidget(measure_group)
        
        # 알람 설정
        alarm_group = QGroupBox("Alarm Settings")
        alarm_layout = QGridLayout()
        
        self.alarm_enabled = QCheckBox("Enable Alarms")
        alarm_layout.addWidget(self.alarm_enabled, 0, 0, 1, 2)
        
        alarm_layout.addWidget(QLabel("Low Level (cm):"), 1, 0)
        self.low_level = QSpinBox()
        self.low_level.setRange(0, 15)
        self.low_level.setValue(3)
        alarm_layout.addWidget(self.low_level, 1, 1)
        
        alarm_layout.addWidget(QLabel("High Level (cm):"), 2, 0)
        self.high_level = QSpinBox()
        self.high_level.setRange(0, 15)
        self.high_level.setValue(12)
        alarm_layout.addWidget(self.high_level, 2, 1)
        
        alarm_group.setLayout(alarm_layout)
        layout.addWidget(alarm_group)
        
        # 탱크 정보 표시
        info_group = QGroupBox("Syringe Tank Info")
        info_layout = QVBoxLayout()
        info_text = f"""
        • Diameter: {SYRINGE_CONFIG['diameter_cm']} cm
        • Length: {SYRINGE_CONFIG['length_cm']} cm
        • Max Volume: {SYRINGE_CONFIG['volume_ml']:.1f} ml
        • Sensor Distance: {SYRINGE_CONFIG['sensor_distance_cm']} cm
        """
        info_label = QLabel(info_text)
        info_label.setStyleSheet(f"color: {COLORS.text_secondary};")
        info_layout.addWidget(info_label)
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        layout.addStretch()
        self.setLayout(layout)

class MainWindow(QMainWindow):
    """메인 윈도우"""
    def __init__(self):
        super().__init__()
        self.sensor_thread = SensorThread()
        self.data_buffer = []
        self.init_ui()
        self.setup_connections()
        
    def init_ui(self):
        self.setWindowTitle("Acconeer A121 Tank Level Monitor - Professional Edition")
        self.setGeometry(100, 100, 1400, 900)
        
        # 스타일 설정
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {COLORS.background};
            }}
            QGroupBox {{
                font-weight: bold;
                border: 2px solid {COLORS.grid_color};
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }}
            QPushButton {{
                background-color: {COLORS.primary_blue};
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {COLORS.accent_orange};
            }}
            QPushButton:disabled {{
                background-color: {COLORS.grid_color};
                color: {COLORS.text_secondary};
            }}
        """)
        
        # 중앙 위젯
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 메인 레이아웃
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # 좌측 패널 (설정 및 컨트롤)
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        left_panel.setLayout(left_layout)
        left_panel.setMaximumWidth(300)
        
        # 설정 패널
        self.config_panel = ConfigurationPanel()
        left_layout.addWidget(self.config_panel)
        
        # 컨트롤 버튼
        control_group = QGroupBox("Control")
        control_layout = QVBoxLayout()
        
        self.connect_btn = QPushButton("Connect")
        self.start_btn = QPushButton("Start")
        self.start_btn.setEnabled(False)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setEnabled(False)
        self.save_btn = QPushButton("Save Data")
        self.save_btn.setEnabled(False)
        
        control_layout.addWidget(self.connect_btn)
        control_layout.addWidget(self.start_btn)
        control_layout.addWidget(self.stop_btn)
        control_layout.addWidget(self.save_btn)
        
        control_group.setLayout(control_layout)
        left_layout.addWidget(control_group)
        
        # 로그
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        left_layout.addWidget(log_group)
        
        main_layout.addWidget(left_panel)
        
        # 우측 패널 (시각화)
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        right_panel.setLayout(right_layout)
        
        # 상단 패널 (게이지와 탱크 시각화)
        top_panel = QWidget()
        top_layout = QHBoxLayout()
        top_panel.setLayout(top_layout)
        
        # 원형 게이지
        self.gauge = CircularGauge()
        top_layout.addWidget(self.gauge)
        
        # 탱크 시각화
        self.tank_viz = TankVisualization()
        top_layout.addWidget(self.tank_viz)
        
        # 실시간 데이터 디스플레이
        data_frame = QFrame()
        data_frame.setFrameStyle(QFrame.Box)
        data_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {COLORS.surface};
                border: 1px solid {COLORS.grid_color};
                border-radius: 5px;
                padding: 10px;
            }}
        """)
        data_layout = QVBoxLayout()
        data_frame.setLayout(data_layout)
        
        self.level_label = QLabel("Level: -- cm")
        self.level_label.setFont(QFont('Arial', 24, QFont.Bold))
        self.volume_label = QLabel("Volume: -- ml")
        self.volume_label.setFont(QFont('Arial', 18))
        self.distance_label = QLabel("Distance: -- cm")
        self.distance_label.setFont(QFont('Arial', 14))
        self.strength_label = QLabel("Signal: --")
        self.strength_label.setFont(QFont('Arial', 14))
        
        data_layout.addWidget(self.level_label)
        data_layout.addWidget(self.volume_label)
        data_layout.addWidget(self.distance_label)
        data_layout.addWidget(self.strength_label)
        
        top_layout.addWidget(data_frame)
        
        right_layout.addWidget(top_panel)
        
        # 하단 패널 (실시간 플롯)
        bottom_panel = QWidget()
        bottom_layout = QHBoxLayout()
        bottom_panel.setLayout(bottom_layout)
        
        # 레벨 플롯
        self.level_plot = RealTimePlot("Fluid Level History", "Level", "cm")
        bottom_layout.addWidget(self.level_plot)
        
        # 신호 강도 플롯
        self.strength_plot = RealTimePlot("Signal Strength", "Strength", "")
        bottom_layout.addWidget(self.strength_plot)
        
        right_layout.addWidget(bottom_panel)
        
        main_layout.addWidget(right_panel)
        
        # 상태바
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.connection_status = QLabel("Disconnected")
        self.connection_status.setStyleSheet(f"color: {COLORS.danger_red};")
        self.fps_label = QLabel("FPS: --")
        self.status_bar.addWidget(self.connection_status)
        self.status_bar.addPermanentWidget(self.fps_label)
        
        # FPS 타이머
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(1000)
        self.fps_count = 0
        
    def setup_connections(self):
        """시그널 연결"""
        self.connect_btn.clicked.connect(self.connect_sensor)
        self.start_btn.clicked.connect(self.start_measurement)
        self.stop_btn.clicked.connect(self.stop_measurement)
        self.save_btn.clicked.connect(self.save_data)
        
        self.sensor_thread.data_received.connect(self.handle_sensor_data)
        self.sensor_thread.error_occurred.connect(self.handle_error)
        
    def log(self, message):
        """로그 메시지 추가"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        
    def connect_sensor(self):
        """센서 연결"""
        conn_type = self.config_panel.conn_type.currentText()
        ip = self.config_panel.ip_input.text()
        port = self.config_panel.port_input.text()
        
        self.log("Connecting to sensor...")
        
        if conn_type == "spi":
            success = self.sensor_thread.setup_sensor("spi")
        else:
            success = self.sensor_thread.setup_sensor("socket", ip, port)
        
        if success:
            # 디텍터 설정
            config = {
                'update_rate': self.config_panel.update_rate.value(),
                'median_filter': self.config_panel.median_filter.value()
            }
            
            if self.sensor_thread.setup_detector(config):
                self.log("Connected successfully!")
                self.connection_status.setText("Connected")
                self.connection_status.setStyleSheet(f"color: {COLORS.success_green};")
                
                self.connect_btn.setEnabled(False)
                self.start_btn.setEnabled(True)
            else:
                self.log("Detector setup failed")
        else:
            self.log("Connection failed")
    
    def start_measurement(self):
        """측정 시작"""
        self.log("Starting measurement...")
        self.data_buffer.clear()
        
        # 플롯 초기화
        self.level_plot.x_data.clear()
        self.level_plot.y_data.clear()
        self.level_plot.start_time = time.time()
        
        self.strength_plot.x_data.clear()
        self.strength_plot.y_data.clear()
        self.strength_plot.start_time = time.time()
        
        # 센서 스레드 시작
        self.sensor_thread.start()
        
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.save_btn.setEnabled(False)
        
        self.log("Measurement started")
    
    def stop_measurement(self):
        """측정 중지"""
        self.log("Stopping measurement...")
        
        self.sensor_thread.stop()
        
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.save_btn.setEnabled(True)
        
        self.log("Measurement stopped")
    
    @Slot(dict)
    def handle_sensor_data(self, data):
        """센서 데이터 처리"""
        self.fps_count += 1
        
        # 데이터 저장
        self.data_buffer.append(data)
        
        # 게이지 업데이트
        self.gauge.update_value(data['percentage'], data['volume_ml'])
        
        # 탱크 시각화 업데이트
        self.tank_viz.update_level(data['fluid_level_cm'], data['distance_cm'])
        
        # 레이블 업데이트
        self.level_label.setText(f"Level: {data['fluid_level_cm']:.1f} cm")
        self.volume_label.setText(f"Volume: {data['volume_ml']:.1f} ml")
        self.distance_label.setText(f"Distance: {data['distance_cm']:.1f} cm")
        self.strength_label.setText(f"Signal: {data['strength']:.2f}")
        
        # 레벨에 따른 색상 변경
        if data['fluid_level_cm'] < 3:
            self.level_label.setStyleSheet(f"color: {COLORS.danger_red};")
        elif data['fluid_level_cm'] < 5:
            self.level_label.setStyleSheet(f"color: {COLORS.accent_orange};")
        else:
            self.level_label.setStyleSheet(f"color: {COLORS.success_green};")
        
        # 플롯 업데이트
        self.level_plot.update_data(data['fluid_level_cm'])
        self.strength_plot.update_data(data['strength'])
        
        # 알람 체크
        if self.config_panel.alarm_enabled.isChecked():
            low_level = self.config_panel.low_level.value()
            high_level = self.config_panel.high_level.value()
            
            if data['fluid_level_cm'] < low_level:
                self.status_bar.showMessage(f"⚠️ Low level warning: {data['fluid_level_cm']:.1f} cm", 2000)
            elif data['fluid_level_cm'] > high_level:
                self.status_bar.showMessage(f"⚠️ High level warning: {data['fluid_level_cm']:.1f} cm", 2000)
    
    @Slot(str)
    def handle_error(self, error_msg):
        """에러 처리"""
        self.log(f"ERROR: {error_msg}")
        QMessageBox.critical(self, "Error", error_msg)
    
    def update_fps(self):
        """FPS 업데이트"""
        self.fps_label.setText(f"FPS: {self.fps_count}")
        self.fps_count = 0
    
    def save_data(self):
        """데이터 저장"""
        if not self.data_buffer:
            QMessageBox.warning(self, "Warning", "No data to save")
            return
        
        filename, _ = QFileDialog.getSaveFileName(
            self, 
            "Save Data", 
            f"tank_level_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV Files (*.csv)"
        )
        
        if filename:
            try:
                with open(filename, 'w', newline='') as csvfile:
                    fieldnames = [
                        'timestamp', 'distance_cm', 'fluid_level_cm', 
                        'volume_ml', 'percentage', 'strength', 'temperature'
                    ]
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    
                    for data in self.data_buffer:
                        writer.writerow(data)
                
                self.log(f"Data saved to {filename}")
                QMessageBox.information(self, "Success", f"Data saved successfully to:\n{filename}")
                
            except Exception as e:
                self.log(f"Failed to save data: {str(e)}")
                QMessageBox.critical(self, "Error", f"Failed to save data:\n{str(e)}")
    
    def closeEvent(self, event):
        """창 닫기 이벤트"""
        if self.sensor_thread.isRunning():
            self.sensor_thread.stop()
        
        if self.sensor_thread.client:
            self.sensor_thread.client.close()
        
        event.accept()

def apply_dark_theme(app):
    """다크 테마 적용 (선택사항)"""
    dark_palette = QPalette()
    dark_palette.setColor(QPalette.Window, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.WindowText, Qt.white)
    dark_palette.setColor(QPalette.Base, QColor(25, 25, 25))
    dark_palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ToolTipBase, Qt.white)
    dark_palette.setColor(QPalette.ToolTipText, Qt.white)
    dark_palette.setColor(QPalette.Text, Qt.white)
    dark_palette.setColor(QPalette.Button, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ButtonText, Qt.white)
    dark_palette.setColor(QPalette.BrightText, Qt.red)
    dark_palette.setColor(QPalette.Link, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.HighlightedText, Qt.black)
    
    app.setPalette(dark_palette)

def main():
    # PyQtGraph 설정
    pg.setConfigOptions(antialias=True)
    pg.setConfigOption('background', COLORS.background)
    pg.setConfigOption('foreground', COLORS.text_primary)
    
    # 애플리케이션 생성
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # 폰트 설정
    font = QFont("Arial", 10)
    app.setFont(font)
    
    # 다크 테마 적용 (원하면 주석 해제)
    # apply_dark_theme(app)
    
    # 메인 윈도우 생성
    window = MainWindow()
    window.show()
    
    # 애플리케이션 실행
    sys.exit(app.exec())

if __name__ == "__main__":
    main()