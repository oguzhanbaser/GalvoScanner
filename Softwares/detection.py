"""
Detection modülü - Galvo Scanner için algılama döngüsü
"""
import cv2
import numpy as np
import time
from collections import deque
import serial
from myDetector import MyDetector


class GalvoDetection:
    """Galvo tarayıcı için algılama ve kontrol sınıfı"""
    
    def __init__(self, app_state, config=None):
        """
        Args:
            app_state: app.py'den paylaşılan global state (frames, locks, etc.)
            config: Yapılandırma sözlüğü
        """
        self.app_state = app_state
        self.config = config or {}
        
        # Varsayılan yapılandırma
        self.serial_port_name = self.config.get('serial_port', 'COM5')
        self.video_source = self.config.get('video_source', "http://192.168.1.14:4500/video")
        self.laser_settings = self.config.get('laser_settings', "laser_trackbar_settings.json")
        self.led_settings = self.config.get('led_settings', "led_settings.json")
        
        # Durum değişkenleri
        self.serial_port = None
        self.serial_connected = False
        self.cap = None
        self.detector = None
        
        # Tracking değişkenleri
        self.step_x = 0
        self.step_y = 0
        self.last_time = 0
        self.laser_buffer = deque(maxlen=3)
        self.led_buffer = deque(maxlen=3)
    
    def init_serial(self):
        """Seri portu başlat"""
        try:
            self.serial_port = serial.Serial(
                self.serial_port_name, 
                115200, 
                timeout=1, 
                dsrdtr=True
            )
            self.serial_port.write(b'G0,0,')
            self.serial_connected = True
            
            # Global state'i güncelle
            self.app_state['serial_port'] = self.serial_port
            self.app_state['serial_connected'] = True
            
            print(f"✅ Seri port bağlandı: {self.serial_port_name}")
            self.app_state['add_log'](f"✅ Seri port bağlandı: {self.serial_port_name}")
            return True
        except Exception as e:
            print(f"⚠️ Seri port bağlantı hatası: {e}")
            self.app_state['add_log'](f"⚠️ Seri port hatası: {e}")
            self.serial_connected = False
            self.serial_port = None
            self.app_state['serial_port'] = None
            self.app_state['serial_connected'] = False
            return False
    
    def init_video(self):
        """Video kaynağını başlat"""
        self.cap = cv2.VideoCapture(self.video_source)
        self.detector = MyDetector(
            laser_settings_file=self.laser_settings, 
            led_settings_file=self.led_settings
        )
        print(f"📹 Video kaynağı: {self.video_source}")
        return self.cap.isOpened()
    
    def wait_for_serial_data(self, timeout=2):
        """Seri porttan veri bekle"""
        if not self.serial_port:
            return None
        mTime = time.time()
        while self.serial_port.in_waiting == 0:
            if time.time() - mTime > timeout:
                return None
            time.sleep(0.01)
        data = self.serial_port.readline().decode('utf-8').rstrip()
        return data
    
    def parse_serial_command(self, command_str):
        """Seri komutu ayrıştır"""
        cmd_data = None
        try:
            parts = command_str.strip().split(',')
            if len(parts) == 3 and parts[0] == '#':
                cmd_data = {'command': parts[1], 'data': parts[2]}
        except Exception as e:
            print(f"Seri komut ayrıştırma hatası: {e}")
        return cmd_data
    
    def process_frame(self, frame):
        """Tek bir frame'i işle"""
        laser_point = self.detector.detect_laser(frame)
        led_points = self.detector.detect_leds(frame)
        
        # Frame'leri güncelle
        with self.app_state['frame_lock']:
            self.app_state['current_frames']['original'] = frame.copy()
            if led_points is not None:
                self.app_state['current_frames']['led'] = led_points['annotated_frame'].copy()
            if laser_point is not None:
                self.app_state['current_frames']['laser'] = laser_point['annotated_frame'].copy()
        
        return laser_point, led_points
    
    def calculate_tracking(self, laser_point, led_points):
        """Takip hesaplamalarını yap"""
        if not self.app_state.get('tracking_enabled', False):
            return None
        
        if laser_point is None or led_points is None:
            return None
        
        self.laser_buffer.append(laser_point['center_point'])
        self.led_buffer.append(led_points['center_point'])
        
        laser_x = int(np.mean([p[0] for p in self.laser_buffer]))
        laser_y = int(np.mean([p[1] for p in self.laser_buffer]))
        led_x = int(np.mean([p[0] for p in self.led_buffer]))
        led_y = int(np.mean([p[1] for p in self.led_buffer]))
        
        diff_x = laser_x - led_x
        diff_y = laser_y - led_y
        
        TOLERANCE = 3
        if abs(diff_x) < TOLERANCE:
            diff_x = 0
        if abs(diff_y) < TOLERANCE:
            diff_y = 0
        
        return {'diff_x': diff_x, 'diff_y': diff_y}
    
    def send_movement(self, diff_x, diff_y):
        """Hareket komutu gönder"""
        if diff_x == 0 and diff_y == 0:
            return False
        
        if time.time() - self.last_time <= 0.1:
            return False
        
        step_size_x = 1
        step_size_y = 1
        
        if diff_x > 0:
            self.step_x -= step_size_x
        elif diff_x < 0:
            self.step_x += step_size_x
        
        if diff_y > 0:
            self.step_y -= step_size_y
        elif diff_y < 0:
            self.step_y += step_size_y
        
        self.last_time = time.time()
        command = f'G{self.step_y},{self.step_x},'
        
        if self.serial_connected and self.serial_port is not None:
            try:
                self.serial_port.write(command.encode())
                recData = self.wait_for_serial_data()
                if recData:
                    cmd_data = self.parse_serial_command(recData)
                    if cmd_data and (cmd_data['command'] != 'M' or cmd_data['data'] != 'O'):
                        self.app_state['add_log'](f"Beklenmeyen yanıt: {recData}")
            except Exception as e:
                self.app_state['add_log'](f"Seri port hatası: {e}")
        
        log_msg = f"DiffX: {diff_x:+4d}, DiffY: {diff_y:+4d} | StepX: {self.step_x:+4d}, StepY: {self.step_y:+4d} | Komut: {command}"
        self.app_state['add_log'](log_msg)
        print(log_msg)
        
        return True
    
    def run(self):
        """Ana algılama döngüsü"""
        self.init_serial()
        
        if not self.init_video():
            print("❌ Video kaynağına bağlanılamadı!")
            return
        
        print("🚀 Algılama döngüsü başlatıldı")
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Video sona erdi veya okunamadı. Yeniden bağlanılıyor...")
                time.sleep(1)
                self.cap = cv2.VideoCapture(self.video_source)
                continue
            
            # Frame'i işle
            laser_point, led_points = self.process_frame(frame)
            
            # Takip hesapla
            tracking_result = self.calculate_tracking(laser_point, led_points)
            
            # Hareket gönder
            if tracking_result:
                self.send_movement(tracking_result['diff_x'], tracking_result['diff_y'])
        
        self.cleanup()
    
    def cleanup(self):
        """Kaynakları temizle"""
        if self.cap:
            self.cap.release()
        if self.serial_port:
            self.serial_port.close()


def detection_loop(app_state, config=None):
    """
    Ana algılama döngüsü fonksiyonu - eski API uyumluluğu için
    
    Args:
        app_state: Global state sözlüğü
        config: Yapılandırma sözlüğü
    """
    detector = GalvoDetection(app_state, config)
    detector.run()
