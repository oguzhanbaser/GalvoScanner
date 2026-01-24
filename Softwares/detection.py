"""
Detection modülü - Galvo Scanner için algılama döngüsü
"""
import cv2
import numpy as np
import time
from collections import deque
import serial
from myDetector import MyDetector
from myCamera import MyCamera
from gpiozero import LED, MCP3008

class GalvoDetection:
    """Galvo tarayıcı için algılama ve kontrol sınıfı"""
    
    def __init__(self, app_state, config=None):
        """
        Args:
            app_state: app.py'den paylaşılan global state (frames, locks, etc.)
            config: Yapılandırma sözlüğü
        """

        self.useCamera = True
        self.app_state = app_state
        self.config = config or {}
        
        # Helper method for logging
        self.add_log = lambda msg: self.app_state.add_log(msg) if hasattr(self.app_state, 'add_log') else None

        self.led1 = LED(17)
        self.led2 = LED(27)
        self.led3 = LED(18)
        self.led4 = LED(22)

        adcVal1 = MCP3008(channel=0)
        adcVal2 = MCP3008(channel=1)
        adcVal3 = MCP3008(channel=2)
        adcVal4 = MCP3008(channel=3)
        
        # Varsayılan yapılandırma
        self.serial_port_name = self.config.get('serial_port', '/dev/ttyS0')
        self.video_source = self.config.get('video_source', "http://192.168.19.18:4500/video_roi")
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

        self.led1.on()
        self.led2.on()
        self.led3.on()
        self.led4.on()

        time.sleep(0.5)

        # self.led1.off()
        self.led2.off()
        self.led3.off()
        self.led4.off()

        if self.useCamera:
            self.camera = MyCamera()
    
    def init_serial(self):
        """Seri portu başlat"""
        try:
            self.app_state.serial_port = serial.Serial(
                self.serial_port_name, 
                115200, 
                timeout=1, 
                dsrdtr=True
            )
            self.app_state.serial_port.write(b'G0,0,')
            self.app_state.serial_connected = True
            
            print(f"✅ Seri port bağlandı: {self.serial_port_name}")
            self.add_log(f"✅ Seri port bağlandı: {self.serial_port_name}")
            return True
        except Exception as e:
            print(f"⚠️ Seri port bağlantı hatası: {e}")
            self.add_log(f"⚠️ Seri port hatası: {e}")
            self.app_state.serial_connected = False
            self.app_state.serial_port = None
            return False
    
    def init_video(self):
        """Video kaynağını başlat"""
        if not self.useCamera:
            self.cap = cv2.VideoCapture(self.video_source, cv2.CAP_FFMPEG)
            # Buffer boyutunu 1 yap - sadece en son frame'i al, eski frame'leri atla
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            # FPS'i artır (eğer kaynak destekliyorsa)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            print(f"📹 Video kaynağı: {self.video_source}")
        else:
            print(f"📹 Kamera kullanılıyor: MyCamera sınıfı")
        
        self.detector = MyDetector(
            laser_settings_file=self.laser_settings, 
            led_settings_file=self.led_settings
        )
        
        if self.useCamera:
            return True  # Kamera her zaman hazır
        return self.cap.isOpened()
    
    def wait_for_serial_data(self, timeout=2):
        """Seri porttan veri bekle"""
        if not self.app_state.serial_port:
            return None
        mTime = time.time()
        while self.app_state.serial_port.in_waiting == 0:
            if time.time() - mTime > timeout:
                return None
            time.sleep(0.01)
        data = self.app_state.serial_port.readline().decode('utf-8').rstrip()
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
        """Tek bir frame'i işle - OPTIMIZE: Gereksiz copy() kaldırıldı"""
        laser_point = self.detector.detect_laser(frame)
        led_points = self.detector.detect_leds(frame)
        
        # cv2.imshow("frame"  , frame)
        # cv2.waitKey(1)

        # Frame'leri güncelle - OPTIMIZE: annotated_frame zaten kopyalandı, tekrar copy() gereksiz
        with self.app_state.frame_lock:
            self.app_state.current_frames['original'] = frame
            if led_points is not None:
                self.app_state.current_frames['led'] = led_points['annotated_frame']
            if laser_point is not None:
                self.app_state.current_frames['laser'] = laser_point['annotated_frame']
        
        return laser_point, led_points
    
    def calculate_tracking(self, laser_point, led_points):
        """Takip hesaplamalarını yap - OPTIMIZE: Numpy vektör işlemleri kullanıldı"""
        if not self.app_state.tracking_enabled:
            return None
        
        # İlk takip başlangıcında motorların son pozisyonundan başla
        if len(self.laser_buffer) == 0 and len(self.led_buffer) == 0:
            self.step_x = self.app_state.motor_position_y
            self.step_y = self.app_state.motor_position_x
            self.add_log(f"📍 Takip başlangıç pozisyonu: X={self.step_y}, Y={self.step_x}")
        
        if laser_point is None or led_points is None:
            return None
        
        self.laser_buffer.append(laser_point['center_point'])
        self.led_buffer.append(led_points['center_point'])
        
        # OPTIMIZE: List comprehension yerine numpy array işlemleri - daha hızlı
        laser_points = np.array(self.laser_buffer)
        led_points_arr = np.array(self.led_buffer)
        
        laser_x = int(np.mean(laser_points[:, 0]))
        laser_y = int(np.mean(laser_points[:, 1]))
        led_x = int(np.mean(led_points_arr[:, 0]))
        led_y = int(np.mean(led_points_arr[:, 1]))
        
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
        command = f'G{self.step_x},{self.step_y},'
        
        # SharedState'teki motor pozisyonlarını güncelle
        self.app_state.motor_position_x = self.step_y
        self.app_state.motor_position_y = self.step_x
        
        if self.app_state.serial_connected and self.app_state.serial_port is not None:
            try:
                self.app_state.serial_port.write(command.encode())
                recData = self.wait_for_serial_data()
                if recData:
                    cmd_data = self.parse_serial_command(recData)
                    if cmd_data and (cmd_data['command'] != 'M' or cmd_data['data'] != 'O'):
                        self.add_log(f"Beklenmeyen yanıt: {recData}")
            except Exception as e:
                self.add_log(f"Seri port hatası: {e}")
        
        log_msg = f"DiffX: {diff_x:+4d}, DiffY: {diff_y:+4d} | StepX: {self.step_x:+4d}, StepY: {self.step_y:+4d} | Komut: {command}"
        self.add_log(log_msg)
        print(log_msg)
        
        return True
    
    def calculate_precision_tracking(self, laser_point, led_points):
        """Hassas takip hesaplamaları - Şu an boş, ileride doldurulacak"""
        if not self.app_state.precision_tracking_enabled:
            return None
        
        # İlk hassas takip başlangıcında motorların son pozisyonundan başla
        if len(self.laser_buffer) == 0 and len(self.led_buffer) == 0:
            self.step_x = self.app_state.motor_position_y
            self.step_y = self.app_state.motor_position_x
            self.add_log(f"🎯 Hassas takip başlangıç pozisyonu: X={self.step_y}, Y={self.step_x}")
        
        # TODO: Hassas takip algoritması buraya gelecek
        # Şimdilik None dönüyor
        return None
    
    def run(self):
        """Ana algılama döngüsü"""
        self.init_serial()
        
        if not self.init_video():
            print("❌ Video kaynağına bağlanılamadı!")
            return
        
        print("🚀 Algılama döngüsü başlatıldı")
        
        while True:

            # if self.app_state.serial_port is not None and self.app_state.serial_connected:
            #     while self.app_state.serial_port.in_waiting > 0:
            #         print(self.app_state.serial_port.read())

            if self.useCamera:
                frame = self.camera.get_frame_roi()
                ret = True
            else:
                ret, frame = self.cap.read()
            
            frame = cv2.resize(frame, (640, 480))

            if not ret:
                print("Video sona erdi veya okunamadı. Yeniden bağlanılıyor...")
                time.sleep(1)
                if not self.useCamera:
                    self.cap = cv2.VideoCapture(self.video_source)
                continue
            
            # Frame'i işle
            laser_point, led_points = self.process_frame(frame)
            
            # Normal takip hesapla
            tracking_result = self.calculate_tracking(laser_point, led_points)
            
            # Hassas takip hesapla (eğer normal takip aktif değilse)
            precision_tracking_result = None
            if not self.app_state.tracking_enabled:
                precision_tracking_result = self.calculate_precision_tracking(laser_point, led_points)
            
            # Hareket gönder (normal veya hassas takip)
            if tracking_result:
                # print(f"Takip sonucu: DiffX={tracking_result['diff_x']}, DiffY={tracking_result['diff_y']}")
                self.send_movement(tracking_result['diff_x'], tracking_result['diff_y'])
            elif precision_tracking_result:
                # Hassas takip hareket gönderimi buraya gelecek
                pass
    

        self.cleanup()
    
    def cleanup(self):
        """Kaynakları temizle"""
        if self.cap and not self.useCamera:
            self.cap.release()
        if self.app_state.serial_port:
            self.app_state.serial_port.close()
        if self.useCamera and self.camera:
            # Kamerayı durdur
            try:
                self.camera.picam2.stop()
            except:
                pass


