import cv2
import numpy as np
import time
from flask import Flask, Response, render_template, jsonify, request
import threading
from collections import deque
# from myDetector import MyDetector
import serial, threading
from datetime import datetime
from detection import GalvoDetection
import psutil

# Paylaşılan state sınıfı - tüm modüller arasında senkron state yönetimi
class SharedState:
    def __init__(self):
        # Frame'leri tutmak için
        self.current_frames = {
            'original': None,
            'laser': None,
            'led': None
        }
        self.frame_lock = threading.Lock()
        
        # Seri port durumu
        self.serial_port = None
        self.serial_connected = False
        self.serial_lock = threading.Lock()
        
        # Tracking durumu
        self.tracking_enabled = False
        self.precision_tracking_enabled = False

        # Test modu
        self.test_mode = True
        self.test_target_x = 32
        self.test_target_y = -68
        
        # Motor pozisyonları (slider'ların son değerleri)
        self.motor_position_x = 34
        self.motor_position_y = -69
        
        # Global detector nesnesi
        self.detector = None
        
        # Seçilen renkler
        self.selected_colors = {
            'blue': None,  # LED
            'red': None    # Laser
        }
        
        # Log buffer
        self.log_buffer = deque(maxlen=100)
        self.log_lock = threading.Lock()

# Global state instance'ı oluştur
shared_state = SharedState()

def add_log(message):
    """Log buffer'a mesaj ekle"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    with shared_state.log_lock:
        shared_state.log_buffer.appendleft(f"[{timestamp}] {message}")

# Flask uygulaması
app = Flask(__name__)

def waitForSerialData(ser, timeout=2):
    mTime = time.time()
    while ser.in_waiting == 0:
        if time.time() - mTime > timeout:
            return None
        time.sleep(0.01)
    data = ser.readline().decode('utf-8').rstrip()
    return data

def parseSerialCommand(command_str):
    cmd_data = None
    try:
        parts = command_str.strip().split(',')
        if len(parts) == 3:
            if parts[0] == '#':
                cmd = parts[1]
                data = parts[2]
                cmd_data = {'command': cmd, 'data': data}
    except Exception as e:
        print(f"Seri komut ayrıştırma hatası: {e}")
    return cmd_data

def send_galvo_command(command, pTimeout=2):
    """Galvo'ya komut gönder"""
    with shared_state.serial_lock:
        if shared_state.serial_connected and shared_state.serial_port is not None:
            try:
                while shared_state.serial_port.in_waiting > 0:
                    shared_state.serial_port.read()  # Önceki verileri temizle

                shared_state.serial_port.write(command.encode())
                recData = waitForSerialData(shared_state.serial_port, timeout=pTimeout)
                if recData is None:
                    return False, "Zaman aşımı: Yanıt alınamadı"
                else:
                    return True, recData
            except Exception as e:
                print(f"Seri port hatası: {e}")
                return False, str(e)
        return False, "Seri port bağlı değil"

def generate_frames(frame_type):
    """Video frame'lerini MJPEG formatında stream et - sadece yeni frame geldiğinde encode yap"""
    last_frame_id = None
    while True:
        with shared_state.frame_lock:
            frame = shared_state.current_frames.get(frame_type)
        
        if frame is not None and id(frame) != last_frame_id:
            # Yeni frame geldi — encode et ve gönder
            last_frame_id = id(frame)
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            # Frame yok veya değişmedi — CPU'yu boşa harcama
            time.sleep(0.03)

@app.route('/')
def index():
    """Ana sayfa"""
    return render_template('index.html')

@app.route('/video_feed/<frame_type>')
def video_feed(frame_type):
    """Video stream endpoint'leri"""
    if frame_type not in ['original', 'laser', 'led']:
        frame_type = 'original'
    return Response(generate_frames(frame_type),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def api_status():
    """Sistem durumunu döndür"""
    # CPU sıcaklığını al
    cpu_temp = None
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            cpu_temp = round(float(f.read()) / 1000.0, 1)
    except:
        cpu_temp = 0.0
    
    # CPU kullanım yüzdesini al
    cpu_usage = psutil.cpu_percent(interval=0.1)
    
    return jsonify({
        'serial_connected': shared_state.serial_connected,
        'status': 'ok',
        'motor_x': shared_state.motor_position_x,
        'motor_y': shared_state.motor_position_y,
        'cpu_temp': cpu_temp,
        'cpu_usage': cpu_usage
    })

@app.route('/api/home', methods=['POST'])
def api_home():
    """Home pozisyonuna git"""
    success, message = send_galvo_command('H', pTimeout=60)
    # if success:
    #     # Ardından 0,0 pozisyonuna git
    #     send_galvo_command('G0,0,')
    send_galvo_command('G34,-69,')  # Home komutundan sonra pozisyonu sıfırla
    shared_state.motor_position_x = 34
    shared_state.motor_position_y = -69
    return jsonify({
        'success': success,
        'message': message if not success else 'Home pozisyonuna gönderildi'
    })

@app.route('/api/move', methods=['POST'])
def api_move():
    """Belirtilen pozisyona git"""
    data = request.get_json()
    x = data.get('x', 0)
    y = data.get('y', 0)
    
    # Motor pozisyonlarını kaydet
    shared_state.motor_position_x = x
    shared_state.motor_position_y = y
    
    command = f'G{y},{x},'
    success, message = send_galvo_command(command)

    # print(success, message)
    
    return jsonify({
        'success': success,
        'message': message if not success else f'Pozisyon: X={x}, Y={y}',
        'command': command
    })

@app.route('/api/logs')
def api_logs():
    """Log buffer'ı döndür"""
    with shared_state.log_lock:
        logs = list(shared_state.log_buffer)
    return jsonify({'logs': logs})

@app.route('/api/tracking/start', methods=['POST'])
def api_tracking_start():
    """Takip sistemini başlat"""
    shared_state.tracking_enabled = True
    add_log("🚀 Takip sistemi başlatıldı")

    # Test modunda ise belirtilen koordinata adım adım git
    if shared_state.test_mode:
        tx = shared_state.test_target_x
        ty = shared_state.test_target_y
        add_log(f"🧪 Test modu: X={tx}, Y={ty} koordinatına adım adım hareket başladı")
        test_thread = threading.Thread(target=move_to_target_gradually, args=(tx, ty), daemon=True)
        test_thread.start()

    return jsonify({'success': True, 'tracking': True})


@app.route('/api/tracking/test', methods=['POST'])
def api_tracking_test():
    """Test modunu ve hedef koordinatları ayarla"""
    data = request.get_json()
    shared_state.test_mode = data.get('test_mode', False)
    shared_state.test_target_x = data.get('x', 0)
    shared_state.test_target_y = data.get('y', 0)
    state = 'aktif' if shared_state.test_mode else 'devre dışı'
    add_log(f"🧪 Test modu {state}: X={shared_state.test_target_x}, Y={shared_state.test_target_y}")
    return jsonify({
        'success': True,
        'test_mode': shared_state.test_mode,
        'x': shared_state.test_target_x,
        'y': shared_state.test_target_y
    })


@app.route('/api/tracking/test/status')
def api_tracking_test_status():
    """Test modu durumunu döndür"""
    return jsonify({
        'test_mode': shared_state.test_mode,
        'x': shared_state.test_target_x,
        'y': shared_state.test_target_y
    })

@app.route('/api/tracking/stop', methods=['POST'])
def api_tracking_stop():
    """Takip sistemini durdur"""
    shared_state.tracking_enabled = False
    add_log("⏹️ Takip sistemi durduruldu")
    return jsonify({'success': True, 'tracking': False})

@app.route('/api/tracking/status')
def api_tracking_status():
    """Takip durumunu döndür"""
    return jsonify({'tracking': shared_state.tracking_enabled})

@app.route('/api/tracking/precision/start', methods=['POST'])
def api_precision_tracking_start():
    """Hassas takip sistemini başlat"""
    shared_state.precision_tracking_enabled = True
    add_log("🎯 Hassas takip sistemi başlatıldı")
    return jsonify({'success': True, 'precision_tracking': True})

@app.route('/api/tracking/precision/stop', methods=['POST'])
def api_precision_tracking_stop():
    """Hassas takip sistemini durdur"""
    shared_state.precision_tracking_enabled = False
    add_log("⏸️ Hassas takip sistemi durduruldu")
    return jsonify({'success': True, 'precision_tracking': False})

@app.route('/api/tracking/precision/status')
def api_precision_tracking_status():
    """Hassas takip durumunu döndür"""
    return jsonify({'precision_tracking': shared_state.precision_tracking_enabled})

@app.route('/api/get_pixel_color', methods=['POST'])
def api_get_pixel_color():
    """Orijinal frame'den piksel rengini al ve detector ayarlarına kaydet"""
    data = request.get_json()
    x = data.get('x', 0)
    y = data.get('y', 0)
    color_type = data.get('color_type', 'blue')  # 'blue' (LED) veya 'red' (Laser)
    
    with shared_state.frame_lock:
        frame = shared_state.current_frames.get('original')
    
    if frame is not None:
        # Frame boyutlarını kontrol et
        h, w = frame.shape[:2]
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        
        # BGR renk değerlerini al
        b, g, r = frame[y, x]
        
        # HSV'ye dönüştür
        pixel_bgr = np.uint8([[[b, g, r]]])
        pixel_hsv = cv2.cvtColor(pixel_bgr, cv2.COLOR_BGR2HSV)
        h_val, s_val, v_val = pixel_hsv[0][0]
        
        color_data = {
            'r': int(r),
            'g': int(g),
            'b': int(b),
            'h': int(h_val),
            's': int(s_val),
            'v': int(v_val)
        }
        
        # Seçilen rengi kaydet
        shared_state.selected_colors[color_type] = color_data
        
        # Detector ayarlarını güncelle ve kaydet
        if shared_state.detector is not None:
            H_TOLERANCE = 15
            S_TOLERANCE = 80
            V_TOLERANCE = 80
            
            if color_type == 'blue':  # LED ayarları
                shared_state.detector.led_params['H_MIN'] = max(0, int(h_val) - H_TOLERANCE)
                shared_state.detector.led_params['H_MAX'] = min(180, int(h_val) + H_TOLERANCE)
                shared_state.detector.led_params['S_MIN'] = max(0, int(s_val) - S_TOLERANCE)
                shared_state.detector.led_params['S_MAX'] = min(255, int(s_val) + S_TOLERANCE)
                shared_state.detector.led_params['V_MIN'] = max(0, int(v_val) - V_TOLERANCE)
                shared_state.detector.led_params['V_MAX'] = min(255, int(v_val) + V_TOLERANCE)
                shared_state.detector.save_led_settings()
                add_log(f"🔵 LED renk ayarları kaydedildi: HSV({h_val},{s_val},{v_val})")
                
            elif color_type == 'red':  # Laser ayarları
                # Kırmızı renk için özel işlem (H 0-10 veya 170-180)
                if h_val <= 10:
                    shared_state.detector.h_min = max(0, int(h_val) - H_TOLERANCE)
                    shared_state.detector.h_max = min(10, int(h_val) + H_TOLERANCE)
                    shared_state.detector.h2_min = max(170, 180 - H_TOLERANCE)
                    shared_state.detector.h2_max = 180
                elif h_val >= 170:
                    shared_state.detector.h_min = 0
                    shared_state.detector.h_max = min(10, H_TOLERANCE)
                    shared_state.detector.h2_min = max(170, int(h_val) - H_TOLERANCE)
                    shared_state.detector.h2_max = min(180, int(h_val) + H_TOLERANCE)
                else:
                    shared_state.detector.h_min = max(0, int(h_val) - H_TOLERANCE)
                    shared_state.detector.h_max = min(180, int(h_val) + H_TOLERANCE)
                    shared_state.detector.h2_min = 170
                    shared_state.detector.h2_max = 180
                
                shared_state.detector.s_min = max(0, int(s_val) - S_TOLERANCE)
                shared_state.detector.s_max = min(255, int(s_val) + S_TOLERANCE)
                shared_state.detector.v_min = max(0, int(v_val) - V_TOLERANCE)
                shared_state.detector.v_max = min(255, int(v_val) + V_TOLERANCE)
                
                # Laser ayarlarını kaydet
                shared_state.detector.settings = {
                    "H_MIN": shared_state.detector.h_min, "H_MAX": shared_state.detector.h_max,
                    "S_MIN": shared_state.detector.s_min, "S_MAX": shared_state.detector.s_max,
                    "V_MIN": shared_state.detector.v_min, "V_MAX": shared_state.detector.v_max,
                    "H2_MIN": shared_state.detector.h2_min, "H2_MAX": shared_state.detector.h2_max,
                    "MIN_ALAN": shared_state.detector.min_area, "MAX_ALAN": shared_state.detector.max_area,
                    "PARLAKLIK_ESIK": shared_state.detector.parlaklik_esigi,
                    "DAIRESELLIK_ESIGI": int(shared_state.detector.dairesellik_esigi * 100)
                }
                import json
                with open(shared_state.detector.LASER_SETTINGS_FILE, "w") as f:
                    json.dump(shared_state.detector.settings, f, indent=2)
                add_log(f"🔴 Laser renk ayarları kaydedildi: HSV({h_val},{s_val},{v_val})")
        else:
            add_log(f"⚠️ Detector henüz hazır değil, renk seçildi ama kaydedilemedi")
        
        return jsonify({
            'success': True,
            'color': color_data,
            'color_type': color_type
        })
    
    return jsonify({'success': False, 'message': 'Frame bulunamadı'})

@app.route('/api/selected_colors')
def api_selected_colors():
    """Seçilen renkleri döndür"""
    return jsonify(shared_state.selected_colors)


def move_to_target_gradually(target_x, target_y):
    """Test modunda hedef koordinatlara adım adım git (normal takip ile aynı hız)"""
    STEP_INTERVAL = 0.1
    while shared_state.tracking_enabled and shared_state.test_mode:
        cur_x = shared_state.motor_position_x
        cur_y = shared_state.motor_position_y

        if cur_x == target_x and cur_y == target_y:
            add_log(f"🧪 Test modu: Hedefe ulaşıldı X={target_x}, Y={target_y}")
            break

        new_x = cur_x + (1 if cur_x < target_x else -1 if cur_x > target_x else 0)
        new_y = cur_y + (1 if cur_y < target_y else -1 if cur_y > target_y else 0)

        command = f'G{new_y},{new_x},'
        shared_state.motor_position_x = new_x
        shared_state.motor_position_y = new_y
        send_galvo_command(command)
        add_log(f"🧪 Test hareketi: X={new_x}, Y={new_y} → Hedef: X={target_x}, Y={target_y}")

        time.sleep(STEP_INTERVAL)


def run_detection():
    """Detection döngüsünü çalıştır"""
    # Detection döngüsünü çalıştır
    galvo_detection.run()


if __name__ == "__main__":
    # Algılama döngüsünü ayrı thread'de başlat
    # Global detector nesnesini oluştur
    galvo_detection = GalvoDetection(shared_state)
    galvo_detection.init_video()
    shared_state.detector = galvo_detection.detector  # myDetector nesnesi

    detection_thread = threading.Thread(target=run_detection, daemon=True)
    detection_thread.start()
    
    print("=" * 50)
    print("🎯 Galvo Scanner Flask Server")
    print("=" * 50)
    print("Web arayüzü: http://localhost:5000")
    print("Raspberry Pi'dan erişim: http://<raspberry-ip>:5000")
    print("=" * 50)
    
    # Flask uygulamasını başlat
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

