import cv2
import numpy as np
import time
from flask import Flask, Response, render_template, jsonify, request
import threading
from collections import deque
from myDetector import MyDetector
import serial
from datetime import datetime

# Global değişkenler - frame'leri tutmak için
current_frames = {
    'original': None,
    'laser': None,
    'led': None
}
frame_lock = threading.Lock()

# Seri port durumu için global değişkenler
serial_port = None
serial_connected = False
serial_lock = threading.Lock()

# Tracking (takip) durumu - başlat/durdur kontrolü
tracking_enabled = False

# Global detector nesnesi - ayar kaydetme için
detector = None

# Seçilen renkler - detector'dan yüklenecek
selected_colors = {
    'blue': None,  # LED
    'red': None    # Laser
}

# Log buffer - son 100 log satırı
log_buffer = deque(maxlen=100)
log_lock = threading.Lock()

def add_log(message):
    """Log buffer'a mesaj ekle"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    with log_lock:
        log_buffer.appendleft(f"[{timestamp}] {message}")

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

def send_galvo_command(command):
    """Galvo'ya komut gönder"""
    global serial_port, serial_connected
    with serial_lock:
        if serial_connected and serial_port is not None:
            try:
                serial_port.write(command.encode())
                recData = waitForSerialData(serial_port)
                return True, recData
            except Exception as e:
                print(f"Seri port hatası: {e}")
                return False, str(e)
        return False, "Seri port bağlı değil"

def generate_frames(frame_type):
    """Video frame'lerini MJPEG formatında stream et"""
    while True:
        with frame_lock:
            frame = current_frames.get(frame_type)
        
        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            time.sleep(0.1)

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
    global serial_connected
    return jsonify({
        'serial_connected': serial_connected,
        'status': 'ok'
    })

@app.route('/api/home', methods=['POST'])
def api_home():
    """Home pozisyonuna git"""
    success, message = send_galvo_command('H')
    if success:
        # Ardından 0,0 pozisyonuna git
        send_galvo_command('G0,0,')
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
    
    command = f'G{y},{x},'
    success, message = send_galvo_command(command)
    
    return jsonify({
        'success': success,
        'message': message if not success else f'Pozisyon: X={x}, Y={y}',
        'command': command
    })

@app.route('/api/logs')
def api_logs():
    """Log buffer'ı döndür"""
    with log_lock:
        logs = list(log_buffer)
    return jsonify({'logs': logs})

@app.route('/api/tracking/start', methods=['POST'])
def api_tracking_start():
    """Takip sistemini başlat"""
    global tracking_enabled
    tracking_enabled = True
    add_log("🚀 Takip sistemi başlatıldı")
    return jsonify({'success': True, 'tracking': True})

@app.route('/api/tracking/stop', methods=['POST'])
def api_tracking_stop():
    """Takip sistemini durdur"""
    global tracking_enabled
    tracking_enabled = False
    add_log("⏹️ Takip sistemi durduruldu")
    return jsonify({'success': True, 'tracking': False})

@app.route('/api/tracking/status')
def api_tracking_status():
    """Takip durumunu döndür"""
    return jsonify({'tracking': tracking_enabled})

@app.route('/api/get_pixel_color', methods=['POST'])
def api_get_pixel_color():
    """Orijinal frame'den piksel rengini al ve detector ayarlarına kaydet"""
    global selected_colors, detector
    data = request.get_json()
    x = data.get('x', 0)
    y = data.get('y', 0)
    color_type = data.get('color_type', 'blue')  # 'blue' (LED) veya 'red' (Laser)
    
    with frame_lock:
        frame = current_frames.get('original')
    
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
        selected_colors[color_type] = color_data
        
        # Detector ayarlarını güncelle ve kaydet
        if detector is not None:
            H_TOLERANCE = 15
            S_TOLERANCE = 80
            V_TOLERANCE = 80
            
            if color_type == 'blue':  # LED ayarları
                detector.led_params['H_MIN'] = max(0, int(h_val) - H_TOLERANCE)
                detector.led_params['H_MAX'] = min(180, int(h_val) + H_TOLERANCE)
                detector.led_params['S_MIN'] = max(0, int(s_val) - S_TOLERANCE)
                detector.led_params['S_MAX'] = min(255, int(s_val) + S_TOLERANCE)
                detector.led_params['V_MIN'] = max(0, int(v_val) - V_TOLERANCE)
                detector.led_params['V_MAX'] = min(255, int(v_val) + V_TOLERANCE)
                detector.save_led_settings()
                add_log(f"🔵 LED renk ayarları kaydedildi: HSV({h_val},{s_val},{v_val})")
                
            elif color_type == 'red':  # Laser ayarları
                # Kırmızı renk için özel işlem (H 0-10 veya 170-180)
                if h_val <= 10:
                    detector.h_min = max(0, int(h_val) - H_TOLERANCE)
                    detector.h_max = min(10, int(h_val) + H_TOLERANCE)
                    detector.h2_min = max(170, 180 - H_TOLERANCE)
                    detector.h2_max = 180
                elif h_val >= 170:
                    detector.h_min = 0
                    detector.h_max = min(10, H_TOLERANCE)
                    detector.h2_min = max(170, int(h_val) - H_TOLERANCE)
                    detector.h2_max = min(180, int(h_val) + H_TOLERANCE)
                else:
                    detector.h_min = max(0, int(h_val) - H_TOLERANCE)
                    detector.h_max = min(180, int(h_val) + H_TOLERANCE)
                    detector.h2_min = 170
                    detector.h2_max = 180
                
                detector.s_min = max(0, int(s_val) - S_TOLERANCE)
                detector.s_max = min(255, int(s_val) + S_TOLERANCE)
                detector.v_min = max(0, int(v_val) - V_TOLERANCE)
                detector.v_max = min(255, int(v_val) + V_TOLERANCE)
                
                # Laser ayarlarını kaydet
                detector.settings = {
                    "H_MIN": detector.h_min, "H_MAX": detector.h_max,
                    "S_MIN": detector.s_min, "S_MAX": detector.s_max,
                    "V_MIN": detector.v_min, "V_MAX": detector.v_max,
                    "H2_MIN": detector.h2_min, "H2_MAX": detector.h2_max,
                    "MIN_ALAN": detector.min_area, "MAX_ALAN": detector.max_area,
                    "PARLAKLIK_ESIK": detector.parlaklik_esigi,
                    "DAIRESELLIK_ESIGI": int(detector.dairesellik_esigi * 100)
                }
                import json
                with open(detector.LASER_SETTINGS_FILE, "w") as f:
                    json.dump(detector.settings, f, indent=2)
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
    return jsonify(selected_colors)


def create_app_state():
    """Detection modülü için app state sözlüğü oluştur"""
    return {
        'current_frames': current_frames,
        'frame_lock': frame_lock,
        'serial_port': serial_port,
        'serial_connected': serial_connected,
        'serial_lock': serial_lock,
        'tracking_enabled': tracking_enabled,
        'add_log': add_log
    }


def run_detection():
    """Detection döngüsünü çalıştır"""
    global detector
    from detection import detection_loop, GalvoDetection
    
    app_state = create_app_state()
    
    # Global detector nesnesini oluştur
    galvo_detection = GalvoDetection(app_state)
    galvo_detection.init_video()
    detector = galvo_detection.detector  # myDetector nesnesi
    
    # App state'i global değişkenlerle senkronize tut
    import threading
    
    def sync_state():
        while True:
            app_state['tracking_enabled'] = tracking_enabled
            app_state['serial_port'] = serial_port
            app_state['serial_connected'] = serial_connected
            time.sleep(0.1)
    
    sync_thread = threading.Thread(target=sync_state, daemon=True)
    sync_thread.start()
    
    # Detection döngüsünü çalıştır
    galvo_detection.run()


if __name__ == "__main__":
    # Algılama döngüsünü ayrı thread'de başlat
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

