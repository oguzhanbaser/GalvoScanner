import cv2
import numpy as np
import math
import json
import os


# Varsayılan parametreler
DEFAULT_PARAMS = {
    'H_MIN': 114,
    'H_MAX': 154,
    'S_MIN': 60,
    'S_MAX': 255,
    'V_MIN': 92,
    'V_MAX': 250,
    'MIN_ALAN': 5,
    'MAX_ALAN': 5000,
    'PARLAKLIK_ESIGI': 225,
    'DAIRESELLIK_ESIGI': 0.4,
    'MAX_LED_SAYISI': 4,
    'H_TOLERANCE': 20,
    'S_TOLERANCE': 80,
    'V_TOLERANCE': 80
}


def detect_leds(frame, params=None):
    """
    Frame üzerinde LED tespiti yapar.
    
    Args:
        frame: İşlenecek görüntü frame'i (BGR formatında)
        params: Tespit parametreleri (dict). None ise varsayılan değerler kullanılır.
        
    Returns:
        dict: {
            'led_coordinates': [(x1, y1), (x2, y2), ...],  # Tespit edilen LED koordinatları
            'center_point': (x, y),  # Tüm LED'lerin orta noktası
            'led_count': int,  # Tespit edilen LED sayısı
            'annotated_frame': frame,  # İşaretlenmiş görüntü
            'mask': mask,  # Renk maskesi
            'inpaint_mask': inpaint_mask  # Inpaint maskesi
        }
    """
    # Parametreleri birleştir
    if params is None:
        params = DEFAULT_PARAMS.copy()
    else:
        p = DEFAULT_PARAMS.copy()
        p.update(params)
        params = p
    
    # Frame kopyası al
    frame_processed = frame.copy()
    
    # Parametreleri çıkar
    h_min = params['H_MIN']
    h_max = params['H_MAX']
    s_min = params['S_MIN']
    s_max = params['S_MAX']
    v_min = params['V_MIN']
    v_max = params['V_MAX']
    min_alan = max(1, params['MIN_ALAN'])
    max_alan = params['MAX_ALAN']
    parlaklik_esigi = params['PARLAKLIK_ESIGI']
    dairesellik_esigi = params['DAIRESELLIK_ESIGI']
    max_led_sayisi = params['MAX_LED_SAYISI']
    
    # HSV aralığı
    lower_bound = (h_min, s_min, v_min)
    upper_bound = (h_max, s_max, v_max)
    
    # PIXEL SUPPLEMENTATION (Inpainting)
    gray = cv2.cvtColor(frame_processed, cv2.COLOR_BGR2GRAY)
    _, inpaint_mask = cv2.threshold(gray, parlaklik_esigi, 255, cv2.THRESH_BINARY)
    frame_inpainted = cv2.inpaint(frame_processed, inpaint_mask, inpaintRadius=2, flags=cv2.INPAINT_TELEA)
    
    # Blur ve HSV dönüşümü
    blurred = cv2.medianBlur(frame_inpainted, 5)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # Renk maskesi
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Connected Components ile filtreleme
    num_labels, labels = cv2.connectedComponents(mask)
    filtered_mask = np.zeros_like(mask, dtype=np.uint8)
    
    for label in range(1, num_labels):
        region_mask = (labels == label).astype("uint8") * 255
        area = cv2.countNonZero(region_mask)
        if min_alan <= area <= max_alan:
            filtered_mask = cv2.bitwise_or(filtered_mask, region_mask)
    
    mask = filtered_mask
    
    # Konturları bul
    contours, _ = cv2.findContours(inpaint_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    led_merkezleri = []
    
    # Her konturu işle
    for c in contours:
        area = cv2.contourArea(c)
        
        # Alan kontrolü
        if area < min_alan or area > max_alan:
            continue
        
        # Dairesellik kontrolü
        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        dairesellik = (4 * np.pi * area) / (perimeter ** 2)
        
        if dairesellik < dairesellik_esigi:
            continue
        
        # Ortalama HSV kontrolü
        mask_c = np.zeros_like(mask)
        cv2.drawContours(mask_c, [c], -1, 255, -1)
        mean_s = cv2.mean(hsv[:, :, 1], mask=mask_c)[0]
        mean_v = cv2.mean(hsv[:, :, 2], mask=mask_c)[0]
        
        if mean_s < 100 or mean_v < 50:
            continue
        
        # Merkez hesapla
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            led_merkezleri.append((cX, cY))
    
    # X koordinatına göre sırala ve maksimum sayıda tut
    led_merkezleri.sort(key=lambda item: item[0])
    led_merkezleri = led_merkezleri[:max_led_sayisi]
    
    # Orta nokta hesapla
    center_point = None
    if len(led_merkezleri) > 0:
        toplam_x = sum([x for x, y in led_merkezleri])
        toplam_y = sum([y for x, y in led_merkezleri])
        center_point = (int(toplam_x / len(led_merkezleri)), int(toplam_y / len(led_merkezleri)))
    
    return {
        'led_coordinates': led_merkezleri,
        'center_point': center_point,
        'led_count': len(led_merkezleri),
        'annotated_frame': frame_inpainted,
        'mask': mask,
        'inpaint_mask': inpaint_mask
    }


def draw_led_annotations(frame, led_coordinates, center_point=None, draw_lines=True, draw_center=True):
    """
    Frame üzerine LED işaretlemelerini çizer.
    
    Args:
        frame: İşaretlenecek frame
        led_coordinates: LED koordinatları listesi [(x1, y1), (x2, y2), ...]
        center_point: Orta nokta (x, y) - None ise hesaplanmaz
        draw_lines: LED'ler arasında çizgi çiz
        draw_center: Orta noktayı çiz
        
    Returns:
        frame: İşaretlenmiş frame
    """
    annotated = frame.copy()
    led_sayisi = len(led_coordinates)
    
    if led_sayisi == 0:
        return annotated
    
    # LED etiket renkleri
    etiket_renkleri = [(0, 0, 255), (0, 0, 0), (128, 0, 128), (255, 0, 0)]
    
    # Her LED'i işaretle
    for i, (cX, cY) in enumerate(led_coordinates):
        led_no = i + 1
        etiket_renk = etiket_renkleri[i % len(etiket_renkleri)]
        
        # Metin rengini arka plan parlaklığına göre ayarla
        b, g, r = annotated[cY, cX]
        parlaklik_toplami = int(r) + int(g) + int(b)
        metin_renk = (0, 0, 0) if parlaklik_toplami > 350 else (255, 255, 255)
        
        # LED numarası ve daire çiz
        # cv2.putText(annotated, f"LED {led_no}", (cX - 30, cY - 30), 
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.7, metin_renk, 2)
        cv2.circle(annotated, (cX, cY), 5, etiket_renk, -1)
    
    # LED'ler arası çizgiler
    if draw_lines and led_sayisi >= 2:
        for i in range(led_sayisi - 1):
            cv2.line(annotated, led_coordinates[i], led_coordinates[i+1], (0, 255, 255), 2)
        # 4 LED varsa dörtgeni kapat
        if led_sayisi == 4:
            cv2.line(annotated, led_coordinates[3], led_coordinates[0], (0, 255, 255), 2)
    
    # Orta nokta
    if draw_center and center_point is not None:
        cv2.circle(annotated, center_point, 5, (255, 255, 0), -1)
        cv2.putText(annotated, "Orta Nokta", (center_point[0] + 10, center_point[1] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    
    # Durum mesajı
    durum = "4 LED TESPIT EDILDI!" if led_sayisi == 4 else f"LED Sayisi: {led_sayisi}"
    renk = (0, 255, 0) if led_sayisi == 4 else (0, 0, 255)
    cv2.putText(annotated, durum, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, renk, 2)
    
    return annotated


def save_settings(settings, filename="led_settings.json"):
    """Ayarları JSON dosyasına kaydeder."""
    with open(filename, "w") as file:
        json.dump(settings, file, indent=2)


def load_settings(filename="led_settings.json"):
    """Ayarları JSON dosyasından yükler."""
    if os.path.exists(filename):
        with open(filename, "r") as file:
            return json.load(file)
    return {}


class LEDDetectorApp:
    """
    LED tespit uygulaması - Çift tıklama ile renk seçimi
    """
    def __init__(self, video_source=None):
        self.video_source = video_source or os.getenv("STREAM_URL", "http://192.168.19.221:5000/video_roi")
        self.params = DEFAULT_PARAMS.copy()
        self.window_name = "LED Tespit - Cift Tik ile Renk Sec"
        self.frame_current = None
        
        # Ayarları yükle
        settings = load_settings()
        if settings:
            self.params.update(settings)
    
    def on_mouse_click(self, event, x, y, flags, param):
        """Mouse callback - çift tıklama ile renk seçimi ve tespit"""
        if event == cv2.EVENT_LBUTTONDBLCLK and self.frame_current is not None:
            bgr_pixel = self.frame_current[y, x]
            b, g, r = int(bgr_pixel[0]), int(bgr_pixel[1]), int(bgr_pixel[2])
            hsv_pixel = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
            h, s, v = int(hsv_pixel[0]), int(hsv_pixel[1]), int(hsv_pixel[2])
            
            print("\n" + "="*60)
            print(f"RENK SEÇİLDİ - Piksel Koordinat: X={x}, Y={y}")
            print("-" * 60)
            print(f"RGB Değeri: R={r}, G={g}, B={b}")
            print(f"HSV Değeri: H={h}, S={s}, V={v}")
            print("-" * 60)
            
            # Toleransları kullanarak aralık hesapla
            h_tolerance = self.params['H_TOLERANCE']
            s_tolerance = self.params['S_TOLERANCE']
            v_tolerance = self.params['V_TOLERANCE']
            
            self.params['H_MIN'] = max(0, h - h_tolerance)
            self.params['H_MAX'] = min(179, h + h_tolerance)
            self.params['S_MIN'] = max(0, s - s_tolerance)
            self.params['S_MAX'] = min(255, s + s_tolerance)
            self.params['V_MIN'] = max(0, v - v_tolerance)
            self.params['V_MAX'] = min(255, v + v_tolerance)
            
            print(f"Yeni Tespit Aralıkları:")
            print(f"  H: {self.params['H_MIN']}-{self.params['H_MAX']}")
            print(f"  S: {self.params['S_MIN']}-{self.params['S_MAX']}")
            print(f"  V: {self.params['V_MIN']}-{self.params['V_MAX']}")
            print("="*60 + "\n")
    
    def run(self):
        """Ana döngü - video akışını işle ve LED tespiti yap"""
        cap = cv2.VideoCapture(self.video_source)
        
        if not cap.isOpened():
            print(f"Hata: Video kaynağı açılamadı: {self.video_source}")
            return
        
        # Pencere oluştur
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)
        
        print("=" * 60)
        print("LED Tespit Uygulaması")
        print("- ÇİFT TIK: Renk seç ve tespit başlat")
        print("- 'Q' tuşu: Ayarları kaydet ve çık")
        print("- 'R' tuşu: Varsayılan ayarlara dön")
        print("=" * 60)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Kamera akışı okunamadı.")
                break
            
            # Frame'i yeniden boyutlandır
            frame = cv2.resize(frame, (640, 480))
            self.frame_current = frame.copy()
            
            # LED tespiti yap
            result = detect_leds(frame, self.params)
            
            # Sonuçları çıkar
            led_coordinates = result['led_coordinates']
            center_point = result['center_point']
            led_count = result['led_count']
            annotated_frame = result['annotated_frame']
            mask = result['mask']
            inpaint_mask = result['inpaint_mask']
            
            # İşaretlemeleri çiz
            annotated_frame = draw_led_annotations(annotated_frame, led_coordinates, center_point)
            
            # Koordinatları yazdır (sadece LED tespit edildiyse)
            if led_count > 0:
                print(f"\rTespit: {led_count} LED", end="", flush=True)
            
            # Görüntüleri göster
            cv2.imshow(self.window_name, annotated_frame)
            cv2.imshow("Maske", mask)
            cv2.imshow("Inpaint Maskesi", inpaint_mask)
            
            # Klavye kontrolü
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                # Ayarları kaydet
                save_settings({
                    "H_MIN": self.params['H_MIN'],
                    "H_MAX": self.params['H_MAX'],
                    "S_MIN": self.params['S_MIN'],
                    "S_MAX": self.params['S_MAX'],
                    "V_MIN": self.params['V_MIN'],
                    "V_MAX": self.params['V_MAX'],
                    "MIN_ALAN": self.params['MIN_ALAN'],
                    "MAX_ALAN": self.params['MAX_ALAN'],
                    "H_TOLERANCE": self.params['H_TOLERANCE'],
                    "S_TOLERANCE": self.params['S_TOLERANCE'],
                    "V_TOLERANCE": self.params['V_TOLERANCE'],
                })
                print("\n\nAyarlar kaydedildi!")
                break
            elif key == ord('r'):
                # Varsayılan ayarlara dön
                self.params = DEFAULT_PARAMS.copy()
                print("\nVarsayılan ayarlar yüklendi!")
        
        cap.release()
        cv2.destroyAllWindows()




# Ana program
if __name__ == "__main__":
    # Video kaynağı - ortam değişkeninden al veya varsayılan kullan
    video_source = os.getenv("STREAM_URL", "http://192.168.19.221:5000/video_roi")
    
    # Uygulamayı başlat
    app = LEDDetectorApp(video_source)
    app.run()

