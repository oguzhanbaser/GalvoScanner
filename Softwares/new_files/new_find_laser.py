import cv2
import numpy as np
import json
import os

# Ayarlar dosyası
SETTINGS_FILE = "laser_trackbar_settings.json"
DAIRESELLIK_ESIGI = 0.4  # Dairesellik eşiği (0-1 arası, 1 = tam daire)

# Renk toleransı (H için +/-, S ve V için +/-)
H_TOLERANCE = 10
S_TOLERANCE = 50
V_TOLERANCE = 50

# Global değişkenler
frame_original = None
clicked_hsv = (0, 0, 0)

def save_settings(settings):
    with open(SETTINGS_FILE, "w") as file:
        json.dump(settings, file)

def load_settings():
    if os.path.exists(SETTINGS_FILE):
        with open(SETTINGS_FILE, "r") as file:
            return json.load(file)
    return {}

# Mouse callback - sol tık ile renk seç ve trackbar'ları güncelle
def get_hsv_at_click(event, x, y, flags, param):
    global clicked_hsv
    
    # Çift tık - sadece bilgi göster
    if event == cv2.EVENT_LBUTTONDBLCLK:
        bgr_pixel = frame_original[y, x]
        hsv_pixel = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
        clicked_hsv = tuple(hsv_pixel)
        print("---------------------------------------------")
        print(f"Piksel Koordinat: X={x}, Y={y}")
        print(f"PIKSEL HSV DEĞERİ: H={clicked_hsv[0]}, S={clicked_hsv[1]}, V={clicked_hsv[2]}")
        print("---------------------------------------------")
    
    # Sol tık - renk seç ve trackbar'ları güncelle
    elif event == cv2.EVENT_LBUTTONDOWN:
        bgr_pixel = frame_original[y, x]
        hsv_pixel = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = int(hsv_pixel[0]), int(hsv_pixel[1]), int(hsv_pixel[2])
        
        print("=============================================")
        print(f"RENK SEÇİLDİ - Piksel: X={x}, Y={y}")
        print(f"HSV Değeri: H={h}, S={s}, V={v}")
        print("Trackbar'lar güncelleniyor...")
        
        # H değeri için özel işlem (kırmızı renk 0-10 ve 170-180 arasında)
        if h <= 10:
            # Alt kırmızı aralık
            h_min = max(0, h - H_TOLERANCE)
            h_max = min(10, h + H_TOLERANCE)
            h2_min = max(170, 180 - H_TOLERANCE)
            h2_max = 180
            print(f"Alt kırmızı aralık seçildi: H={h_min}-{h_max}, H2={h2_min}-{h2_max}")
        elif h >= 170:
            # Üst kırmızı aralık
            h_min = 0
            h_max = min(10, H_TOLERANCE)
            h2_min = max(170, h - H_TOLERANCE)
            h2_max = min(180, h + H_TOLERANCE)
            print(f"Üst kırmızı aralık seçildi: H={h_min}-{h_max}, H2={h2_min}-{h2_max}")
        else:
            # Normal renk aralığı
            h_min = max(0, h - H_TOLERANCE)
            h_max = min(179, h + H_TOLERANCE)  # H max 179
            h2_min = 170
            h2_max = 180
            print(f"Normal aralık seçildi: H={h_min}-{h_max}")
        
        # S ve V aralıkları - sınırları kontrol et
        s_min = max(0, s - S_TOLERANCE)
        s_max = min(255, s + S_TOLERANCE)
        v_min = max(0, v - V_TOLERANCE)
        v_max = min(255, v + V_TOLERANCE)
        
        # Trackbar'ları güncelle (int değerler olarak)
        cv2.setTrackbarPos('H Min', 'Ayarlar', int(h_min))
        cv2.setTrackbarPos('H Max', 'Ayarlar', int(h_max))
        cv2.setTrackbarPos('S Min', 'Ayarlar', int(s_min))
        cv2.setTrackbarPos('S Max', 'Ayarlar', int(s_max))
        cv2.setTrackbarPos('V Min', 'Ayarlar', int(v_min))
        cv2.setTrackbarPos('V Max', 'Ayarlar', int(v_max))
        cv2.setTrackbarPos('H2 Min', 'Ayarlar', int(h2_min))
        cv2.setTrackbarPos('H2 Max', 'Ayarlar', int(h2_max))
        
        print(f"Yeni aralıklar: H=[{h_min}-{h_max}], S=[{s_min}-{s_max}], V=[{v_min}-{v_max}]")
        print("=============================================")
        
        # Görüntüyü hemen yeniden işle
        process_image()

# Trackbar callback - her değişiklikte görüntüyü güncelle
def process_image(x=None):
    global frame_original
    
    if frame_original is None:
        return
    
    try:
        frame = frame_original.copy()
        
        # Trackbar değerlerini al
        h_min = cv2.getTrackbarPos('H Min', 'Ayarlar')
        h_max = cv2.getTrackbarPos('H Max', 'Ayarlar')
        s_min = cv2.getTrackbarPos('S Min', 'Ayarlar')
        s_max = cv2.getTrackbarPos('S Max', 'Ayarlar')
        v_min = cv2.getTrackbarPos('V Min', 'Ayarlar')
        v_max = cv2.getTrackbarPos('V Max', 'Ayarlar')
        h2_min = cv2.getTrackbarPos('H2 Min', 'Ayarlar')
        h2_max = cv2.getTrackbarPos('H2 Max', 'Ayarlar')
        min_area = cv2.getTrackbarPos('Min Alan', 'Ayarlar')
        max_area = cv2.getTrackbarPos('Max Alan', 'Ayarlar')
        parlaklik_esigi = cv2.getTrackbarPos('Parlaklik Esik', 'Ayarlar')
        dairesellik_esigi = cv2.getTrackbarPos('Dairesellik x100', 'Ayarlar') / 100.0
    except cv2.error:
        # Trackbar'lar henüz oluşturulmamışsa fonksiyondan çık
        return
    
    if min_area == 0:
        min_area = 1
    
    # PIXEL SUPPLEMENTATION (Inpainting)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask_parlak = cv2.threshold(gray, parlaklik_esigi, 255, cv2.THRESH_BINARY)[1]
    frame_inpainted = cv2.inpaint(frame, mask_parlak, inpaintRadius=2, flags=cv2.INPAINT_TELEA)
    
    # Median Blur uygula
    blurred = cv2.medianBlur(frame_inpainted, 5)
    
    # BGR'den HSV'ye dönüştür
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # Kırmızı renk için HSV aralıkları
    lower_red1 = np.array([h_min, s_min, v_min])
    upper_red1 = np.array([h_max, s_max, v_max])
    
    # Maskeler oluştur
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    
    # İkinci aralık sadece kırmızı renk için (H <= 10 veya H >= 170)
    # Diğer renkler için sadece mask1 kullan
    if (h_min <= 10 and h_max <= 10) or (h_min >= 170 or h_max >= 170):
        # Kırmızı renk tespit ediliyor - iki aralık birleştir
        lower_red2 = np.array([h2_min, s_min, v_min])
        upper_red2 = np.array([h2_max, s_max, v_max])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
    else:
        # Diğer renkler - sadece tek aralık kullan
        mask = mask1
    
    # Connected Components ile filtreleme
    num_labels, labels = cv2.connectedComponents(mask)
    filtered_mask = np.zeros_like(mask, dtype=np.uint8)
    
    for label in range(1, num_labels):
        region_mask = (labels == label).astype("uint8") * 255
        area = cv2.countNonZero(region_mask)
        if area >= min_area and area <= max_area:
            filtered_mask = cv2.bitwise_or(filtered_mask, region_mask)
    
    mask = filtered_mask
    
    # Konturları bul
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    laser_merkezleri = []
    
    # Konturları işle - alan ve dairesellik kontrolü
    for c in contours:
        area = cv2.contourArea(c)
        
        # Alan kontrolü
        if area < min_area or area > max_area:
            continue
        
        # Dairesellik kontrolü - çembere benziyor mu?
        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        
        dairesellik = (4 * np.pi * area) / (perimeter * perimeter)
        
        # Sadece çembere benzeyen alanları işaretle
        if dairesellik < dairesellik_esigi:
            continue
        
        # Merkez hesapla
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            laser_merkezleri.append((cx, cy))
            
            # Görselleştir - çember olarak çiz
            radius = int(np.sqrt(area / np.pi))
            cv2.circle(frame_inpainted, (cx, cy), radius, (0, 255, 0), 2)
            cv2.circle(frame_inpainted, (cx, cy), 3, (0, 0, 255), -1)
            cv2.putText(frame_inpainted, f'Laser {dairesellik:.2f}', (cx - 40, cy - radius - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Durum bilgisi
    durum = f"Laser Sayisi: {len(laser_merkezleri)}"
    renk = (0, 255, 0) if len(laser_merkezleri) > 0 else (0, 0, 255)
    cv2.putText(frame_inpainted, durum, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, renk, 2)
    
    # Sonuçları göster
    cv2.imshow('Orijinal', frame)
    cv2.imshow('İşlenmiş', frame_inpainted)
    cv2.imshow('Maske', mask)
    cv2.imshow('Inpaint Maskesi', mask_parlak)

# Kaydedilmiş ayarları yükle
settings = load_settings()

# Kamera bağlantısını aç
STREAM_URL = os.getenv("STREAM_URL", "http://192.168.19.221:5000/video_roi")
cap = cv2.VideoCapture(STREAM_URL)

if not cap.isOpened():
    print(f"Kamera açılamadı: {STREAM_URL}")
    exit()

print(f"Kamera bağlantısı kuruldu: {STREAM_URL}")

# Trackbar penceresi oluştur
cv2.namedWindow('Ayarlar')
cv2.namedWindow('Orijinal')

# Mouse callback ayarla
cv2.setMouseCallback('Orijinal', get_hsv_at_click)

# Kullanım talimatı yazdır
print("=" * 60)
print("KULLANIM TALİMATI:")
print("- SOL TIK: Renk seç ve trackbar'ları otomatik ayarla")
print("- ÇİFT TIK: Sadece HSV değerlerini göster")
print("- 'Q' tuşu: Ayarları kaydet ve çık")
print("=" * 60)

# Boş callback fonksiyonu - trackbar oluşturma için
def nothing(x):
    process_image()

# HSV trackbar'ları oluştur (nothing callback ile) - H max değeri 179 olmalı
cv2.createTrackbar('H Min', 'Ayarlar', min(settings.get('H_MIN', 0), 179), 179, nothing)
cv2.createTrackbar('H Max', 'Ayarlar', min(settings.get('H_MAX', 10), 179), 179, nothing)
cv2.createTrackbar('S Min', 'Ayarlar', min(settings.get('S_MIN', 120), 255), 255, nothing)
cv2.createTrackbar('S Max', 'Ayarlar', min(settings.get('S_MAX', 255), 255), 255, nothing)
cv2.createTrackbar('V Min', 'Ayarlar', min(settings.get('V_MIN', 70), 255), 255, nothing)
cv2.createTrackbar('V Max', 'Ayarlar', min(settings.get('V_MAX', 255), 255), 255, nothing)

# İkinci kırmızı aralık için
cv2.createTrackbar('H2 Min', 'Ayarlar', min(settings.get('H2_MIN', 170), 179), 179, nothing)
cv2.createTrackbar('H2 Max', 'Ayarlar', min(settings.get('H2_MAX', 180), 179), 179, nothing)

# Diğer parametreler
cv2.createTrackbar('Min Alan', 'Ayarlar', settings.get('MIN_ALAN', 5), 500, nothing)
cv2.createTrackbar('Max Alan', 'Ayarlar', settings.get('MAX_ALAN', 5000), 10000, nothing)
cv2.createTrackbar('Parlaklik Esik', 'Ayarlar', settings.get('PARLAKLIK_ESIK', 225), 255, nothing)
cv2.createTrackbar('Dairesellik x100', 'Ayarlar', settings.get('DAIRESELLIK_ESIGI', 40), 100, nothing)

# Ana döngü - kamera görüntüsünü işle
while True:
    # Kameradan frame oku
    ret, frame = cap.read()
    if not ret:
        print("Kamera görüntüsü okunamadı.")
        break
    
    # Frame'i yeniden boyutlandır (opsiyonel)
    # frame = cv2.resize(frame, (640, 480))
    frame_original = frame.copy()
    
    # Görüntüyü işle
    process_image()
    
    # Klavye kontrolü
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('q'):
        # Ayarları kaydet
        h_min = cv2.getTrackbarPos('H Min', 'Ayarlar')
        h_max = cv2.getTrackbarPos('H Max', 'Ayarlar')
        s_min = cv2.getTrackbarPos('S Min', 'Ayarlar')
        s_max = cv2.getTrackbarPos('S Max', 'Ayarlar')
        v_min = cv2.getTrackbarPos('V Min', 'Ayarlar')
        v_max = cv2.getTrackbarPos('V Max', 'Ayarlar')
        h2_min = cv2.getTrackbarPos('H2 Min', 'Ayarlar')
        h2_max = cv2.getTrackbarPos('H2 Max', 'Ayarlar')
        min_area = cv2.getTrackbarPos('Min Alan', 'Ayarlar')
        max_area = cv2.getTrackbarPos('Max Alan', 'Ayarlar')
        parlaklik_esigi = cv2.getTrackbarPos('Parlaklik Esik', 'Ayarlar')
        dairesellik_esigi = cv2.getTrackbarPos('Dairesellik x100', 'Ayarlar')
        
        settings = {
            "H_MIN": h_min, "H_MAX": h_max,
            "S_MIN": s_min, "S_MAX": s_max,
            "V_MIN": v_min, "V_MAX": v_max,
            "H2_MIN": h2_min, "H2_MAX": h2_max,
            "MIN_ALAN": min_area, "MAX_ALAN": max_area,
            "PARLAKLIK_ESIK": parlaklik_esigi,
            "DAIRESELLIK_ESIGI": dairesellik_esigi
        }
        save_settings(settings)
        break

# Kamerayı kapat
cap.release()
cv2.destroyAllWindows()
