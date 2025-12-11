import cv2
import numpy as np
import json
import os


class MyDetector:
    """
    Laser noktası tespit eden sınıf.
    HSV renk uzayında çalışır ve tıklama ile renk seçimi yapar.
    """
    
    def __init__(self, laser_settings_file="laser_trackbar_settings.json", led_settings_file="led_settings.json"):
        """
        MyDetector sınıfını başlatır.
        
        Args:
            laser_settings_file: Laser ayarlarının kaydedileceği JSON dosyası
            led_settings_file: LED ayarlarının kaydedileceği JSON dosyası
        """
        # Sabitler
        self.LASER_SETTINGS_FILE = laser_settings_file
        self.LED_SETTINGS_FILE = led_settings_file
        self.H_TOLERANCE = 15
        self.S_TOLERANCE = 80
        self.V_TOLERANCE = 80
        
        # Değişkenler
        self.frame_original = None
        self.clicked_hsv = (0, 0, 0)
        self.settings = {}
        self.cap = None
        self.frame_inpainted = None
        
        # Tespit parametreleri - tıklama ile güncellenecek
        self.h_min = 0
        self.h_max = 10
        self.s_min = 120
        self.s_max = 255
        self.v_min = 70
        self.v_max = 255
        self.h2_min = 170
        self.h2_max = 180
        self.min_area = 5
        self.max_area = 5000
        self.parlaklik_esigi = 225
        self.dairesellik_esigi = 0.4
        
        # LED tespit parametreleri
        self.led_params = {
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
        
        # Laser ayarlarını yükle
        self.settings = self.load_settings()
        if self.settings:
            self.h_min = self.settings.get('H_MIN', 0)
            self.h_max = self.settings.get('H_MAX', 10)
            self.s_min = self.settings.get('S_MIN', 120)
            self.s_max = self.settings.get('S_MAX', 255)
            self.v_min = self.settings.get('V_MIN', 70)
            self.v_max = self.settings.get('V_MAX', 255)
            self.h2_min = self.settings.get('H2_MIN', 170)
            self.h2_max = self.settings.get('H2_MAX', 180)
            self.min_area = self.settings.get('MIN_ALAN', 5)
            self.max_area = self.settings.get('MAX_ALAN', 5000)
            self.parlaklik_esigi = self.settings.get('PARLAKLIK_ESIK', 225)
            self.dairesellik_esigi = self.settings.get('DAIRESELLIK_ESIGI', 40) / 100.0
        
        # LED ayarlarını yükle
        self.load_led_settings()
        
    
    def save_settings(self, settings=None):
        """Ayarları JSON dosyasına kaydeder."""
        if settings is None:
            settings = self.settings
        with open(self.SETTINGS_FILE, "w") as file:
            json.dump(settings, file)
    
    def load_settings(self):
        """Laser ayarlarını JSON dosyasından yükler."""
        if os.path.exists(self.LASER_SETTINGS_FILE):
            with open(self.LASER_SETTINGS_FILE, "r") as file:
                return json.load(file)
        return {}
    
    def set_led_config_path(self, config_path):
        """LED konfigürasyon dosyasının yolunu ayarlar ve ayarları yükler.
        
        Args:
            config_path: LED ayarları JSON dosyasının yolu
        """
        self.LED_SETTINGS_FILE = config_path
        self.load_led_settings()
        print(f"LED konfigürasyon dosyası güncellendi: {config_path}")
    
    def load_led_settings(self):
        """LED ayarlarını JSON dosyasından yükler."""
        if os.path.exists(self.LED_SETTINGS_FILE):
            with open(self.LED_SETTINGS_FILE, "r") as file:
                loaded = json.load(file)
                self.led_params.update(loaded)
                print(f"LED ayarları yüklendi: {self.LED_SETTINGS_FILE}")
    
    def save_led_settings(self):
        """LED ayarlarını JSON dosyasına kaydeder."""
        with open(self.LED_SETTINGS_FILE, "w") as file:
            json.dump(self.led_params, file, indent=2)
        print(f"LED ayarları kaydedildi: {self.LED_SETTINGS_FILE}")
    
    def get_hsv_at_click(self, event, x, y, flags, param):
        """
        Mouse callback fonksiyonu.
        Sol tık: Renk seç ve parametreleri güncelle
        Çift tık: HSV değerlerini göster
        """
        if self.frame_original is None:
            return
        
        # Çift tık - sadece bilgi göster
        if event == cv2.EVENT_LBUTTONDBLCLK:
            bgr_pixel = self.frame_original[y, x]
            b, g, r = int(bgr_pixel[0]), int(bgr_pixel[1]), int(bgr_pixel[2])
            hsv_pixel = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
            h, s, v = int(hsv_pixel[0]), int(hsv_pixel[1]), int(hsv_pixel[2])
            self.clicked_hsv = (h, s, v)
            print("\n" + "-"*60)
            print(f"BİLGİ - Piksel Koordinat: X={x}, Y={y}")
            print(f"RGB: R={r}, G={g}, B={b}")
            print(f"HSV: H={h}, S={s}, V={v}")
            print("-"*60 + "\n")
        
        # Sol tık - renk seç ve parametreleri güncelle
        elif event == cv2.EVENT_LBUTTONDOWN:
            bgr_pixel = self.frame_original[y, x]
            b, g, r = int(bgr_pixel[0]), int(bgr_pixel[1]), int(bgr_pixel[2])
            hsv_pixel = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
            h, s, v = int(hsv_pixel[0]), int(hsv_pixel[1]), int(hsv_pixel[2])
            
            print("\n" + "="*60)
            print(f"RENK SEÇİLDİ - Piksel Koordinat: X={x}, Y={y}")
            print("-" * 60)
            print(f"RGB Değeri: R={r}, G={g}, B={b}")
            print(f"HSV Değeri: H={h}, S={s}, V={v}")
            print("-" * 60)
            print("Renk parametreleri güncelleniyor...")
            
            # H değeri için özel işlem (kırmızı renk 0-10 ve 170-180 arasında)
            if h <= 10:
                # Alt kırmızı aralık
                self.h_min = int(max(0, h - self.H_TOLERANCE))
                self.h_max = int(min(10, h + self.H_TOLERANCE))
                self.h2_min = int(max(170, 180 - self.H_TOLERANCE))
                self.h2_max = 180
                print(f"Alt kırmızı aralık seçildi:")
                print(f"  H Aralık 1: {self.h_min}-{self.h_max}")
                print(f"  H Aralık 2: {self.h2_min}-{self.h2_max}")
            elif h >= 170:
                # Üst kırmızı aralık
                self.h_min = 0
                self.h_max = int(min(10, self.H_TOLERANCE))
                self.h2_min = int(max(170, h - self.H_TOLERANCE))
                self.h2_max = int(min(180, h + self.H_TOLERANCE))
                print(f"Üst kırmızı aralık seçildi:")
                print(f"  H Aralık 1: {self.h_min}-{self.h_max}")
                print(f"  H Aralık 2: {self.h2_min}-{self.h2_max}")
            else:
                # Normal renk aralığı
                self.h_min = int(max(0, h - self.H_TOLERANCE))
                self.h_max = int(min(180, h + self.H_TOLERANCE))
                self.h2_min = 170
                self.h2_max = 180
                print(f"Normal renk aralığı seçildi:")
                print(f"  H Aralık: {self.h_min}-{self.h_max}")
            
            # S ve V aralıkları
            self.s_min = int(max(0, s - self.S_TOLERANCE))
            self.s_max = int(min(255, s + self.S_TOLERANCE))
            self.v_min = int(max(0, v - self.V_TOLERANCE))
            self.v_max = int(min(255, v + self.V_TOLERANCE))
            
            print(f"  S Aralık: {self.s_min}-{self.s_max}")
            print(f"  V Aralık: {self.v_min}-{self.v_max}")
            print("="*60 + "\n")
            
            # Görüntüyü yeniden işle
            self.process_image()
    
    def process_image(self, pFrame):
        """
        Görüntüyü işler ve laser noktalarını tespit eder.
        """
        if pFrame is None:
            return None
        
        frame = pFrame.copy()

        cx = None
        cy = None
        
        # Sınıf değişkenlerinden parametreleri al
        h_min = self.h_min
        h_max = self.h_max
        s_min = self.s_min
        s_max = self.s_max
        v_min = self.v_min
        v_max = self.v_max
        h2_min = self.h2_min
        h2_max = self.h2_max
        min_area = self.min_area
        max_area = self.max_area
        parlaklik_esigi = self.parlaklik_esigi
        dairesellik_esigi = self.dairesellik_esigi
        
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
        lower_red2 = np.array([h2_min, s_min, v_min])
        upper_red2 = np.array([h2_max, s_max, v_max])
        
        # Maskeler oluştur
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
        
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
        
        # En büyük ve en dairesel noktayı bul
        best_contour = None
        best_score = -1
        best_area = 0
        best_circularity = 0
        
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
            
            # En iyi skoru hesapla (alan * dairesellik)
            # Hem büyük hem de dairesel olanı seç
            score = area * dairesellik
            
            if score > best_score:
                best_score = score
                best_contour = c
                best_area = area
                best_circularity = dairesellik
        
        laser_merkezleri = []
        
        # En iyi kontur varsa işaretle
        if best_contour is not None:
            # Merkez hesapla
            M = cv2.moments(best_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                laser_merkezleri.append((cx, cy))
                
                # Görselleştir - çember olarak çiz
                radius = int(np.sqrt(best_area / np.pi))
                cv2.circle(frame_inpainted, (cx, cy), radius, (0, 255, 0), 2)
                cv2.circle(frame_inpainted, (cx, cy), 3, (0, 0, 255), -1)
                cv2.putText(frame_inpainted, f'Laser {best_circularity:.2f}', (cx - 40, cy - radius - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            cx, cy = -1, -1
            return None
        
        # Durum bilgisi
        durum = f"Laser Sayisi: {len(laser_merkezleri)}"
        renk = (0, 255, 0) if len(laser_merkezleri) > 0 else (0, 0, 255)
        cv2.putText(frame_inpainted, durum, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, renk, 2)
        
        # # Sonuçları göster
        # cv2.imshow('Orijinal', self.frame_inpainted)
        # cv2.imshow('Maske', mask)
        # cv2.imshow('Inpaint Maskesi', mask_parlak)

        # return cx, cy
        return {
            'center_point': [cx, cy],
            'annotated_frame': frame_inpainted,
            'mask': mask
        }
    
    def detect_leds(self, frame):
        """
        Frame üzerinde LED tespiti yapar (tespit_led_new.py'den uyarlanmış).
        
        Args:
            frame: İşlenecek görüntü frame'i (BGR formatında)
            
        Returns:
            dict: {
                'led_coordinates': [(x1, y1), (x2, y2), ...],
                'center_point': (x, y),
                'led_count': int,
                'annotated_frame': frame,
                'mask': mask,
                'inpaint_mask': inpaint_mask
            }
        """
        # Frame kopyası al
        frame_processed = frame.copy()
        
        # Parametreleri çıkar
        h_min = self.led_params['H_MIN']
        h_max = self.led_params['H_MAX']
        s_min = self.led_params['S_MIN']
        s_max = self.led_params['S_MAX']
        v_min = self.led_params['V_MIN']
        v_max = self.led_params['V_MAX']
        min_alan = max(1, self.led_params['MIN_ALAN'])
        max_alan = self.led_params['MAX_ALAN']
        parlaklik_esigi = self.led_params['PARLAKLIK_ESIGI']
        dairesellik_esigi = self.led_params['DAIRESELLIK_ESIGI']
        max_led_sayisi = self.led_params['MAX_LED_SAYISI']
        
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

        # Konturları bul - PARLALIK MASKESİ ÜZERİNDEN (tespit_led.py ile aynı)
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
            
            # Ortalama HSV kontrolü - tespit_led.py ile aynı
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
    
    def run(self, source_type='image'):
        """
        Ana döngüyü başlatır.
        
        Args:
            source_type: 'image' veya 'video' - işlenecek kaynak tipi
        """
        if source_type == 'image' and self.frame_original is None:
            raise ValueError("Önce bir görüntü yüklemelisiniz.")
        
        if source_type == 'video' and self.cap is None:
            raise ValueError("Önce bir video kaynağı açmalısınız.")
        
        # Pencere oluştur
        cv2.namedWindow('Orijinal')
        
        # Mouse callback ayarla
        cv2.setMouseCallback('Orijinal', self.get_hsv_at_click)
        
        # Kullanım talimatı yazdır
        print("=" * 60)
        print("KULLANIM TALİMATI:")
        print("- SOL TIK: Renk seç (otomatik tespit başlar)")
        print("- ÇİFT TIK: Sadece HSV değerlerini göster")
        print("- 'Q' tuşu: Ayarları kaydet ve çık")
        print("=" * 60)
        
        # İlk görüntüyü işle (sadece resim modu için)
        if source_type == 'image':
            self.process_image()
        
        # Ana döngü
        while True:
            # Video modu için frame oku
            if source_type == 'video':
                ret, frame = self.cap.read()
                if not ret:
                    print("Video sona erdi veya okunamadı.")
                    break
                self.frame_original = frame
                self.process_image()
            
            # Klavye kontrolü
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                # Ayarları kaydet
                self.settings = {
                    "H_MIN": int(self.h_min), 
                    "H_MAX": int(self.h_max),
                    "S_MIN": int(self.s_min), 
                    "S_MAX": int(self.s_max),
                    "V_MIN": int(self.v_min), 
                    "V_MAX": int(self.v_max),
                    "H2_MIN": int(self.h2_min), 
                    "H2_MAX": int(self.h2_max),
                    "MIN_ALAN": int(self.min_area), 
                    "MAX_ALAN": int(self.max_area),
                    "PARLAKLIK_ESIK": int(self.parlaklik_esigi),
                    "DAIRESELLIK_ESIGI": int(self.dairesellik_esigi * 100)
                }
                self.save_settings(self.settings)
                print("Ayarlar kaydedildi!")
                break
        
        # Video kaynağını kapat
        if source_type == 'video':
            self.close_video()
        
        cv2.destroyAllWindows()


# Ana program - geriye dönük uyumluluk için
# if __name__ == "__main__":
#     # Kullanım örnekleri:
    
#     # ÖRNEK 1: Resim ile kullanım
#     # image_path = 'C:\\Users\\baser-huawei\\Documents\\GitHub\\GalvoScanner\\Softwares\\new_files\\image.jpg'
#     # try:
#     #     detector = MyDetector(image_path=image_path)
#     #     detector.run(source_type='image')
#     # except ValueError as e:
#     #     print(f"Hata: {e}")
    
#     # ÖRNEK 2: Video dosyası ile kullanım (yorumlu)
#     # video_path = 'path/to/video.mp4'
#     # try:
#     #     detector = MyDetector(video_source=video_path)
#     #     detector.run(source_type='video')
#     # except ValueError as e:
#     #     print(f"Hata: {e}")
    
#     # ÖRNEK 3: Kamera ile kullanım (yorumlu)
#     try:
#         detector = MyDetector(video_source="http://192.168.19.221:5000/video_roi")  # 0 = varsayılan kamera
#         # detector.run(source_type='video')

#         while True:
#             ret, frame = detector.cap.read()
#             if not ret:
#                 print("Video sona erdi veya okunamadı.")
#                 break
#             detector.frame_original = frame
#             [cx, cy] = detector.process_image()
#             print(f"Laser Koordinatları: X={cx}, Y={cy}")
#             cv2.imshow('Orijinal', detector.frame_inpainted)
#             cv2.waitKey(10)
#     except ValueError as e:
#         print(f"Hata: {e}")
