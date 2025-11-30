import cv2
import numpy as np
import math
import json
import os


H_MIN = 114
H_MAX = 154
S_MIN = 60
S_MAX = 255
V_MIN = 92
V_MAX = 250

MIN_ALAN_BASLANGIC = 5
MAX_ALAN_BASLANGIC = 5000

DAIRESELLIK_ESIGI = 0.4 


ERODE_ITERATIONS = 2
DILATE_ITERATIONS = 2
KERNEL = np.ones((5,5), np.uint8)


clicked_hsv = (0, 0, 0)
PENCERE_ADI = "1. Canli Akis ve Tespit"
SETTINGS_FILE = "trackbar_settings.json"


MAX_ALAN_FAKTOR_UST = 2.0 



def save_settings(settings):
    with open(SETTINGS_FILE, "w") as file:
        json.dump(settings, file)

def load_settings():
    if os.path.exists(SETTINGS_FILE):
        with open(SETTINGS_FILE, "r") as file:
            return json.load(file)
    return {}

def empty(a):
    pass

def get_hsv_at_click(event, x, y, flags, param):
    global clicked_hsv
    if event == cv2.EVENT_LBUTTONDBLCLK:
        bgr_pixel = param['frame'][y, x]
        hsv_pixel = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
        clicked_hsv = tuple(hsv_pixel)
        print("---------------------------------------------")
        print(f"Piksel Koordinat: X={x}, Y={y}")
        print(f"PIKSEL HSV DEĞERİ: H={clicked_hsv[0]}, S={clicked_hsv[1]}, V={clicked_hsv[2]}")
        print("---------------------------------------------")

cv2.namedWindow("Trackbar Ayarlari")
cv2.resizeWindow("Trackbar Ayarlari", 640, 300)
cv2.namedWindow(PENCERE_ADI)

cv2.createTrackbar("H Min", "Trackbar Ayarlari", H_MIN, 179, empty)
cv2.createTrackbar("H Max", "Trackbar Ayarlari", H_MAX, 179, empty)
cv2.createTrackbar("S Min", "Trackbar Ayarlari", S_MIN, 255, empty)
cv2.createTrackbar("S Max", "Trackbar Ayarlari", S_MAX, 255, empty)
cv2.createTrackbar("V Min", "Trackbar Ayarlari", V_MIN, 255, empty)
cv2.createTrackbar("V Max", "Trackbar Ayarlari", V_MAX, 255, empty)
cv2.createTrackbar("Min Alan", "Trackbar Ayarlari", MIN_ALAN_BASLANGIC, 500, empty)
cv2.createTrackbar("Max Alan", "Trackbar Ayarlari", MAX_ALAN_BASLANGIC, 10000, empty)

settings = load_settings()
H_MIN = settings.get("H_MIN", H_MIN)
H_MAX = settings.get("H_MAX", H_MAX)
S_MIN = settings.get("S_MIN", S_MIN)
S_MAX = settings.get("S_MAX", S_MAX)
V_MIN = settings.get("V_MIN", V_MIN)
V_MAX = settings.get("V_MAX", V_MAX)
MIN_ALAN_BASLANGIC = settings.get("MIN_ALAN_BASLANGIC", MIN_ALAN_BASLANGIC)
MAX_ALAN_BASLANGIC = settings.get("MAX_ALAN_BASLANGIC", MAX_ALAN_BASLANGIC)
STREAM_URL = os.getenv("STREAM_URL", "http://192.168.19.221:5000/video_roi")  # Varsayılan olarak 0 (webcam)
cap = cv2.VideoCapture(STREAM_URL)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Kamera akişi okunamadi.")
        break
    
    h, w, _ = frame.shape
    yarim_yukseklik = h // 2

    # frame = cv2.flip(frame, 1)
    cv2.setMouseCallback(PENCERE_ADI, get_hsv_at_click, {'frame': frame})
    frame = cv2.resize(frame, (640, 480))
    h_min = cv2.getTrackbarPos("H Min", "Trackbar Ayarlari")
    h_max = cv2.getTrackbarPos("H Max", "Trackbar Ayarlari")
    s_min = cv2.getTrackbarPos("S Min", "Trackbar Ayarlari")
    s_max = cv2.getTrackbarPos("S Max", "Trackbar Ayarlari")
    v_min = cv2.getTrackbarPos("V Min", "Trackbar Ayarlari")
    v_max = cv2.getTrackbarPos("V Max", "Trackbar Ayarlari")
    MIN_LED_ALAN = cv2.getTrackbarPos("Min Alan", "Trackbar Ayarlari")
    MAX_LED_ALAN = cv2.getTrackbarPos("Max Alan", "Trackbar Ayarlari")

    if MIN_LED_ALAN == 0:
        MIN_LED_ALAN = 1

    BLUE_LOWER = (h_min, s_min, v_min)
    BLUE_UPPER = (h_max, s_max, v_max)

    # PIXEL SUPPLEMENTATION (Inpainting) 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    parlaklik_esigi = 225 
    mask_parlak = cv2.threshold(gray, parlaklik_esigi, 255, cv2.THRESH_BINARY)[1] 

    frame_inpainted = cv2.inpaint(frame, mask_parlak, inpaintRadius=2, flags=cv2.INPAINT_TELEA)
    
    blurred = cv2.medianBlur(frame_inpainted, 5)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    
    # Debugging: Print HSV range values to ensure trackbars are updating
    print(f"HSV Range: H_MIN={h_min}, H_MAX={h_max}, S_MIN={s_min}, S_MAX={s_max}, V_MIN={v_min}, V_MAX={v_max}")

    # Temporarily disable erosion and dilation for debugging
    # mask = cv2.erode(mask, KERNEL, iterations=ERODE_ITERATIONS)
    # mask = cv2.dilate(mask, KERNEL, iterations=DILATE_ITERATIONS)

    num_labels, labels = cv2.connectedComponents(mask)
    filtered_mask = np.zeros_like(mask, dtype=np.uint8)

    for label in range(1, num_labels):
        region_mask = (labels == label).astype("uint8") * 255
        area = cv2.countNonZero(region_mask)
        if area >= MIN_LED_ALAN and area <= MAX_LED_ALAN:
            filtered_mask = cv2.bitwise_or(filtered_mask, region_mask)

    mask = filtered_mask
    
   
    contours, _ = cv2.findContours(mask_parlak.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    parlak_led_merkezleri = []

    # Optimize alan ve dairesellik kontrolleri
    for c in contours:
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c, True)
        dairesellik = (4 * np.pi * area) / (perimeter**2) if perimeter > 0 else 0

        # Küçük parazitleri dışla
        if area < MIN_LED_ALAN or area > MAX_LED_ALAN or dairesellik < DAIRESELLIK_ESIGI:
            continue

        # Ortalama HSV kontrolü
        mask_c = np.zeros_like(mask)
        cv2.drawContours(mask_c, [c], -1, 255, -1)
        mean_h = cv2.mean(hsv[:, :, 0], mask=mask_c)[0]
        mean_s = cv2.mean(hsv[:, :, 1], mask=mask_c)[0]
        mean_v = cv2.mean(hsv[:, :, 2], mask=mask_c)[0]
        if mean_s < 100 or mean_v < 50:
            continue

        # Geçerli LED merkezini hesapla
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            parlak_led_merkezleri.append((cX, cY))

    led_sayisi_ham = len(parlak_led_merkezleri)
    print("\nTespit Edilen LED Koordinatlari (X, Y)")
    
    parlak_led_merkezleri.sort(key=lambda item: item[0]) 

    parlak_led_merkezleri = parlak_led_merkezleri[:4] 

    led_sayisi = len(parlak_led_merkezleri) 

    if led_sayisi > 0:
        
        nokta_koordinatlari = []
        etiket_renkleri = [(0, 0, 255), (0, 0, 0), (128, 0, 128), (255, 0, 0)] 

        
        for i, (cX, cY) in enumerate(parlak_led_merkezleri):
            led_no = i + 1
            etiket_renk = etiket_renkleri[i % len(etiket_renkleri)]
            nokta_koordinatlari.append((cX, cY))
            
            b, g, r = frame_inpainted[cY, cX]
            parlaklik_toplami = r + g + b
            metin_renk = (0, 0, 0) if parlaklik_toplami > 350 else (255, 255, 255)
            
            print(f"LED {led_no}: X={cX}, Y={cY}")
            cv2.putText(frame_inpainted, f"LED {led_no}", (cX - 30, cY - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, metin_renk, 2)
            cv2.circle(frame_inpainted, (cX, cY), 5, etiket_renk, -1)

        
        if len(nokta_koordinatlari) >= 2:
            for i in range(len(nokta_koordinatlari) - 1):
                cv2.line(frame_inpainted, nokta_koordinatlari[i], nokta_koordinatlari[i+1], (0, 255, 255), 2)
            if len(nokta_koordinatlari) == 4:
                cv2.line(frame_inpainted, nokta_koordinatlari[3], nokta_koordinatlari[0], (0, 255, 255), 2)

        
        toplam_x = sum([x for x, y in parlak_led_merkezleri])
        toplam_y = sum([y for x, y in parlak_led_merkezleri])
        orta_nokta = (int(toplam_x / led_sayisi), int(toplam_y / led_sayisi))
        cv2.circle(frame_inpainted, orta_nokta, 5, (255, 255, 0), -1)
        cv2.putText(frame_inpainted, "Orta Nokta", (orta_nokta[0] + 10, orta_nokta[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        print(f"Orta Nokta: X={orta_nokta[0]}, Y={orta_nokta[1]}")
        print("---------------------------------------------")

    
    durum = "4 LED TESPIT EDILDI!" if led_sayisi == 4 else f"LED Sayisi: {led_sayisi}"
    renk = (0, 255, 0) if led_sayisi == 4 else (0, 0, 255)
    cv2.putText(frame_inpainted, durum, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, renk, 2)


    cv2.imshow(PENCERE_ADI, frame_inpainted)
    cv2.imshow("2. Maske (Renk Izolasyonu + Filtreleme)", mask)
    cv2.imshow("3. Inpaint Maskesi", mask_parlak)  

    if cv2.waitKey(1) & 0xFF == ord('q'):
        settings = {
            "H_MIN": h_min, "H_MAX": h_max,
            "S_MIN": s_min, "S_MAX": s_max,
            "V_MIN": v_min, "V_MAX": v_max,
            "MIN_ALAN_BASLANGIC": MIN_LED_ALAN,
            "MAX_ALAN_BASLANGIC": MAX_LED_ALAN,
        }
        save_settings(settings)
        break

cap.release()
cv2.destroyAllWindows()