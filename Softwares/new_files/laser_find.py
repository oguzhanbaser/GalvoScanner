import cv2
import numpy as np

# Boş callback fonksiyonu
def nothing(x):
    pass

# Kamerayı başlat
cap = cv2.VideoCapture("http://192.168.19.221:5000/video_roi")

# Trackbar penceresi oluştur
cv2.namedWindow('Ayarlar')

# HSV trackbar'ları oluştur
cv2.createTrackbar('H Min', 'Ayarlar', 0, 180, nothing)
cv2.createTrackbar('H Max', 'Ayarlar', 10, 180, nothing)
cv2.createTrackbar('S Min', 'Ayarlar', 120, 255, nothing)
cv2.createTrackbar('S Max', 'Ayarlar', 255, 255, nothing)
cv2.createTrackbar('V Min', 'Ayarlar', 70, 255, nothing)
cv2.createTrackbar('V Max', 'Ayarlar', 255, 255, nothing)

# İkinci kırmızı aralık için
cv2.createTrackbar('H2 Min', 'Ayarlar', 170, 180, nothing)
cv2.createTrackbar('H2 Max', 'Ayarlar', 180, 180, nothing)

# Diğer parametreler
cv2.createTrackbar('Min Alan', 'Ayarlar', 500, 5000, nothing)
cv2.createTrackbar('Dairesellik x100', 'Ayarlar', 70, 100, nothing)

while True:
    # Kare yakala
    ret, frame = cap.read()
    if not ret:
        break
    
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
    circularity_threshold = cv2.getTrackbarPos('Dairesellik x100', 'Ayarlar') / 100.0
    
    # BGR'den HSV'ye dönüştür
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Kırmızı renk için HSV aralıkları (trackbar değerleri ile)
    lower_red1 = np.array([h_min, s_min, v_min])
    upper_red1 = np.array([h_max, s_max, v_max])
    lower_red2 = np.array([h2_min, s_min, v_min])
    upper_red2 = np.array([h2_max, s_max, v_max])
    
    # Maskeler oluştur
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    
    # Gürültüyü azalt
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Konturları bul
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Konturları çiz
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            # Dairesellik hesapla
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            # Sadece yuvarlak şekilleri işaretle
            if circularity > circularity_threshold:
                x, y, w, h = cv2.boundingRect(contour)
                # Merkezini bul
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    # Daire çiz
                    radius = int(np.sqrt(area / np.pi))
                    cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)  # Merkez noktası
                    cv2.putText(frame, f'Daire {circularity:.2f}', (cx - 40, cy - radius - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Sonuçları göster
    cv2.imshow('Orijinal', frame)
    cv2.imshow('Maske', mask)
    
    # 'q' tuşuna basılırsa çık
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Temizlik
cap.release()
cv2.destroyAllWindows()
