import cv2
import numpy as np

def process_color_beacon_image(img_hsv, color_range_hsv, min_area=3):
    """
    Belirli bir renk aralığında beacon ışığı tespiti:
    1) HSV eşikleme
    2) Labeling
    3) Pixel supplementation (kapanma)
    4) Gürültü temizleme
    """

    
    # 2. Belirli renk aralığında eşikleme (örneğin kırmızı için)
    lower_hsv, upper_hsv = color_range_hsv
    mask = cv2.inRange(img_hsv, lower_hsv, upper_hsv)
    # cv2.imwrite(f"{output_prefix}_color_mask.png", mask)
    cv2.imshow("Mask", mask)

    # 3. Labeling görselleştir (Şekil 6 benzeri)
    num_labels, labels = cv2.connectedComponents(mask)
    label_hue = np.uint8(179 * labels / np.max(labels))
    blank_ch = 255 * np.ones_like(label_hue)
    labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])
    labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)
    labeled_img[label_hue == 0] = 0
    # cv2.imwrite(f"{output_prefix}_after_labeling.png", labeled_img)
    cv2.imshow("Labeled Image", labeled_img)

    # 4. Pixel supplementation (Şekil 7)
    kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (3,1))
    kernel_v = cv2.getStructuringElement(cv2.MORPH_RECT, (1,3))
    supplemented = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_h)
    supplemented = cv2.morphologyEx(supplemented, cv2.MORPH_CLOSE, kernel_v)
    # cv2.imwrite(f"{output_prefix}_after_supplementation.png", supplemented)
    cv2.imshow("Supplemented Image", supplemented)

    # 5. Gürültü temizleme (Şekil 8)
    num2, labels2, stats, _ = cv2.connectedComponentsWithStats(supplemented)
    clean = np.zeros_like(supplemented)
    for i in range(1, num2):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= min_area:
            clean[labels2 == i] = 255
    # cv2.imwrite(f"{output_prefix}_after_noise_removal.png", clean)
    cv2.imshow("Cleaned Image", clean)
    

    print("Renkli işleme tamamlandı.")

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def mouseRGB_callback(event,x,y,flags,param):
    global red_lower1, red_upper1

    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        hVal = img_hsv[y,x,0]
        sVal = img_hsv[y,x,1]
        gVal = img_hsv[y,x,2]
        
        hMax = hVal + 20
        hLow = hVal - 20
        sMax = sVal + 20
        sLow = sVal - 20
        vMax = gVal + 20
        vLow = gVal - 20

        hMax = clamp(hMax, 0, 255)
        hLow = clamp(hLow, 0, 255)
        sMax = clamp(sMax, 0, 255)
        sLow = clamp(sLow, 0, 255)
        vMax = clamp(vMax, 0, 255)
        vLow = clamp(vLow, 0, 255)

        red_lower1 = np.array([hLow, sLow, vLow])
        red_upper1 = np.array([hMax, sMax, vMax])

        print(hMax, hLow, sMax, sLow, vMax, vLow)

        print("X: {0}, Y: {1}, H: {2}, S: {3}, V: {4}".format(x, y, hVal, sVal, gVal))

    if event == cv2.EVENT_MOUSEMOVE:  # Trigger on mouse movement
        zoom_factor = 10
        h, w, _ = img_bgr.shape

        # Define the 20x20 region around the cursor
        x1, y1 = max(0, x - 10), max(0, y - 10)
        x2, y2 = min(w, x + 10), min(h, y + 10)

        # Crop the region and resize it for zoom effect
        cropped = img_bgr[y1:y2, x1:x2]
        zoomed = cv2.resize(cropped, (200, 200), interpolation=cv2.INTER_LINEAR)

        #draw cross zoomed img
        cv2.line(zoomed, (100, 0), (100, 200), (0, 255, 0), 1)
        cv2.line(zoomed, (0, 100), (200, 100), (0, 255, 0), 1)

        # Display the zoomed area in a separate window
        cv2.imshow("Zoomed Area", zoomed)

if __name__ == "__main__":
    # Örnek: Kırmızı için HSV aralığı (iki aralık gerekebilir çünkü kırmızı hue 0/180 civarında çift bölgelidir)
    red_lower1 = np.array([98, 148, 244])
    red_upper1 = np.array([118, 168, 255])

    # Maskeyi dosya gibi kaydedip ardından `process_color_beacon_image` fonksiyonunu çağırabilirsiniz.
    # Ya da yukarıdaki fonksiyon içine entegre edebilirim.

    input_path="images/20240711_111126.jpg"
    
    img_bgr = cv2.imread(input_path)
    if img_bgr is None:
        raise FileNotFoundError(f"'{input_path}' bulunamadı.")
    
    
    img_bgr = cv2.resize(img_bgr, (800, 600))
    cv2.imshow("Original Image", img_bgr)
    cv2.setMouseCallback("Original Image", mouseRGB_callback)
        
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    while True:
        # Bu örnekte sadece ilk kırmızı aralığı işleniyor:

        # 1. Görüntüyü oku ve HSV uzayına dönüştür

        process_color_beacon_image(
            img_hsv=img_hsv,
            color_range_hsv=(red_lower1, red_upper1)
        )

        k = cv2.waitKey(0)
        if (k == 27): 
            cv2.destroyAllWindows()
            break
