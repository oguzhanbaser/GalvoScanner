import cv2
import numpy as np
import json
import os
from myDetector import MyDetector  # detector_module, MyDetector sınıfını içermelidir


# Ana program - geriye dönük uyumluluk için
if __name__ == "__main__":
    # Kullanım örnekleri:
    
    # ÖRNEK 1: Resim ile kullanım
    # image_path = 'C:\\Users\\baser-huawei\\Documents\\GitHub\\GalvoScanner\\Softwares\\new_files\\image.jpg'
    # try:
    #     detector = MyDetector(image_path=image_path)
    #     detector.run(source_type='image')
    # except ValueError as e:
    #     print(f"Hata: {e}")
    
    # ÖRNEK 2: Video dosyası ile kullanım (yorumlu)
    # video_path = 'path/to/video.mp4'
    # try:
    #     detector = MyDetector(video_source=video_path)
    #     detector.run(source_type='video')
    # except ValueError as e:
    #     print(f"Hata: {e}")
    
    # ÖRNEK 3: Kamera ile kullanım (yorumlu)

    cap = cv2.VideoCapture("http://192.168.19.221:5000/video_roi")

    try:
        detector = MyDetector(laser_settings_file="laser_trackbar_settings.json", led_settings_file="led_trackbar_settings.json")  # 0 = varsayılan kamera

        # detector.run(source_type='video')

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Video sona erdi veya okunamadı.")
                break
            
            laser_point = detector.process_image(frame)
            led_points = detector.detect_leds(frame)

            cv2.imshow('LED Tespit', led_points['annotated_frame'])
            cv2.waitKey(10)

            if laser_point is not None:
                print(f"Laser Koordinatları: X={laser_point['center_point'][0]}, Y={laser_point['center_point'][1]}")
                cv2.imshow('Orijinal', laser_point['annotated_frame'])
                cv2.waitKey(10)
            else:
                cv2.imshow('Orijinal', frame)
                cv2.waitKey(10)
    except ValueError as e:
        print(f"Hata: {e}")