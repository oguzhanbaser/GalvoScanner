import cv2
import numpy as np
import json
import os, time
from myDetector import MyDetector  # detector_module, MyDetector sınıfını içermelidir
import serial

def waitForSerialData(ser, timeout=2):
    mTime = time.time()
    while ser.in_waiting == 0:
        if time.time() - mTime > timeout:  # 2 saniye zaman aşımı
            return None
        time.sleep(0.01)  # Küçük bir gecikme ekleyerek CPU kullanımını azalt

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
        pass

    return cmd_data

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

    detector = MyDetector(laser_settings_file="laser_trackbar_settings.json", led_settings_file="led_settings.json")  # 0 = varsayılan kamera

    
    sendCommand = False

    if sendCommand == True:
        ser = serial.Serial('/dev/ttyS0', 115200, timeout=1, dsrdtr=True)
    
    # ser.write(b'H')
    # waitForSerialData(ser, timeout=30)

        ser.write(b'G0,0,')
    # detector.run(source_type='video')

    last_time = 0
    step_x = 0
    step_y = 0
    
    # Koordinat filtreleme için buffer (son N frame ortalaması)
    from collections import deque
    laser_buffer = deque(maxlen=3)  # Son 3 frame
    led_buffer = deque(maxlen=3)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Video sona erdi veya okunamadı.")
            break

        # frame = cv2.imread("C:\\Users\\baser_7rlgtle\\Desktop\\MyFolders\\myGithub\\GalvoScanner\\ss_led_laser.png")
        frame = cv2.resize(frame, (640, 480))

        laser_point = detector.detect_laser(frame)
        led_points = detector.detect_leds(frame)

        cv2.imshow('Orjinal', frame)

        # if laser_point is not None:
        #     laser_x, laser_y = laser_point['center_point']
        #     print(f"Laser Koordinatları: X={laser_point['center_point'][0]}, Y={laser_point['center_point'][1]}")
        #     cv2.imshow('Lazer Tespit', laser_point['annotated_frame'])

        # if led_points is not None:
        #     led_x, led_y = led_points['center_point']
        #     print("LED Merkezleri:", led_points['center_point'])

        if (laser_point is not None) and (led_points is not None):
            # Ham koordinatları buffer'a ekle
            laser_buffer.append(laser_point['center_point'])
            led_buffer.append(led_points['center_point'])
            
            # Filtrelenmiş koordinatlar (ortalaması)
            laser_x = int(np.mean([p[0] for p in laser_buffer]))
            laser_y = int(np.mean([p[1] for p in laser_buffer]))
            led_x = int(np.mean([p[0] for p in led_buffer]))
            led_y = int(np.mean([p[1] for p in led_buffer]))
            
            # Farkları hesapla (lazer - hedef)
            diff_x = laser_x - led_x
            diff_y = laser_y - led_y
            
            # Tolerans kontrolü (hedefe ulaşıldı mı?)
            TOLERANCE = 3
            if abs(diff_x) < TOLERANCE:
                diff_x = 0
            if abs(diff_y) < TOLERANCE:
                diff_y = 0

            # Hareket gerekli mi?
            if (diff_x != 0 or diff_y != 0) and (time.time() - last_time > 0.1):
                # Farka göre adım boyutu belirle (daha büyük fark = daha büyük adım)
                # step_size_x = 1 if abs(diff_x) < 20 else (2 if abs(diff_x) < 50 else 3)
                # step_size_y = 1 if abs(diff_y) < 20 else (2 if abs(diff_y) < 50 else 3)
                step_size_x = 1
                step_size_y = 1
                
                # Hareket yönünü belirle
                # diff_x > 0: lazer sağda, galvo'yu sola kaydır (-)
                # diff_x < 0: lazer solda, galvo'yu sağa kaydır (+)
                if diff_x > 0:
                    step_x -= step_size_x
                elif diff_x < 0:
                    step_x += step_size_x
                
                if diff_y > 0:
                    step_y -= step_size_y
                elif diff_y < 0:
                    step_y += step_size_y

                last_time = time.time()

                # Komutu gönder
                command = f'G{step_y},{step_x},'
                # command = f'G0,{step_x},'

                if sendCommand == True:
                    ser.write(command.encode())
                    
                    recData = waitForSerialData(ser)
                    if recData:
                        cmd_data = parseSerialCommand(recData)
                        if(cmd_data['command'] != 'M' or cmd_data['data'] != 'O'):
                            print(f"Beklenmeyen yanıt: {recData}")
                        

                print(f"DiffX: {diff_x:+4d}, DiffY: {diff_y:+4d} | StepX: {step_x:+4d}, StepY: {step_y:+4d} | Komut: {command}")
            
            cv2.imshow('Lazer Tespit', laser_point['annotated_frame'])

        cv2.imshow('LED Tespit', led_points['annotated_frame'])

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
            