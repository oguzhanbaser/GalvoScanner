# Tubitak 1002 Projesi kapsamında geliştirilen Galbo Taraycılı Haberleşme Sistemi

## Genel Amaç
Galvo tabanlı optik yönlendirme, kamera kalibrasyonu, lazer/LED sürücü kartları ve simülasyon (Gazebo) bileşenlerini içeren çok bileşenli Ar-Ge çalışma alanı.

## Dizın Özeti
```
Softwares/
  calib_images/        (İlk kamera kalibrasyon ham görüntüleri)
  new_calib/           (Yeni kalibrasyon çekimleri)
  images/              (Deney / mesafe / çıktı görüntü setleri)
  esp_software/
    main_drv8825.cpp.bak.cpp   (DRV8825 step/dir sürücüsü için yedek ana kod)
    esp32-stepper/             (PlatformIO ESP32 step motor kontrol projesi)
    python-test/
      test1.py                 (Seri / kontrol test scripti)
      myenv*/                  (Yerel sanal ortam klasörleri - repoda tutulması gerekmez)
  opencv-ws/
    camera_calib.py            (Genel kalibrasyon scripti)
    camera_calib_real.py       (Gerçek donanım verisi ile kalibrasyon)
    camera_params.py           (Örnek/çıktı parametre dosyası)
    camera_params_rpi.py       (Raspberry Pi için parametre varyasyonu)
    find_focal_len.py          (Odak uzunluğu yaklaşık hesabı)
    my_utils.py                (Ortak yardımcı fonksiyonlar)
    test*.py / test*_iter.py   (Deneme / iteratif test scriptleri)
    images/, files/, __pycache__/ (Veri ve derleme çıktıları)
  rpi_calib/
    calib_images/              (Raspberry Pi kamera kalibrasyon görüntüleri)

Gazebo-Sim/
  my_test_pkg/
    CMakeLists.txt             (ROS paketi derleme tanımı)
    package.xml                (ROS paket meta verisi)
    launch/camera_gazebo.launch (Kamera modelini Gazebo’da başlatma)
    urdf/camera_model.xacro    (Parametrik kamera URDF/Xacro modeli)
    src/                       (Şu an boş / ileride eklenecek)

Devreler/
  beacon_led/
    led_tester*/               (LED test kartı revizyonları - KiCad .kicad_pcb/.kicad_sch)
    led_tester_v2/led_cnc/     (CNC üretim G-code çıktıları)
    led_tester_v2/production/  (BOM, konum, gerber paketleri)
  esp_stepper/
    esp_stepper.*              (ESP tabanlı stepper sürücü kart şematik & PCB)
    kicad_esp/                 (Espressif kütüphane paketi - footprint/simge)
    libs/pcb, libs/sch         (Özel footprint/schematic kütüphaneleri)
    outputs/                   (PDF çizimler)
    production/                (Üretim dosyaları, yedekler)
  laser_com/
    board/                     (Ön/alternatif ethernet tabanlı taslak kart)
    laser_com_v1/              (Lazer iletişim kartı modüler şemalar: laser_driver, regulator, step_up, fotodiyot)
    production/, output/, libs/ (Üretim & kütüphane)

```

## Önemli Bileşenler
- Kamera Kalibrasyonu: Softwares/opencv-ws/* (Chessboard / iç parametre türetme)
- ESP32 Step Motor Kontrolü: Softwares/esp_software/esp32-stepper (Galvo veya mekanik tarayıcı prototip sürüşü)
- Donanım Kartları: Devreler/* (LED beacon, stepper sürücü, lazer iletişim)
- Simülasyon: Gazebo-Sim/my_test_pkg (Kamera modelleme & ileride galvo sim eklenebilir)
- Kalibrasyon Görselleri: calib_images, new_calib, rpi_calib/calib_images (Distorsiyon/fütür LUT üretim girdileri)

## Kurulum (ESP32 Firmware - PlatformIO)
```
cd Softwares/esp_software/esp32-stepper
pio run
pio run -t upload
pio device monitor
```

## Kamera Kalibrasyon Akışı (Örnek)
1. Satranç tahtası görüntülerini calib_images/ veya new_calib/ altına koy.  
2. opencv-ws dizinine geç:
```
cd Softwares/opencv-ws
python camera_calib.py
```
3. Üretilen parametreleri (örn. camera_params.py) uygulama tarafında içe aktar.  
4. Raspberry Pi için ayrı parametre gerekirse camera_params_rpi.py referans alınır.

## Gazebo Simülasyonu
```
cd Gazebo-Sim/my_test_pkg
# ROS ortamını kaynakla (ör: source /opt/ros/<distro>/setup.bash)
roslaunch my_test_pkg camera_gazebo.launch
```
camera_model.xacro içindeki parametrelerle sanal kamera testleri yapılır.

## Donanım Üretim Notları
- production/ klasörleri: BOM (bom.csv), yerleşim (positions.csv), gerber paketleri (.zip)
- pcb & sch kütüphane dosyalarını libs/ altına izole ederek sürüm kontrolü yapılmış.
- Büyük revizyonlarda outputs/ PDF çizimleri referans saklama.

## Temizlik / .gitignore Önerisi
Aşağıdakiler repoda tutulmayabilir:
- __pycache__/, *.pyc
- Softwares/esp_software/esp32-stepper/.pio/
- Softwares/esp_software/python-test/myenv*/ (sanal ortamlar)
- Büyük ham görüntüler (Git LFS ile: *.jpg)

Örnek ekleme:
```
__pycache__/
*.pyc
.pio/
myenv*/
*.bak.*
```

## Olası Geliştirmeler
- Otomatik kalibrasyon raporu (Markdown çıktısı)
- Galvo açı ↔ piksel koordinat eşleştirme modülü
- ROS düğümü: gerçek donanım + simülasyon arayüzü
- Lazer iletişim protokol dokümantasyonu (ayrı docs/ önerilir)

## Lisans
(Lisans dosyası eklenmeli: MIT / Apache-2.0 / GPL seçilmemiş.)

## Katkı
Fork -> feature branch -> PR.  
İsimlendirme: feat-*, fix-*, doc-*.

## Uyarı
main_drv8825.cpp.bak.cpp ve büyük görüntü setleri sürüm geçmişini şişirebilir; gerekirse arşivleyin veya LFS.

## Hızlı Başlangıç
```
git clone <repo-url>
cd GalvoScanner
# ESP32
cd Softwares/esp_software/esp32-stepper && pio run
# Kalibrasyon
cd ../../opencv-ws && python camera_calib.py
```

## İletişim
Sorumlu / Ekip: (ekleyin)  
E-posta: (ekleyin)

---
Bu README mevcut görünen dizin yapısına göre derlenmiştir; yeni klasörler eklendikçe güncellenmelidir.