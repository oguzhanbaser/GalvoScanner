#!/usr/bin/env python3
"""
Raspberry Pi kamera görüntüsünü MP4 formatında kaydeden script.
"""

import cv2
import time
import os
from datetime import datetime
from myCamera import MyCamera


class VideoRecorder:
    """Raspberry Pi kamera görüntüsünü video dosyasına kaydeden sınıf."""
    
    def __init__(self, use_roi=True, output_dir="recordings", fps=30.0):
        """
        VideoRecorder'ı başlatır.
        
        Args:
            use_roi: True ise ROI kullan, False ise tam çözünürlük
            output_dir: Kaydedilen videoların saklanacağı klasör
            fps: Saniyedeki frame sayısı
        """
        self.use_roi = use_roi
        self.output_dir = output_dir
        self.fps = fps
        self.camera = None
        self.out = None
        self.is_recording = False
        self.frame_count = 0
        self.output_path = None
        
        # Çıkış klasörünü oluştur
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"Çıkış klasörü oluşturuldu: {self.output_dir}")
    
    def connect_camera(self):
        """Raspberry Pi kamerasına bağlan."""
        print("Raspberry Pi kamerası başlatılıyor...")
        self.camera = MyCamera()
        
        # İlk frame'i al ve boyutları belirle
        if self.use_roi:
            frame = self.camera.get_frame_roi()
            print(f"Kamera başlatıldı - ROI Modu")
        else:
            frame = self.camera.get_frame()
            print(f"Kamera başlatıldı - Tam Çözünürlük")
        
        height, width = frame.shape[:2]
        print(f"Çözünürlük: {width}x{height}")
        
        return width, height
    
    def start_recording(self, width, height, filename=None):
        """Video kaydını başlat."""
        if filename is None:
            # Otomatik dosya adı oluştur (tarih_saat formatında)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"video_{timestamp}.mp4"
        
        self.output_path = os.path.join(self.output_dir, filename)
        
        # VideoWriter oluştur - MP4 codec (H264)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # veya 'H264', 'X264'
        self.out = cv2.VideoWriter(self.output_path, fourcc, self.fps, (width, height))
        
        if not self.out.isOpened():
            raise Exception("VideoWriter açılamadı!")
        
        self.is_recording = True
        self.frame_count = 0
        print(f"\n{'='*60}")
        print(f"Kayıt başladı: {self.output_path}")
        print(f"FPS: {self.fps}, Çözünürlük: {width}x{height}")
        print(f"{'='*60}\n")
    
    def record_frame(self, frame):
        """Bir frame'i kaydet."""
        if self.is_recording and self.out is not None:
            self.out.write(frame)
            self.frame_count += 1
    
    def stop_recording(self):
        """Kaydı durdur ve dosyayı kapat."""
        if self.out is not None:
            self.out.release()
            self.is_recording = False
            
            # İstatistikleri göster
            duration = self.frame_count / self.fps
            file_size = os.path.getsize(self.output_path) / (1024 * 1024)  # MB
            
            print(f"\n{'='*60}")
            print(f"Kayıt tamamlandı!")
            print(f"Dosya: {self.output_path}")
            print(f"Toplam Frame: {self.frame_count}")
            print(f"Süre: {duration:.2f} saniye")
            print(f"Dosya Boyutu: {file_size:.2f} MB")
            print(f"{'='*60}\n")
    
    def release(self):
        """Kaynakları serbest bırak."""
        if self.out is not None:
            self.out.release()
        cv2.destroyAllWindows()
        print("Kaynaklar serbest bırakıldı")
    
    def run(self, max_duration=None, show_preview=True):
        """
        Video kaydını başlat ve çalıştır.
        
        Args:
            max_duration: Maksimum kayıt süresi (saniye), None ise sınırsız
            show_preview: Önizleme penceresi göster
        """
        try:
            # Kameraya bağlan
            width, height = self.connect_camera()
            
            # Kaydı başlat
            self.start_recording(width, height)
            
            # Kullanım talimatı
            print("KULLANIM:")
            print("- 'Q' tuşu: Kaydı durdur ve çık")
            print("- 'P' tuşu: Kayıt duraklatma/devam ettir")
            if show_preview:
                print("- Önizleme penceresi açık")
            print("-" * 60)
            
            start_time = time.time()
            paused = False
            
            while True:
                # Raspberry Pi kamerasından frame al
                try:
                    if self.use_roi:
                        frame = self.camera.get_frame_roi()
                    else:
                        frame = self.camera.get_frame()
                    
                    # picamera2 RGB formatında gelir, ancak kayıt için renk düzeltme yapma
                    # (VideoWriter ve görüntüleme için aynı format kullanılıyor)
                except Exception as e:
                    print(f"Frame okunamadı: {e}")
                    break
                
                # Frame'i kaydet (duraklatılmadıysa)
                if not paused:
                    self.record_frame(frame)
                
                # Önizleme göster
                if show_preview:
                    # Bilgi ekle
                    display_frame = frame.copy()
                    status = "PAUSED" if paused else "RECORDING"
                    color = (0, 165, 255) if paused else (0, 0, 255)
                    
                    cv2.putText(display_frame, f"{status} - Frame: {self.frame_count}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    
                    elapsed = time.time() - start_time
                    cv2.putText(display_frame, f"Time: {elapsed:.1f}s", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    cv2.imshow('Video Recorder - Press Q to Stop', display_frame)
                
                # Klavye kontrolü
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    print("Kullanıcı tarafından durduruldu...")
                    break
                elif key == ord('p') or key == ord('P'):
                    paused = not paused
                    status_msg = "DURAKLATILDI" if paused else "DEVAM EDİYOR"
                    print(f"Kayıt {status_msg}")
                
                # Maksimum süre kontrolü
                if max_duration and (time.time() - start_time) >= max_duration:
                    print(f"Maksimum süre ({max_duration}s) aşıldı, kayıt sonlandırılıyor...")
                    break
            
        except KeyboardInterrupt:
            print("\nCtrl+C ile durduruldu...")
        except Exception as e:
            print(f"Hata oluştu: {e}")
        finally:
            # Kaydı durdur ve temizle
            self.stop_recording()
            self.release()


def main():
    """Ana fonksiyon - özelleştirilebilir parametreler."""
    
    # AYARLAR - İhtiyaca göre değiştirin
    USE_ROI = False  # True: ROI modu (küçük alan), False: Tam çözünürlük
    OUTPUT_DIR = "recordings"  # Kaydedilen videoların klasörü
    FPS = 30.0  # Saniyedeki frame sayısı
    MAX_DURATION = None  # Maksimum süre (saniye), None=sınırsız
    SHOW_PREVIEW = False  # Önizleme penceresi göster
    
    print("="*60)
    print("Raspberry Pi Kamera Video Kaydedici")
    print("="*60)
    print(f"Mod: {'ROI (Küçük Alan)' if USE_ROI else 'Tam Çözünürlük'}")
    print(f"FPS: {FPS}")
    print(f"Çıkış Klasörü: {OUTPUT_DIR}")
    print("="*60)
    
    # Video kaydediciyi başlat
    recorder = VideoRecorder(
        use_roi=USE_ROI,
        output_dir=OUTPUT_DIR,
        fps=FPS
    )
    
    # Kaydı çalıştır
    recorder.run(max_duration=MAX_DURATION, show_preview=SHOW_PREVIEW)


if __name__ == "__main__":
    main()
