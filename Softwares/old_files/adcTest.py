#!/usr/bin/env python3
"""
MCP3008 ADC Chip'inden SPI ile veri okuma
Raspberry Pi + GPIO Zero kütüphanesi kullanılarak
"""

from gpiozero import MCP3008
import time

def main():
    """
    MCP3008'in 8 kanalından veri okur ve ekrana yazdırır
    
    Bağlantı şeması (MCP3008 -> Raspberry Pi):
    - VDD     -> 3.3V
    - VREF    -> 3.3V
    - AGND    -> GND
    - DGND    -> GND
    - CLK     -> GPIO 11 (SCLK)
    - DOUT    -> GPIO 9  (MISO)
    - DIN     -> GPIO 10 (MOSI)
    - CS/SHDN -> GPIO 8  (CE0)
    """
    
    # MCP3008'in 8 kanalını tanımla
    channels = {
        0: MCP3008(channel=0),
        1: MCP3008(channel=1),
        2: MCP3008(channel=2),
        3: MCP3008(channel=3),
        4: MCP3008(channel=4),
        5: MCP3008(channel=5),
        6: MCP3008(channel=6),
        7: MCP3008(channel=7)
    }
    
    print("MCP3008 ADC Okuma Başlatıldı")
    print("Çıkmak için Ctrl+C basın\n")
    print("-" * 80)
    
    try:
        while True:
            # Her kanaldan veri oku ve ekrana yazdır
            print(f"\n{time.strftime('%H:%M:%S')} - ADC Değerleri:")
            
            for ch_num, adc in channels.items():
                # Ham değer (0.0 - 1.0 arası)
                raw_value = adc.value
                
                # 10-bit değer (0 - 1023 arası)
                digital_value = int(raw_value * 1023)
                
                # Voltaj değeri (3.3V referans için)
                voltage = raw_value * 3.3
                
                print(f"  Kanal {ch_num}: {raw_value:.4f} | "
                      f"Digital: {digital_value:4d} | "
                      f"Voltaj: {voltage:.3f}V")


            x,y = GetQPDCoordinate([channels[i].value for i in range(4,8)])
            print(f"  QPD Koordinatları: X={x:.4f}, Y={y:.4f}")
            
            print("-" * 80)
            time.sleep(1)  # 1 saniye bekle
            
    except KeyboardInterrupt:
        print("\n\nProgram sonlandırıldı.")
    finally:
        # Kaynakları temizle
        for adc in channels.values():
            adc.close()
        print("ADC kanalları kapatıldı.")

def GetQPDCoordinate(vals):
    """
    QPD (Quadrant Photodiode) koordinatlarını hesaplar.
    
    Args:
        vals (list): 4 kanal değerleri listesi
    
    Returns:
        tuple: (x, y) koordinatları
    """
    if len(vals) != 4:
        raise ValueError("QPD koordinat hesaplaması için 4 kanal değeri gerekir.")
    
    # QPD kanalları: [top-left, top-right, bottom-left, bottom-right]
    # tr, tl, bl, br = vals
    tl = vals[3]
    tr = vals[1]
    bl = vals[2]
    br = vals[0]

    # X ve Y koordinatlarını hesapla
    x = (tr + br - tl - bl) / (tl + tr + bl + br)
    y = (tl + tr - bl - br) / (tl + tr + bl + br)
    
    return x, y



def read_single_channel(channel=0):
    """
    Tek bir kanaldan veri okuma örneği
    
    Args:
        channel (int): Okunacak kanal numarası (0-7)
    """
    adc = MCP3008(channel=channel)
    
    print(f"Kanal {channel} okunuyor...")
    
    try:
        for i in range(10):
            raw_value = adc.value
            digital_value = int(raw_value * 1023)
            voltage = raw_value * 3.3
            
            print(f"Okuma {i+1}: Ham={raw_value:.4f}, "
                  f"Digital={digital_value:4d}, "
                  f"Voltaj={voltage:.3f}V")
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nOkuma durduruldu.")
    finally:
        adc.close()


if __name__ == "__main__":
    # Tüm kanalları oku
    main()
    
    # Veya sadece tek bir kanal okumak için:
    # read_single_channel(channel=0)
