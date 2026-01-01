flowchart TD
    Start([Program Başlat]) --> Loop{Ana Döngü}

    
    Loop --> DetectLaser[Laser Tespiti<br/>HSV Filtresi + Inpainting]
    DetectLaser --> DetectLED[LED Tespiti<br/>HSV Filtresi + Şekil Analizi]
    
    DetectLED --> CheckBoth{Hem Laser<br/>Hem LED<br/>Bulundu mu?}
    
    CheckBoth -->|Evet| AddBuffer[Koordinatları Buffer'a Ekle<br/>3 Frame Ortalaması]
    AddBuffer --> CalcDiff[Fark Hesapla]
    CalcDiff --> CheckTolerance{Fark Tolerans<br/>İçinde mi?<br/>±3 piksel}
    
    CheckTolerance -->|Hayır| CalcSteps[Adım Hesapla]
    CalcSteps --> SendCommand[Motor Komutu Gönder]
    SendCommand --> WaitResponse[ESP'den Yanıt Bekle]
    WaitResponse --> PrintStatus[Durum Bilgisi Yazdır]
    
    CheckTolerance -->|Evet| PrintStatus
    CheckBoth -->|Hayır| PrintStatus
    
    PrintStatus --> Display[Görüntüleri Göster<br/>Laser, LED, Orijinal]
    
    Display --> Loop

    
    style Start fill:#90EE90
    style DetectLaser fill:#87CEEB
    style DetectLED fill:#87CEEB
    style SendCommand fill:#FFD700
    style WaitResponse fill:#FFD700
    style CalcSteps fill:#FFA07A