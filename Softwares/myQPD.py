
from gpiozero import MCP3008
import time
import threading
from collections import deque

class MyQPD:
    def __init__(self):
        # MCP3008'in 4 kanalını QPD için tanımla
        self.br = MCP3008(channel=4)  # Bottom-Right
        self.tr = MCP3008(channel=5)  # Top-Right
        self.bl = MCP3008(channel=6)  # Bottom-Left
        self.tl = MCP3008(channel=7)  # Top-Left

        # Her kanal için son 10 okumayı tutan buffer'lar
        self._buf_tl = deque(maxlen=10)
        self._buf_tr = deque(maxlen=10)
        self._buf_bl = deque(maxlen=10)
        self._buf_br = deque(maxlen=10)
        self._lock = threading.Lock()

        # Buffer'ı ilk değerlerle doldur (ilk okuma anında boş olmasın)
        for _ in range(10):
            self._buf_tl.append(self.tl.value)
            self._buf_tr.append(self.tr.value)
            self._buf_bl.append(self.bl.value)
            self._buf_br.append(self.br.value)

        # Arka plan okuma thread'ini başlat
        self._running = True
        self._thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._thread.start()

    def _sample_loop(self):
        """Her 10 ms'de bir 4 kanalı okur ve buffer'lara ekler."""
        while self._running:
            tl = self.tl.value
            tr = self.tr.value
            bl = self.bl.value
            br = self.br.value
            with self._lock:
                self._buf_tl.append(tl)
                self._buf_tr.append(tr)
                self._buf_bl.append(bl)
                self._buf_br.append(br)
            time.sleep(0.01)  # 10 ms

    def stop(self):
        """Arka plan thread'ini durdur."""
        self._running = False

    def getValues(self):
        with self._lock:
            return [
                sum(self._buf_tl) / len(self._buf_tl),
                sum(self._buf_tr) / len(self._buf_tr),
                sum(self._buf_bl) / len(self._buf_bl),
                sum(self._buf_br) / len(self._buf_br),
            ]

    def getvaluesDigital(self):
        vals = self.getValues()
        return [int(v * 4096) for v in vals]

    def get_coordinates(self):
        """Son 10 okumanın ortalamasını kullanarak QPD koordinatlarını döndürür.
        Bloklamaz — arka plan thread'i örneklemeyi sürdürür."""
        threshold = 0.02

        with self._lock:
            tl = sum(self._buf_tl) / len(self._buf_tl)
            tr = sum(self._buf_tr) / len(self._buf_tr)
            bl = sum(self._buf_bl) / len(self._buf_bl)
            br = sum(self._buf_br) / len(self._buf_br)

        if tl < threshold:
            tl = 0
        if tr < threshold:
            tr = 0
        if bl < threshold:
            bl = 0
        if br < threshold:
            br = 0

        total = tl + tr + bl + br

        if total == 0:
            return 0.0, 0.0

        x = (tr + br - tl - bl) / total
        y = (tl + tr - bl - br) / total

        print(f"QPD Ortalama: TL={tl:.4f}, TR={tr:.4f}, BL={bl:.4f}, BR={br:.4f} | Koordinatlar: X={x:.4f}, Y={y:.4f}")

        return x, y

# mm = MyQPD()

# while True:
#     vals = mm.getValues()
#     x, y = mm.get_coordinates()
#     print(f"QPD Değerleri: {vals} | Koordinatlar: X={x:.4f}, Y={y:.4f}")
#     time.sleep(0.5)