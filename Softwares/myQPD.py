
from gpiozero import MCP3008
import time

class MyQPD:
    def __init__(self):
        # MCP3008'in 4 kanalını QPD için tanımla
        self.tl = MCP3008(channel=4)  # Top-Left
        self.tr = MCP3008(channel=5)  # Top-Right
        self.bl = MCP3008(channel=6)  # Bottom-Left
        self.br = MCP3008(channel=7)  # Bottom-Right

    def getValues(self):
        return [self.tl.value, self.tr.value, self.bl.value, self.br.value]

    def getvaluesDigital(self):
        return [int(self.tl.value * 4096), int(self.tr.value * 4096), 
                int(self.bl.value * 4096), int(self.br.value * 4096)]

    def get_coordinates(self):

        # qpd değerleri belli bir eşiğin altında ise 0 olarak kabul edelim
        threshold = 0.05  # Eşik değeri
        if self.tl.value < threshold:
            tl = 0
        else:
            tl = self.tl.value
            
        if self.tr.value < threshold:
            tr = 0
        else:
            tr = self.tr.value
            
        if self.bl.value < threshold:
            bl = 0
        else:
            bl = self.bl.value
            
        if self.br.value < threshold:
            br = 0
        else:
            br = self.br.value

        total = tl + tr + bl + br

        if total == 0:
            return 0.0, 0.0  # Işık yoksa koordinatlar sıfır olur
        
        x = (tr + br - tl - bl) / total
        y = (tl + tr - bl - br) / total
        return x, y
    
# mm = MyQPD()

# while True:
#     vals = mm.getValues()
#     x, y = mm.get_coordinates()
#     print(f"QPD Değerleri: {vals} | Koordinatlar: X={x:.4f}, Y={y:.4f}")
#     time.sleep(0.5)