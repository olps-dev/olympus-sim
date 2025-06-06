import random
import math
import time


class BME680Stub:
    """Stub for BME680 environmental sensor with realistic data patterns."""
    
    def read(self):
        """Generate realistic temperature and IAQ readings."""
        t = time.time()
        temp = 21 + 2 * math.sin(t / 60) + random.gauss(0, 0.3)
        iaq = 50 + 5 * math.sin(t / 43) + random.gauss(0, 1)
        return {"temperature": temp, "iaq": iaq}


class SSD1306Stub:
    """Stub for SSD1306 OLED display."""
    
    def draw_banner(self, text="OK"):
        """Simulate drawing text on the OLED display."""
        print(f"[OLED] {text}")


class BatteryADC:
    """Stub for battery voltage monitoring with discharge curve."""
    
    t0 = time.time()  # Class variable for simulation start time
    
    def voltage(self):
        """Return voltage following a 9-day discharge curve."""
        start = BatteryADC.t0
        dt = time.time() - start
        # Simple linear discharge from 4.2V to 3.7V over 9 days
        return 4.2 - max(0, dt / (9 * 24 * 3600)) * 0.5 