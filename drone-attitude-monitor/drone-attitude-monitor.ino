#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21, 22);

  Serial.println("\n--- I2C Scanner ---");
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("Device found at 0x%02X", addr);
      if (addr == 0x68) Serial.print("  <-- MPU6050");
      Serial.println();
      found++;
    }
  }

  if (found == 0) Serial.println("No devices found. Check wiring.");
  else Serial.printf("%d device(s) found.\n", found);
}

void loop() {}
```

---

**Step 3 — Save the file:**
```
Ctrl+S