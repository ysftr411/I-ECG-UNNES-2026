# Skema Wiring Hardware I-ECG

| Dari | Ke | Fungsi |
|------|----|--------|
| ESP32 3V3 | ADS1115 VDD | Catu daya ADC |
| ESP32 3V3 | AD8232 3.3V | Catu daya sensor |
| ESP32 GND | ADS1115 GND & AD8232 GND | Ground bersama |
| ADS1115 ADDR | GND | Alamat I²C = 0x48 |
| ESP32 GPIO22 | ADS1115 SCL | Clock I²C |
| ESP32 GPIO21 | ADS1115 SDA | Data I²C |
| AD8232 OUTPUT | ADS1115 A0 | Sinyal EKG analog |
| AD8232 LO+ | ESP32 GPIO19 | Deteksi leads-off + |
| AD8232 LO− | ESP32 GPIO18 | Deteksi leads-off − |