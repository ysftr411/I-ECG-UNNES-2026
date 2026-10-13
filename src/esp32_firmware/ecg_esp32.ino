// ================================================================
// I-ECG | ESP32 + ADS1115 (AD8232 -> A0)
// Keluaran serial: SATU ANGKA per baris = tegangan (mV) x 100
//   (Python membagi 100 lewat normalisasi() sehingga kembali ke mV)
// Laju sampling: 360 Hz, dijadwalkan dengan micros() (bukan delay())
// ================================================================
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define ADS_ADDR      0x48
#define FS_HZ         360
#define SDA_PIN       21
#define SCL_PIN       22

// Set ke 1 bila pin LO+ / LO- AD8232 terhubung (GPIO19 = LO+, GPIO18 = LO-).
// Bila elektroda lepas, ESP32 mengirim 0 (garis datar) agar sinyal sampah tidak dianalisis.
#define USE_LEADOFF   0
#define PIN_LO_PLUS   19
#define PIN_LO_MINUS  18

Adafruit_ADS1115 ads;

const unsigned long PERIOD_US = 1000000UL / FS_HZ;   // ~2777 us
unsigned long nextT;
int16_t lastRaw = 0;

// Membaca register konversi ADS1115 langsung lewat I2C (tanpa delay bawaan library),
// agar loop benar-benar mampu mencapai 360 Hz.
bool readConversion(int16_t &out) {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write((uint8_t)0x00);                 // pointer ke conversion register
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)ADS_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  out = (int16_t)((hi << 8) | lo);
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);                     // I2C 400 kHz

#if USE_LEADOFF
  pinMode(PIN_LO_PLUS, INPUT);
  pinMode(PIN_LO_MINUS, INPUT);
#endif

  if (!ads.begin(ADS_ADDR)) {
    // Jangan kirim teks ke serial (bisa merusak parsing di Python).
    // LED internal berkedip cepat sebagai penanda error.
    pinMode(2, OUTPUT);
    while (1) { digitalWrite(2, !digitalRead(2)); delay(150); }
  }

  ads.setGain(GAIN_TWO);                     // +/-2.048 V -> 0.0625 mV/bit
  ads.setDataRate(RATE_ADS1115_860SPS);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);   // continuous, A0
  Wire.setClock(400000);                     // pastikan clock tetap 400 kHz setelah begin()

  delay(50);
  nextT = micros();
}

void loop() {
  // Tunggu jadwal sampel berikutnya (aman terhadap overflow micros()).
  while ((long)(micros() - nextT) < 0) { }
  nextT += PERIOD_US;

  int16_t raw;
  if (readConversion(raw)) lastRaw = raw;
  else raw = lastRaw;                        // bila I2C gagal sesaat, pakai nilai terakhir

#if USE_LEADOFF
  if (digitalRead(PIN_LO_PLUS) || digitalRead(PIN_LO_MINUS)) {
    Serial.println(0);
    return;
  }
#endif

  // mV = raw * 0.0625  ->  mV x 100 = raw * 6.25
  long v100 = lroundf(raw * 6.25f);
  Serial.println(v100);
}
