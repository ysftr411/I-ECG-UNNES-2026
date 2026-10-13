# I-ECG: Explainable ECG System

### Performa Algoritma Pan-Tompkins dan Analisis HRV untuk Explainable AI (XAI) Diagnosa Kesehatan Jantung pada Sistem Monitoring Elektrokardiogram Portable

[![Python](https://img.shields.io/badge/Python-3.9+-blue.svg)](https://python.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-ESP32-green.svg)](https://espressif.com)

---

## 📖 Deskripsi Proyek

Proyek skripsi S1 Teknik Elektro Universitas Negeri Semarang (UNNES) yang mengembangkan **sistem monitoring elektrokardiogram (EKG) portabel** berbasis **ESP32 + AD8232 + ADS1115** dengan antarmuka Python.

Sistem mengintegrasikan:
1. **Akuisisi sinyal EKG real-time** pada frekuensi sampling 360 Hz
2. **Prapemrosesan sinyal 4 tahap** (Notch, Bandpass, Moving Average, Wavelet db4)
3. **Deteksi puncak R** dengan **algoritma Pan-Tompkins (1985)**
4. **Analisis Heart Rate Variability (HRV)** domain waktu
5. **Klasifikasi berbasis aturan** (rule-based XAI)

**Peneliti:**
- **Yusuf Toro** (2305100074) — Pan-Tompkins, HRV, XAI
- **Tya Rizsfa Dhemika** — Prapemrosesan & Denoising

---

## 🎯 Fitur Utama

- ✅ Real-time monitoring EKG 360 Hz
- ✅ Deteksi puncak R otomatis (Pan-Tompkins)
- ✅ Analisis HRV dual-window (60s & 300s)
- ✅ Klasifikasi XAI transparan dengan narasi Indonesia
- ✅ Import dataset CSV/Excel untuk mode playback
- ✅ Ekspor hasil ke Excel

---

## 🔧 Hardware

| Komponen | Spesifikasi |
|----------|-------------|
| ESP32 DevKit | Dual-core 240 MHz |
| AD8232 | CMRR > 80 dB |
| ADS1115 | ADC 16-bit, I²C, 0,0625 mV/LSB |
| Elektroda | 3-lead (RA, LA, RL) |

---

## 🚀 Cara Menjalankan

### Instalasi

```bash
git clone https://github.com/USERNAME/I-ECG-UNNES-2026.git
cd I-ECG-UNNES-2026

python -m venv venv
venv\Scripts\activate         # Windows
source venv/bin/activate      # Mac/Linux

pip install -r requirements.txt