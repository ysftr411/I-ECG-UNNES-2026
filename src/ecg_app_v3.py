import numpy as np
import scipy.signal as signal
import pywt
import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.gridspec import GridSpec
import tkinter as tk
from tkinter import messagebox, ttk, filedialog
import pandas as pd
import time
import threading
from collections import deque
import os
import warnings
warnings.filterwarnings('ignore')

# ================================================================
# KONFIGURASI & TEMA
# ================================================================
# --- Parameter akuisisi sinyal EKG (berlaku untuk seluruh sistem: Tya & Toro) ---
FS               = 360     # Frekuensi sampling (Hz). Menentukan resolusi waktu semua tahap
                            # pemfilteran (Tya) dan deteksi puncak R (Toro). Nilai 360 Hz
                            # memenuhi kriteria Nyquist untuk komponen EKG 0,5-40 Hz.
DISPLAY_WIN_SEC  = 6       # Panjang jendela tampilan grafik real-time (detik)
FAST_HRV_WIN_SEC      = 60    # Jendela ultra-short-term: pemantauan awal (detik ke-0 sampai <300)
STANDARD_HRV_WIN_SEC  = 300   # Jendela short-term 5 menit: analisis HRV utama (Task Force ESC/NASPE 1996)
FAST_HRV_RECALC_SEC   = 2.0   # Interval pembaruan estimasi cepat (setiap 2 detik)
STANDARD_HRV_RECALC_SEC = 5.0 # Interval pembaruan analisis utama (setiap 5 detik)
BUFFER_SIZE      = FS * (STANDARD_HRV_WIN_SEC + 10)   # Cukup untuk jendela 300 detik + sedikit cadangan
WINDOW_DISP      = FS * DISPLAY_WIN_SEC
BAUD_RATE        = 115200  # Baud rate komunikasi serial dengan ESP32
RT_INTERVAL_MS   = 250     # Interval pembaruan tampilan real-time (ms). 250 ms = 4 fps, cukup mulus & ringan.

PLAYBACK_DISP_WIN_SEC = 15
WINDOW_DISP_PLAYBACK  = FS * PLAYBACK_DISP_WIN_SEC

# --- Palet warna UI (tidak berkaitan dengan algoritma, murni tampilan) ---
BG_MAIN     = '#F4F7F6'
BG_PLOT     = '#FFFFFF'
BG_RIGHT    = '#FFFFFF'
HEADER_BG   = '#2C3E50'
LINE_RAW    = '#BDC3C7'
LINE_NOTCH  = '#F39C12'
LINE_BP     = '#9B59B6'
LINE_MA     = '#3498DB'
LINE_CLN    = '#2ECC71'
LINE_HR     = '#E67E22'
LINE_RR     = '#3498DB'
R_COLOR     = '#E74C3C'
BTN_ACCENT  = '#2980B9'
BTN_HOVER   = '#3498DB'
TEXT_DARK   = '#2C3E50'
GRID_CLR    = '#E0E6ED'

def normalisasi(v): return v / 100.0
# ^ Mengubah nilai mentah dari ESP32 (dikirim dalam skala x100 agar presisi desimal
#   tidak hilang saat dikirim sebagai teks lewat serial) kembali ke satuan mV.

# --- Basis referensi klinis yang dipakai sebagai dasar ambang batas (threshold) pada
#     modul Analyzer (Toro) untuk sistem pakar berbasis aturan / rule-based XAI.
#     Setiap flag klinis pada Analyzer.analyze() menunjuk ke salah satu kunci di sini,
#     sehingga narasi yang dihasilkan sistem selalu bisa ditelusuri sumber literaturnya. ---
REFERENCES = {
    'nsr':           'Normal Sinus Rhythm: HR 60-100 bpm (AHA/ACC Guidelines).',
    'spodick':       'Spodick DH (1996). Normal Sinus Heart Rate: 50-90 bpm — Framingham & EPICORE.',
    'ahaacls':       'AHA ACLS Guidelines 2016 — Bradikardia simtomatik <60 bpm, Takikardia >100 bpm.',
    'athlete_hr':    'Mesirca P, et al. (2021) Circulation Research — Bradikardia fisiologis atlet 30-60 bpm.',
    'hrv_taskforce': 'Task Force of ESC and NASPE (1996). Heart Rate Variability. Circulation 93(5):1043-1065.',
    'rr_cv':         'Regenstrief Institute (LOINC 76644-4) — Variabilitas RR menurun = risiko aritmia.',
    'af_pattern':    'US Patent 8195280 — Variabilitas RR ekstrem indikasi fibrilasi atrium.',
}

class ESP32Reader:
    """
    Menangani komunikasi serial dengan ESP32 (pembacaan sinyal EKG mentah dari modul
    AD8232 melalui ADC ADS1115). Kelas ini murni akuisisi data (I/O), belum melakukan
    pemfilteran (Tya) maupun deteksi puncak R (Toro) — data yang keluar dari sini masih 'raw'.

    Format yang diharapkan dari ESP32: SATU ANGKA per baris (mV x 100), laju 360 Hz.
    """
    def __init__(self, port=None):
        self.port      = port or self._detect()
        self.ser       = None
        self.connected = False
        self.buffer    = deque(maxlen=BUFFER_SIZE)        # buffer sirkular untuk tampilan real-time
        self.full_record = []                     # menyimpan seluruh riwayat data untuk diekspor
        self.running   = False
        self.start_t   = 0

    def _detect(self):
        # Mencoba menebak port COM ESP32 secara otomatis berdasarkan deskripsi chip USB-to-serial
        # yang umum dipakai board ESP32 (CP210x, CH340/341, FTDI, dsb).
        kw = ['CP210','CH340','CH341','UART','USB','SILICON','FTDI']
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if any(k in p.description.upper() for k in kw):
                return p.device
        return ports[0].device if ports else None

    def connect(self):
        # Membuka koneksi serial ke ESP32 pada BAUD_RATE yang telah ditentukan.
        try:
            self.ser = serial.Serial(self.port, BAUD_RATE, timeout=1)
            time.sleep(2)   # jeda 2 detik: menunggu ESP32 selesai boot/reset
            self.ser.reset_input_buffer()   # buang sisa data boot/teks lama
            self.connected = True; return True
        except Exception:
            return False

    def read_loop(self):
        # Loop pembacaan serial yang berjalan pada thread terpisah (lihat method start()),
        # agar UI (Tkinter) tidak freeze menunggu data masuk.
        #
        # Perbaikan: pembacaan dilakukan per-blok (read) dengan timeout, bukan polling
        # in_waiting tanpa jeda. Dengan cara ini thread tidur saat belum ada data, sehingga
        # tidak menghabiskan CPU / berebut GIL dengan GUI (penyebab lag sebelumnya).
        self.running = True
        self.start_t = time.time()
        leftover = b''
        while self.running:
            try:
                n = self.ser.in_waiting
                chunk = self.ser.read(n if n else 1)   # blok sampai ada >=1 byte (maks timeout 1 dtk)
                if not chunk:
                    continue
                leftover += chunk
                *lines, leftover = leftover.split(b'\n')
                if len(leftover) > 1024:               # pengaman: tidak ada newline terlalu lama
                    leftover = b''
                for ln in lines:
                    try:
                        v = normalisasi(float(ln.decode('ascii', errors='ignore').strip()))
                    except ValueError:
                        continue   # baris rusak/teks non-angka -> dilewati
                    if not np.isfinite(v):
                        continue
                    self.buffer.append(v)
                    if len(self.full_record) < 2000000:   # batas pengaman memori
                        # Waktu dihitung dari nomor sampel / FS (bukan time.time()), karena
                        # pembacaan serial datang bergelombang sehingga jam PC tidak seragam.
                        self.full_record.append({
                            'Waktu_s': round(len(self.full_record)/FS, 4),
                            'Raw_mV':  round(v, 4)
                        })
            except Exception:
                time.sleep(0.01)

    def start(self): threading.Thread(target=self.read_loop, daemon=True).start()
    def stop(self):
        self.running = False
        try:
            if self.ser: self.ser.close()
        except Exception:
            pass
    def get_data(self):
        # Offset DC keluaran AD8232 (±1,5 V) dibuang dengan median agar sinyal berpusat di
        # nol, seperti pada dataset. Tidak memengaruhi isi EKG karena bandpass juga membuangnya.
        arr = np.array(list(self.buffer))
        return arr - np.median(arr) if len(arr) else arr


class DatasetReader:
    """
    Pembaca alternatif selain ESP32: memuat sinyal EKG dari berkas CSV (mis. dataset
    Kaggle/PhysioNet) atau Excel (hasil ekspor sistem ini sendiri), untuk mode 'playback'.
    Berguna untuk menguji algoritma Pan-Tompkins & HRV Toro secara offline/berulang
    tanpa perlu perangkat keras ESP32 terhubung.
    """
    def __init__(self, filepath):
        self.filepath = filepath
        self.fs_hint  = None

        ext = os.path.splitext(filepath)[1].lower()
        if ext in ('.xlsx', '.xls'):
            sig, fs_detected = self._load_from_excel(filepath)
            self.fs_hint = fs_detected
        else:
            sig = self._load_from_csv(filepath)

        sig = np.asarray(sig, dtype=float)
        mx = np.max(np.abs(sig)) if len(sig) else 0
        if mx > 20: sig = sig / (mx / 2.0)   # normalisasi kasar bila skala amplitudo dataset jauh berbeda
        self.full_record = sig

    def _load_from_csv(self, filepath):
        # Menebak kolom sinyal EKG dari header CSV; jika tidak ketemu kata kunci,
        # fallback ke kolom paling kanan.
        df = pd.read_csv(filepath)
        cols = list(df.columns)
        target_col = cols[-1]
        for c in cols:
            if isinstance(c, str) and any(k in c.lower()
               for k in ['ecg','sig','val','mv','raw','sample']):
                target_col = c; break
        sig = df[target_col].dropna().values.astype(float)
        return sig

    def _load_from_excel(self, filepath):
        # Mirip _load_from_csv, tetapi juga mencoba mendeteksi frekuensi sampling asli
        # dataset dari kolom waktu (jika ada), agar dapat di-resample ke FS sistem (360 Hz).
        try:
            xls = pd.ExcelFile(filepath)
        except Exception:
            raise ValueError('File Excel tidak dapat dibuka / formatnya tidak dikenali.')

        sheet_name = None
        for s in xls.sheet_names:
            sl = s.lower()
            if 'raw' in sl or 'sinyal' in sl:
                sheet_name = s; break
        if sheet_name is None:
            sheet_name = xls.sheet_names[0]

        df = pd.read_excel(xls, sheet_name=sheet_name)
        if df.empty:
            raise ValueError(f"Sheet '{sheet_name}' pada file Excel ini kosong.")
        cols = list(df.columns)

        val_col = None
        for c in cols:
            if isinstance(c, str) and any(k in c.lower()
               for k in ['raw_mv','ecg','sig','val','mv','raw','sample']):
                val_col = c; break
        if val_col is None:
            val_col = cols[-1]
        sig = df[val_col].dropna().values.astype(float)

        # Estimasi frekuensi sampling asli = 1 / median(selisih antar-waktu)
        fs_detected = None
        time_col = None
        for c in cols:
            if isinstance(c, str) and ('waktu' in c.lower() or 'time' in c.lower()):
                time_col = c; break
        if time_col is not None:
            t = df[time_col].dropna().values.astype(float)
            if len(t) > 1:
                dt = np.diff(t)
                dt = dt[dt > 0]
                if len(dt):
                    fs_est = 1.0/np.median(dt)
                    if fs_est > 1:
                        fs_detected = int(round(fs_est))

        return sig, fs_detected


# ================================================================
# BAGIAN TYA RIZSFA DHEMIKA — Pemfilteran Sinyal EKG (Digital Signal Processing)
# Fokus: menghilangkan derau/noise agar sinyal EKG bersih sebelum diproses lebih
# lanjut oleh algoritma Pan-Tompkins (Toro). Tahapan mengikuti pipeline berjenjang:
#   Raw -> Notch 50 Hz -> Bandpass Butterworth -> Moving Average -> Wavelet DWT
# ================================================================
class Denoiser:
    def __init__(self):
        nyq = FS/2.0   # frekuensi Nyquist = setengah frekuensi sampling
        # --- Filter Notch 50 Hz: membuang gangguan jala-jala listrik (Power Line
        #     Interference/PLI) yang umum di Indonesia (frekuensi PLN 50 Hz). Q=30
        #     menentukan seberapa sempit rentang frekuensi yang ditolak (semakin besar
        #     Q, semakin sempit dan presisi notch-nya). ---
        self.bn, self.an = signal.iirnotch(50.0, Q=30, fs=FS)
        # --- Filter Bandpass Butterworth orde 4, rentang 0,5-40 Hz: mengisolasi pita
        #     frekuensi yang relevan secara diagnostik pada sinyal EKG, sekaligus
        #     membuang baseline drift frekuensi sangat rendah (<0,5 Hz) dan derau
        #     frekuensi tinggi (>40 Hz, mis. EMG otot). ---
        self.bb, self.ab = signal.butter(4, [0.5/nyq, 40.0/nyq], btype='band')

    def _get_metrics(self, raw, clean):
        # Menghitung metrik kualitas sinyal untuk membandingkan sinyal sebelum (raw)
        # dan sesudah (clean) tiap tahap filter:
        #   - SNR (Signal-to-Noise Ratio, dB): makin tinggi = makin bersih
        #   - MSE (Mean Squared Error): makin kecil = makin mirip sinyal aslinya (tanpa distorsi)
        #   - PRD (Percentage Root-mean-square Difference, %): makin kecil = makin sedikit distorsi
        if len(raw) != len(clean) or len(raw) == 0: return 0.0, 0.0, 0.0
        diff  = raw - clean
        sum_c2, sum_d2, sum_r2 = np.sum(clean**2), np.sum(diff**2), np.sum(raw**2)
        snr = 10*np.log10(sum_c2/sum_d2) if sum_d2 > 1e-10 else 100.0
        mse = np.mean(diff**2)
        prd = np.sqrt(sum_d2/sum_r2)*100 if sum_r2 > 1e-10 else 0.0
        return round(float(snr),2), round(float(mse),5), round(float(prd),2)

    def process(self, ecg):
        # Pipeline utama pemfilteran Tya. Mengembalikan sinyal pada setiap tahap
        # (stages) beserta metrik kualitasnya (metrics), sehingga bisa dibandingkan
        # satu sama lain di Dashboard 'Tahapan Filter'.
        stages  = {'raw': ecg}
        metrics = {}
        if len(ecg) < 50:
            # Sinyal terlalu pendek untuk difilter secara stabil (filtfilt butuh
            # panjang data minimum) -> lewati proses, kembalikan sinyal asli saja.
            for k in ['notch','bandpass','ma','wavelet']: stages[k] = ecg
            return stages, metrics

        # --- Tahap 1 (T1): Notch Filter 50 Hz. filtfilt = filter maju-mundur (zero-phase),
        #     sehingga tidak menggeser posisi puncak gelombang EKG (penting agar deteksi
        #     puncak R oleh Toro tetap akurat). ---
        s_notch = signal.filtfilt(self.bn, self.an, ecg)
        stages['notch'] = s_notch

        # --- Tahap 2 (T2): Bandpass Butterworth 0,5-40 Hz, diterapkan pada hasil notch. ---
        s_bp = signal.filtfilt(self.bb, self.ab, s_notch)
        stages['bandpass'] = s_bp

        # --- Tahap 3 (T3): Moving Average untuk menghilangkan baseline wander (naik-turun
        #     garis dasar akibat napas/gerakan pasien). Caranya: hitung tren garis dasar
        #     dengan rata-rata bergerak berjendela lebar (0,2*FS sampel, dibuat ganjil
        #     dengan '|1'), lalu kurangkan dari sinyal bandpass agar tren lambat tersebut hilang. ---
        w  = max(1, int(0.2*FS)|1)
        bl = np.convolve(s_bp, np.ones(w)/w, mode='same')   # estimasi baseline (tren lambat)
        s_ma = s_bp - bl                                     # sinyal - baseline = sinyal tanpa drift
        stages['ma'] = s_ma

        # --- Tahap 4 (T4): Discrete Wavelet Transform (DWT) Daubechies-4 (db4), level 4,
        #     untuk denoising halus tambahan menggunakan soft-thresholding universal
        #     (metode Donoho): estimasi level noise dari koefisien detail terhalus (co[-1]),
        #     lalu semua koefisien detail di-threshold agar komponen noise kecil dibuang
        #     tanpa merusak bentuk gelombang EKG. ---
        s_wave = s_ma
        if len(s_ma) >= 16:
            try:
                co = pywt.wavedec(s_ma, 'db4', level=4)                 # dekomposisi wavelet
                sg = np.median(np.abs(co[-1]))/0.6745                   # estimasi std noise (MAD-based)
                th = sg*np.sqrt(2*np.log(max(len(s_ma),1)))             # ambang universal Donoho
                ct = [co[0]] + [pywt.threshold(c, th, 'soft') for c in co[1:]]  # soft-threshold tiap level detail
                s_wave = pywt.waverec(ct, 'db4')[:len(ecg)]             # rekonstruksi sinyal bersih
            except: pass   # jika dekomposisi gagal (sinyal terlalu pendek/tidak stabil), pakai s_ma apa adanya
        stages['wavelet'] = s_wave

        # Menghitung metrik kualitas (SNR/MSE/PRD) untuk tiap tahap terhadap sinyal asli,
        # sehingga efektivitas masing-masing filter bisa dibandingkan secara kuantitatif.
        metrics['notch']    = self._get_metrics(ecg, s_notch)
        metrics['bandpass'] = self._get_metrics(ecg, s_bp)
        metrics['ma']       = self._get_metrics(ecg, s_ma)
        metrics['wavelet']  = self._get_metrics(ecg, s_wave)
        return stages, metrics


# ================================================================
# BAGIAN YUSUF TORO — Deteksi Puncak R (Algoritma Pan-Tompkins) & Analisis HRV/XAI
# Input kelas ini adalah sinyal yang SUDAH bersih (keluaran Denoiser milik Tya, biasanya
# stages['wavelet']). Fokus Toro: menemukan lokasi puncak R secara akurat, lalu
# menurunkan fitur HRV dan klasifikasi kondisi jantung berbasis aturan (rule-based XAI).
# ================================================================
class PanTompkins:
    def __init__(self):
        # Periode refrakter minimum antar-detak jantung (dalam sampel), digunakan agar
        # algoritma tidak mendeteksi dua puncak R pada satu kompleks QRS yang sama.
        # 0,2*FS = 200 milidetik, mendekati periode refrakter fisiologis jantung.
        self.ref = int(0.2*FS)

    def detect(self, ecg):
        # Implementasi 4 tahap klasik algoritma Pan & Tompkins (1985):
        # (1) turunan -> (2) pengkuadratan -> (3) integrasi jendela bergerak -> (4) deteksi puncak.
        if len(ecg) < FS: return np.array([])   # data kurang dari 1 detik, tidak cukup untuk dianalisis

        # --- Tahap 1: Penapisan Turunan (Derivative Filter) ---
        # Kernel 5-titik [1, 2, 0, -2, -1] menonjolkan kemiringan (slope) sinyal, yang
        # paling curam terjadi pada kompleks QRS dibanding gelombang P/T yang lebih landai.
        # Faktor (1/8)*FS menormalisasi skala turunan terhadap frekuensi sampling.
        b    = np.array([1,2,0,-2,-1])*(1/8)*FS
        diff = np.convolve(ecg, b, mode='same')

        # --- Tahap 2 & 3 digabung dalam satu baris: Pengkuadratan lalu Integrasi Jendela
        #     Bergerak (Moving Window Integration/MWI). Pengkuadratan (diff**2) membuat
        #     semua nilai positif dan menonjolkan komponen frekuensi tinggi QRS. MWI dengan
        #     jendela 0,15*FS sampel (150 ms, mendekati durasi tipikal kompleks QRS)
        #     menghasilkan sinyal 'amplop' yang lebar puncaknya sebanding durasi QRS. ---
        intg = np.convolve(diff**2, np.ones(int(0.15*FS))/int(0.15*FS), mode='same')

        # --- Tahap 4a: Ambang Adaptif (Adaptive Threshold). Ambang ditetapkan 5% dari
        #     nilai puncak tertinggi sinyal integrasi pada jendela yang sedang diproses,
        #     sehingga otomatis menyesuaikan skala amplitudo sinyal (adaptif per rekaman/pasien). ---
        thr  = np.max(intg)*0.05

        # --- Tahap 4b: Pencarian puncak kandidat pada sinyal hasil integrasi, dengan
        #     batas jarak minimum antar-puncak (self.ref = periode refrakter) untuk
        #     mencegah deteksi ganda pada satu kompleks QRS. ---
        pks,_ = signal.find_peaks(intg, distance=self.ref, height=thr)

        # --- Tahap 4c: Penghalusan Lokasi Puncak (Peak Refinement). Karena proses MWI
        #     menggeser & melebarkan posisi puncak, lokasi puncak R yang presisi dicari
        #     ulang pada sinyal EKG asli (bukan sinyal integrasi), yaitu titik dengan
        #     nilai absolut tertinggi dalam jendela pendek ±0,05*FS sampel (±50 ms) di
        #     sekitar tiap kandidat puncak. ---
        wk = int(0.05*FS)
        out = [np.argmax(np.abs(ecg[max(0,pk-wk):min(len(ecg),pk+wk)]))+max(0,pk-wk)
               for pk in pks]
        return np.array(sorted(set(out)))   # urutkan & buang indeks duplikat


class Analyzer:
    """
    Menurunkan fitur HRV (Heart Rate Variability) dari deretan puncak R hasil
    PanTompkins.detect(), lalu menjalankan sistem pakar berbasis aturan (rule-based
    expert system / rule-based XAI): setiap keputusan klasifikasi dibandingkan langsung
    terhadap ambang klinis yang bersumber literatur (lihat dict REFERENCES), sehingga
    seluruh proses dapat ditelusuri (explainable), bukan model black-box.
    """
    def analyze(self, ecg, peaks, profile='umum', window_sec_actual=None):
        if len(peaks) < 4: return None, None   # minimal 4 puncak R agar statistik HRV bermakna

        # --- Interval R-R (dalam milidetik) dari selisih antar-indeks puncak R berurutan,
        #     dikonversi dari satuan sampel ke waktu (dibagi FS) lalu ke milidetik (*1000). ---
        rr  = np.diff(peaks)/FS*1000
        # Menyaring interval yang secara fisiologis tidak masuk akal (mis. akibat deteksi
        # puncak R yang meleset), yaitu di luar rentang 300-1800 ms (setara ~33-200 bpm).
        rr  = rr[(rr>300)&(rr<1800)]
        if len(rr) < 3: return None, None

        # --- Fitur dasar: rata-rata interval R-R dan Heart Rate (HR = 60000/mean RR ms->bpm) ---
        mr   = np.mean(rr); hr = 60000/mr

        # --- SDNN (Standard Deviation of NN Intervals): variabilitas keseluruhan interval R-R. ---
        sdnn = np.std(rr)

        df = np.diff(rr)   # selisih antar-interval R-R berurutan, dasar RMSSD & pNN50
        # --- RMSSD (Root Mean Square of Successive Differences): mencerminkan variabilitas
        #     jangka pendek / aktivitas parasimpatis. ---
        rmss = np.sqrt(np.mean(df**2)) if len(df) else 0
        # --- pNN50: persentase pasangan interval R-R berurutan yang berbeda >50 ms. ---
        pnn  = (np.sum(np.abs(df)>50)/len(df)*100) if len(df) else 0
        # --- Regularitas irama (didefinisikan dalam penelitian ini, berbasis prinsip
        #     koefisien variasi SDNN/mean RR): 100% berarti irama sangat konsisten,
        #     dibatasi (di-clip) maksimum 100 agar tidak menghasilkan nilai negatif. ---
        reg  = round(100 - min(sdnn/mr*100 if mr else 0, 100), 1)

        f = {
            'HR':       round(hr,1),
            'Mean_RR':  round(mr,1),
            'SDNN':     round(sdnn,2),
            'RMSSD':    round(rmss,2),
            'pNN50':    round(pnn,1),
            'Reg':      reg,
            'N_Beats':  len(rr)+1,
            'Window_Sec': window_sec_actual,
        }

        # ============================================================
        # SISTEM PAKAR BERBASIS ATURAN (RULE-BASED XAI)
        # Setiap fitur dibandingkan terhadap ambang klinis; skor kumulatif menentukan
        # label umum (SEHAT/BATAS NORMAL/PERLU EVALUASI), sedangkan flag klinis bersifat
        # kondisional independen (bisa langsung memicu label 'KELAINAN TERDETEKSI'
        # meski skor kumulatif tinggi), agar kondisi berbahaya tidak "tertutupi" rata-rata.
        # ============================================================

        # Ambang bawah HR disesuaikan profil: atlet punya bradikardia sinus fisiologis
        # (denyut istirahat rendah adalah wajar), sehingga ambangnya diturunkan ke 40 bpm.
        hr_low = 40 if profile == 'atlet' else 60
        skor = 0; maks = 4
        if hr_low <= hr <= 100: skor += 1   # HR dalam rentang normal (sesuai profil)
        if sdnn > 40:           skor += 1   # variabilitas keseluruhan baik
        if rmss > 30:           skor += 1   # variabilitas jangka pendek/parasimpatis baik
        if reg  > 85:           skor += 1   # irama sangat teratur
        pct = (skor/maks)*100               # skor kesesuaian keseluruhan, dalam persen

        # --- Flag klinis: kondisi tegas yang langsung disertai rujukan literaturnya
        #     (mengambil dari dict REFERENCES), sehingga narasi XAI selalu bisa ditelusuri. ---
        flags = []
        if hr > 100:
            flags.append(f"⚠ TAKIKARDI — HR={hr:.1f} bpm (>100). Ref: {REFERENCES['ahaacls']}")
        if hr < hr_low:
            ref_b = REFERENCES['athlete_hr'] if profile=='atlet' else REFERENCES['ahaacls']
            flags.append(f"⚠ BRADIKARDI — HR={hr:.1f} bpm (<{hr_low}). Ref: {ref_b}")
        if sdnn < 20:
            flags.append(f"⚠ HRV SANGAT RENDAH — SDNN={sdnn:.1f} ms (<20). Ref: {REFERENCES['hrv_taskforce']}")
        if reg < 75:
            flags.append(f"⚠ IRAMA TIDAK TERATUR — Reg={reg:.1f}% (<75%). Ref: {REFERENCES['af_pattern']}")

        # --- Penentuan label akhir: flag klinis diprioritaskan; bila belum 300 detik
        #     label diberi status 'ESTIMASI AWAL' agar tidak mengklaim keadaan 'sehat'
        #     dari data yang belum mencukupi rekomendasi Task Force ESC/NASPE 1996. ---
        win    = window_sec_actual or 0   # durasi jendela aktif (detik)
        n_beat = f['N_Beats']
        if flags:
            label = 'PERINGATAN AWAL — BUTUH EVALUASI DOKTER'
        elif win < STANDARD_HRV_WIN_SEC:
            label = 'ESTIMASI AWAL — LANJUTKAN REKAMAN HINGGA 5 MENIT'
        elif pct >= 75:
            label = 'PARAMETER DALAM RENTANG OPERASIONAL SISTEM'
        elif pct >= 50:
            label = 'BATAS NORMAL — OBSERVASI DISARANKAN'
        else:
            label = 'EVALUASI KESEHATAN DISARANKAN'

        # --- Validitas statistik jendela waktu: 3 tingkat sesuai durasi rekaman ---
        if win < FAST_HRV_WIN_SEC:
            validity = (
                f"⚠ Data hanya {win:.0f} detik ({n_beat} detak). "
                f"Belum cukup untuk analisis HRV ultra-short-term."
            )
        elif win < STANDARD_HRV_WIN_SEC:
            validity = (
                f"⚠ Estimasi ultra-short-term HRV dari {win:.0f} detik "
                f"({n_beat} detak). Hasil digunakan untuk pemantauan awal "
                f"dan belum digunakan sebagai interpretasi HRV standar. "
                f"Analisis utama aktif setelah rekaman mencapai "
                f"{STANDARD_HRV_WIN_SEC} detik."
            )
        else:
            validity = (
                f"✓ Analisis short-term HRV menggunakan jendela "
                f"{win:.0f} detik ({n_beat} detak). "
                f"Durasi memenuhi jendela standar 5 menit untuk evaluasi "
                f"HRV domain waktu. Hasil tetap merupakan skrining awal, "
                f"bukan diagnosis klinis definitif."
            )

        # --- Narasi klinis otomatis: wujud konkret 'Explainable AI' pada sistem ini ---
        jenis_analisis = (
            "analisis HRV short-term 5 menit"
            if win >= STANDARD_HRV_WIN_SEC
            else "estimasi HRV ultra-short-term"
        )
        naratif = (
            f"Berdasarkan {jenis_analisis} selama ±{win:.0f} detik "
            f"(Profil Pasien: {profile.upper()}), sistem menghasilkan:\n\n"
            f"  📌  {label}\n\n"
            f"METRIK TERUKUR:\n"
            f"  • Heart Rate  : {f['HR']} bpm\n"
            f"  • Jarak R-R   : {f['Mean_RR']} ms (rata-rata)\n"
            f"  • SDNN (var)  : {f['SDNN']} ms\n"
            f"  • RMSSD       : {f['RMSSD']} ms\n"
            f"  • Regularitas : {f['Reg']} %\n\n"
        )

        if pct >= 75 and not flags and win >= STANDARD_HRV_WIN_SEC:
            naratif += (
                "PENJELASAN:\n"
                "Parameter yang dihitung berada dalam rentang operasional "
                "yang digunakan sistem. Hasil ini menunjukkan tidak adanya "
                "flag berdasarkan aturan yang diterapkan pada rekaman saat ini. "
                "Hasil tidak dapat digunakan untuk menyingkirkan seluruh "
                "kelainan jantung dan bukan pengganti pemeriksaan klinis.\n\n"
                f"{validity}"
            )
        elif pct >= 75 and not flags:
            # Skor baik tapi durasi belum 300 dtk — tampilkan narasi estimasi awal
            naratif += (
                "PENJELASAN:\n"
                "Parameter terukur saat ini berada dalam rentang operasional sistem. "
                "Namun durasi rekaman belum mencapai 5 menit (300 detik) yang "
                "direkomendasikan Task Force ESC/NASPE 1996, sehingga hasil ini "
                "hanya bersifat estimasi awal dan belum dapat digunakan sebagai "
                "dasar interpretasi HRV standar. Lanjutkan perekaman.\n\n"
                f"{validity}"
            )
        else:
            naratif += "PENJELASAN KELAINAN / CATATAN:\n"
            for fl in flags:
                naratif += f"  {fl}\n"
            if not flags:
                naratif += (
                    "Beberapa indikator HRV berada di bawah rentang optimal. "
                    "Kondisi ini bisa disebabkan oleh kelelahan, stres, atau sinyal belum stabil. "
                    "Disarankan perekaman ulang dalam kondisi istirahat."
                )
            naratif += f"\n\n{validity}"

        info = {
            'label':    label,
            'pct':      pct,
            'flags':    flags,
            'naratif':  naratif,
            'validity': validity,
            'rr_ms':    rr,
        }
        return f, info


# ================================================================
# APLIKASI GUI (Tkinter + Matplotlib) — menghubungkan modul akuisisi (ESP32Reader/
# DatasetReader), pemfilteran (Denoiser milik Tya), dan deteksi/analisis (PanTompkins
# & Analyzer milik Toro) ke dalam satu dashboard interaktif dengan 2 tab:
#   Tab 1 'Dashboard Real-Time & XAI'      -> menonjolkan hasil kerja Toro
#   Tab 2 'Dashboard Tahapan Filter'       -> menonjolkan hasil kerja Tya
# ================================================================
class ECGApp:
    def __init__(self, root):
        self.root = root
        self.root.title('Explainable ECG System (I-ECG) | UNNES 2026')
        self.root.configure(bg=BG_MAIN)
        self.root.state('zoomed')

        # Instansiasi modul inti: satu Denoiser (Tya), satu PanTompkins & satu Analyzer (Toro)
        self.denoiser = Denoiser()
        self.pt       = PanTompkins()
        self.analyzer = Analyzer()

        self.source      = None     # sumber data aktif: ESP32Reader atau array dataset
        self.mode        = 'esp32'  # 'esp32' (real-time) atau 'playback' (dataset)
        self.running     = False
        self.anim        = None     # (tidak dipakai lagi; diganti timer root.after)
        self._rt_job     = None     # id timer root.after untuk loop real-time
        self.t_start     = 0
        self.export_hrv  = []
        self.export_mtrx = []
        self.dataset_sig = None
        self.current_playback_idx = 0

        self.hr_history       = deque(maxlen=120)   # riwayat HR untuk grafik tren
        self.t_history        = deque(maxlen=120)
        self.last_fast_hrv_calc_t   = 0
        self.last_std_hrv_calc_t    = 0
        self.cached_fast_feats      = None   # fitur HRV jendela 60 dtk (ultra-short-term)
        self.cached_fast_info       = None
        self.cached_std_feats       = None   # fitur HRV jendela 300 dtk (standard short-term)
        self.cached_std_info        = None
        self.active_hrv_mode        = 'belum_cukup'  # 'belum_cukup' | 'ultra_short' | 'standard'

        self.user_profile = tk.StringVar(value='umum')   # profil pasien: memengaruhi ambang HR (Analyzer)
        self.show_ax1     = tk.BooleanVar(value=True)
        self.show_ax2     = tk.BooleanVar(value=True)
        self.show_ax3     = tk.BooleanVar(value=True)
        self.show_rpeak_tab1 = tk.BooleanVar(value=True)
        self.t_vars       = [tk.BooleanVar(value=True) for _ in range(5)]  # toggle 5 tahap filter Tya

        self._panel_visible_tab = {0: True, 1: True}

        self._tab2_zoom_idx = None
        self._axes_tab2_stage_map = {}

        # Inisialisasi atribut grafik agar aman diakses sebelum _rebuild_* pertama selesai
        self.ax1 = self.ax2 = self.ax3 = None
        self.ln_clean = self.ln_hr = self.sc_r = None
        self.axes_tab2 = []; self.lines_tab2 = []; self._active_t = []; self.sc_r2 = None

        self._build_ui()
        self._rebuild_tab1()
        self._rebuild_tab2()

    def _build_ui(self):
        # Membangun seluruh elemen antarmuka: toolbar atas, dua tab grafik (Matplotlib
        # ditanam dalam Tkinter via FigureCanvasTkAgg), panel kanan (Toro/Tya, saling
        # bertukar sesuai tab aktif), serta baris tombol kontrol & playback di bawah.
        root = self.root

        tbar = tk.Frame(root, bg=HEADER_BG, height=40)
        tbar.pack(fill='x', side='top')
        tbar.pack_propagate(False)
        tk.Label(tbar,
                 text='  I-ECG: EXPLAINABLE ECG SYSTEM  |  UNNES 2026',
                 font=('Segoe UI',11,'bold'), bg=HEADER_BG, fg='white').pack(side='left', pady=8)

        self.btn_toggle_panel = tk.Button(
            tbar,
            text='◀ Sembunyikan Panel',
            font=('Segoe UI',8,'bold'),
            bg='#1A252F', fg='#BDC3C7',
            activebackground='#2C3E50', activeforeground='white',
            relief='flat', cursor='hand2', padx=10,
            command=self._toggle_panel_kanan)
        self.btn_toggle_panel.pack(side='right', padx=10, pady=6)

        # Penghitung durasi perekaman (detik berlalu sejak tombol Mulai ditekan)
        self.lbl_elapsed = tk.Label(
            tbar, text='⏱ Durasi: 00:00 (0 dtk)',
            font=('Segoe UI',10,'bold'), bg='#1A252F', fg='#2ECC71', padx=10)
        self.lbl_elapsed.pack(side='right', padx=6, pady=6)

        self._body_frame = tk.Frame(root, bg=BG_MAIN)
        self._body_frame.pack(fill='both', expand=True, padx=10, pady=(6,0))

        self._left_frame = tk.Frame(self._body_frame, bg=BG_MAIN)
        self._left_frame.pack(side='left', fill='both', expand=True)

        style = ttk.Style()
        style.theme_use('default')
        style.configure('TNotebook', background=BG_MAIN, borderwidth=0)
        style.configure('TNotebook.Tab', padding=[15,6],
                        font=('Segoe UI',9,'bold'),
                        background='#D5D8DC', foreground=TEXT_DARK)
        style.map('TNotebook.Tab',
                  background=[('selected', BTN_ACCENT)],
                  foreground=[('selected','white')])

        self.notebook = ttk.Notebook(self._left_frame)
        self.notebook.pack(fill='both', expand=True)
        self.notebook.bind('<<NotebookTabChanged>>', self._on_tab_changed)

        self.tab1 = tk.Frame(self.notebook, bg=BG_MAIN)   # Tab 1: fokus Toro (real-time & XAI)
        self.tab2 = tk.Frame(self.notebook, bg=BG_MAIN)   # Tab 2: fokus Tya (tahapan filter)
        self.notebook.add(self.tab1, text='  Dashboard (Real-Time & XAI)  ')
        self.notebook.add(self.tab2, text='  Dashboard (Tahapan Filter)  ')

        self.fig1 = plt.figure(figsize=(11,7.5), facecolor=BG_MAIN)
        self.canvas1 = FigureCanvasTkAgg(self.fig1, master=self.tab1)
        self.canvas1.get_tk_widget().pack(fill='both', expand=True)

        self.fig2 = plt.figure(figsize=(11,7.5), facecolor=BG_MAIN)
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=self.tab2)
        self.canvas2.get_tk_widget().pack(fill='both', expand=True)
        self.fig2.canvas.mpl_connect('button_press_event', self._on_tab2_click)

        self.right_outer = tk.Frame(self._body_frame, bg=BG_MAIN, width=460)
        self.right_outer.pack(side='right', fill='y', padx=(10,0))
        self.right_outer.pack_propagate(False)

        self.lbl_right_title = tk.Label(
            self.right_outer,
            text='XAI: KLASIFIKASI & METRIK HRV',
            font=('Segoe UI',12,'bold'),
            bg=HEADER_BG, fg='white', padx=8, pady=10)
        self.lbl_right_title.pack(fill='x')

        self.canvas_r   = tk.Canvas(self.right_outer, bg=BG_RIGHT, highlightthickness=0)
        scrollbar_r     = tk.Scrollbar(self.right_outer, orient='vertical', command=self.canvas_r.yview)
        self.canvas_r.configure(yscrollcommand=scrollbar_r.set)
        self.canvas_r.pack(side='left', fill='both', expand=True)
        scrollbar_r.pack(side='right', fill='y')

        def _on_mw(event):
            self.canvas_r.yview_scroll(int(-1*(event.delta/120)), 'units')
        self.canvas_r.bind('<Enter>', lambda e: self.canvas_r.bind_all('<MouseWheel>', _on_mw))
        self.canvas_r.bind('<Leave>', lambda e: self.canvas_r.unbind_all('<MouseWheel>'))

        # Dua frame panel kanan disiapkan sekaligus di awal (Toro & Tya), lalu hanya
        # salah satu yang ditampilkan (di-swap) tergantung tab mana yang aktif.
        self.rf_toro = tk.Frame(self.canvas_r, bg=BG_RIGHT)
        self.rf_tya  = tk.Frame(self.canvas_r, bg=BG_RIGHT)
        self._make_right_toro()
        self._make_right_tya()

        self._rw = self.canvas_r.create_window((0,0), window=self.rf_toro, anchor='nw', width=440)

        def _cfg(e): self.canvas_r.configure(scrollregion=self.canvas_r.bbox('all'))
        self.rf_toro.bind('<Configure>', _cfg)
        self.rf_tya .bind('<Configure>', _cfg)

        bottom = tk.Frame(root, bg='#EAECEE', pady=6)
        bottom.pack(fill='x', side='bottom')

        row1 = tk.Frame(bottom, bg='#EAECEE')
        row1.pack(fill='x', padx=10)

        self.btn_start = self._big_btn(row1, '▶ Mulai Real-Time', self.cmd_start)
        self.btn_stop  = self._big_btn(row1, '■ Hentikan',        self.cmd_stop, state='disabled')
        self.btn_clear = self._big_btn(row1, '↺ Bersihkan',       self.cmd_clear)
        tk.Frame(row1, width=8, bg='#EAECEE').pack(side='left')
        self._small_btn(row1, '💾 Ekspor Excel',             self.cmd_export)
        self._small_btn(row1, '🔌 Hubungkan ESP32',          self.cmd_esp32)
        self._small_btn(row1, '📂 Import Dataset CSV (Kaggle)', self.cmd_import_csv)

        pf = tk.LabelFrame(row1, text='Profil Pasien (Acuan XAI)',
                           font=('Segoe UI',9,'bold'), bg='#EAECEE', fg=TEXT_DARK)
        pf.pack(side='right', padx=10, fill='y')
        tk.Radiobutton(pf, text='Orang Biasa/Umum', variable=self.user_profile,
                       value='umum', bg='#EAECEE', font=('Segoe UI',9)).pack(side='left', padx=4)

        row2 = tk.Frame(bottom, bg='#EAECEE')
        row2.pack(fill='x', padx=10, pady=(4,0))

        self.f_tog1 = tk.LabelFrame(row2, text='Toggle Grafik',
                                    font=('Segoe UI',9,'bold'), bg='#EAECEE', fg=TEXT_DARK)
        tk.Checkbutton(self.f_tog1, text='Puncak R (△)',
           variable=self.show_rpeak_tab1, bg='#EAECEE',
           font=('Segoe UI',9,'bold'), fg='#E74C3C',
           command=self._on_toggle_tab1).pack(side='left', padx=4)
        for txt, var in [('Sinyal EKG', self.show_ax1),
                         ('Distribusi R-R', self.show_ax2),
                         ('Tren HR (BPM)', self.show_ax3)]:
            tk.Checkbutton(self.f_tog1, text=txt, variable=var, bg='#EAECEE',
                           font=('Segoe UI',9), command=self._on_toggle_tab1).pack(side='left', padx=4)

        self.f_tog2 = tk.LabelFrame(row2, text='Toggle Tahapan Filter',
                                    font=('Segoe UI',9,'bold'), bg='#EAECEE', fg=TEXT_DARK)
        for i, txt in enumerate(['T0: Raw','T1: Notch','T2: Bandpass','T3: Mov.Avg','T4: Wavelet']):
            tk.Checkbutton(self.f_tog2, text=txt, variable=self.t_vars[i], bg='#EAECEE',
                           font=('Segoe UI',9), command=self._on_toggle_tab2).pack(side='left', padx=4)

        self.f_play = tk.Frame(bottom, bg='#D5D8DC')
        tk.Label(self.f_play, text='  ◄► PLAYBACK (Dataset CSV)  ',
                 font=('Segoe UI',10,'bold'), bg='#D5D8DC', fg=TEXT_DARK).pack(side='left', padx=8)
        self.lbl_play_pos = tk.Label(self.f_play, text='0.0 s',
                                     font=('Segoe UI',10,'bold'),
                                     bg='#D5D8DC', fg=BTN_ACCENT, width=8)
        self.lbl_play_pos.pack(side='left')
        self.slider_play = tk.Scale(
            self.f_play, from_=0, to=100, orient='horizontal',
            bg='#D5D8DC', activebackground=BTN_ACCENT,
            highlightthickness=0, length=900, showvalue=False,
            command=self._on_playback_scroll)
        self.slider_play.pack(side='left', padx=6, pady=2, fill='x', expand=True)
        self.lbl_play_end = tk.Label(self.f_play, text='— s',
                                     font=('Segoe UI',9),
                                     bg='#D5D8DC', fg='#7F8C8D', width=10)
        self.lbl_play_end.pack(side='left', padx=(0,8))

        statusbar = tk.Frame(root, bg='#D5D8DC')
        statusbar.pack(fill='x', side='bottom')

        self.lbl_status = tk.Label(statusbar,
                                   text='Status: Sistem Siap  |  Pilih Profil → [▶ Mulai Real-Time]',
                                   font=('Segoe UI',9), bg='#D5D8DC', fg=TEXT_DARK,
                                   anchor='w', padx=10, pady=2)
        self.lbl_status.pack(fill='x', side='left', expand=True)

        self.btn_show_playback = tk.Button(
            statusbar, text='▶ Tampilkan Playback',
            command=self._toggle_playback_visibility,
            font=('Segoe UI',8,'bold'),
            bg='#95A5A6', fg='white',
            activebackground=BTN_ACCENT, activeforeground='white',
            relief='flat', cursor='hand2', padx=8,
            state='disabled')
        self.btn_show_playback.pack(side='right', padx=8, pady=2)

        self._on_tab_changed(None)

    def _toggle_panel_kanan(self):
        # Menyembunyikan/menampilkan panel kanan agar grafik bisa diperlebar saat presentasi.
        sel = self.notebook.index(self.notebook.select())
        self._panel_visible_tab[sel] = not self._panel_visible_tab[sel]
        self._apply_panel_visibility(sel)

    def _apply_panel_visibility(self, sel):
        visible = self._panel_visible_tab[sel]
        if visible:
            self.right_outer.pack(side='right', fill='y', padx=(10,0))
        else:
            self.right_outer.pack_forget()
        self.btn_toggle_panel.config(
            text='◀ Sembunyikan Panel' if visible else '▶ Tampilkan Panel')

    def _on_tab_changed(self, _event):
        # Saat berpindah tab, panel kanan ikut ditukar: tab 1 (Real-Time & XAI) menampilkan
        # panel milik Toro (rf_toro), tab 2 (Tahapan Filter) menampilkan panel milik Tya (rf_tya).
        sel = self.notebook.index(self.notebook.select())
        if sel == 0:
            self.lbl_right_title.config(text='XAI: KLASIFIKASI & METRIK HRV')
            self.canvas_r.itemconfig(self._rw, window=self.rf_toro)
            self.f_tog1.pack(side='left', fill='y', padx=(0,10))
            self.f_tog2.pack_forget()
        else:
            self.lbl_right_title.config(text='TEORI & TAHAPAN PEMFILTERAN')
            self.canvas_r.itemconfig(self._rw, window=self.rf_tya)
            self.f_tog2.pack(side='left', fill='y', padx=(0,10))
            self.f_tog1.pack_forget()
        self._apply_panel_visibility(sel)
        self.canvas_r.update_idletasks()
        self.canvas_r.configure(scrollregion=self.canvas_r.bbox('all'))
        # Tab yang baru dibuka langsung digambar ulang (karena tab tersembunyi tidak digambar
        # selama real-time demi mengurangi beban CPU).
        self._redraw_current_frame()

    def _on_toggle_tab1(self):
        self._rebuild_tab1(); self._redraw_current_frame()

    def _on_toggle_tab2(self):
        self._rebuild_tab2(); self._redraw_current_frame()

    def _redraw_current_frame(self):
        # Menggambar ulang frame terakhir (tanpa menunggu data baru) setiap kali
        # pengaturan toggle grafik diubah, agar tampilan langsung menyesuaikan.
        if self.mode == 'playback' and self.dataset_sig is not None:
            self._render_playback_frame(self.current_playback_idx)
        elif self.source is not None:
            raw_full = self.source.get_data()
            if len(raw_full) >= FS*2:
                disp = raw_full[-min(WINDOW_DISP, len(raw_full)):]
                hrv_raw, hrv_win_sec, hrv_mode = self._select_hrv_window(raw_full)
                self._process_and_draw(
                    disp=disp,
                    hrv_raw=hrv_raw,
                    is_realtime=(self.mode != 'playback'),
                    skip_hrv=False,
                    hrv_win_sec=hrv_win_sec,
                    hrv_mode=hrv_mode
                )

    def _rebuild_tab1(self):
        # Membangun ulang layout grafik Tab 1 (sinyal EKG bersih + puncak R, distribusi
        # R-R, tren HR) sesuai kombinasi toggle yang aktif saat ini.
        self.fig1.clf()
        self.ax1 = self.ax2 = self.ax3 = None
        self.ln_clean = self.ln_hr = self.sc_r = None
        active = [self.show_ax1.get(), self.show_ax2.get(), self.show_ax3.get()]
        n = sum(active)
        if n == 0: self.canvas1.draw(); return

        ratios = []
        if active[0]: ratios.append(3)
        if active[1]: ratios.append(1)
        if active[2]: ratios.append(1)

        gs = GridSpec(n, 1, figure=self.fig1, height_ratios=ratios, hspace=0.45)
        self.fig1.subplots_adjust(left=0.07, right=0.97, top=0.95, bottom=0.07)

        idx = 0

        if active[0]:
            self.ax1 = self.fig1.add_subplot(gs[idx]); idx += 1
            self._style_ax(self.ax1, f'Sinyal EKG Bersih (Wavelet) — {DISPLAY_WIN_SEC} detik', 'Amplitudo (mV)')
            self.ln_clean, = self.ax1.plot([], [], color=LINE_CLN, lw=1.8, label='Sinyal Filtered (T4)')
            # Scatter puncak R: menandai lokasi hasil algoritma Pan-Tompkins (Toro) di atas
            # sinyal bersih hasil pemfilteran (Tya) — di sinilah pekerjaan kedua mahasiswa bertemu.
            self.sc_r = self.ax1.scatter([], [], c=R_COLOR, marker='^', s=60, zorder=5, label='Puncak R (Pan-Tompkins)')
            self.sc_r.set_visible(self.show_rpeak_tab1.get())
            self.ax1.legend(loc='upper right', fontsize=8, facecolor=BG_PLOT)

        if active[1]:
            self.ax2 = self.fig1.add_subplot(gs[idx]); idx += 1
            self._style_ax(self.ax2, 'Distribusi R-R Interval', 'Frekuensi')
            self.ax2.set_xlabel('R-R Interval (ms)', fontsize=8, color=TEXT_DARK)

        if active[2]:
            self.ax3 = self.fig1.add_subplot(gs[idx])
            self._style_ax(self.ax3, 'Tren Heart Rate (BPM) — Real-Time', 'BPM')
            self.ax3.set_xlabel('Waktu (detik)', fontsize=8, color=TEXT_DARK)
            self.ln_hr, = self.ax3.plot([], [], color=LINE_HR, lw=2)
            # Garis bantu ambang klinis HR (60, 100, dan 40 bpm untuk profil atlet) —
            # visualisasi langsung dari ambang yang dipakai Analyzer.analyze().
            self.ax3.axhline(60,  color='#3498DB', alpha=0.6, ls='--', lw=1, label='60 bpm')
            self.ax3.axhline(100, color='#E74C3C', alpha=0.6, ls='--', lw=1, label='100 bpm')
            self.ax3.axhline(40,  color='#2ECC71', alpha=0.6, ls='--', lw=1, label='40 bpm (Atlet)')
            self.ax3.legend(fontsize=7, loc='upper right')

        self.canvas1.draw()

    def _rebuild_tab2(self):
        # Membangun ulang layout grafik Tab 2 (5 tahap pemfilteran Tya: Raw, Notch,
        # Bandpass, Moving Average, Wavelet), termasuk mode "zoom" satu tahap saja
        # ketika grafik tahap tersebut di-klik dua kali (double-click).
        self.fig2.clf()
        self.axes_tab2 = []; self.lines_tab2 = []; self._active_t = []; self.sc_r2 = None
        active = [v.get() for v in self.t_vars]
        n = sum(active)
        self._axes_tab2_stage_map = {}
        if n == 0: self.canvas2.draw(); return

        titles = [
            'T0: Sinyal Raw (Asli dari Sensor)',
            'T1: Notch Filter 50 Hz (Buang PLI / Noise Listrik)',
            'T2: Butterworth Bandpass 0.5–40 Hz (Isolasi Frekuensi EKG)',
            'T3: Moving Average — Baseline Drift Removal',
            'T4: Discrete Wavelet Transform db4 (Denoising Halus)',
        ]
        colors = [LINE_RAW, LINE_NOTCH, LINE_BP, LINE_MA, LINE_CLN]

        zoom_idx = self._tab2_zoom_idx
        if zoom_idx is not None and active[zoom_idx]:
            # Mode zoom: hanya tampilkan satu subplot penuh untuk tahap yang dipilih.
            gs = GridSpec(1, 1, figure=self.fig2)
            self.fig2.subplots_adjust(left=0.06, right=0.97, top=0.92, bottom=0.09)
            ax = self.fig2.add_subplot(gs[0])
            self._style_ax(ax, titles[zoom_idx] + '   (Diperbesar — klik 2x pada grafik untuk kembali)', 'mV')
            ax.tick_params(labelsize=9)
            ln, = ax.plot([], [], color=colors[zoom_idx], lw=1.6)

            self.axes_tab2  = [ax]
            self.lines_tab2 = [ln]
            self._active_t  = [zoom_idx]
            self._axes_tab2_stage_map[ax] = zoom_idx
            self.sc_r2 = None

            self.canvas2.draw()
            return
        else:
            self._tab2_zoom_idx = None

        gs = GridSpec(n, 1, figure=self.fig2, hspace=0.55)
        self.fig2.subplots_adjust(left=0.07, right=0.94, top=0.95, bottom=0.07)

        self.axes_tab2  = []
        self.lines_tab2 = []
        self.sc_r2      = None
        self._active_t  = []

        idx = 0
        for i, is_on in enumerate(active):
            if is_on:
                ax = self.fig2.add_subplot(gs[idx])
                self._style_ax(ax, titles[i], 'mV')
                ax.tick_params(labelsize=7)
                ln, = ax.plot([], [], color=colors[i], lw=1.2)
                self.axes_tab2.append(ax)
                self.lines_tab2.append(ln)
                self._active_t.append(i)
                self._axes_tab2_stage_map[ax] = i
                idx += 1

        self.canvas2.draw()

    def _on_tab2_click(self, event):
        # Menangani double-click pada salah satu subplot Tab 2 untuk masuk/keluar
        # mode zoom tahap filter tersebut.
        if not getattr(event, 'dblclick', False) or event.inaxes is None:
            return
        if self._tab2_zoom_idx is not None:
            self._tab2_zoom_idx = None
        else:
            stage_idx = self._axes_tab2_stage_map.get(event.inaxes)
            if stage_idx is None:
                return
            self._tab2_zoom_idx = stage_idx
        self._rebuild_tab2()
        self._redraw_current_frame()

    def _make_right_toro(self):
        # Panel kanan Tab 1: menampilkan seluruh keluaran pekerjaan Toro — metrik HRV
        # terukur, catatan validitas statistik jendela waktu, label klasifikasi XAI,
        # narasi klinis otomatis, dan daftar flag klinis yang terdeteksi.
        rf = self.rf_toro
        pad = {'padx':10,'pady':3,'anchor':'w'}

        def section(txt, bg='#EAECEE', fg=TEXT_DARK):
            tk.Label(rf, text=txt, font=('Segoe UI',11,'bold'), bg=bg, fg=fg,
                     padx=8, pady=6).pack(fill='x', pady=(8,3))

        def val_row(label_txt, attr_name):
            row = tk.Frame(rf, bg=BG_RIGHT); row.pack(fill='x', **pad)
            tk.Label(row, text=label_txt+':', font=('Segoe UI',10,'bold'),
                     bg=BG_RIGHT, fg='#7F8C8D', width=14, anchor='w').pack(side='left')
            lbl = tk.Label(row, text='—', font=('Segoe UI',12,'bold'),
                           bg='#F4F6F9', fg=BTN_ACCENT, width=14,
                           relief='flat', padx=6, pady=2)
            lbl.pack(side='left', padx=4)
            setattr(self, attr_name, lbl)

        section('📈 METRIK HRV TERUKUR')
        val_row('HR (BPM)',        'rv_hr')
        val_row('Mean R-R (ms)',   'rv_rr')
        val_row('SDNN (ms)',       'rv_sdnn')
        val_row('RMSSD (ms)',      'rv_rmss')
        val_row('pNN50 (%)',       'rv_pnn')
        val_row('Regularitas (%)', 'rv_reg')
        val_row('Jml Detak',       'rv_nbeat')
        val_row('Durasi Jendela',  'rv_window')

        section('⚠ VALIDITAS STATISTIK', bg='#FCF3CF', fg='#7D6608')
        self.rv_validity = tk.Text(rf, height=4, font=('Segoe UI',9,'italic'),
                                   bg='#FEFEF0', fg='#7D6608', wrap='word',
                                   relief='flat', state='disabled', padx=8, pady=4)
        self.rv_validity.pack(fill='x', padx=10, pady=3)

        section('🤖 HASIL KLASIFIKASI XAI', bg='#D4E6F1', fg='#154360')
        self.rv_label = tk.Label(rf, text='Menunggu sinyal...',
                                 font=('Segoe UI',13,'bold'), bg='#F4F6F9', fg=TEXT_DARK,
                                 wraplength=420, justify='center', relief='flat', padx=8, pady=8)
        self.rv_label.pack(fill='x', padx=10, pady=3)
        self.rv_skor = tk.Label(rf, text='Skor Kesesuaian: —',
                                font=('Segoe UI',10,'bold'), bg=BG_RIGHT, fg='#7F8C8D')
        self.rv_skor.pack(anchor='w', padx=10, pady=2)

        section('🩺 PENJELASAN NARATIF (XAI — Berbasis Paper)')
        tk.Label(rf, text='Teks ini menjelaskan hasil analisis untuk dokter & orang awam.',
                 font=('Segoe UI',9,'italic'), bg=BG_RIGHT, fg='#7F8C8D',
                 wraplength=420, justify='left').pack(fill='x', padx=10, pady=(0,4))
        self.rv_naratif = tk.Text(rf, height=18, font=('Segoe UI',10),
                                  bg='#F8F9F9', fg=TEXT_DARK, wrap='word',
                                  relief='flat', state='disabled', padx=10, pady=8, spacing2=3)
        self.rv_naratif.pack(fill='x', padx=10, pady=3)

        section('⚠ FLAG KLINIS TERDETEKSI', bg='#FADBD8', fg='#78281F')
        self.rv_flags = tk.Text(rf, height=6, font=('Segoe UI',9,'bold'),
                                bg='#FEF9F9', fg=R_COLOR, wrap='word',
                                relief='flat', state='disabled', padx=8, pady=4)
        self.rv_flags.pack(fill='x', padx=10, pady=3)

    def _make_right_tya(self):
        # Panel kanan Tab 2: menampilkan penjelasan teori tiap tahap filter beserta
        # rujukannya, dan metrik kualitas sinyal (SNR/MSE/PRD) hasil kerja Tya.
        rf = self.rf_tya

        def section(txt, bg='#EAECEE', fg=TEXT_DARK):
            tk.Label(rf, text=txt, font=('Segoe UI',11,'bold'), bg=bg, fg=fg,
                     padx=8, pady=6).pack(fill='x', pady=(8,3))

        def desc(title, body_txt, ref=''):
            tk.Label(rf, text=title, font=('Segoe UI',10,'bold'),
                     bg=BG_RIGHT, fg=BTN_ACCENT, anchor='w').pack(fill='x', padx=10, pady=(8,0))
            tk.Label(rf, text=body_txt, font=('Segoe UI',9),
                     bg=BG_RIGHT, fg=TEXT_DARK, anchor='w',
                     justify='left', wraplength=420).pack(fill='x', padx=14, pady=2)
            if ref:
                tk.Label(rf, text=f'📚 Ref: {ref}', font=('Segoe UI',8,'italic'),
                         bg=BG_RIGHT, fg='#7F8C8D',
                         wraplength=420, justify='left').pack(fill='x', padx=14, pady=(0,4))

        section('🛠 TAHAPAN PEMFILTERAN SINYAL')
        desc('1. Notch Filter — 50 Hz',
             'Menghilangkan gangguan jala-jala listrik (Power Line Interference / PLI).',
             'Rangayyan RM (2002). Biomedical Signal Analysis. IEEE Press.')
        desc('2. Butterworth Bandpass 0.5–40 Hz',
             'Mengekstrak frekuensi komponen EKG (0.5–40 Hz).',
             'AHA (1990). Recommendations for Standardization of ECG Frequency Response.')
        desc('3. Moving Average — Baseline Drift',
             'Membuang Baseline Wander akibat gerakan napas pasien.',
             'Bhatt V, Bhatt C (2019). Baseline Wander Removal Techniques. IJAREEIE.')
        desc('4. Wavelet DWT db4',
             'Denoising halus dengan Daubechies-4 (db4), level 4.',
             'Mallat S (1989). A Theory for Multiresolution Signal Decomposition. IEEE TPAMI.')

        section('📊 METRIK KUALITAS SINYAL (Realtime)', bg='#D4EFDF', fg='#145A32')

        def mrow(lbl, attr):
            row = tk.Frame(rf, bg=BG_RIGHT); row.pack(fill='x', padx=10, pady=3)
            tk.Label(row, text=lbl+':', font=('Segoe UI',10,'bold'),
                     bg=BG_RIGHT, fg='#7F8C8D', width=22, anchor='w').pack(side='left')
            lv = tk.Label(row, text='—', font=('Segoe UI',12,'bold'),
                          bg='#F4F6F9', fg='#145A32', width=12, relief='flat', padx=6, pady=2)
            lv.pack(side='left', padx=4)
            setattr(self, attr, lv)

        mrow('SNR Wavelet (dB)',  'rv_snr')
        mrow('MSE Wavelet',       'rv_mse')
        mrow('PRD Wavelet (%)',   'rv_prd')
        mrow('SNR Bandpass (dB)', 'rv_snr_bp')
        mrow('PRD Bandpass (%)',  'rv_prd_bp')

        section('📚 REFERENSI FILTER')
        ref_txt = '\n'.join([f"• [{k}]: {v}" for k,v in REFERENCES.items()])
        txt = tk.Text(rf, height=12, font=('Segoe UI',8), bg='#F8F9F9', fg='#5D6D7E',
                      wrap='word', relief='flat', state='normal', padx=8, pady=6)
        txt.insert('end', ref_txt)
        txt.config(state='disabled')
        txt.pack(fill='x', padx=10, pady=3)

    def _style_ax(self, ax, title, ylabel):
        # Fungsi bantu styling grafik (murni tampilan, tidak memengaruhi algoritma).
        ax.set_facecolor(BG_PLOT)
        ax.set_title(title, fontsize=9, fontweight='bold', color=TEXT_DARK, pad=6)
        ax.set_ylabel(ylabel, fontsize=8, color=TEXT_DARK)
        ax.tick_params(labelsize=8, colors=TEXT_DARK)
        ax.grid(True, alpha=0.6, color=GRID_CLR, lw=0.6)
        for sp in ax.spines.values(): sp.set_edgecolor('#BDC3C7'); sp.set_linewidth(1)

    def _big_btn(self, parent, text, cmd, state='normal'):
        btn = tk.Button(parent, text=text, command=cmd,
                        font=('Segoe UI',10,'bold'), bg=BTN_ACCENT, fg='white',
                        activebackground=BTN_HOVER, activeforeground='white',
                        relief='flat', width=16, state=state, cursor='hand2')
        btn.pack(side='left', padx=4, pady=4)
        return btn

    def _small_btn(self, parent, text, cmd):
        tk.Button(parent, text=text, command=cmd,
                  font=('Segoe UI',9,'bold'), bg='#BDC3C7', fg=TEXT_DARK,
                  activebackground='#A6ACAF', relief='flat', width=28,
                  cursor='hand2').pack(side='left', padx=4, pady=4)

    def cmd_start(self):
        # Memulai sesi perekaman/analisis real-time: mereset riwayat, lalu menjalankan
        # timer (root.after) yang secara berkala memanggil _rt_frame() setiap RT_INTERVAL_MS.
        # Timer root.after dipakai (bukan FuncAnimation) agar tidak ada penggambaran ganda
        # pada figure yang sedang tidak terlihat dan frame tidak menumpuk saat komputer sibuk.
        if self.running: return
        self.export_hrv = []; self.export_mtrx = []
        self.hr_history.clear(); self.t_history.clear()
        self.t_start = time.time()
        self._update_elapsed(0)
        self.last_fast_hrv_calc_t = 0
        self.last_std_hrv_calc_t  = 0
        self.cached_fast_feats = self.cached_fast_info = None
        self.cached_std_feats  = self.cached_std_info  = None
        self.active_hrv_mode = 'belum_cukup'

        if self.mode == 'playback':
            self.mode = 'esp32'; self._close_playback()

        if self.mode == 'esp32':
            if not self.source or not self.source.connected:
                self._set_status('ESP32 belum terhubung!'); return
            self._set_status(f'▶ ESP32 | Profil: {self.user_profile.get().upper()}')

        self.running = True
        self.btn_start.config(state='disabled', bg='#7F8C8D')
        self.btn_stop .config(state='normal',   bg='#E74C3C')
        self._rt_job = self.root.after(RT_INTERVAL_MS, self._rt_tick)

    def _update_elapsed(self, detik=None):
        # Memperbarui label durasi perekaman (mm:ss dan total detik).
        if detik is None:
            detik = time.time() - self.t_start if self.t_start else 0
        detik = max(0, int(detik))
        self.lbl_elapsed.config(
            text=f'⏱ Durasi: {detik//60:02d}:{detik%60:02d} ({detik} dtk)')

    def _rt_tick(self):
        # Satu siklus timer real-time. Jadwal berikutnya baru dibuat SETELAH frame selesai
        # diproses, sehingga bila pemrosesan lambat, frame tidak menumpuk (tidak ngelag).
        self._rt_job = None
        if not self.running: return
        self._update_elapsed()
        try:
            self._rt_frame(None)
        except Exception as e:
            print('Error frame real-time:', e)
        if self.running:
            self._rt_job = self.root.after(RT_INTERVAL_MS, self._rt_tick)

    def cmd_stop(self):
        if not self.running: return
        self.running = False
        if self._rt_job is not None:
            try: self.root.after_cancel(self._rt_job)
            except Exception: pass
            self._rt_job = None
        self.btn_start.config(state='normal',   bg=BTN_ACCENT)
        self.btn_stop .config(state='disabled', bg='#7F8C8D')
        self._set_status('■ Dihentikan | Data siap diekspor.')

    def cmd_clear(self):
        # Mereset seluruh state aplikasi ke kondisi awal (buffer, riwayat, mode, panel kanan).
        self.cmd_stop()
        if self.source and hasattr(self.source,'buffer'):
            self.source.buffer.clear()
            if hasattr(self.source, 'full_record'): self.source.full_record.clear()
        self.export_hrv = []; self.export_mtrx = []
        self.hr_history.clear(); self.t_history.clear()
        self.cached_fast_feats = self.cached_fast_info = None
        self.cached_std_feats  = self.cached_std_info  = None
        self.active_hrv_mode = 'belum_cukup'
        self.dataset_sig = None; self.mode = 'esp32'
        self._update_elapsed(0)
        self._close_playback()
        self.btn_show_playback.config(state='disabled', bg='#95A5A6', text='▶ Tampilkan Playback')
        self._rebuild_tab1(); self._rebuild_tab2()
        self._reset_right_panel()
        self._set_status('↺ Sistem dibersihkan.')

    def cmd_import_csv(self):
        # Memuat dataset EKG eksternal (CSV Kaggle/PhysioNet atau Excel hasil ekspor
        # sistem ini) untuk mode playback: sinyal di-resample ke FS sistem (360 Hz)
        # bila frekuensi sampling aslinya berbeda, agar tetap kompatibel dengan
        # parameter algoritma Pan-Tompkins (Toro) dan filter (Tya) yang sudah ditetapkan.
        fnm = filedialog.askopenfilename(
            title='Pilih File Dataset EKG (Kaggle/PhysioNet) atau Hasil Ekspor I-ECG (.xlsx)',
            filetypes=[('CSV / Excel Files','*.csv *.xlsx *.xls'),
                       ('CSV Files','*.csv'),
                       ('Excel Files (Hasil Ekspor I-ECG)','*.xlsx *.xls'),
                       ('All Files','*.*')])
        if not fnm: return
        try:
            self.cmd_stop()
            dr = DatasetReader(fnm)
            sinyal_asli = dr.full_record
            is_excel = os.path.splitext(fnm)[1].lower() in ('.xlsx', '.xls')

            fs_asli = getattr(dr, 'fs_hint', None)

            if fs_asli is None:
                # Coba tebak FS dari nama berkas (mis. 'dataset_360hz.csv') bila tidak
                # terdeteksi dari kolom waktu.
                basename = os.path.basename(fnm).lower()
                for hint in ['360','500','250','1000','100','125','200']:
                    if hint in basename:
                        try:
                            fs_asli = int(hint); break
                        except: pass
            if fs_asli is None:
                try:
                    if not is_excel:
                        df_peek = pd.read_csv(fnm, nrows=1, header=None)
                        ncols = df_peek.shape[1]
                        fs_asli = 360 if ncols >= 186 else 360
                    else:
                        fs_asli = FS
                except: fs_asli = 360

            if fs_asli != FS and fs_asli > 0:
                # Resampling sinyal (interpolasi Fourier) agar jumlah sampel per detik
                # sesuai FS sistem (360 Hz), tanpa mengubah durasi total sinyal.
                n_target = int(len(sinyal_asli) * FS / fs_asli)
                sinyal_asli = signal.resample(sinyal_asli, max(n_target, 10))

            self.dataset_sig = sinyal_asli
            total_sampel     = len(self.dataset_sig)
            total_detik      = total_sampel / FS
            total_mnt        = int(total_detik // 60)
            total_sisa       = total_detik % 60

            self.mode = 'playback'
            self.current_playback_idx = 0

            max_start = max(0, total_sampel - WINDOW_DISP_PLAYBACK)
            self.slider_play.config(from_=0, to=max_start)
            self.slider_play.set(0)
            self.lbl_play_pos.config(text='0.0 s')
            self.lbl_play_end.config(text=f'{total_detik:.1f} s')

            self.f_play.pack(fill='x', padx=10, pady=(2,0))

            self.btn_show_playback.config(state='normal', bg='#E74C3C', text='✕ Tutup Playback')

            sumber_txt = 'Hasil Ekspor I-ECG (.xlsx)' if is_excel else 'CSV Kaggle/PhysioNet'
            self._set_status(
                f'📂 {os.path.basename(fnm)} | Sumber: {sumber_txt} | '
                f'FS: {fs_asli}Hz → {FS}Hz | '
                f'{total_sampel:,} sampel | '
                f'{total_mnt} mnt {total_sisa:.1f} dtk | '
                f'Geser slider untuk playback.'
            )
            self._render_playback_frame(0)

            messagebox.showinfo(
                'Import Dataset Berhasil',
                f'File          : {os.path.basename(fnm)}\n'
                f'Sumber        : {sumber_txt}\n'
                f'FS terdeteksi : {fs_asli} Hz  →  sistem pakai {FS} Hz\n'
                f'Jumlah sampel : {total_sampel:,} sampel\n'
                f'Durasi        : {total_mnt} menit {total_sisa:.1f} detik\n'
                f'                ({total_detik:.1f} detik total)\n\n'
                f'Geser slider ◄► untuk memutar di seluruh durasi.\n'
                f'Analisis XAI tersedia di panel kanan.'
            )
        except Exception as e:
            messagebox.showerror('Gagal Import CSV/Excel', str(e))

    def cmd_export(self):
        # Mengekspor seluruh riwayat perekaman ke satu berkas Excel dengan 4 sheet:
        # sinyal mentah, hasil HRV+XAI (Toro), metrik kualitas filter (Tya), dan referensi.
        if not self.export_hrv:
            messagebox.showinfo('Info','Belum ada data rekaman untuk diekspor.')
            return
        fnm = filedialog.asksaveasfilename(
            defaultextension='.xlsx',
            initialfile=f'IECG_{self.user_profile.get().upper()}_{int(time.time())}.xlsx',
            title='Simpan Data Excel',
            filetypes=[('Excel Workbook','*.xlsx')])
        if not fnm: return

        df_hrv  = pd.DataFrame(self.export_hrv)
        df_mtrx = pd.DataFrame(self.export_mtrx)
        sig_data = getattr(self.source,'full_record',[]) if self.source else []
        df_sig  = pd.DataFrame(sig_data) if sig_data else pd.DataFrame()
        df_ref  = pd.DataFrame([{'Kode':k,'Sitasi':v} for k,v in REFERENCES.items()])
        try:
            with pd.ExcelWriter(fnm, engine='openpyxl') as writer:
                if not df_sig.empty:
                    df_sig.to_excel(writer, sheet_name='1_Sinyal_Raw', index=False)
                df_hrv .to_excel(writer, sheet_name='2_HRV_XAI',    index=False)
                df_mtrx.to_excel(writer, sheet_name='3_Metrik_Filter', index=False)
                df_ref .to_excel(writer, sheet_name='4_Referensi',   index=False)
            messagebox.showinfo('✅ Ekspor Berhasil',
                                f'Tersimpan:\n{fnm}\n\n4 Sheet: Sinyal, HRV+XAI, Metrik Filter, Referensi\n\n'
                                f'💡 Tip: file ini bisa di-import lagi lewat tombol\n'
                                f'"📂 Import Dataset CSV (Kaggle)" untuk melihat ulang sinyalnya.')
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def cmd_esp32(self):
        # Membuka jendela popup untuk memilih port COM dan menghubungkan ke ESP32.
        if self.running: self.cmd_stop()
        pop = tk.Toplevel(self.root)
        pop.title('Koneksi ESP32'); pop.geometry('400x200')
        pop.configure(bg=BG_MAIN); pop.grab_set()
        tk.Label(pop, text='Hubungkan ESP32 / AD8232',
                 font=('Segoe UI',11,'bold'), bg=BG_MAIN).pack(pady=(14,4))
        ports = [p.device for p in serial.tools.list_ports.comports()] or ['(tidak ada)']
        sel = tk.StringVar(value=ports[0])
        tk.Label(pop, text='Port COM:', font=('Segoe UI',9), bg=BG_MAIN).pack()
        tk.OptionMenu(pop, sel, *ports).pack(pady=4)
        res = tk.Label(pop, text='', font=('Segoe UI',9,'bold'), bg=BG_MAIN); res.pack(pady=4)
        def do_conn():
            res.config(text='Menghubungkan...', fg='#F39C12'); pop.update()
            # Tutup koneksi ESP32 lama (bila ada) agar port tidak terkunci & thread lama berhenti
            if isinstance(self.source, ESP32Reader):
                self.source.stop()
            rd = ESP32Reader(port=sel.get())
            if rd.connect():
                rd.start(); self.source=rd; self.mode='esp32'
                res.config(text=f'✓ Terhubung: {sel.get()}', fg='#27AE60')
                self._set_status(f'🔌 ESP32 terhubung: {sel.get()}')
                pop.after(1500, pop.destroy)
            else:
                res.config(text='✗ Gagal — cek kabel & port (tutup Serial Monitor Arduino)', fg='#E74C3C')
        tk.Button(pop,text='Hubungkan',command=do_conn,
                  font=('Segoe UI',10,'bold'),bg='#27AE60',fg='white',
                  relief='flat',width=16).pack(pady=4)

    def _on_playback_scroll(self, val):
        # Dipanggil saat slider playback digeser: memindahkan posisi baca dataset
        # dan menggambar ulang frame pada posisi baru tersebut.
        if self.mode == 'playback' and self.dataset_sig is not None:
            start = int(float(val))
            pos_s = start / FS
            self.lbl_play_pos.config(text=f'{pos_s:.1f} s')
            self._render_playback_frame(start)

    def _toggle_playback_visibility(self):
        if self.mode == 'playback':
            self._close_playback()
        else:
            self._reopen_playback()

    def _close_playback(self):
        self.f_play.pack_forget()
        if self.mode == 'playback':
            self.mode = 'esp32'
        if self.dataset_sig is not None:
            self.btn_show_playback.config(state='normal', bg='#95A5A6', text='▶ Tampilkan Playback')

    def _reopen_playback(self):
        if self.dataset_sig is None:
            messagebox.showinfo('Info', 'Belum ada data rekaman/dataset yang tersimpan untuk diputar.')
            return
        if self.running:
            self.cmd_stop()

        self.mode = 'playback'
        total_sampel = len(self.dataset_sig)
        max_start = max(0, total_sampel - WINDOW_DISP_PLAYBACK)
        self.slider_play.config(from_=0, to=max_start)

        idx = min(self.current_playback_idx, max_start)
        self.slider_play.set(idx)
        self.lbl_play_pos.config(text=f'{idx/FS:.1f} s')
        self.lbl_play_end.config(text=f'{total_sampel/FS:.1f} s')

        self.f_play.pack(fill='x', padx=10, pady=(2,0))
        self._render_playback_frame(idx)
        self._set_status('📂 Playback ditampilkan kembali dari posisi terakhir.')
        self.btn_show_playback.config(state='normal', bg='#E74C3C', text='✕ Tutup Playback')

    def _render_playback_frame(self, start_idx):
        # Mengambil potongan sinyal dataset pada posisi start_idx untuk jendela tampilan
        # (disp) dan jendela HRV (hrv_raw), memilih jendela 60 atau 300 detik secara otomatis.
        if self.dataset_sig is None: return
        self.current_playback_idx = start_idx
        end_idx = min(start_idx + WINDOW_DISP_PLAYBACK, len(self.dataset_sig))
        disp    = self.dataset_sig[start_idx:end_idx]
        if len(disp) < FS: return

        available_sec = end_idx / FS
        if available_sec >= STANDARD_HRV_WIN_SEC:
            hrv_start   = end_idx - FS * STANDARD_HRV_WIN_SEC
            hrv_win_sec = STANDARD_HRV_WIN_SEC
            hrv_mode    = 'standard'
        elif available_sec >= FAST_HRV_WIN_SEC:
            hrv_start   = end_idx - FS * FAST_HRV_WIN_SEC
            hrv_win_sec = FAST_HRV_WIN_SEC
            hrv_mode    = 'ultra_short'
        else:
            hrv_start   = 0
            hrv_win_sec = end_idx / FS
            hrv_mode    = 'ultra_short'

        hrv_raw = (
            self.dataset_sig[hrv_start:end_idx]
            if hrv_mode != 'belum_cukup'
            else None
        )
        self.active_hrv_mode = hrv_mode
        self._process_and_draw(
            disp=disp,
            hrv_raw=hrv_raw,
            is_realtime=False,
            skip_hrv=False,
            hrv_win_sec=hrv_win_sec,
            hrv_mode=hrv_mode
        )

    def _select_hrv_window(self, raw_full):
        """
        Memilih jendela HRV berdasarkan durasi data yang tersedia:
          - < 60 dtk  : ultra_short (memakai seluruh data yang ada, durasi aktual)
          - 60–299 dtk: ultra_short (jendela 60 detik)
          - >= 300 dtk : standard   (jendela 300 detik)
        Mengembalikan (hrv_raw, hrv_win_sec, hrv_mode).
        """
        durasi_tersedia = len(raw_full) / FS
        if durasi_tersedia >= STANDARD_HRV_WIN_SEC:
            hrv_raw = raw_full[-FS * STANDARD_HRV_WIN_SEC:]
            return hrv_raw, STANDARD_HRV_WIN_SEC, 'standard'
        if durasi_tersedia >= FAST_HRV_WIN_SEC:
            hrv_raw = raw_full[-FS * FAST_HRV_WIN_SEC:]
            return hrv_raw, FAST_HRV_WIN_SEC, 'ultra_short'
        # Data < 60 dtk: HRV tetap dihitung dari seluruh data yang sudah terkumpul,
        # durasi aktual (detik berlalu) ditampilkan di panel metrik.
        return raw_full, durasi_tersedia, 'ultra_short'

    def _rt_frame(self, _frame_idx):
        # Dipanggil oleh _rt_tick() tiap RT_INTERVAL_MS saat mode real-time berjalan.
        # Dua timer terpisah: fast (setiap 2 dtk) untuk jendela 60 dtk,
        # standard (setiap 5 dtk) untuk jendela 300 dtk, agar CPU tidak terbebani.
        if not self.running or not self.source: return
        raw_full = self.source.get_data()
        if len(raw_full) < FS * 2: return

        disp = raw_full[-min(WINDOW_DISP, len(raw_full)):]
        hrv_raw, hrv_win_sec, hrv_mode = self._select_hrv_window(raw_full)

        if hrv_mode == 'belum_cukup':
            self.active_hrv_mode = 'belum_cukup'
            self._process_and_draw(
                disp=disp,
                hrv_raw=None,
                is_realtime=True,
                skip_hrv=True,
                hrv_win_sec=0,
                hrv_mode='belum_cukup'
            )
            return

        now = time.time()
        if hrv_mode == 'standard':
            skip_hrv = (now - self.last_std_hrv_calc_t) < STANDARD_HRV_RECALC_SEC
            if not skip_hrv:
                self.last_std_hrv_calc_t = now
        else:
            skip_hrv = (now - self.last_fast_hrv_calc_t) < FAST_HRV_RECALC_SEC
            if not skip_hrv:
                self.last_fast_hrv_calc_t = now

        self.active_hrv_mode = hrv_mode
        self._process_and_draw(
            disp=disp,
            hrv_raw=hrv_raw,
            is_realtime=True,
            skip_hrv=skip_hrv,
            hrv_win_sec=hrv_win_sec,
            hrv_mode=hrv_mode
        )

    def _process_and_draw(self, disp, hrv_raw, is_realtime=True, skip_hrv=False,
                          hrv_win_sec=0, hrv_mode='belum_cukup'):
        # Fungsi inti yang menyatukan seluruh pipeline setiap frame:
        #   1) Denoiser.process()   -> pemfilteran (Tya), menghasilkan 5 tahap sinyal
        #   2) PanTompkins.detect() -> deteksi puncak R pada sinyal bersih (Toro)
        #   3) Analyzer.analyze()   -> fitur HRV + klasifikasi XAI (Toro), dihitung
        #                              pada jendela 60 dtk (ultra_short) atau 300 dtk (standard)
        #   4) Update seluruh elemen grafik & panel kanan sesuai hasil di atas.
        stages, metrics = self.denoiser.process(disp)
        clean_disp = stages['wavelet']
        peaks_disp = self.pt.detect(clean_disp)
        x = np.arange(len(disp))

        if hrv_raw is not None and not skip_hrv:
            # Jendela HRV dihitung terpisah dari jendela tampilan (disp), karena HRV
            # butuh durasi lebih panjang agar puncak R yang terdeteksi cukup banyak
            # untuk statistik yang bermakna. Disimpan ke cache sesuai mode aktif.
            hrv_stg, _ = self.denoiser.process(hrv_raw)
            hrv_peaks  = self.pt.detect(hrv_stg['wavelet'])
            feats_baru, info_baru = self.analyzer.analyze(
                hrv_stg['wavelet'], hrv_peaks,
                profile=self.user_profile.get(),
                window_sec_actual=hrv_win_sec
            )
            if feats_baru and info_baru:
                info_baru['hrv_mode'] = hrv_mode
                if hrv_mode == 'standard':
                    self.cached_std_feats = feats_baru
                    self.cached_std_info  = info_baru
                elif hrv_mode == 'ultra_short':
                    self.cached_fast_feats = feats_baru
                    self.cached_fast_info  = info_baru

        # Ambil cache sesuai mode aktif — 300 dtk diprioritaskan bila sudah tersedia
        if hrv_mode == 'standard':
            feats = self.cached_std_feats
            info  = self.cached_std_info
        elif hrv_mode == 'ultra_short':
            feats = self.cached_fast_feats
            info  = self.cached_fast_info
        else:
            feats = None
            info  = None

        # Tab yang sedang terlihat: hanya figure pada tab ini yang digambar ulang (hemat CPU).
        sel = self.notebook.index(self.notebook.select())

        if self.ax1 and self.ln_clean:
            self.ln_clean.set_data(x, clean_disp)
            self.ax1.set_title(
                f'Sinyal EKG Bersih (Wavelet) — {len(disp)/FS:.0f} detik',
                fontsize=9, fontweight='bold', color=TEXT_DARK, pad=6)
            if self.sc_r:
                if self.show_rpeak_tab1.get() and len(peaks_disp):
                    self.sc_r.set_offsets(np.column_stack([peaks_disp, clean_disp[peaks_disp]]))
                    self.sc_r.set_visible(True)
                else:
                    self.sc_r.set_visible(False)
            if len(clean_disp):
                pad = max((np.max(clean_disp)-np.min(clean_disp))*0.18, 0.15)
                self.ax1.set_xlim(0, len(disp))
                self.ax1.set_ylim(np.min(clean_disp)-pad, np.max(clean_disp)+pad)

        if self.ax2 and not skip_hrv and info and len(info['rr_ms']) > 1:
            # Judul histogram dinamis sesuai jendela aktif
            mode_label = (
                f'Short-Term ({STANDARD_HRV_WIN_SEC} dtk)'
                if hrv_mode == 'standard'
                else f'Ultra-Short-Term ({hrv_win_sec:.0f} dtk)'
            )
            self.ax2.cla()
            self._style_ax(self.ax2, f'Distribusi R-R Interval — {mode_label}', 'Frekuensi')
            self.ax2.set_xlabel('R-R Interval (ms)', fontsize=8, color=TEXT_DARK)
            rr = info['rr_ms']
            self.ax2.hist(rr, bins=min(15,len(rr)), color=LINE_RR, alpha=0.85, ec='white', lw=0.7)
            self.ax2.axvline(np.mean(rr), color=R_COLOR, ls='--', lw=1.5,
                             label=f'Mean={np.mean(rr):.0f} ms')
            self.ax2.legend(fontsize=8)

        if self.ax3 and self.ln_hr and is_realtime and not skip_hrv and feats:
            elapsed = round(time.time()-self.t_start, 1)
            self.hr_history.append(feats['HR'])
            self.t_history .append(elapsed)
            self.ln_hr.set_data(np.array(self.t_history), np.array(self.hr_history))
            self.ax3.set_xlim(max(0,elapsed-120), elapsed+2)
            ha = np.array(self.hr_history)
            if len(ha): self.ax3.set_ylim(max(30,ha.min()-10), min(200,ha.max()+15))

        # Menggambar ulang tiap tahap filter yang sedang aktif di Tab 2 (pekerjaan Tya).
        # Hanya dikerjakan bila Tab 2 sedang terlihat.
        if sel == 1:
            sig_list = [stages['raw'],stages['notch'],stages['bandpass'],stages['ma'],stages['wavelet']]
            for pi, oi in enumerate(self._active_t):
                if pi >= len(self.lines_tab2): break
                rs = sig_list[oi]
                self.lines_tab2[pi].set_data(x, rs)
                if len(rs):
                    p2 = max((np.max(rs)-np.min(rs))*0.15, 0.1)
                    self.axes_tab2[pi].set_xlim(0, len(disp))
                    self.axes_tab2[pi].set_ylim(np.min(rs)-p2, np.max(rs)+p2)
            self.canvas2.draw_idle()
        else:
            self.canvas1.draw_idle()

        if not skip_hrv and hrv_raw is not None and not feats and hrv_win_sec > 0:
            self.rv_window.config(text=f'{hrv_win_sec:.0f} dtk')
            self._set_text(self.rv_validity,
                           f'Mengumpulkan data... {hrv_win_sec:.0f} detik berlalu, '
                           f'menunggu minimal 4 puncak R untuk menghitung HRV.')

        if not skip_hrv and feats and info:
            self._update_right_toro(feats, info)
            self._update_right_tya(metrics)
            if is_realtime: self._log_data(feats, info, metrics)

    def _update_right_toro(self, f, info):
        # Menyalin hasil Analyzer (fitur HRV, label XAI, narasi, flag) ke widget panel kanan.
        self.rv_hr   .config(text=f"{f['HR']} bpm")
        self.rv_rr   .config(text=f"{f['Mean_RR']} ms")
        self.rv_sdnn .config(text=f"{f['SDNN']} ms")
        self.rv_rmss .config(text=f"{f['RMSSD']} ms")
        self.rv_pnn  .config(text=f"{f['pNN50']} %")
        self.rv_reg  .config(text=f"{f['Reg']} %")
        self.rv_nbeat.config(text=str(f['N_Beats']))
        self.rv_window.config(text=f"{f['Window_Sec']:.0f} dtk" if f['Window_Sec'] else '—')
        self._set_text(self.rv_validity, info['validity'])
        lbl = info['label']
        if 'RENTANG OPERASIONAL' in lbl or 'NORMAL' in lbl:
            clr = '#27AE60'   # hijau — parameter baik
        elif 'PERINGATAN' in lbl or 'KELAINAN' in lbl or 'EVALUASI KESEHATAN' in lbl:
            clr = '#C0392B'   # merah — perlu perhatian
        elif 'ESTIMASI AWAL' in lbl:
            clr = '#D68910'   # oranye-emas — estimasi, belum cukup durasi
        else:
            clr = '#E67E22'   # oranye — batas/observasi
        self.rv_label.config(text=lbl, fg=clr)
        self.rv_skor .config(text=f"Skor Kesesuaian: {info['pct']:.1f}%")
        self._set_text(self.rv_naratif, info['naratif'])
        self._set_text(self.rv_flags,
                       '\n\n'.join(info['flags']) if info['flags']
                       else '✅ Tidak ada flag klinis terdeteksi.')

    def _update_right_tya(self, metrics):
        # Menyalin metrik kualitas sinyal (SNR/PRD tahap wavelet & bandpass) ke panel kanan Tya.
        if not metrics: return
        self.rv_snr   .config(text=f"{metrics['wavelet'][0]} dB")
        self.rv_mse   .config(text=str(metrics['wavelet'][1]))
        self.rv_prd   .config(text=f"{metrics['wavelet'][2]} %")
        self.rv_snr_bp.config(text=f"{metrics['bandpass'][0]} dB")
        self.rv_prd_bp.config(text=f"{metrics['bandpass'][2]} %")

    def _log_data(self, f, info, metrics):
        # Mencatat satu baris data (fitur HRV+XAI Toro dan metrik filter Tya) ke
        # dua list terpisah, untuk kemudian diekspor bersama lewat cmd_export().
        t = round(time.time()-self.t_start, 1)
        self.export_hrv.append({
            'Waktu_s': t, 'Profil': self.user_profile.get().upper(),
            'HR_bpm': f['HR'], 'Mean_RR_ms': f['Mean_RR'],
            'SDNN_ms': f['SDNN'], 'RMSSD_ms': f['RMSSD'],
            'pNN50_%': f['pNN50'], 'Reg_%': f['Reg'],
            'N_Beats': f['N_Beats'], 'Win_sec': f['Window_Sec'],
            'Label': info['label'], 'Skor_%': info['pct'],
            'Flags': ' | '.join(info['flags']) if info['flags'] else '—',
            'Narasi_XAI': info['naratif'],
        })
        self.export_mtrx.append({
            'Waktu_s': t,
            'SNR_Notch': metrics['notch'][0],   'PRD_Notch': metrics['notch'][2],
            'SNR_BP':    metrics['bandpass'][0], 'PRD_BP':   metrics['bandpass'][2],
            'SNR_MA':    metrics['ma'][0],       'PRD_MA':   metrics['ma'][2],
            'SNR_Wave':  metrics['wavelet'][0],  'PRD_Wave': metrics['wavelet'][2],
        })

    def _set_text(self, widget, txt):
        # Fungsi bantu untuk mengisi widget Tkinter Text yang bersifat read-only (disabled).
        widget.config(state='normal')
        widget.delete('1.0','end')
        widget.insert('end', txt)
        widget.config(state='disabled')

    def _reset_right_panel(self):
        for a in ['rv_hr','rv_rr','rv_sdnn','rv_rmss','rv_pnn','rv_reg','rv_nbeat','rv_window']:
            getattr(self, a).config(text='—')
        self.rv_label.config(text='Menunggu sinyal...', fg=TEXT_DARK)
        self.rv_skor .config(text='Skor Kesesuaian: —')
        for w in [self.rv_validity, self.rv_naratif, self.rv_flags]:
            self._set_text(w, '—')
        for a in ['rv_snr','rv_mse','rv_prd','rv_snr_bp','rv_prd_bp']:
            getattr(self, a).config(text='—')

    def _set_status(self, msg):
        self.lbl_status.config(text=f'Status: {msg}')

    def on_close(self):
        self.cmd_stop()
        if isinstance(self.source, ESP32Reader):
            self.source.stop()
        plt.close('all'); self.root.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    root.state('zoomed')
    try:
        import ctypes; ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except: pass
    app = ECGApp(root)
    root.protocol('WM_DELETE_WINDOW', app.on_close)
    root.mainloop()
