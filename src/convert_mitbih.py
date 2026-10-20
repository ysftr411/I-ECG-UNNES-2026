import wfdb
import pandas as pd
import os

RAW_DIR = "data/mitdb_raw"
OUTPUT_DIR = "data"
RECORDS = ['100', '101', '102', '103', '104']

print("Mulai mengkonversi ke CSV...\n")

for rec_id in RECORDS:
    print(f"Memproses rekaman {rec_id}...")
    try:
        # Membaca rekaman WFDB
        record = wfdb.rdrecord(f"{RAW_DIR}/{rec_id}")
        
        # Mengambil lead pertama (biasanya MLII)
        signal = record.p_signal[:, 0]
        
        # Membuat DataFrame dengan nama kolom yang dikenali oleh GUI Anda
        df = pd.DataFrame({
            'time_s': [i / record.fs for i in range(len(signal))],
            'Raw_mV': signal
        })
        
        # Simpan ke folder data/
        output_file = f"{OUTPUT_DIR}/mitbih_{rec_id}.csv"
        df.to_csv(output_file, index=False)
        
        print(f"  ✓ Berhasil: {output_file} ({len(signal)} baris)")
    except Exception as e:
        print(f"  ✗ Gagal memproses {rec_id}: {e}")

print("\n✅ Selesai! Semua file CSV siap digunakan.")