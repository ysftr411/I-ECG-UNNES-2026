import urllib.request
import os

OUTPUT_DIR = "data/mitdb_raw"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Link server Zenodo (sangat stabil, tidak akan 404)
BASE_URL = "https://zenodo.org/records/3477543/files/"

records = ['100', '101', '102', '103', '104']
extensions = ['dat', 'hea', 'atr']

print("Memulai download dari server Zenodo...\n")

for rec in records:
    print(f"Mengunduh rekaman {rec}...")
    for ext in extensions:
        filename = f"{rec}.{ext}"
        # Tambahkan parameter ?download=1 agar langsung terunduh
        url = f"{BASE_URL}{filename}?download=1"
        filepath = os.path.join(OUTPUT_DIR, filename)
        
        try:
            urllib.request.urlretrieve(url, filepath)
            print(f"  ✓ {filename}")
        except Exception as e:
            print(f"  ✗ Gagal {filename}: {e}")
    print("-" * 20)

print("\n✅ Selesai! Silakan cek folder data/mitdb_raw")