# Algoritma Pan-Tompkins

## 5 Tahap

1. **Filter Diferensial** — kernel [1, 2, 0, -2, -1] × (1/8)·f_s
2. **Pengkuadratan** — y[n] = (x[n])²
3. **Moving Window Integration** — jendela 54 sampel (150 ms)
4. **Adaptive Thresholding** — 5% × max
5. **Peak Refinement** — ±18 sampel (±50 ms)

## Parameter

| Parameter | Nilai |
|-----------|-------|
| Frekuensi sampling | 360 Hz |
| Jendela MWI | 54 sampel |
| Ambang | 5% |
| Refrakter | 72 sampel (200 ms) |