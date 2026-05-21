# fuzzy-eco-Guardian
# 🌡️ IT2-FuzzyPID Thermal Management — PLTS Inverter SIL Simulation

> **Evaluasi Tengah Semester — Pemrograman Kontroller**
> Kelompok 9 | Prodi D4 Teknologi Rekayasa Instrumentasi | Institut Teknologi Sepuluh Nopember | 2026

---

## 📌 Deskripsi Proyek

Repositori ini berisi implementasi lengkap sistem **manajemen termal real-time berbasis Interval Type-2 Fuzzy-PID** untuk inverter Panel Listrik Tenaga Surya (PLTS), yang divalidasi melalui pendekatan **Software-in-the-Loop (SIL)** menggunakan Proteus 8 Professional dan mikrokontroler virtual **STM32F401VE**.

Sistem ini dirancang untuk menjawab 15 *research gap* dari literatur jurnal internasional bereputasi (Scopus/WoS, 2021–2026) yang merekomendasikan validasi algoritma kendali fuzzy pada perangkat keras tertanam (*embedded*) berbiaya rendah secara real-time. Algoritma IT2 Fuzzy-PID dieksekusi sepenuhnya dalam bahasa C (bare-metal) tanpa RTOS komersial, menjadikannya sangat efisien dalam penggunaan memori dan latensi komputasi.

---

## ✨ Fitur Utama

- **Interval Type-2 Fuzzy Logic** dengan mekanisme *Footprint of Uncertainty* (FOU) untuk menangani ketidakpastian pembacaan sensor secara otomatis
- **Adaptive Setpoint** yang menyesuaikan target suhu secara dinamis berdasarkan intensitas iradiasi matahari (sensor LDR)
- **Type Reduction Karnik-Mendel** untuk defuzzifikasi dua jalur (upper & lower membership function)
- **Anti-windup PID** dengan pembatasan integral pada rentang ±300
- **PWM 100 Hz** untuk kendali kecepatan motor DC kipas pendingin secara proporsional
- **Monitoring real-time** via UART (Virtual Terminal), LCD I2C 16×2, dan Oscilloscope virtual
- **Ekspor data CSV** untuk analisis grafik ilmiah menggunakan GNUPlot
- **Indikator LED** (hijau/merah) untuk status beban pendinginan

---

## 🏗️ Arsitektur Sistem

```
Panel Surya → Inverter PLTS → Disipasi Panas
                                    ↓
                          [Sensor LM35 + LDR]
                                    ↓
                        [STM32F401VE — ADC 12-bit]
                                    ↓
                    [Algoritma IT2 Fuzzy-PID (C bare-metal)]
                                    ↓
               [PWM Generator] → [Motor Driver L298] → [Fan DC]
                    ↓                    ↓
            [UART Terminal]         [LCD I2C]
            [Oscilloscope]          [LED Indikator]
```

**6 Blok Fungsional:** Input Sensor → Mikrokontroler → Aktuator → Display → Monitoring → Analisis Data

---

## 🛠️ Perangkat Lunak & Teknologi

| Perangkat Lunak | Versi | Fungsi |
|---|---|---|
| **Proteus Professional** | 8 | Platform simulasi SIL (VSM) |
| **Keil µVision** | 5 | IDE & kompilasi firmware C (ARM Compiler 6) |
| **STM32CubeMX** | — | Konfigurasi periferal STM32 secara grafis |
| **GNUPlot** | — | Visualisasi grafik ilmiah dari data CSV |

---

## ⚙️ Spesifikasi Mikrokontroler

| Parameter | Nilai |
|---|---|
| Mikrokontroler | STM32F401VE (ARM Cortex-M4 + FPU) |
| Clock | HSI 16 MHz (tanpa PLL) |
| Flash / RAM | 512 KB / 96 KB |
| Resolusi ADC | 12-bit (0–4095) |
| Frekuensi PWM | 100 Hz (Prescaler 159, Period 999) |
| UART | USART2, 4800 baud |
| Display | Soft I2C — PCF8574 + LCD 16×2 |

---

## 🔬 Algoritma: Interval Type-2 Fuzzy-PID

### Fuzzifikasi
Dua variabel input (suhu & cahaya) dipetakan ke 3 himpunan linguistik: `LOW`, `MEDIUM`, `HIGH`, masing-masing dengan **Upper MF** dan **Lower MF** membentuk FOU.

### Evaluasi Aturan
9 aturan IF-THEN (3 suhu × 3 cahaya) dievaluasi dengan operasi MIN untuk mendapatkan *firing strength* pada jalur upper dan lower.

### Type Reduction (Karnik-Mendel)

$$y_{upper} = \frac{\sum w_U \cdot g}{\sum w_U}, \quad y_{lower} = \frac{\sum w_L \cdot g}{\sum w_L}$$

$$\text{fuzzy\_gain} = \frac{y_{upper} + y_{lower}}{2}$$

### Adaptive Setpoint

$$\text{adaptive\_sp} = 50 + (500 - \text{cahaya}) \times 0.02 \quad (°C)$$

### Sinyal Kendali PID

$$U = \text{fuzzy\_gain} \times \left(K_p \cdot e + K_i \int e \, dt + K_d \frac{de}{dt}\right)$$

**Parameter awal:** `Kp = 4.0`, `Ki = 0.02`, `Kd = 1.0` | **Saturasi output:** 0–999

---

## 📊 Hasil Pengujian Simulasi

### Kondisi Normal (T = 33,9°C, L = 369 Lux)
- Adaptive setpoint: **52,62°C**
- PWM output: **337** (~33,7% duty cycle) → LED Hijau aktif
- Motor berputar pada kecepatan rendah, konsumsi daya minimal

### Kondisi Beban Puncak (T = 74,9°C, L = 749 Lux)
- Adaptive setpoint: **45,02°C** — error: **+29,88°C**
- PWM output: **853–999** (~97–100% duty cycle) → LED Merah aktif
- Kipas beroperasi pada kecepatan maksimum

### Respons Transisi (T: 67°C → 29°C secara mendadak)
- PWM jatuh dari **850** → **2** dalam satu siklus sampling (respons instan)
- Sistem stabil kembali pada PWM **118** (1,4V) setelah error diredam
- Demonstrasi **zero overshoot recovery** berkat mekanisme derivative + FOU

---

## 📁 Struktur Repositori

```
├── firmware/
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c              # Loop utama & task scheduling
│   │   │   ├── it2_fuzzy_pid.c     # Implementasi algoritma IT2 Fuzzy-PID
│   │   │   └── lcd_i2c.c           # Driver LCD via PCF8574
│   │   └── Inc/
│   │       ├── it2_fuzzy_pid.h
│   │       └── lcd_i2c.h
│   └── output/
│       └── firmware.hex            # File hex untuk Proteus
├── proteus/
│   └── thermal_management.pdsprj  # Skema rangkaian Proteus
├── gnuplot/
│   ├── data/
│   │   └── log_data.csv            # Data log dari Virtual Terminal
│   └── plot_response.gp            # Script GNUPlot
├── docs/
│   ├── block_diagram.png
│   ├── flowchart.png
│   └── laporan_ETS.pdf
└── README.md
```

---

## 🚀 Cara Menjalankan Simulasi

1. **Clone repositori ini**
   ```bash
   git clone https://github.com/username/it2-fuzzypid-thermal-plts.git
   ```

2. **Kompilasi firmware** menggunakan Keil µVision
   - Buka project di `firmware/`
   - Build → output: `firmware/output/firmware.hex`

3. **Jalankan simulasi Proteus**
   - Buka `proteus/thermal_management.pdsprj`
   - Pastikan path `.hex` di properti STM32F401VE sudah mengarah ke file hasil kompilasi
   - Klik **Run** (▶)

4. **Analisis data**
   - Salin log dari Virtual Terminal ke `gnuplot/data/log_data.csv`
   - Jalankan script GNUPlot:
     ```bash
     gnuplot gnuplot/plot_response.gp
     ```

---

## 📚 Referensi Jurnal Utama

Sistem ini dibangun berdasarkan sintesis 15 jurnal internasional bereputasi (Scopus/WoS, 2021–2026), antara lain:

- Liu et al. (2026) — STM32 intelligent sensor platform
- Chatzipapas & Karnavas (2025) — IT2 Fuzzy Logic pada mikrokontroler berbiaya rendah
- Sain & Lee (2026) — Novel IT2 Takagi-Sugeno Fuzzy PID (MagLev case study)
- Mezouari et al. (2025) — Real-time Fuzzy Logic Control pada konverter daya
- Yang et al. (2025) — PSO-Fuzzy pada STM32 untuk battery management

> Daftar lengkap 15 referensi tersedia di `docs/laporan_ETS.pdf`

---

## 👨‍💻 Tim Pengembang

| Nama | NRP |
|---|---|
| Mahendra Dwi Fahreza | 2042241023 |
| Athaya Zhafari Saputra | 2042241019 |

**Kelas 4B** — Prodi D4 Teknologi Rekayasa Instrumentasi
Departemen Teknik Instrumentasi, Fakultas Vokasi
**Institut Teknologi Sepuluh Nopember (ITS)** — 2026

**Dosen Pengampu:** Ahmad Radhy, S.Si., M.Si

---

## 📄 Lisensi

Proyek ini dibuat untuk keperluan akademik (Evaluasi Tengah Semester). Penggunaan ulang kode untuk tujuan pembelajaran diperbolehkan dengan mencantumkan atribusi kepada tim pengembang.

---

*#teknikInstrumentasi #STM32 #FuzzyLogic #EmbeddedSystems #PLTS #SoftwareInTheLoop #ITS*
