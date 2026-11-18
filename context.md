Buat aplikasi GCS (Ground Control Station) robot bawah air dengan Python 3.10+
dan Tkinter. Satu file, satu class App(tk.Tk), komentar singkat bahasa
Indonesia.

BATASAN LIBRARY (WAJIB):
Hanya bawaan Python: tkinter, tkinter.ttk, messagebox, filedialog, csv, json,
datetime, random, math. Satu-satunya library luar yang boleh: matplotlib,
di-import dalam try-except (jika tidak ada, tampilkan pesan dan lewati,
program tetap jalan). Tanpa pyserial, threading, socket, OpenCV, YOLO.
Semua data robot DISIMULASIKAN dengan random dan after().

ATURAN OUTPUT (PENTING):
- Beri SATU FILE LENGKAP yang langsung bisa dijalankan tanpa edit.
- Jangan dipotong, jangan tulis "...sisanya sama" atau placeholder.
- Studi kasus di bawah SELALU menang atas deskripsi umum ini. Jika ada
  perbedaan, ikuti studi kasus.
- Jangan tambah fitur di luar studi kasus dan daftar di bawah.
- Kerjakan sesuai urutan PRIORITAS. Jika terlalu panjang, kurangi prioritas
  paling bawah, bukan yang di atasnya.

STUDI KASUS DARI SOAL:
Studi Kasus Latihan: ROV Thruster Control & Sensor Monitoring Dashboard
Deskripsi Project: Saat ROV diturunkan ke dalam kolam pengujian, operator di darat tidak hanya perlu mengendalikan pergerakan motor penggerak (thruster), tetapi juga wajib memantau kondisi vital wahana. Saat ini, sistem kontrol dan pemantauan sensor masih terpisah dan berbasis command line, sehingga sangat tidak intuitif. Anda ditugaskan untuk merancang sebuah aplikasi antarmuka grafis (GUI) menggunakan Python yang menggabungkan kontrol mekanis dan pemantauan telemetri ke dalam satu layar "Dashboard Terintegrasi".
Aplikasi ini harus terbagi menjadi dua fungsi utama dalam satu layar:
Fungsi Monitoring Sensor: Terdapat area khusus di mana penguji (sebagai simulator sensor) dapat memasukkan nilai Suhu Air (°C), Kedalaman (meter), dan Sisa Baterai (%).
Fungsi Kontrol Thruster: Operator dapat memilih spesifik thruster mana yang akan diuji (misalnya: Thruster Kiri Depan, Kanan Belakang, atau Vertikal). Disediakan juga kontrol visual yang intuitif (seperti slider, bukan sekadar mengetik angka) untuk mengatur kecepatan motor dari 0 hingga 100%, serta opsi arah putaran motor (Maju atau Mundur).
Di dalam antarmuka tersebut, harus tersedia tombol "Kirim Perintah" (Send Command) ke motor, dan sebuah tombol darurat "KILL SWITCH / EMERGENCY STOP" yang terlihat sangat menonjol. Setiap kali perintah berhasil dieksekusi, atau ketika tombol darurat ditekan, sistem harus memunculkan riwayat aktivitas tersebut secara real-time di sebuah area log (text area), lengkap dengan status thruster, kecepatan, arah, serta pembacaan suhu dan baterai saat itu.
Karena ini berkaitan dengan perangkat keras berharga, sistem keamanan (safety interlock) adalah prioritas mutlak. Sistem Anda harus memblokir perintah dan memunculkan jendela pesan peringatan (Messagebox) apabila:
Teknisi menekan "Kirim Perintah" tetapi belum memilih thruster mana pun.
Teknisi langsung menyetel kecepatan motor secara ekstrem ke 100% dari posisi 0% secara tiba-tiba (mencegah lonjakan arus listrik).
[Peringatan Sensor]: Nilai Suhu yang dimasukkan berada di atas 45°C ATAU sisa Baterai berada di bawah 15%. (Cegah motor menyala dalam kondisi ekstrem ini agar komponen tidak terbakar/mati mendadak di kedalaman).
Jika tombol darurat (KILL SWITCH) ditekan, indikator kecepatan di layar harus seketika kembali ke angka nol dan log mencetak peringatan merah bahwa sistem telah dihentikan secara paksa.
Tugas Peserta:
Analisis seluruh kebutuhan kontrol dan monitoring di atas, lalu tentukan secara mandiri komponen widget GUI apa yang paling efisien (misal: Dropdown untuk thruster, Slider untuk kecepatan, Radio Button untuk arah, dan Entry/Spinbox untuk simulasi nilai sensor).
Rancang tata letak (layout) GUI sedemikian rupa agar area "Monitoring" dan "Kontrol" tertata rapi, logis, dan tidak membingungkan operator saat kondisi darurat.
Implementasikan GUI beserta logika programnya (event handling) untuk memastikan pembacaan sensor dapat memblokir perintah yang membahayakan thruster.
Terapkan validasi input (misalnya memastikan input kedalaman dan suhu diisi dengan angka, bukan huruf) agar aplikasi tidak error/crash.
Selesaikan aplikasi ini dalam waktu 60 menit dan bersiaplah untuk mendemonstrasikan kelancaran UI/UX serta alur kode program di depan penguji.



FITUR WAJIB DARI SOAL (jangan ada yang terlewat):
1. [...]
2. [...]
3. [...]


UNTUK ISI GCS UtaMAKAN YANG DI SOAL, jangan buat d luar soal

=== PRIORITAS 1: KESESUAIAN, FUNGSI, VALIDASI ===
- Pakai widget yang relevan: Label, Entry, Combobox, Checkbutton, Button,
  Treeview/Listbox, Canvas, messagebox.
- Setiap tombol terhubung ke fungsi callback (command=).
- Validasi input: try-except + messagebox untuk input kosong, bukan angka,
  atau di luar rentang. Program tidak boleh crash karena input salah.
- Pengolahan data memakai if/for/fungsi, list, dan dictionary; hasil tampil
  di tabel atau log.
- Telemetri simulasi (depth, heading, pitch, roll, baterai turun pelan, suhu)
  diperbarui tiap 500 ms memakai after(). Tidak ada sleep atau while True.
- Indikator status berwarna: NORMAL (hijau), WARNING (kuning), BAHAYA (merah)
  sesuai batas dari soal.
- Indikator LINK OK / LINK LOST berdasarkan selisih waktu data terakhir
  (datetime): lebih dari 2 detik = LOST (merah). Sediakan tombol
  "Putuskan Simulasi" untuk mengujinya. Saat LOST, tombol kontrol nonaktif.
- Tombol ARM/DISARM dan tombol Reset.

=== PRIORITAS 2: MULTIPAGE, LOG, COMMAND, CSV, PORT ===
- ttk.Notebook dengan 3 halaman:
  1. Dashboard: tile telemetri (judul kecil, nilai besar, satuan), indikator
     status, LINK; bagian yang bisa di-scroll (Canvas + Scrollbar) jika
     kontennya banyak.
  2. Kontrol & Log:
     - Combobox port (isi: "SIMULASI", "COM1", "COM3", "COM5") + tombol
       Connect/Disconnect. Hanya "SIMULASI" yang berfungsi; port lain
       menampilkan messagebox "Port tidak tersedia pada mode simulasi".
     - Tombol arah (Maju, Mundur, Kiri, Kanan, Naik, Turun) dan STOP, aktif
       hanya saat sudah Connect, ARM, dan LINK OK.
     - Entry command + tombol Kirim (command dicatat dan dibalas "ACK"
       simulasi).
     - Text/Listbox serial log berisi TX/RX ber-timestamp, ada scrollbar,
       dan tombol Bersihkan Log.
  3. Data: Treeview log telemetri (maksimal 500 baris, scrollbar), tombol
     Simpan CSV (filedialog, ada header), dan tombol Reset.
- Baris pesan status di bawah window ("Siap", "Terhubung (simulasi)", dll).

=== PRIORITAS 3: FITUR LANJUTAN (SIMULASI, jika P1 dan P2 beres) ===
- Halaman "Grafik": grafik depth dan baterai terhadap waktu (matplotlib,
  FigureCanvasTkAgg), maksimal 100 titik terakhir, mengisi ruang tersedia.
- Halaman "Peta": Canvas tampak atas, gambar posisi robot dan jejak
  (trajectory) dari heading dan kecepatan simulasi (dead reckoning
  sederhana), ikon robot mengikuti heading, tombol Clear Jejak.
- Halaman "Simulasi": Canvas 2D dua pandangan, tampak atas (berputar sesuai
  heading) dan tampak samping (miring sesuai pitch), bentuk kapal selam dari
  oval dan poligon sederhana. Bukan 3D asli.
- Halaman "Kamera": Canvas dengan HUD (heading, depth, status LINK) dan kotak
  deteksi SIMULASI acak berlabel (misalnya "objek 0.87"). Beri komentar
  bahwa bagian ini tempat memasang detektor sungguhan nanti.

=== TAMPILAN (semua halaman) ===
- Layout rapi dan konsisten: ttk.LabelFrame untuk mengelompokkan; grid untuk
  form (label kiri, input kanan, lebar kolom konsisten); jangan campur pack
  dan grid dalam satu container.
- TANPA area kosong berlebih: fill="both" + expand=True (pack) atau
  sticky="nsew" + weight=1 (grid); padding maksimal 6 px; Treeview, Text,
  Canvas, dan grafik mengembang mengikuti window.
- Window 1000x650 dengan minsize; tetap rapi saat di-maximize.
- Font seragam (satu keluarga, 2-3 ukuran). Pakai tema gelap sederhana
  (konstanta warna di atas file) selama teks tetap terbaca.
- Kode PEP 8, nama jelas, fungsi pendek, tanpa trailing whitespace.
- Simpan log ke JSON selain CSV (filedialog).