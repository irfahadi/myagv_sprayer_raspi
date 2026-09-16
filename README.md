# myagv_sprayer_raspi

Workspace ROS 2 Humble untuk **Raspberry Pi 4B di dalam myAGV** (Elephant
Robotics) — sisi *sensor dan roda* robot penyiram greenhouse. Semua pengambilan
keputusan navigasi ada di Jetson Nano
([`myagv_sprayer_jetson`](../myagv_sprayer_jetson)); antarmuka bersama ada di
[`myagv_sprayer_msgs`](../myagv_sprayer_msgs).

Basis kode diadaptasi dari
[xaatim/beam_agrobot_v2](https://github.com/xaatim/beam_agrobot_v2) dan
[Abhishek1010006/Mobile-Manipulator-Robot-Simulation](https://github.com/Abhishek1010006/Mobile-Manipulator-Robot-Simulation).

## Yang berubah dari versi sebelumnya

**Nav2 dihapus seluruhnya.** Perencanaan jalur pindah ke Jetson, ke satu node yang
menampung VFH-QL, CQL, SARSA, A* dan D* Lite di belakang satu ruang aksi yang
sama — syarat agar kelimanya bisa dibandingkan secara setara. Yang tersisa di
sini adalah peta, lokalisasi, sensor, dan roda.

Karena jalur perintah kini melintasi LAN, ditambahkan
**`cmd_vel_watchdog_node`**: satu-satunya publisher `/cmd_vel`. Ia merelai
`/cmd_vel_nav`, membatasi kecepatan, meredam percepatan, dan **menolkan roda
dalam 0,5 detik** kalau aliran perintah dari Jetson berhenti. Link putus →
berhenti, bukan meluncur dengan perintah terakhir.

```
        LAN 192.168.10.0/24 · ROS_DOMAIN_ID 42 · CycloneDDS di eth0
   ┌────────────────────────────────┐        ┌──────────────────────────────────┐
   │  Raspberry Pi (myAGV)  .10     │        │  Jetson Nano            .20      │
   │                                │        │                                  │
   │  driver myAGV  → /odom, TF     │        │  nav_controller_node             │
   │  ydlidar       → /scan  ───────┼───────▶│  vfh_ql·cql·sarsa·astar·dstar    │
   │  camera_stream → JPEG   ───────┼───────▶│  aruco_detector + visual_servo   │
   │  map_server    → /map   ───────┼───────▶│  tray_detector (identitas)       │
   │  localization  → TF map→odom   │        │  relay · tank · spray_manager    │
   │  cmd_vel_watchdog ◀── /cmd_vel_nav ─────┤  mission_node                    │
   │      └→ /cmd_vel → roda        │        │                                  │
   └────────────────────────────────┘        └──────────────────────────────────┘
```

## Paket

| paket | isi |
|---|---|
| `myagv_sprayer_description` | URDF/xacro myAGV (4 roda mecanum, LiDAR, kamera di puncak tiang nozzle) + payload sprayer (LiPo, buck, relai, Jetson, tangki 1 L + HC-SR04, pompa, nozzle TeeJet) |
| `myagv_sprayer_gazebo` | World **`nursery_room.world`** ruang persemaian 8 × 6 m sesuai denah, **plus marker ArUco di muka tiap baki**, dan `scenario_spawner_node` untuk menaruh rintangan Skenario 2 (persisten/transient) saat runtime |
| `myagv_sprayer_navigation` | `map_server_node` (pembaca .pgm/.yaml sendiri, tanpa Nav2), `localization_node` (identity / **drift** untuk eksperimen lokalisasi ad hoc / slam_toolbox), peta tersimpan, dan `tray_waypoints.yaml` |
| `myagv_sprayer_bringup` | `hardware.launch.py`, `sim_full.launch.py`, `camera_stream_node`, `cmd_vel_watchdog_node`, `robot_health_node`, `tray_marker_node` |

Koordinat: frame `map` = pusat ruangan, x ke timur, y ke utara.

| baki | pusat (m) | pose parkir (x, y, yaw) | marker |
|---|---|---|---|
| baki_1_barat_laut | (−2.2, 2.2) | (−2.2, 1.40, +90°) | ArUco id 1 |
| baki_2_timur_laut | (2.2, 2.2) | (2.2, 1.40, +90°) | ArUco id 2 |
| baki_3_barat_daya | (−2.6, −2.2) | (−2.6, −1.40, −90°) | ArUco id 3 |
| baki_4_timur_daya | (2.6, −2.2) | (2.6, −1.40, −90°) | ArUco id 4 |
| home / refill | pintu selatan | (0.0, −2.3, ±90°) | — |

## Marker ArUco

```bash
cd src/myagv_sprayer_gazebo
python3 tools/generate_aruco_markers.py
```

Menghasilkan model Gazebo (`models/aruco_marker_1..4/`) sekaligus berkas cetak di
`print/`. DICT_4X4_50, sisi hitam **tepat 100 mm** — angka itu masuk ke
`marker_length_m` di repo Jetson dan salah sedikit akan menskalakan seluruh
estimasi jarak. Pasang di muka baki yang menghadap posisi parkir, tinggi ±0,30 m.

## Simulasi (PC Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11)

```bash
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs \
     ros-humble-slam-toolbox ros-humble-xacro ros-humble-cv-bridge
cd myagv_sprayer_raspi
vcs import src < myagv_sprayer.repos     # menarik sprayer_msgs (ganti URL CHANGEME)
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash

ros2 launch myagv_sprayer_bringup sim_full.launch.py
```

Lalu di terminal lain (workspace Jetson, boleh di PC yang sama):

```bash
ros2 launch sprayer_bringup jetson.launch.py sim:=true algorithm:=vfh_ql auto_start:=true
```

### Menjalankan skenario eksperimen

Matriks skenario saat ini ada di `docs/experiment_protocol.md` (repo Jetson):
skenario 1 (ideal) dan skenario 2 (rintangan tak terpetakan, varian persisten
dan transient). Skenario lokalisasi-noise/stale-map/multi-target versi
sebelumnya sudah tidak ada di protokol saat ini.

```bash
# 1: kondisi ideal, tanpa rintangan tak terpetakan
ros2 launch myagv_sprayer_bringup sim_full.launch.py scenario:=1_ideal trial:=0

# 2a: rintangan tak terpetakan (selang/ember) di jalur, bertahan sepanjang episode
ros2 launch myagv_sprayer_bringup sim_full.launch.py scenario:=2_unmapped_persistent trial:=0

# 2b: rintangan yang sama, tapi hilang sendiri ~24 detik ke dalam episode
ros2 launch myagv_sprayer_bringup sim_full.launch.py scenario:=2_unmapped_transient trial:=0

# lokalisasi melenceng — independen dari skenario, untuk eksperimen ad hoc
ros2 launch myagv_sprayer_bringup sim_full.launch.py localization_mode:=drift
```

Rintangan ditaruh dari benih `(scenario, trial)` memakai hash yang sama dengan
benchmark offline di repo Jetson, sehingga setiap algoritma menghadapi dunia yang
benar-benar identik dan sebuah trial bisa direproduksi cukup dari nomornya.

Membuat peta baru dengan LiDAR (peta bawaan sudah cocok dengan world):

```bash
ros2 launch myagv_sprayer_navigation slam.launch.py use_sim_time:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r /cmd_vel:=/cmd_vel_manual
ros2 run nav2_map_server map_saver_cli -f src/myagv_sprayer_navigation/maps/nursery_room
```

(Teleop diarahkan ke `/cmd_vel_manual` karena `/cmd_vel` milik watchdog; watchdog
memberi prioritas ke perintah manual selama masih terbit.)

## Hardware (Raspberry Pi 4B di myAGV)

Prasyarat: Ubuntu 22.04 arm64 + ROS 2 Humble. Driver myAGV dan YDLidar ditarik
lewat `myagv_sprayer.repos`.

```bash
git clone <repo> ~/myagv_sprayer_raspi && cd ~/myagv_sprayer_raspi
bash setup/install_raspi.sh
sudo cp setup/netplan-eth0.yaml /etc/netplan/60-myagv-sprayer.yaml && sudo netplan apply
ros2 launch myagv_sprayer_bringup hardware.launch.py
```

Kalau nama launch driver berbeda dari versi upstream Anda:

```bash
ros2 launch myagv_sprayer_bringup hardware.launch.py \
  myagv_driver_launch:="myagv_odometry myagv_active.launch.py" \
  lidar_launch:="ydlidar_ros2_driver ydlidar_launch.py"
```

Autostart: `sudo cp setup/myagv-sprayer.service /etc/systemd/system/ && sudo systemctl enable --now myagv-sprayer`.

Kalibrasi kamera:
`ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=/camera/image_raw`,
lalu isi `fx fy cx cy` di `myagv_sprayer_bringup/config/camera.yaml` **dan** di
`sprayer_docking/config/docking.yaml` (ArUco memakai intrinsik yang sama).

Catatan mekanis: kamera bawaan myAGV dipindah ke puncak tiang nozzle (±0,50 m dari
lantai, menunduk ±32°) dan nozzle di ±0,42 m menunduk ±14°, agar kanopi bibit
(±0,30 m) dan marker (±0,30 m) sama-sama terlihat dari jarak parkir 0,45 m.

## Mengubah ruangan

Ubah `myagv_sprayer_gazebo/worlds/nursery_room.world`, lalu samakan di tiga
tempat: `tools/generate_assets.py` (ROOM_X/ROOM_Y/TRAYS → jalankan untuk membuat
ulang tekstur dan peta Nav), `tools/generate_aruco_markers.py` (posisi marker),
dan `sprayer_nav/sim2d/layout.py` di repo Jetson (geometri simulator 2D). Terakhir
perbarui `tray_waypoints.yaml` di kedua repo.

## Lisensi

MIT — lihat `LICENSE`.
