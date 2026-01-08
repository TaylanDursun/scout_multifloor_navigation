# Taylan

## Terminal Komutları
```bash
source ~/scout_ws/install/setup.bash
ros2 launch scout_gazebo_sim scout_hospital_floors.launch.py floor:=1
```
Hangi kat isteniyorsa floor:1 veya floor:2 yazılmalı

```bash
source ~/scout_ws/install/setup.bash
ros2 launch ros2_aruco aruco_recognition.launch.py
```

```bash
source ~/scout_ws/install/setup.bash
ros2 run scout_gazebo_sim floor_detector.py
```

## ✅ Şu Ana Kadar Tamamlananlar
- [x] **Haritalama:** Lidar kullanılarak hastane haritası (SLAM) çıkarıldı.
- [x] **Dünya Düzeni:** Gazebo ortamına Kat 1 (ID:0) ve Kat 2 (ID:1) markerları eklendi. (`scout_hospital_floors.launch.py`) for dynamic floor selection.
- [x] **Algılama:**ZED kamera entegrasyonu ve ArUco okuma sistemi kuruldu.
- [x] **Karar Mekanizması:** Robotun bulunduğu katı tespit edip log basan `floor_detector` node'u yazıldı.

## Önemli Dosyalar
world dosyası:
`/home/taylan/scout_ws/src/scout/scout_gazebo_sim/worldshospital_two_floors_markes.world`

launch dosyası:
`/home/taylan/scout_ws/src/scout/scout_gazebo_sim/launch/scout_hospital_floors.launch.py`

Robotun okuduğu barkod ID'sine (0 veya 1) göre hangi katta olduğunu anlayan Python algoritması:
`scout/scout_gazebo_sim/scripts/floor_detector.py`





