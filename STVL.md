# Q1 SCI Calisma Dosyasi

## Yeni Konumlandirma

Onerilen yeni baslik:

**Uncertainty-Aware Negative Obstacle Risk Representation for ROS 2 Navigation with Two-Layer STVL Costmap Integration**

Bu calisma artik yalnizca "RANSAC + depth discontinuity ile negatif engel tespiti" olarak sunulmamali. Ana iddia su olmalidir:

> Negatif engeller Nav2/STVL icin sadece bir algilama problemi degil, bir temsil problemidir. Standart costmap bos alani guvenli gibi yorumlayabilir. Bu calisma, tek RGB-D kameradan gelen geometrik ipuclarini belirsizlik-duyarli bir risk alanina donusturur ve bu alani hem yerel STVL hem de global Nav2 costmap'e aktararak robotun dusme riski tasiyan bolgelerden planlama asamasinda kacinmasini saglar.

Bu konumlandirma, klasik geometrik bilesenlerin zayif yenilik algisini azaltir. Yenilik artik RANSAC'in kendisinde degil; geometri, belirsizlik, zamansal kararlilik ve costmap temsili arasindaki ucuca risk mimarisindedir.

## Ana Bilimsel Katkilar

1. **Belirsizlik-duyarli negatif engel risk alani**  
   Sistem her piksel veya costmap hucreleri icin yalnizca SAFE/DROP etiketi uretmez. Bunun yerine `P_drop`, `P_unknown`, `confidence`, `temporal_stability` ve `risk_score` uretir.

2. **Dereceli risk-to-costmap donusumu**  
   Binary lethal isaretleme yerine risk skoru `0-254` arasi Nav2 uyumlu maliyete donusturulur. Yuksek guvenli drop kenarlari yine lethal kalabilir; belirsiz bolgeler ise high-cost olarak temsil edilir.

3. **Iki katmanli risk-aware Nav2/STVL entegrasyonu**  
   Ayni risk bilgisi yerel STVL tarafinda reaktif kacinma, global costmap tarafinda ise planlama oncesi rota secimi icin kullanilir.

4. **Hard-negative odakli degerlendirme**  
   Rampa, golge, parlak zemin ve duzensiz yuzey gibi yanlis alarm ureten sahneler ayri metriklerle incelenir.

5. **Gercek zamanli ve uygulanabilir ROS 2 mimarisi**  
   Sistem STVL cekirdegini degistirmeden ROS 2 topic ve parametre arayuzuyle calisir. Gomme CPU sinifi donanimlar hedeflenir.

## Yontem Ozeti

### 1. Geometrik Kanitlar

RGB-D derinlik goruntusunden su kanitlar cikarilir:

- `depth_jump`: komsu pikseller arasindaki ani derinlik farki
- `height_residual`: RANSAC zemin duzlemine gore asagi sapma
- `missing_depth`: destek beklenen yerde gecersiz derinlik
- `edge_evidence`: drop-edge siniri
- `ramp_consistency`: surekli egimli yuzey olasiligi

Bu kanitlar klasik esitlenmis etiket yerine yumusak risk alanina beslenir.

### 2. Risk Kanallari

Yeni kodda uretilen ana kanallar:

```text
P_drop              negatif engel / dusme olasiligi
P_unknown           destek yoklugu veya guvenilmez derinlik olasiligi
confidence          algi guven skoru
temporal_stability  riskin zaman icindeki kararliligi
risk_score          birlesik navigasyon riski
risk_cost           Nav2 uyumlu 0-254 maliyet
```

Bu ciktilar `GeometricRiskAnalyzer.analyze()` sonucunda mevcuttur:

```python
result["drop_probability"]
result["unknown_probability"]
result["confidence"]
result["temporal_stability"]
result["risk_field"]["risk_score"]
result["risk_cost"]
```

### 3. Zamansal Risk Birlesimi

Tek karelik gürültüye bagli yanlis riskleri azaltmak icin risk alani onceki kareyle birlestirilir:

```text
S_t = alpha * S_{t-1} + (1 - alpha) * R_t
```

Bu mekanizma eski nokta biriktirme mantigini daha bilimsel bir seviyeye tasir. Artik zaman sadece PointCloud2 tamponunda degil, risk temsilinin icinde de vardir.

### 4. Costmap'e Aktarim

Risk skoru su sekilde maliyete donusturulur:

```text
risk_cost = round(254 * risk_score)
```

Yuksek riskli ve guvenilir drop sinirlari `254` lethal cost alir. Daha belirsiz ama tehlikeli bolgeler `high-cost` olarak kalir. Bu sayede planlayici sadece "yasak/serbest" ayrimi degil, risk derecesine gore rota secimi yapabilir.

## Kodda Yapilan Degisiklikler

Yeni dosya:

- `src/negative_obstacle_perception/negative_obstacle_perception/risk_field.py`

Guncellenen dosyalar:

- `geometric_risk_analyzer.py`: klasik risk maskesine ek olarak belirsizlik-duyarli risk alani uretir.
- `semantic_drop_publisher.py`: `/semantic_drop_points` yaninda `/risk/drop_probability` ve `/risk/cost_image` yayinlar.
- `semantic_drop_publisher_params.yaml`: ground estimator varsayilan olarak acildi ve yeni topic'ler belgelendi.
- `test_geometric_risk.py`: risk alani sekil/aralik testleri eklendi.

ROS topic ciktilari:

```text
/semantic_drop_points       PointCloud2, STVL marking source
/risk/drop_probability      32FC1, P(drop)
/risk/cost_image            mono8, 0-254 risk/cost alani
```

## Sensor Suruculeri ve Donanim Ayarlari

Kullanilacak donanimlar:

| Donanim | Surucu/Paket | Ana Topic | Durum |
|---|---|---|---|
| Intel RealSense RGB-D | `realsense2_camera` | `/camera/depth/image_raw`, `/camera/depth/points` | opsiyonel launch |
| RPLiDAR/Slamtec | `rplidar_ros` veya `sllidar_ros2` | `/scan` | opsiyonel launch |
| GPS NMEA | `nmea_navsat_driver` | `/fix` | `gps_mode:=true` |
| Witmotion HWT905 IMU | `witmotion_hwt905_driver` | `/imu/data` | `gps_mode:=true` |

Kurulum:

```bash
# GPS - NMEA
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/evenator/nmea_navsat_driver.git
cd ~/ros2_ws
colcon build --packages-select nmea_navsat_driver
source install/setup.bash

# RPLiDAR / Slamtec
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git

# Intel RealSense
sudo apt install ros-$ROS_DISTRO-librealsense2*
sudo apt install ros-$ROS_DISTRO-realsense2-*

# Witmotion HWT905 IMU
cd ~/ros2_ws/src
git clone https://github.com/Nervxz/ROS2_HWT905WitmotionIMU.git
cd ~/ros2_ws
colcon build --packages-select witmotion_hwt905_driver
source install/setup.bash
```

Tek tek test:

```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch rplidar_ros view_rplidar_launch.py
ros2 launch negative_obstacle_bringup gps_localization.launch.py gps_port:=/dev/ttyUSB1 imu_port:=/dev/ttyUSB2
```

Tam sistemde sensorleri baslatma:

```bash
# RealSense dahil
ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py start_realsense:=true

# RPLiDAR dahil
ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py start_lidar:=true

# GPS + IMU + EKF dahil
ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py gps_mode:=true gps_port:=/dev/ttyUSB1 imu_port:=/dev/ttyUSB2

# Tum gercek dis ortam sensörleri
ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py start_realsense:=true start_lidar:=true gps_mode:=true
```

Notlar:

- RealSense topicleri farkli gelirse `camera_depth_topic:=...` ve `camera_info_topic:=...` ile semantic node baglanir.
- RPLiDAR paketi sistemde `sllidar_ros2` olarak gorunuyorsa:

```bash
ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py start_lidar:=true lidar_package:=sllidar_ros2 lidar_launch:=view_sllidar_a1_launch.py
```

- URDF'de `camera_link`, `camera_optical_frame`, `lidar_link`, `imu_link`, `gps_link` tanimlidir.

## Veri Seti Organizasyonu

Calisma veri seti iki ana bolumden olusmalidir: sentetik veri ve gercek robot verisi. Q1 icin yalnizca sentetik veri yeterli degildir; gercek robot kismi nicel olarak tamamlanmalidir.

### A. Sentetik Veri Seti

Amac:

- kontrollu etiket uretmek
- farkli negatif engel geometrilerini sistematik test etmek
- hard-negative sahnelerde yanlis alarm davranisini olcmek

Mevcut sentetik kapsam:

| Grup | Sahne Tipleri | Ornek |
|---|---|---:|
| Pozitif negatif engel | rectangular pit, circular pit, platform edge, trench, stairs descent, curb drop | 230 |
| Hard-negative | ramp, shadow, uneven ground | 110 |
| Toplam | 9 aktif sahne tipi | 340 |

Her ornek sunlari icermelidir:

```text
depth.npy
meta.json
risk_mask.png
edge_mask.png
risk_field.npz        onerilen yeni cikti
overlay.png
```

`risk_field.npz` icinde su alanlar saklanabilir:

```text
p_drop
p_unknown
confidence
temporal_stability
risk_score
risk_cost
```

### B. Gercek Robot Veri Seti

Q1 icin gercek veri seti mutlaka eklenmelidir. Veri ikiye ayrilir:

- **Ic mekan:** platform, merdiven basi, ic mekan cukur/kenar.
- **Dis ortam:** kaldirim dususu, dis merdiven, hendek/kanal, rampa, golge/parlak zemin, duzensiz zemin.

Onerilen en kucuk ama savunulabilir protokol:

| Sahne | Tur | Tekrar |
|---|---|---:|
| Platform kenari | gercek drop | 20 |
| Merdiven basi | gercek drop | 20 |
| Ic mekan cukur/kenar | gercek drop | 20 |
| Kaldirim dususu | gercek drop | 20 |
| Rampa | hard-negative | 20 |
| Golge/parlak zemin | hard-negative | 20 |
| Duzensiz zemin | hard-negative | 20 |

Dis ortam icin veri miktari hedefi:

| Seviye | Senaryo | Tekrar | Deneme | Yaklasik Frame |
|---|---:|---:|---:|---:|
| Minimum | 8 | 20 | 160 | 16.000 |
| Ideal | 8 | 30 | 240 | 24.000 |
| Guclu | 10 | 30 | 300 | 30.000 |

Hedef: en az **160 dis ortam denemesi**, mumkunse **240 deneme**. Hesap 10 saniye kayit ve 10 Hz uzerindendir.

Her gercek kayit sunlari icermelidir:

```text
rosbag
depth frames
camera_info
robot odometry
cmd_vel
local costmap
global costmap
rviz screenshots
rviz short video
manual/semiautomatic risk annotation
scenario metadata
```

Gercek robot metrikleri:

- drop oncesi durma basarisi
- minimum drop mesafesi
- hedefe ulasma orani
- gereksiz durma orani
- global planin drop bolgesini kesme sayisi
- yeniden planlama sayisi
- hard-negative false positive orani

## Tek Launch ve RViz Deneme Plani

Deneme testleri tek launch dosyasi ile baslatilmalidir. RViz acik testlerde gorsel dogrulama,
RViz kapali testlerde ise veri kaydi ve performans olcumu yapilir.

### Simulasyon

```bash
ros2 launch negative_obstacle_bringup full_system_sim.launch.py
```

Secenekler:

```bash
world:=pit_scene
world:=platform_edge
rviz:=true
rviz:=false
map_mode:=true map:=/path/map.yaml
```

### Gercek Robot

```bash
ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py
```

Secenekler:

```bash
rviz:=true
rviz:=false
port:=/dev/ttyUSB0
gps_mode:=true
map_mode:=true map:=/path/map.yaml
```

### 4WD Serial Motor Kontrolu

Gercek robotta motor kontrolu `4WD_USE_manual.tex` protokolune uyar:

```text
Baud: 115200, 8N1
Komut: "L R\n"
Ornek: "100 100\n" ileri, "100 -100\n" sag pivot, "-100 100\n" sol pivot, "0 0\n" dur
```

Kurallar:

- Serial komutlar yalnizca **CH2 pasifken** etkili olur.
- Pair PWM komutu taze kalmalidir; yaklasik **400 ms** icinde yenilenmezse Arduino beklemeye gecer.
- ROS surucusu `/cmd_vel` girdisini periyodik olarak `L R\n` pair PWM komutuna cevirir.
- ROS standardinda `angular.z > 0` sol donustur; Arduino'ya `-L +R` olarak gider.
- Tek teker test komutlari (`FL 100`, `FR 100` vb.) manuel test icindir; Nav2 surusunde kullanilmaz.

### Manuel Motor Kontrol GUI

GUI ayri bir ROS node olarak calisir. Genel surus icin `/cmd_vel`, tek teker testleri icin
`/wheel_test_command` yayinlar.

Tam sistemle birlikte:

```bash
ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py motor_gui:=true
```

Sadece GUI:

```bash
ros2 launch negative_obstacle_bringup motor_control_gui.launch.py
```

GUI islevleri:

- Baslat / Dur / Acil STOP
- ileri, geri, sag, sol
- genel hiz orani
- FL, FR, RL, RR tekerlerini ayri ayri ileri/geri RPM testi
- tek teker STOP

Guvenlik notu: GUI ile tek teker testi yaparken CH2 pasif olmali ve robot kaldirilmissa tekerler
serbest donecek sekilde sabitlenmelidir.

### RViz ile Kontrol Edilecekler

```text
/camera/depth/image_raw
/semantic_drop_points
/risk/drop_probability
/risk/cost_image
/local_costmap/costmap
/global_costmap/costmap
```

Basarili bir denemede beklenenler:

- drop kenari gorulunce `/semantic_drop_points` olusur
- `/risk/drop_probability` drop bolgesinde yukselir
- `/risk/cost_image` drop/unknown support bolgesinde high-cost veya lethal cost uretir
- local costmap robotu durdurur veya kacar
- global costmap varsa plan drop bolgesini bastan kesmez

### Deneme Kaydi

Her testte tek rosbag kaydi alinmalidir:

```text
/camera/depth/image_raw
/camera/depth/camera_info
/semantic_drop_points
/risk/drop_probability
/risk/cost_image
/local_costmap/costmap
/global_costmap/costmap
/odom
/tf
/tf_static
/cmd_vel
```

Her deneme klasoru ayrica RViz gorselleri icermelidir:

```text
trial_001/
  bag/
  meta.yaml
  annotations/
  rviz/
    before.png          # robot drop/hard-negative bolgeye yaklasmadan once
    detection.png       # risk/costmap belirginlestigi an
    stop_or_avoid.png   # robot durdugu veya kactigi an
    overview.png        # local/global costmap + robot pozu
    short_clip.mp4      # 5-15 saniyelik kisa RViz video, mumkunse
```

RViz ekraninda en az su katmanlar gorunmelidir:

```text
RobotModel
/semantic_drop_points
/risk/drop_probability veya /risk/cost_image
/local_costmap/costmap
/global_costmap/costmap
TF
```

Not: Performans/latency olcumlerinde RViz kapali tutulmali (`rviz:=false`). RViz gorselleri
ayri bir gorsel dogrulama kosusunda alinmali; makalede nitel sonuc ve hata analizi icin kullanilmalidir.

## Deney Tasarimi

Yeni ablasyon seti su sekilde olmali:

| Kod | Aciklama |
|---|---|
| B0 | Standart STVL, negatif engel modulu yok |
| A1 | Edge-only depth discontinuity |
| A2 | RANSAC zemin + edge |
| A3 | A2 + temporal point persistence |
| A6 | Risk field, temporal fusion yok |
| A7 | Risk field + confidence |
| A8 | Risk field + confidence + temporal stability |
| A9 | Risk field + hareket/hiz uyarlamali decay |

Birincil guvenlik metrikleri:

- `False Safe Rate`: gercek drop bolgesinin guvenli kalma orani
- `Drop Recall`: drop bolgesini yakalama orani
- `HN-FPR`: hard-negative sahnelerde yanlis drop/risk orani
- `Risk Calibration`: yuksek risk verilen bolgelerin gercek tehlikeyle uyumu
- `Costmap Flicker`: ardarda karelerde costmap kararsizligi

Navigasyon metrikleri:

- basarili kacis / durma
- minimum guvenli mesafe
- rota uzamasi
- hedefe ulasma suresi
- gereksiz durma

## Makale Yapisi

1. Introduction  
   Negatif engelin Nav2/STVL icin temsil problemi oldugunu anlat.

2. Related Work  
   Negatif engel tespiti, traversability, voxel/costmap temsili ve risk-aware planning ayrimi.

3. Problem Definition  
   Drop, unknown support, risk score ve costmap temsili formal tanimlansin.

4. Method  
   Geometrik kanitlar, belirsizlik-duyarli risk alani, temporal fusion, risk-to-costmap donusumu.

5. Dataset  
   Sentetik ve gercek veri ayrik ama ayni etiket semasiyla sunulsun.

6. Experiments  
   Binary geometrik sistem ile risk-aware sistem ablasyonlu karsilastirilsin.

7. Results  
   Sadece piksel metrikleri degil, navigasyon davranisi da verilsin.

8. Discussion and Limitations  
   FSR, domain gap, kalibrasyon, dinamik sahne sinirlari acikca yazilsin.

## Dikkat Edilecek Iddialar

Kullanilabilir:

- "negative obstacle representation"
- "uncertainty-aware risk field"
- "risk-aware costmap integration"
- "two-layer local-global Nav2 protection"
- "lightweight geometric evidence model"

Kacinilmasi gerekenler:

- "tam guvenli navigasyon"
- "tum negatif engelleri tespit eder"
- "state-of-the-art"  
- "gercek dunyada genellenir"  
- "FSR=0.286 yeterlidir"

Dogru ve savunulabilir iddia:

> Onerilen risk temsili, standart STVL'nin negatif engelleri bos/guvenli alan gibi yorumlama problemini azaltir ve planlayicinin belirsiz veya destek kaybi iceren bolgeleri dereceli maliyet olarak gormesini saglar.

## Kisa Yol Haritasi

1. Risk field ciktisini sentetik veri uzerinde kaydet.
2. A6-A8 ablasyonlarini calistir.
3. Gercek robotta en az 6 sahne x 20 tekrar topla.
4. Global-only, local-only ve local+global costmap karsilastirmasi yap.
5. Makale basligini ve katkilarini risk representation ekseninde guncelle.
6. Sonuclari "tespit" degil "risk temsili + navigasyon etkisi" olarak tartis.
