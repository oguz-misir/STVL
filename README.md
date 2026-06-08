# STVL Negative Obstacle Risk Navigation

ROS 2/Nav2/STVL tabanli mobil robot navigasyonu icin belirsizlik-duyarli negatif engel risk temsili.

Ana fikir: negatif engel yalnizca bir tespit problemi degil, costmap uzerinde risk temsili problemidir.

## Icerik

- RGB-D kameradan `P_drop`, `P_unknown`, `confidence`, `temporal_stability` ve `risk_cost` uretimi
- `/semantic_drop_points` ile STVL entegrasyonu
- RealSense, RPLiDAR, GPS NMEA ve Witmotion IMU destekli gercek robot launch yapisi
- 4WD serial motor kontrolu ve manuel GUI
- Sentetik + gercek veri toplama protokolu

Ayrintili calisma plani, donanim kurulumu ve deney protokolu icin:

- [STVL.md](STVL.md)
