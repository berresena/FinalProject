# FinalProject
🚀 Qt-Based ROS Integrated 3D Mapping and Obstacle Avoidance System
🇹🇷 Açıklama (Türkçe):

Bu proje, ROS (Robot Operating System) altyapısı ve Qt arayüzü kullanılarak geliştirilmiş bir mobil robot kontrol ve haritalama sistemidir. Sistem, robotun gerçek zamanlı olarak PointCloud2 verilerini alarak çevresel harita oluşturmasını, engellerden kaçınmasını ve coverage planning algoritması ile sistematik alan taraması yapmasını sağlamaktadır.

Öne Çıkan Özellikler:

    Qt ile geliştirilmiş modern ve kullanıcı dostu GUI

    ROS topic’lerinden gelen sensor_msgs/PointCloud2 verilerinin OpenGL ile görselleştirilmesi

    VFH (Vector Field Histogram) algoritması ile dinamik engel tespiti ve yönelim planlaması

    Kapsama algoritması ile tam alan gezintisi (coverage planner)![vokoscreen-2025-06-04_00-25-57-_online-video-cutter com_](https://github.com/user-attachments/assets/d6180143-7f63-4146-b150-d2282a3ab1f8)


    geometry_msgs/Twist mesajları ile hareket komutu gönderimi

    ROS TF dönüşümleri ile hassas konumlandırma

🇬🇧 Description (English):

This project is a mobile robot control and mapping system developed using ROS (Robot Operating System) infrastructure and a modern Qt-based GUI. The system enables real-time processing of sensor_msgs/PointCloud2 data for environmental 3D mapping, obstacle avoidance, and systematic area coverage planning.

Key Features:

    Modern and user-friendly GUI built with Qt

    Visualization of incoming PointCloud2 data using OpenGL

    Dynamic obstacle detection and heading control using the VFH (Vector Field Histogram) algorithm

    Full-area coverage planning with a dedicated planner module

    Motion control via geometry_msgs/Twist messages

    Accurate localization using ROS TF transformations
