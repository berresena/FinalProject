# pcl_qt_viewer – Qt & ROS tabanlı 3D Haritalama ve Navigasyon Sistemi

📌 **Türkçe / English Description Below**

## 🚀 Proje Tanımı

Bu proje, Qt tabanlı modern bir arayüz ile entegre edilmiş ve **ROS (Robot Operating System)** kullanılarak geliştirilmiş bir **mobil robot haritalama ve navigasyon** sistemidir. 

**Başlıca Özellikler:**

- `sensor_msgs/PointCloud2` verilerini OpenGL ile 3D görselleştirme  
- VFH (Vector Field Histogram) ile dinamik engel tespiti ve yönelim planlaması  
- Sistematik alan taraması (Coverage Planner)  
- ROS TF dönüşümleri ile lokalizasyon  
- `geometry_msgs/Twist` ile robot hareket kontrolü  
- Qt tabanlı GUI: haritalama durumu, engel algılama, kamera görüntüsü ve konum takibi

---

## 🖥️ Sistem Gereksinimleri

- Ubuntu 20.04  
- ROS Noetic  
- Qt5 / Qt Creator  
- PCL (Point Cloud Library)  
- OpenGL  
- catkin (Catkin-based workspace)

---

## 🔧 Kurulum

1. ROS Noetic kurulu değilse:  
   [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)

2. Workspace'i klonlayın:

```bash
cd ~
git clone https://github.com/kullanici-adi/pcl_qt_viewer.git
cd pcl_qt_viewer
```
3.src/ klasörüne geçin, varsa eksik bağımlılıkları yükleyin ve projeyi derleyin:
``` bash 
cd src
rosdep install --from-paths . --ignore-src -r -y
cd ..
catkin_make
source devel/setup.bash
```
Projedeki run.sh ile direkt olarak çalıştırmanız mümkündür.
