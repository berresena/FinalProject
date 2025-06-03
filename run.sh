#!/bin/bash

gnome-terminal --tab --title="GAZEBO" -- bash -c "
  echo -e '\033[1;35mTURTLEBOT EVİ AÇILIYOR...\033[0m';
  roslaunch turtlebot3_gazebo turtlebot3_world.launch;
  exec bash"

# 2️⃣ PCL VIEWER'I 5 SANİYE BEKLETİP BAŞLAT (Gazebo'nun açılması için)
gnome-terminal --tab --title="TF DÖNÜŞÜM" -- bash -c "
  sleep 5;
  echo -e '\033[1;34mPCL HARİTALAYICI AÇILIYOR...\033[0m';
  rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint camera_rgb_optical_frame 100;
  exec bash"

gnome-terminal --tab --title="PCL_VIEWER" -- bash -c "
  sleep 5;
  echo -e '\033[1;34mPCL HARİTALAYICI AÇILIYOR...\033[0m';
  rosrun pcl_qt_viewer pcl_qt_viewer_node;
  exec bash"

echo -e "\033[1;33mTÜM PENCERELER AÇILDI!🚀\033[0m"

