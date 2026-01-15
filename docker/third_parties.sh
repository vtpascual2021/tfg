#!/bin/bash

cd /root/ros2_humble_ws/src/Pangolin || { echo "No se pudo entrar en /ros2_humble_ws/src/Pangolin"; exit 1; }

# instalador de dependencias recomendadas por Pangolin, necesarias para compilarlo
./scripts/install_prerequisites.sh recommended

cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release

# con todos los nucleos disponibles, crea el fichero 'PangolinConfig.cmake', necesario para 'find_package(Pangolin REQUIRED)'
cmake --build . -j$(nproc)
# reconocer Pangolin como paquete instalable
sudo cmake --install .
# actualiza el enlazador dinámico
sudo ldconfig
# actualizar la variable de la libreria dinamica
LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
# actualiza el enlazador dinámico
sudo ldconfig

# añade la ruta de instalación para que CMake de ORBSLAM3 detecte Pangolin
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH

cd /root/ros2_humble_ws/src/ORB_SLAM3/ || { echo "No se pudo entrar en  ORB_SLAM3"; exit 1; }

chmod +x build.sh
./build.sh

cd /root/ros2_humble_ws/
colcon build --symlink-install --packages-ignore pangolin ORB_SLAM3 easynav_pointcloud_common easynav_pointcloud_maps_manager pcl_ros

source install/setup.bash

exit 0
