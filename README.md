# Índice
- [Configuracion y creación del Docker](#configuración-del-docker)
  - [Compilación e instalación de librerías](#compilación-e-instalación-de-librerías---thirdparties)
- [Paquetes](#paquetes-utilizados)
  - [URJC-excavation](#urjc---excavation-world-simulation)
  - [Aerostack2](#aerostack2)
    - [Lanzar Aerostack](#lanzar-aerostack)
    - [Lanzar Aerostack - Manual](#lanzamiento-aerostack-manual)
    - [Lanzar Aerostack - Script](#lanzamiento-mod-repo-as2-ex)
- [Técnicas SLAM](#técnicas-de-slam)
  - [Lidarslam](#lidar-slam)
    - [Lidarslam y as2](#lidarslam--as2)
  - [ORB-SLAM3](#orb-slam-3)
    - [Pangolin](#pangolin)
    - [ORB-SLAM3 y ROS2](#orb-slam3-para-ros2)
- [Creación de mapa 3D](#creación-de-mapas)

# Configuración del docker

Para simular las condiciones de la placa Jetson Orin Nano, se configura un docker con las mismas características software. Es decir, el contenedor debe tener:
- Ubuntu 22.04
- Ros2 Humble
- CUDA 12.6

Para crear la estructura de *Docker Compose*, se precisa de un [Dockerfile](docker/Dockerfile) y varios ficheros `yaml`.

Para iniciar un docker, en una terminal:

1. Construir el contenedor

```sh
cd docker
docker compose -f docker-compose-gui-nvidia.yaml build
```

2. Montarlo

```sh
docker compose -f docker-compose-gui-nvidia.yaml up -d
```

3. Ejecutarlo

```sh
docker compose -f docker-compose-gui-nvidia.yaml exec tfg_nvidia /bin/bash
```

4. Pararlo

Cerrar el contenedor (CTRL+D) y después ejecutar:

```sh
docker compose -f docker-compose-gui-nvidia.yaml down
```

> [!TIP]
> Si alguna herramienta de visualización no funciona, como rviz o gazebo, ejecutar (con el contenedor cerrado):
```sh
xhost +Local:*
xhost
```

Tras iniciarlo, comprobar que se ha creado con las especificaciones:
```
$ lsb_release -a
No LSB modules are available.
Distributor ID:	Ubuntu
Description:	Ubuntu 22.04.4 LTS
Release:	22.04
Codename:	jammy

$ printenv ROS_DISTRO
humble
```

> [!NOTE]  
> Una alternativa es ejecutar el fichero `launch_docker.sh`, incluido en la raiz del workspace. Este fichero ejecuta los siguientes comandos:
```sh
#!/bin/bash

usage() {
  echo "./launch_docker.sh [-b]"
  exit 1
}

docker_up() {
  docker compose -f docker-compose-gui-nvidia.yaml up -d

  docker compose -f docker-compose-gui-nvidia.yaml exec tfg_nvidia /bin/bash
}


if [ $# -gt 1 ]; then
  usage

elif [ $# -eq 1 ]; then
  xhost +Local:*
  xhost

  cd docker

  if [ "$1" = "-b" ]; then
    echo "BUILDING DOCKER..."
    docker compose -f docker-compose-gui-nvidia.yaml build
    docker_up
  else
    echo "ARG NOT VALID"
    usage
  fi

else
  xhost +Local:*
  xhost

  cd docker

  docker_up

fi

exit 0
```

## Compilación e Instalación de librerías - Thirdparties

Para poder compilar haciendo colcon del workspace dentro del docker, se necesita hacer dos cosas por separado:

1. Hacer build de las librerías de terceros utilizadas por otros paquetes
2. Hacer colcon de aquellos paquetes que lo necesitan

Para automatizar este proceso, se ejecuta el fichero [third_parties](docker/third_parties.sh). Este se encarga de:

· Compilar el paquete **Pangolin**: instalar las dependencias recomendadas, compilar el paquete (como paquete instalable), añadirlo a la variable de 'librería dinámica' (LD_LIBRARY_PATH) y añadirlo a la ruta de instalación de cmake (CMAKE_PREFIX_PATH). Esto último es necesario para que el paquete *ORB-SLAM3* lo pueda enlazar en su cmake.

· Compilar el paquete **ORB-SLAM3**: ejecutar su script *build.sh*

· Compilar el workspace con los flags `--symlink-install` y  `--packages-ignore pangolin ORB_SLAM3`.
Este último es muy importante, ya que si no se ignoran ambos, dará un error, ya que su modo de compilación es mediante scripts, no mediante colcon.


```sh
$ ./third_parties.sh
```

# Paquetes utilizados

Para usar la aplicación, se necesita: modelo del mundo, modelo del dron y simulador.

Por un lado, el modelo del dron y el simulador utilizado (Gazebo Ignition) vienen definidos en **Aerostack2 (as2)**.
Por otro lado, el modelo del mundo viene definido en **Urjc-Excavation-World**.

## URJC - Excavation (World Simulation)

Clonar el repositorio [urjc-excavation-world](https://github.com/juanscelyg/urjc-excavation-world). Este define varios modelos para mundos en formato `.world`. Está preparado para ROS2 Jazzy y Ubuntu 24.04 donde funciona gazebo (**gz**). Sin embargo, el contenedor creado es de una versión anterior donde se utiliza gazebo ignition (**ign**). Es por eso que, en caso de querer utilizar el launcher o, para unificar todos los directorios del workspace para que estén preparados para la misma versión, es necesario cambiar:

1. El launcher:
```python
# Versión original en el launcher
gazebo_server_cmd_line = [
        'gz', 'sim', '-r', '-v4', world]


# Se cambia por :
gazebo_server_cmd_line = [
        'ign', 'gazebo', '-r', '-v4', world]

```

2. Modelo del mundo:

```xml
<!-- ANTES -->
<plugin
    filename="gz-sim-navsat-system"
    name="gz::sim::systems::NavSat">
</plugin>

<!-- DESPUES -->
<plugin
    filename="ignition-gazebo-navsat-system"
    name="ignition::gazebo::systems::NavSat">
</plugin>
```

Tras hacer estos cambios, se puede lanzar Gazebo Ignition desde el propio paquete con el siguiente comando:

```sh
ros2 launch urjc_excavation_world urjc_excavation.launch.py
```

> [!WARNING]
> Aún sin cambiar las referencias a gz en la definición del mundo, si que se abre la ventana de ign, apareciendo este error:
```bash
[ign gazebo-1] [ignition::plugin::Loader::LookupPlugin] Failed to get info for [gz::sim::systems::NavSat]. Could not find a plugin with that name or alias.
[ign gazebo-1] [Err] [SystemLoader.cc:125] Failed to load system plugin [gz::sim::systems::NavSat] : could not instantiate from library [gz-sim-navsat-system] from path [/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-navsat-system.so].
```

## Gazebo Cave World (World Simulation)

Contiene el modelo de un 'mundo cueva' para poder testear el dron en interiores. Primero, clonar el repositiorio [gazebo_cave_world](https://github.com/LTU-CEG/gazebo_cave_world.git). Como solo se necesita el modelo, añadir un fichero `COLCON_IGNORE` en el directorio.

Modificaciones:
1. Cambiar la extensión del fichero del modelo del mundo de `.world` a `.sdf`
2. Añadir la definición de los plugins que se necesitan para la simulación:
```xml
<?xml version="1.0"?>
<sdf version='1.6'>
  <world name='cave_world_generic'>

    <!-- Generic Plugins -->
    <plugin
        filename="ignition-gazebo-physics-system"
        name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
        filename="ignition-gazebo-user-commands-system"
        name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin
        filename="ignition-gazebo-scene-broadcaster-system"
        name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>

    <!-- Sensors plugin -->
    <plugin
      filename="ignition-gazebo-sensors-system"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="ignition-gazebo-imu-system"
      name="ignition::gazebo::systems::Imu">
    </plugin>
    <light name='sun' type='directional'>
      [...]
```

NOTA : Para que se lance de forma automática al iniciar el dron, poner el nombre del mundo al fihcero de lanzamiento de aerostack. MÁS ADELANTE
## Aerostack2
Este paquete se instala, no por repositorio, si no por binarios a través del Dockerfile:

```Dockerfile

RUN apt-get install -y \
        # aerostack
        ros-humble-aerostack2 \
```

### Lanzar el modelo del mundo en as2

Para poder utilizar el mundo definido en *urjc excavation* en *aerostack2*, es necesario hacer:

0. Lanzar el contenedor, con las herramientas gráficas funcionales y los paquetes compilados.

1. Exportar la variable de entorno 'GZ_SIM_RESOURCE_PATH', que define donde buscar el modelo del mundo

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_humble_ws/src/urjc-excavation-world/worlds &&
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_humble_ws/src/urjc-excavation-world/models
```
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_humble_ws/src/gazebo_cave_world/worlds &&
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_humble_ws/src/gazebo_cave_world/worlds/models
```
``` bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_humble_ws/src/drone_pkg/models/x500.sdf
```


2. Cambiar la extensión del fichero que define el mundo de `.world` a `.sdf` o `.sdf.jinja`. Esto es por que as2 busca los modelos con esas extensiones en específico. Sin embargo, eso no quiere decir que el tipo de modelo `.world` este mal definido.

### Lanzar Aerostack

Para [crear una aplicación con as2](https://youtu.be/9YS7so52e7U?feature=shared), se necesitan lanzar una serie de modulos:

1. Aerial Platform
2. State Estimator
3. Motion Controller
4. Behaviours
5. Mission

Además, dado que ahora mismo se trabaja con simulación, es necesario también lanzar primero un nodo que configure los parámetros de la misma: Gazebo Assets.

Cada uno de los nodos se lanzará en una terminal diferente.

### Lanzamiento Aerostack MANUAL
**NO TERMINADO**

0. **Gazebo assets**

El objetivo es configurar `as2` con:
- Gazebo IGN
- Drone cuadricoptero
- Mundo *urjc*

Para ello, se crea un fichero de configuración `json`, donde se definen estos parámetros.

El template está en [config_file](https://github.com/aerostack2/aerostack2/tree/main/as2_simulation_assets/as2_gazebo_assets/#config-file). El resultado es el siguiente:

```json
{
  "world_name": "urjc_excavation_model",
  "drones": [
        {
            "model_type": "quadrotor_base",
            "model_name": "drone_sim_0"
        }
      ]
}
```

Para lanzar el nodo, se ejecuta el siguiente comando:
```bash
ros2 launch as2_gazebo_assets launch_simulation.py simulation_config_file:=/root/ros2_humble_ws/src/drone_pkg/config/config_file_prueba.json
```

> [!IMPORTANT]  
> Además de crear este fichero, dado que cada nodo trabajará en una terminal diferente, es necesario exportar la variable:
>```
> export AEROSTACK2_SIMULATION_DRONE_ID=drone_sim_0 # o aquel nombre establecido en el fichero de conguración json del paso anterior
> ```

1. **Aerial Platform**

Lanzar la plataforma de vuelo:

```bash
ros2 launch as2_platform_multirotor_simulator as2_platform_multirotor_simulator.launch.py
```

2. **State Estimator**

Lanzar el estimador de estado:

Hay varios launchers:
- *ground_truth_state_estimator.launch.py* : el más básico
- *mocap_pose_state_estimator.launch.py*
- *raw_odometry_state_estimator.launch.py*
- *state_estimator_launch.py* : ¿¿¿DA ERROR???

La diferencia es que cada uno lanza un plugin distinto.

<!-- ????? Además, necesita como argumentos:
????? 
????? - Archivo de configuración `json` : definido en el paso '0'. 
????? - Nombre `plugin` en `yaml`: sirve para implementar un funcionalidad concreta
?????  -->

```bash
ros2 launch as2_state_estimator ground_truth_state_estimator.py
```

3. **Motion Controller**

Lanzar el controlador de movimiento:

Hay varios launchers:
- controller_launch.py
- differential_flatness_controller.launch.py
- pid_speed_controller.launch.py

```bash
ros2 launch as2_motion_controller pid_speed_controller.launch.py plugin_name:=pid_speed_controller
```

4. **Behaviours**

5. **Mission**

La misión puede ser un fichero python en el que se detalle cada uno de los putnos que tiene que cumplir el dron al iniciarse.


--

En el fichero de configuración se definen los objetos y/o sensores del robot. En este caso el fichero no tiene ningún sensor. No obstante, uno de los nodos que se lanzarán (as2_alphanumeric_viewer), el encargado de mostrar en pantalla lo información del dron de los topics, sensores, plataforma etc, se suscribe a topics aunque no se publique nada en ellos. Un ejemplo es /drone0/sensor_measurements/gps. Este topic NO va a enviar info en ningún momento ya que la clave del proyecto es trabajar sin GPS.

--

### Lanzamiento MOD. REPO AS2 EX.

Abrir al menos dos terminales.

1. Exportar variables en ambas terminales

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_humble_ws/src/urjc-excavation-world/worlds &&
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_humble_ws/src/urjc-excavation-world/models
```
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_humble_ws/src/gazebo_cave_world/worlds &&
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_humble_ws/src/gazebo_cave_world/worlds/models
```

2. En la terminal 1, lanzar el dron y la simulación:

```bash
root@vera-ASUS-TUF:~/ros2_humble_ws/src/project_gazebo_as2# ./launch_as2.bash 
# opción '-b' hace build del docker
```


3. En la terminal 2, lanzar la GCS:

> [!NOTE]  
> La teleoperación no funciona debido a una ¿incompatibilidad de versiones? Una alternativa a lanzar la GCS con el script es lanzarlo manualmente con el comando : ros2 run as2_alphanumeric_viewer as2_alphanumeric_viewer_node --ros-args -r __ns:=/drone0 
> En otra terminal se lanza entonces la misión a ejecutar, que debe estar predefinida en e.g. un fichero python : python3 mission.py

> [!WANING]
> Si se lanza una misión, cambiar el plugin utilizado para el takeoff. El plugin que controla el takeoff behaviour por posición no funciona, seguramente por las características del mapa.

Al lanzar una mision, hay que tener en cuenta que el mapa utilizado tiene el origen en z = -5.78. También hay que saber que, el dron, al hacer takeoff, necesita comparar la altura necesaria para dar como success la acción o no. Esta comparación se hace con respecto a la info publicada en el topic 'pose':

```python
# extracto definición objeto 'TakeoffBase' :
# aerostack2/as2_behaviors/as2_behaviors_motion/takeoff_behavior/include/takeoff_behavior/takeoff_base.hpp
virtual void state_callback(
    geometry_msgs::msg::PoseStamped & pose_msg,
    geometry_msgs::msg::TwistStamped & twist_msg)
  {
    actual_pose_ = pose_msg;

    feedback_.actual_takeoff_height = actual_pose_.pose.position.z;
    feedback_.actual_takeoff_speed = twist_msg.twist.linear.z;

    localization_flag_ = true;
    return;
  }
```

```bash
root@vera-ASUS-TUF:~/ros2_humble_ws/src/project_gazebo_as2# ./launch_ground_station.bash -t
```

El argumento `-t` es para utilizar la plataforma de teleoperación. Otra alternativa es crear una misión (con BT o con fichero .py) y lanzarla. 
Para utilizar la teleoperación hay que instalar un nuevo paquete:

En el Dockerfile:

```bash
python3 -m pip install --upgrade --extra-index-url https://PySimpleGUI.net/install PySimpleGUI 
```

**Nota :** este comando no es el que viene en la documentación de Aerostack2, ya que en esta te da el comando de instalación anterior. Al crear el docker y lanzar la opción de teleoperación anterior, te da el comando de arriba para que instales la nueva versión.

---

Para que se lance la simulación con el mundo del repositorio `urjc-excavation` es necesario cambiar el fichero de configuración que lanza el script `launch_as2.bash`.
Este fichero es `world.yaml`:

```yaml
world_name: "urjc_excavation_model"
origin: 
    latitude: 0.0
    longitude: 0.0
    altitude: 0.0
drones:
  - model_type: "quadrotor_base"
    model_name: "drone0"
    flight_time: 60
    xyz:
      - 0.0
      - 0.0
      - 0.3
    payload:
      - model_type: "gps"
        model_name: "gps"
      - model_type: "gimbal_speed"
        model_name: "gimbal"
        payload:
          model_type: "hd_camera"
          model_name: "camera"
```

Otra opción es cambiar la ruta del fichero de configuración por la del `json` configurado en el paso '0' del lanzamiento manual de aerostack:

```bash
# FILE : launch_as2.bash
# Set simulation world description config file
if [[ ${swarm} == "true" ]]; then
  # given config file
  simulation_config="config/world_swarm.yaml"
else
  # my config file
  simulation_config="/root/ros2_humble_ws/src/drone_pkg/config/config_file_prueba.json"
fi
```

---

# TÉCNICAS DE SLAM

## Lidar SLAM

Para utilizar el paquete [lidarslam_ros2](lidarslam_ros2) se descargan las librerías `ros-humble-libg2o` y `libeigen3-dev` en el docker, por lo que se añade como comando adicional en el Dockerfile.
Además, para la visualización del mapa creado (en PointCloud), es cómodo utilizar la librería `pcl-tools`, por lo que se añade también.

Para lanzar lidarslam:

```bash
$ ros2 launch lidarslam lidarslam.launch.py
```

Este comando abre una ventana de rviz. Sin embargo, para que funcione con as2 es necesario cambiar lo siguiente en la configuración del paquete.

### Lidarslam + as2

Para empezar, hay que hacer que el dron de la simulación lleve como `payload`un sensor lidar. 
Aerostack2 da soporte con el paquete [as2_gazebo_assets](https://github.com/aerostack2/aerostack2/blob/main/as2_simulation_assets/as2_gazebo_assets/README.md#sensor-models) a 3 tipos de lidar diferentes :

1. **point_lidar** : 	Single point lidar with 40 meter range.

2. **planar_lidar** :	Planar scanning two-dimension lidar with 30 meter range.

3. **3d_lidar** :	Three-dimensional scan with 100 meter range.



Para montarlo, en el fichero de configuración, se añade la siguiente información (`world.yaml` de project_gazebo_as2):

```yaml
world_name: "urjc_excavation_model"
origin: 
    latitude: 0.0
    longitude: 0.0
    altitude: 0.0
drones:
  - model_type: "quadrotor_base"
    model_name: "drone0"
    flight_time: 60
    xyz:
      - 0.0
      - 0.0
      - 0.3
    payload:
      - model_type: "lidar_3d"
        model_name: "lidar_3d"
```

Al añadir como payload el sensor lidar, se crean 2 nuevos topics: `/drone0/sensor_measurements/lidar_3d/scan` y `/drone0/sensor_measurements/lidar_3d/points`

Estos tienen la siguiente interfaz: `sensor_msgs/msg/LaserScan`

```bash
$ ros2 interface show sensor_msgs/msg/LaserScan
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
  builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec
  string frame_id
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
```

Al intentar representar la información en rviz que manda el dron por los topics:
`/drone0/sensor_measurements/lidar_3d/scan` y `/drone0/sensor_measurements/lidar_3d/points` 
aparece el siguiente mensaje:
```bash
[rviz2-4] [INFO] [1755625451.805852742] [rviz]: Message Filter dropping message: frame 'drone0/lidar_3d/lidar_3d/gpu_ray' at time 778.700 for reason 'discarding message because the queue is full'
```

Para evitarlo, hay que hacer que el fixed frame de rviz sea `/drone0/map`. Esto es necesario para que rviz pueda representar adecuadamente los 
valores que envía el drone con respecto al mapa.

Este problema esta relacionado con que no se están publicando las tf *base_link* y *velodyne* (buscadas por lidarslam para crear el mapa) del mundo, aunque del drone sí. Por lo que hay que publicarlas. El problema por el que no se conecta tiene que ver con el namespace de las tfs publicadas por lidarslam.

```python 

#launcher lidarslam.launch.py
# hardcoreado el namespace utilizado por aerostack --> 'drone0'
ns = 'drone0' + '/'

tf = launch_ros.actions.Node(
  package='tf2_ros',
  executable='static_transform_publisher',
  arguments=['0','0','0','0','0','0','1', ns + 'base_link', ns + 'velodyne']
  )

```

En el *yaml* de configuración (`lidarslam.yaml`) también hay que cambiar el namespace de las tfs:
```yaml
global_frame_id: "drone0/map"
robot_frame_id: "drone0/base_link"
``` 

y añadir la correspondiente a odom y el parámetro:
```yaml
odom_frame_id: "drone0/odom"
use_sim_time: true
```

Además de tener que añadir el namespace a las tfs correspondientes, también hay que cambiar el topic remapeado en el que se publican los puntos
del lidar. Esto es, en el launcher de lidarslam, es necesario cambiar:

```python 

#launcher lidarslam.launch.py
mapping = launch_ros.actions.Node(
  package='scanmatcher',
  executable='scanmatcher_node',
  parameters=[main_param_dir],
  remappings=[('/input_cloud','/drone0/sensor_measurements/lidar_3d/points')],
  # anteriormente:
  # remappings=[('/input_cloud','/velodyne_points')],
  output='screen'
  )

```

Por otro lado, para agilizar el proceso de rviz, lidarslam cuenta con un fichero de configuración (`mapping.rviz`) para que salgan ya seleccionados los topics con los que está trabajando. En este fichero está la TF *map* como el *fixed framed*. Sin embargo, para poder integrarlo con as2, es encesario que tenga también el namespace. Es decir:
```
Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: drone0/map
    Frame Rate: 30
```

Para guardar el mapa creado hasta el momento se ejecuta el siguiente servicio:

```bash
ros2 service call /map_save std_srvs/Empty
```

Que guarda en el directorio en el que se ejecuta el comando dos ficheros:

1. pose_graph.g2o : grafo de optimización
2. map.pcd : pointcloud

Para visualizar el mapa creado en forma de pcl (`pcd`), se utiliza *pcl_viewer*:

```bash
pcl_viewer map.pcd
```

Para obtener información del grafo de optimización (`g2o`), se utiliza *g2o*:

```bash
g2o pose_graph.g2o 
```


## ORB SLAM 3

Clonar el repo del siguiente enlace: [orb slam 3](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/c%2B%2B14_comp)

**Es importante clonar la rama c++14, ya que si no no funciona la compilación en esta versión de Ubuntu**

Para usar el paquete orb slam 3, hace falta suplir las siguientes dependencias:

```
# Dockerfile

libopencv-dev \
ninja-build \
wayland-protocols \
libc++-dev \
libepoxy-dev \
```

Además de estos paquetes, es necesario instalar Pangolin. 

---

### Pangolin
Esta dependencia es un paquete en sí, con sus propias dependencias:

```
# Dockerfile

libpython2.7-dev \
libglew-dev \
libgl1-mesa-dev \
libgtk-3-dev \
pkg-config \
libavcodec-dev libavformat-dev libswscale-dev \
libjpeg-dev libpng-dev libtiff-dev libopenexr-dev \
libxkbcommon-dev wayland-protocols \
libboost-all-dev \
libsuitesparse-dev \
ros-humble-cv-bridge \
```

Para compilar Pangolin, se puede hacer de forma manual, o bien, ejecutar el fichero [third_parties](#compilación-e-instalación-de-librerías---thirdparties) del docker.

---

Para este tipo de SLAM, se utiliza una cámara. En este caso, el paquete [as2_gazebo_assets](https://github.com/aerostack2/aerostack2/blob/main/as2_simulation_assets/as2_gazebo_assets/README.md#sensor-models) de aerostack, proporciona 4 tipos de cámaras para gazebo :

1. **hd_camera** :	RGB Camera with 1280x960 resolution.
2. **vga_camera** :	RGB Camera with 640x480 resolution.
3. **semantic_camera** :	RGB Camera with 1280x960 resolution with semantic segmentation data.
4. **rgbd_camera** :	RGBD Camera with 640x480 resolution and 10 meters of depth sensing.

> [!WARNING]
> vga_camera info es incorrecta. El topic camera_info publica:  `height: 960 width: 1280`

Utilizada la cámara RGBD. 

Para añadir una cámara como payload del dron, se edita el fichero `yaml` de configuración del mundo (world.yaml):

```yaml
world_name: "urjc_excavation_model"
origin: 
    latitude: 0.0
    longitude: 0.0
    altitude: 0.0
drones:
  - model_type: "quadrotor_base"
    model_name: "drone0"
    flight_time: 60
    xyz:
      - 0.0
      - 0.0
      - 0.3
    payload:
      - model_type: "rgbd_camera"
        model_name: "rgbd_camera"
```

Al lanzar el simulador con el nuevo payload se crean los siguientes topics:

- /drone0/sensor_measurements/rgbd_camera/camera_info
- /drone0/sensor_measurements/rgbd_camera/depth
- /drone0/sensor_measurements/rgbd_camera/depth/camera_info
- /drone0/sensor_measurements/rgbd_camera/image_raw
- /drone0/sensor_measurements/rgbd_camera/points

Según la información del paquete ORB-SLAM3, es necesario calibrar la cámara antes de utilizarla. Ya que de momento se va a utilizar una cámara simulada en gazebo, no es necesario este proceso. 
Lo que si que es necesario es crear un fichero `yaml`de configuración con los parámetros característicos de la cámara (intrínsecos, extrínsecos, distancia y centro focal).

Para completar el fichero de configuración, cambio el fichero de ejemplo del propio paquete ORB_SLAM3, ajustando los valores a aquellos que porporciona el topic `camera_info`:

```bash
---
header:
  stamp:
    sec: 187
    nanosec: 600000000
  frame_id: /drone0/rgbd_camera/rgbd_camera/rgbd_camera/optical_frame
height: 480
width: 640
distortion_model: plumb_bob
d:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
k:
- 554.3
- 0.0
- 320.5
- 0.0
- 554.3
- 240.5
- 0.0
- 0.0
- 1.0
r:
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
p:
- 277.0
- 0.0
- 160.0
- 0.0
- 0.0
- 277.0
- 120.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: false
---
```

Dados estos valores, el fichero de configuración resulta en:
```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------
Camera.type: "PinHole"

# Camera calibration and distortion parameters (OpenCV)
Camera.fx: 554.3
Camera.fy: 554.3
Camera.cx: 320.5
Camera.cy: 240.5


# distortion parameters
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

# Camera resolution
Camera.width: 640
Camera.height: 480

# Camera frames per second 
Camera.fps: 30.0

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 0

# Image scale, it changes the image size to be processed (<1.0: reduce, >1.0: increase)
Camera.imageScale: 1.0 # 0.5 #0.7071 # 1/sqrt(2)

# ==========================
# Depth configuration
# ==========================
# Si depth es 32FC1 en metros (lo habitual en Gazebo): 1.0
# Si depth fuera 16UC1 en milímetros: 1000.0
# Close/Far threshold. Baseline times.
# Deptmap values factor
DepthMapFactor: 1.0

ThDepth: 40.0

# stereo baseline times fx
Camera.bf: 0.0

# ==========================
# (Opcional) IMU extrinsics (body -> camera)
# Si no usas IMU, déjalo identidad.
# ==========================
# Transformation from body-frame (imu) to left camera
Tbc: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1]

# Do not insert KFs when recently lost
InsertKFsWhenLost: 0

# IMU noise (Use those from VINS-mono)
IMU.NoiseGyro: 1e-3 # 2.44e-4 #1e-3 # rad/s^0.5
IMU.NoiseAcc: 1e-2 # 1.47e-3 #1e-2 # m/s^1.5
IMU.GyroWalk: 1e-6 # rad/s^1.5
IMU.AccWalk: 1e-4 # m/s^2.5
IMU.Frequency: 200

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------
# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1250

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500
```

## ORB SLAM3 PARA ROS2

Uno de los principales problemas de ORB-SLAM3 es que está diseñado para ros1 (Melodic, para Ubuntu 18.04).
Es por eso que para que funcione con las imágenes captadas por el dron desde el módulo de Aerostack2 hay varias opciones:

1. offline : grabar un rosbag con la información publicada en los topics que le interesan a ORB-SLAM3. Una vez grabada, se pasaría al módulo de slam a través de (por ejemlo) un nodo para que lo vaya procesando frame por frame
2. online : crear (o buscar) una versión que haga un bridge entre ros2 humble y orb-slam3. 

### Opción 2

Para acelerar el proceso, utilizo el código del repositorio [ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2).

Para poder utilizarlo, se clona y se añade al fichero de *docker compose* como :

```yaml
../ORBSLAM3_ROS2:/root/ros2_humble_ws/src/orb_slam3_ros2
```

Dentro, para evitar posibles conflictos con la compilación, se le cambia el nombre al paquete en el package.xml y en el CMakeLists.txt al asignado en el docker (orb_slam3_ros2).

Este repositorio incluye los nodos necesarios para poder lanzar 4 tipos de cámaras:
1. monocular
2. rgbd
3. stereo
4. stereo-inertial

Para lanzar cada uno, se sigue el siguiente formato (información extraída del README del repositorio  original):
```
$ ros2 run orb_slam3_ros2 NODE PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```

Al lanzar el nodo `rgbd`, surge el siguiente error:
```bash
Shutdown

Saving keyframe trajectory to KeyFrameTrajectory.txt ...
double free or corruption (out)
[ros2run]: Aborted
```

Este problema estaba reportado y solucionado en los [issues](https://github.com/zang09/ORB_SLAM3_ROS2/issues/24) del repositorio. Está relacionado con la forma con la que se creaban los suscritores a los topics. 

#### Conexión con Gazebo

Primero es necesario crear el fichero de configuración de la cámara que necesita ORB-SLAM3 para funcionar según los parámetros de la cámara simulada en Gazebo Ignition.

! Este fichero se detalla en el apartado anterior. Se guarga dentro del paquete `ORBSLAM3_ROS2/config/rgb-d/rgbd_gazebo.yaml` .  

Para lanzar entonces el nodo con este fichero de configuración:
```bash
$ ros2 run orb_slam3_ros2 rgbd orb_slam3_ros2/vocabulary/ORBvoc.txt orb_slam3_ros2/config/rgb-d/rgbd_gazebo.yaml 
```

Este nodo se suscribe a los topics `camera/rgb` y `camera/depth`. Ahora, se necesita hacer un remaping de estos topics de entrada de los nodos con con los de gazebo y así poder utilizar la cámara simulada del dron de aerostack2.
Para hacer este 

#### -- EXTRA  

En ORB_SLAM3 hay una clase `System.h` que lo que hace es dar un api para comunicarse con el paquete.

el constructor es:
```cpp
System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string());
```
donde los argumentos obligatorios son:
1. El fichero del vocabulario
2. El fichero `yaml` de configuración de la camara
3. El sensor que se va a utilizar



# CREACIÓN DE MAPAS

Crear un mapa navegable a partir de un pointcloud puede signficar, o bien crear un mapa 2D, o bien uno 3D.

## Mapa 2D vs 3D

## Crear mapas 3D

Hay varias opciones, ya desarrolladas para crear un mapa 3D dado un pcl. A continucación, se compara Octomap con Bonxai.

Ambos crean una estructura de datos:
- Dispersa (sparse): almacena solo los vóxeles ocupados
- Ilimitada (unbounded): no necesita que se definan los límites del espacio con antelación, es decir, crece dinámicamente.

Para nubes de puntos muy densas -> mejor Bonxai.

### Octomap

Crea un mapa a partir de un pcl agrupándolos en voxels y utilizando una estructura jerárquica (**Octree**).

Clonar el repositorio de [octomap para ros2](https://github.com/Taeyoung96/OctoMap-ROS2/tree/master). Este trae un brige para ros2 humble y octomap con la información de una nube de puntos. En este caso, la nube de puntos es la que da como output lidarslam y a partir de la cual crea el fichero map.pcd (pointcloud data).

El paquete trae un launcher un poco antiguo (en formato xml), por lo que creo otro:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='octomap_server',
      executable='octomap_server_node',
      name='octomap_server',
      output='screen',
      parameters=[{
          'frame_id': 'drone0/map',
          'base_frame_id': 'base_link',
          'resolution': 0.5, # tamaño de voxel (m)
          'sensor_model/max_range': 20.0,
          'occupancy_min_z': 0.1,
          'occupancy_max_z': 1.0,
      }],
      remappings=[
          ('cloud_in', '/map'),
      ]
    )
])
```

La información sobre lo que significa cada parámetro se puede encontrar en la web de [octomap](https://wiki.ros.org/octomap_server#Parameters).

Se debe lanzar después de la plataforma de vuelo, de la GCS y de LidarSLAM con el siguiente comando:

`ros2 launch drone_pkg octomap.launch.py`

Una vez lanzado, se ejecuta la misión para que obtenga información. 


### Bonxai

Más rápido que Octomap. no utiliza una estructura jerárquica si no que accede a los datos directamente mediante claves.

Para usar Bonxai, utilizar el plugin del paquete [Easy Navegation](https://github.com/EasyNavigation/EasyNavigation)

# NUEVO - MAPA CAVERNA 

1. Abrir X terminales con el docker lanzado y el entorno ros2 compilado e inicializado.
2. TERMINAL 1: Lanzar el mapa e iniciar el dron lanzando el fichero `project_gazebo_as2/launch_as2_lidar.bash`
3. TERMINAL 2: Lanzar la GCS con el comando `ros2 run as2_alphanumeric_viewer as2_alphanumeric_viewer_node --ros-args -r __ns:=/drone0`
4. TERMINAL 3: Lanzar el nodo `drone_pkg/drone_nav.cpp`
5. TERMINAL 4: Grabar un rosbag con todos los topics y así poder ejecutar después lidarslam y octomap/bonxai.



colcon-argcomplete.bash

# EasyNav

Para hacer el mapeado se hace este fichero de configuracion:

```yaml

controller_node:
  ros__parameters:
    use_sim_time: true
    controller_types: [dummy]
    dummy:
      rt_freq: 30.0
      plugin: easynav_controller/DummyController
      cycle_time_nort: 0.01
      cycle_time_rt: 0.001

localizer_node:
  ros__parameters:
    use_sim_time: true
    localizer_types: [dummy]
    dummy:
      rt_freq: 50.0
      freq: 5.0
      reseed_freq: 0.1
      plugin: easynav_localizer/DummyLocalizer
      cycle_time_nort: 0.01
      cycle_time_rt: 0.001

maps_manager_node:
  ros__parameters:
    use_sim_time: true
    map_types: [bonxai, navmap]
    bonxai:
      freq: 10.0
      plugin: easynav_bonxai_maps_manager/BonxaiMapsManager
      package: drone_pkg
      # bonxai_path_file: maps/new_bonxai_map.pcd
    navmap:
      freq: 10.0
      plugin: easynav_navmap_maps_manager/NavMapMapsManager
      package: drone_pkg
      # navmap_path_file: maps/map.navmap

planner_node:
  ros__parameters:
    use_sim_time: true
    planner_types: [dummy]
    dummy:
      freq: 1.0
      plugin: easynav_planner/DummyPlanner
      cycle_time_nort: 0.2
      cycle_time_rt: 0.001

sensors_node:
  ros__parameters:
    use_sim_time: true
    forget_time: 0.5
    perception_default_frame: odom

system_node:
  ros__parameters:
    use_sim_time: true
    use_real_time: false
    position_tolerance: 0.3
    angle_tolerance: 0.15

```

Lo que hace es que pone todos los nodos inactivos (dummy) excepto el 'mapeador' (maps_manager_node). Ahi se ponen a n¡bonxai y a navmap como los programas necesarios.
! Es importante tener Bonxai clonado dentro del proyecto

Para guardar los mapas:

```bash
ros2 service call /maps_manager_node/navmap/savemap std_srvs/srv/Trigger #NAVMAP
ros2 service call /maps_manager_node/bonxai/savemap std_srvs/srv/Trigger # BONXAI
``` 

**NOTA** : para que se guarden los mapas creados ambos path_files deben estar sin rellenar (notar como en el snippet están comentados)

## Problema 1

El nodo drone_nav tiene un problema de míinimos locales. Hasta el momento seguía esta lógica de programa para la navegación:

- Comprueba que no haya ningún obstáculo cercano en los laterales (umbral de seguridad lateral). Si lo hay:
  - Gira hacia el lado contrario (v = 0, w > 0) hasta que la distancia sea mayor que el umbral de seguridad

- Comprueba que no haya ningún obstáculo al frente (unmbral de seguridad frontal). Si lo hay:
  - Comprueba que lateral está más alejado del dron y gira hacia él (v = 0, w > 0)

- En el caso de que no haya ningún obstáculo pasado los umbrales de seguridad, vuela recto (v > 0, w = 0)

Este algoritmo de navegación local aunque es eficiente tiene un problema bastante común, se dan casos de **mínimos locales** en esquinas, paredes cóncavas o pasillos estrechos. 
A nivel de software sucede en la 2ª condición (con obstaculos frontales, girar hacia el lado más lejano).
Para solucionarlo, se '**fija**' la dirección de giro hacia aquel lado en el que al detectar el obstculo frontal se encuentra más alejado. La condición de parada es ahora que el obstaculo frontal se encuentre un poco mas lejano que el umbral frontal.

## Problema 2

Al guardar el mapa de Bonxai este solo contiene los puntos del pcl que se publican en el instante de llamar al servicio. Por ello, para conseguir el PCD (objetivo) hay varias alternativas:
1. El nodo que se encarga de la navegacion (drone_nav) no publique solo los puntos recopilados en ese instante, si no que vaya publicando el pcl de forma incremental. 


El problema con esto es la carga computacional que supone almacenar un pcl de dimensiones desconocidas a priori.

2. Crear un nodo auxiliar que vaya recibiendo los datos del pcl y los vaya almacenando y publicando. Grabar un rosbag con su salida y que sea esa la entrada de easynav.

Problema: complejidad.

3. No usar un bonxai para crear el PCD, si no lidarlam, ya testeado en el proyecto y comprobado que funciona.

Problema: no integrado en easynav, lo cual es prioritario

