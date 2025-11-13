text
# Norsic PAS - ROS2 Projekat

## Opis
Ovaj repozitorij sadrži ROS2 pakete i kod za projekt vezan za robotiku.  
Omogućava vizualizaciju i ostale funkcionalnosti za rad sa robotom.

## Upute za pokretanje

### 1. Otvori terminal u mapi gdje želiš klonirati repozitorij

### 2. Kloniraj samo `zadatak1_01` branch u podmapu `src`:
git clone --branch zadatak1_01 --single-branch https://github.com/Norsx/norsic_PAS.git src


### 3. Aktiviraj ROS2 Humble environment:
source /opt/ros/humble/setup.bash


### 4. Buildaj ROS2 workspace:
colcon build


### 5. Sourcaj buildane pakete:
source install/setup.bash


### 6. Pokreni program:
ros2 launch robot_visualize display.launch.py
