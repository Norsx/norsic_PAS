# Zadatak 2 - D* Path Planning

ROS2 paket za navigaciju mobilnog robota s D* algoritmom.

## Kloniranje Repozitorija

```bash
# Kloniraj samo zadatak2_01 branch
git clone -b zadatak2_01 --single-branch https://github.com/Norsx/norsic_PAS.git ~/Documents/PAS/zadatak2
cd ~/Documents/PAS/zadatak2
```

## Setup

```bash
cd ~/Documents/PAS/zadatak2/pas/zadatak2
colcon build --packages-select zadatak2
source install/setup.bash
```

## Pokretanje

**Terminal 1 - Lokalizacija:**
```bash
cd ~/Documents/PAS/zadatak2/pas/zadatak2
source install/setup.bash
ros2 launch zadatak2 localization_complete_launch.py map_name:=map_02
```

**Terminal 2 - D* Planner:**
```bash
cd ~/Documents/PAS/zadatak2/pas/zadatak2
source install/setup.bash
ros2 launch zadatak2 d_star_path_planner.launch.py
```

**U RViz:** Klikni "2D Goal Pose" i postavi cilj na mapi.

## Dostupne Mape

- `map_name:=map_01`
- `map_name:=map_02`

## Ostali Launch Fajlovi

```bash
# A* algoritam
ros2 launch zadatak2 a_star_path_planner.launch.py

# Mapiranje
ros2 launch zadatak2 mapping_complete_launch.py

# Nav2 navigacija
ros2 launch zadatak2 navigation_complete_nav2.launch.py
```
