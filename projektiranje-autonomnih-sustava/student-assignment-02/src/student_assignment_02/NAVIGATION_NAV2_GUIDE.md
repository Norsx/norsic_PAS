# Navigation s Nav2 Guideom

## 🎯 Pregled

Ovo je **PREPORUČENI pristup** za robotsku navigaciju s A* globalnim planerom i Nav2 lokalnim planerom.

```
A* Global Planer (Planira putanju)
          ↓ (/planned_path)
Nav2 Adapter (Preusmjerava putanju)
          ↓ (FollowPath akcija)
Nav2 Local Planer (Sljedi putanju)
          ↓ (/cmd_vel)
Robot (Se vrtoj!)
```

## 📦 Što je Instalirano

- **A* Path Planner** - Globalni planer koji planira putanju po mapi
- **Nav2 Stack** - Kompletan navigacijski sustav (lokalni planer, kontroler, itd.)
- **Nav2 Adapter** - Srednji sloj koji povezuje A* s Nav2
- **RViz** - Vizualizacija

## 🚀 Brzina Start

### 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-select student_assignment_02
source install/setup.bash
```

### 2. Pokrenite s Stage Simulatorom

**Terminal 1 - Stage:**
```bash
ros2 launch student_assignment_02 stage_launch.py
```

**Terminal 2 - Lokalizacija (AMCL):**
```bash
ros2 launch student_assignment_02 localization_complete_launch.py
```

**Terminal 3 - Navigation s Nav2:**
```bash
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py \
    goal_x:=5.0 \
    goal_y:=5.0
```

## 🎮 Korištenje

### Osnovna komanda

```bash
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py
```

### S argumentima

```bash
# Postavi goal
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py \
    goal_x:=10.0 \
    goal_y:=10.0

# Sa većim bufferom oko prepreka
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py \
    inflation_distance_m:=1.0 \
    goal_x:=3.0 \
    goal_y:=3.0
```

### Dinamički Goal iz RViz-a

1. RViz se pokreće s launch datotekom
2. Koristite "2D Goal Pose" tool
3. Klikni na mapu da postaviš novu goal poziciju
4. A* će automatski replanirati
5. Nav2 će pratiti novu putanju

## 🔌 Arhitektura

### Čvorovi

| Čvor | Paket | Uloga |
|------|-------|-------|
| `a_star_path_planner` | student_assignment_02 | Globalni planer |
| `planner_server` | nav2_planner | Globalni planer (Nav2) - **ISKLJUČEN** |
| `controller_server` | nav2_controller | Lokalni planer - **AKTIVAN** |
| `nav2_adapter` | student_assignment_02 | Adapter A* → Nav2 |
| `lifecycle_manager` | nav2_lifecycle_manager | Upravljanje lifecycle-om |
| `velocity_smoother` | nav2_velocity_smoother | Glačanje brzina |
| `bt_navigator` | nav2_bt_navigator | Behavior tree navigator |

### Topici

| Topic | Tip | Smjer | Opis |
|-------|-----|-------|------|
| `/map` | OccupancyGrid | IN | Mapa s topologijom |
| `/planned_path` | Path | A* → Adapter | Putanja od A* |   
| `/cmd_vel` | Twist | OUT | Zapovijedne brzine |
| `/odom` | Odometry | IN | Odometrija robota |
| `/scan` | LaserScan | IN | LIDAR senzor (ako postoji) |

### Akcije

| Akcija | Tip | Opis |
|--------|-----|------|
| `navigate_to_pose` | NavigateToPose | Nav2 akcija za navigaciju |
| `follow_path` | FollowPath | Nav2 akcija za sljenje putanje |

## ⚙️ Konfiguracija

### Nav2 Parametri (`config/nav2_params.yaml`)

Ključni parametri za controller server:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    
    # Regulated Pure Pursuit (preporučeni lokalni planer)
    local_costmap:
      local_costmap:
        use_sim_time: true
        update_frequency: 5.0
        publish_frequency: 2.0
        global_frame: odom
        robot_base_frame: base_link
        rolling_window: true
        width: 3
        height: 3
        resolution: 0.05
        robot_radius: 0.1
```

### A* Parametri

```bash
# Via launch argumenti
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py \
    inflation_distance_m:=0.5     # Buffer oko prepreka (m) \
    goal_x:=5.0                   # Goal X (m)
    goal_y:=5.0                   # Goal Y (m)
```

## 📊 Monitoring

### Provjera topika

```bash
# Provjera putanje od A*
ros2 topic echo /planned_path --once

# Provjera zapovijednih brzina
ros2 topic echo /cmd_vel --once

# Provjera statusu Nav2
ros2 topic echo /amcl_pose
```

### Provjera servisa

```bash
# Lista svih Nav2 servisa
ros2 service list | grep nav2

# Provjera lokalizacije
ros2 service call /global_localization std_srvs/srv/Empty
```

### RViz Vizualizacija

1. **Map** - `/map` (OccupancyGrid)
2. **Path** - `/planned_path` (Path from A*)
3. **LocalCostmap** - `/local_costmap/costmap` (Local planner costmap)
4. **GlobalCostmap** - `/global_costmap/costmap` (Global costmap)
5. **TF** - Svi transformacije (map, odom, base_link, itd.)
6. **Arrows** - `/cmd_vel` (Trenutne zapovijedne brzine)

## 🚨 Troubleshooting

### Problem: Robot se vrti u mjestu

**Uzrok**: Nav2 kontroler ima loše tuning ili nema TF transformacija

**Rješenje**:
```bash
# 1. Provjeri je li AMCL lokalizacija aktivna
ros2 topic echo /amcl_pose

# 2. Provjeri TF stablo
ros2 run tf2_tools view_frames.py
ros2 run tf2_ros tf2_echo map base_link

# 3. Resetiraj lokalizaciju u RViz-u (Estimate Pose)
```

### Problem: Robot ide krivo ili po krivoj putanji

**Uzrok**: Loša kalibracija ili kontroler parametri

**Rješenje**:
```bash
# Smanjite max_vel i povećajte lookahead u nav2_params.yaml
controller:
  min_vel_x: 0.0
  max_vel_x: 0.3        # Smanjenje brzine
  max_vel_theta: 0.5    # Smanjenje kutne brzine
```

### Problem: "Putanja nije pronađena!"

**Uzrok**: Goal je u prepreci ili není dostižan

**Rješenje**:
1. Provjerite je li goal u slobodnom prostoru
2. Povećajte `inflation_distance_m`
3. Poveća `max_iterations` u A* čvoru

### Problem: Nav2 se ne pokreće

**Uzrok**: Nije instaliran ili nema nav2_params.yaml

**Rješenje**:
```bash
# 1. Instalirajte Nav2
sudo apt install ros-$ROS_DISTRO-nav2*

# 2. Provjerite nav2_params.yaml
ls -la ~/ros2_ws/src/student_assignment_02/config/nav2_params.yaml

# 3. Provjerite build
colcon build --packages-select student_assignment_02
```

## 📋 Cijeli Primjer Workflow-a

### Scenarij: Slanja robota s točke A do točke B

**1. Pokrenite sve dijelove sustava**
```bash
# Terminal 1
ros2 launch student_assignment_02 stage_launch.py

# Terminal 2
ros2 launch student_assignment_02 localization_complete_launch.py

# Terminal 3
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py
```

**2. Čekajte da se lokalizacija stabilizira**
- RViz će pokazati "Estimate Pose" tool
- Klikni na mapu gdje je robot u stvarnosti

**3. Postavite goal u RViz-u**
- Koristi "2D Goal Pose" tool
- Klikni na željenu lokaciju

**4. Promatrajte navigaciju**
- A* planira putanju (vidite crvene/žute točke)
- Nav2 sljedi putanju
- Robot se kreće prema cilju

**5. Pratite u terminalima**
```bash
# Terminal 4 - Pratite cmd_vel
ros2 topic echo /cmd_vel

# Terminal 5 - Pratite A* planiranje
ros2 topic echo /planned_path
```

## 🔧 Napredna Tuning

### Brža navigacija

```yaml
# u nav2_params.yaml
controller_server:
  FollowPath:
    max_lookahead_dist: 0.8      # Više lookahead
    min_lookahead_dist: 0.1
    max_vel_x: 1.0              # Brža brzina
    max_vel_theta: 2.0          # Brže okretanje
```

### Preciznija navigacija

```yaml
controller_server:
  FollowPath:
    max_lookahead_dist: 0.3      # Manje lookahead
    min_lookahead_dist: 0.1
    max_vel_x: 0.2              # Spora brzina
    max_vel_theta: 0.5          # Spora okretanja
```

## 📚 Dalje Čitanje

- [Nav2 Documentation](https://nav2.org/)
- [Nav2 Configuration](https://nav2.org/configuration/)
- [ROS 2 Navigation Stack](https://docs.ros.org/en/foxy/Guides/Navigation2-Setup.html)

## ✅ Checklist Pre Korištenja

- [ ] ROS 2 instaliran
- [ ] Nav2 paket instaliran (`ros-$ROS_DISTRO-nav2-*`)
- [ ] student_assignment_02 paketa građen
- [ ] Stage ili drugi simulator instaliran
- [ ] Mapa dostupna u `/map` topiku
- [ ] AMCL lokalizacija pokrenut
- [ ] TF stablo ispravno (map → odom → base_link)

## 🎓 Lekcije

1. **Globalni vs Lokalni Planer** - A* planira putanju, Nav2 je sljedi
2. **Adapter Pattern** - Adapter povezuje različite dijelove sustava
3. **ROS 2 Actions** - Nav2 koristi actions za asinkronu komunikaciju
4. **Lifecycle Management** - Nav2 koristi lifecycle za upravljanje čvorovima
5. **Cost Maps** - Lokalni i globalni costmap za sigurnu navigaciju

---

**Verzija**: 1.0  
**Datum**: 2026-01-08  
**Autor**: Kresimir Hartl
