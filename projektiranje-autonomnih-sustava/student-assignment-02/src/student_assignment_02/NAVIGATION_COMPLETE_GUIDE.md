# Navigation Complete Guide

## Pregled

Ova datoteka opisuje kako koristiti kompletnu navigacijsku konfiguraciju koja integrira:
- **A* Path Planning** - planiranje putanje koristeći A* algoritam
- **Path Following** - slijenje putanje koristeći Pure Pursuit algoritam
- **RViz Visualization** - vizualizacija u RViz-u

## Struktura

```
launch/
├── navigation_complete.launch.py    # Glavni launch file
├── a_star_path_planner.launch.py   # A* planer (zastarjelo, koristi navigation_complete)
└── ...

student_assignment_02/
├── a_star_path_planner.py          # A* algoritam
├── path_follower_node.py           # Pure Pursuit kontroler (NOVO)
├── goal_navigation_node.py         # Navigation node
└── ...

setup.py                            # Dodaj 'path_follower' entry point
```

## Instalacija

### 1. Rebuild paketa

```bash
cd ~/ros2_ws
colcon build --packages-select student_assignment_02
source install/setup.bash
```

### 2. Provjera instalacije

```bash
ros2 pkg executables student_assignment_02 | grep -E "path_follower|a_star"
```

Očekivani output:
```
student_assignment_02 a_star_path_planner
student_assignment_02 path_follower
...
```

## Korištenje

### Osnovna komanda

```bash
ros2 launch student_assignment_02 navigation_complete.launch.py
```

### S argumentima

```bash
# Sa prilagođenim golom
ros2 launch student_assignment_02 navigation_complete.launch.py \
    goal_x:=10.0 \
    goal_y:=10.0 \
    start_x:=0.0 \
    start_y:=0.0

# Sa prilagođenim parametrima Pure Pursuit
ros2 launch student_assignment_02 navigation_complete.launch.py \
    lookahead_distance:=0.5 \
    max_linear_velocity:=1.0 \
    max_angular_velocity:=2.0

# Sa bufferm oko prepreka
ros2 launch student_assignment_02 navigation_complete.launch.py \
    inflation_distance_m:=0.3 \
    inflation_radius:=2
```

## Čvorovi i Topici

### A* Path Planner Node

**Executable:** `a_star_path_planner`

**Subscriptions:**
- `/map` (nav_msgs/OccupancyGrid) - mapa s topologijom
- `/goal_pose` (geometry_msgs/PoseStamped) - dinamički goal iz RViza

**Publishers:**
- `/planned_path` (nav_msgs/Path) - pronađena putanja
- `/path_planning_visualization` (visualization_msgs/MarkerArray) - istraživane stanice
- `/planning_frontier` (visualization_msgs/MarkerArray) - čelna fronta pretraživanja
- `/inflation_buffer_visualization` (visualization_msgs/MarkerArray) - inflation buffer zona

**Parametri:**
- `goal_x` - X koordinata cilja (m) [default: 5.0]
- `goal_y` - Y koordinata cilja (m) [default: 5.0]
- `start_x` - X koordinata početka (m) [default: 0.0]
- `start_y` - Y koordinata početka (m) [default: 0.0]
- `allow_diagonal` - dozvoli dijagonalno kretanje [default: true]
- `inflation_radius` - inflation buffer u stanicama [default: 1]
- `inflation_distance_m` - inflation buffer u metrima [default: 0.5]
- `inflation_cost_threshold` - threshold za prepreke [default: 60]
- `max_iterations` - maksimalan broj iteracija A* [default: 50000]
- `search_radius` - ograničenje pretraživanja (-1 = bez) [default: -1]

### Path Follower Node

**Executable:** `path_follower`

**Subscriptions:**
- `/planned_path` (nav_msgs/Path) - putanja od A* planera

**Publishers:**
- `/cmd_vel` (geometry_msgs/Twist) - zapovijedne brzine za robota

**Parametri:**
- `lookahead_distance` - lookahead distanca za Pure Pursuit (m) [default: 0.3]
- `max_linear_velocity` - maksimalna linearna brzina (m/s) [default: 0.5]
- `max_angular_velocity` - maksimalna kutna brzina (rad/s) [default: 1.0]
- `goal_tolerance` - tolerancija do cilja (m) [default: 0.1]
- `path_topic` - topic putanje [default: '/planned_path']
- `cmd_vel_topic` - topic zapovijednih brzina [default: '/cmd_vel']

## Tok podataka

```
┌─────────────────────────────────────────────────┐
│  ROS 2 Sustav                                   │
├─────────────────────────────────────────────────┤
│                                                 │
│  /map topic (OccupancyGrid)                    │
│      │                                          │
│      ↓                                          │
│  ┌─────────────────────────────────┐           │
│  │ A* Path Planner                 │           │
│  │ (a_star_path_planner)          │           │
│  │                                 │           │
│  │ - Planira putanju               │           │
│  │ - Vizualizira pretraživanje     │           │
│  └─────────────────────────────────┘           │
│      │                                          │
│      ↓                                          │
│  /planned_path (nav_msgs/Path)                 │
│      │                                          │
│      ↓                                          │
│  ┌─────────────────────────────────┐           │
│  │ Path Follower                   │           │
│  │ (path_follower)                │           │
│  │                                 │           │
│  │ - Sljedi putanju                │           │
│  │ - Pure Pursuit kontrola         │           │
│  └─────────────────────────────────┘           │
│      │                                          │
│      ↓                                          │
│  /cmd_vel (geometry_msgs/Twist)                │
│      │                                          │
│      ↓                                          │
│  Robot (Stage/Gazebo)                          │
│                                                 │
└─────────────────────────────────────────────────┘

/goal_pose (iz RViza)  ──→  A* Path Planner
```

## Primjer korištenja sa Stage Simulatorom

### Terminal 1 - Stage Simulator

```bash
# Pokrenite Stage simulator s mapom
ros2 launch student_assignment_02 stage_launch.py
```

### Terminal 2 - Lokalizacija (ako trebate)

```bash
# AMCL lokalizacija
ros2 launch student_assignment_02 localization_complete_launch.py
```

### Terminal 3 - Navigation (A* + Path Follower)

```bash
ros2 launch student_assignment_02 navigation_complete.launch.py \
    goal_x:=5.0 \
    goal_y:=5.0
```

### Terminal 4 (Opciono) - Postavljanje goal-a u RViz-u

```bash
# U RViz-u:
# 1. Klikni na "2D Goal Pose" tool
# 2. Klikni na kartu da postaviš novu goal poziciju
# A* će automatski replanirati putanju
```

## Troubleshooting

### Problem: "Putanja nije pronađena!"

**Uzrok:** Goal je u prepreci ili nije dostižan

**Rješenje:**
1. Provjerite je li goal u slobodnom prostoru
2. Povećajte `max_iterations` parametar
3. Smanjite `inflation_distance_m` ako je premali razmak

### Problem: Robot se ne miče

**Uzrok:** Path follower nije aktivan ili nema putanje

**Rješenje:**
1. Provjerite je li `/planned_path` topic objavljen:
   ```bash
   ros2 topic echo /planned_path --once
   ```
2. Provjerite je li `/cmd_vel` topic aboniran:
   ```bash
   ros2 topic list | grep cmd_vel
   ```
3. Provjerite je li robot u dobroj poziciji na mapi

### Problem: Spora navigacija

**Uzrok:** Premale lookahead_distance ili max_linear_velocity

**Rješenje:**
```bash
ros2 launch student_assignment_02 navigation_complete.launch.py \
    lookahead_distance:=0.5 \
    max_linear_velocity:=1.0
```

### Problem: Robot se vrtio oko sebe

**Uzrok:** Loša Pure Pursuit tuning ili problem s kutnom brzinom

**Rješenje:**
```bash
# Smanji max_angular_velocity
ros2 launch student_assignment_02 navigation_complete.launch.py \
    max_angular_velocity:=0.5 \
    lookahead_distance:=0.5
```

## Napredni Parametri

### A* Planiranje

- **Brža pretraga:** `max_iterations:=10000` (manje iteracija)
- **Detaljnije:** `max_iterations:=100000` (više iteracija)
- **Sigurnije:** `inflation_distance_m:=1.0` (veći buffer)
- **Tanje putanje:** `allow_diagonal:=false` (4-povezanost)

### Path Following

- **Preciznije:** `lookahead_distance:=0.2` (manja lookahead distanca)
- **Brže:** `lookahead_distance:=0.8` (veća lookahead distanca)
- **Agresivnije:** `max_angular_velocity:=3.0` (brže okretanje)
- **Konzervativnije:** `max_linear_velocity:=0.2` (sporije)

## RViz Vizualizacija

Dodajte u RViz:

1. **Map** - `/map` topic (OccupancyGrid)
2. **Path** - `/planned_path` topic (Path marker)
3. **Markers** - `/path_planning_visualization` (istraživane stanice)
4. **Markers** - `/planning_frontier` (čelna fronta)
5. **Markers** - `/inflation_buffer_visualization` (inflation zona)
6. **Arrows** - `/cmd_vel` (trenutne zapovijedne brzine)

## Čest korišteni Launching Scenariji

### Scenarij 1: Samo planiranje bez sljeđenja

```bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py \
    goal_x:=10.0 goal_y:=10.0
```

### Scenarij 2: Planiranje + slijenje

```bash
ros2 launch student_assignment_02 navigation_complete.launch.py \
    goal_x:=10.0 goal_y:=10.0
```

### Scenarij 3: Dinamički goal iz RViza

```bash
# Pokrenite s RViz-om
ros2 launch student_assignment_02 navigation_complete.launch.py

# U RViz-u koristite "2D Goal Pose" tool za postavljanje novih ciljeva
```

## Dalje Razvoj

Možete proširiti sustav s:

1. **Custom kontroleri** - zamjena Pure Pursuit s drugim strategijama
2. **Avoidance** - dinamičko izbjegavanje prepreka
3. **Multi-goal** - putanja kroz više ciljeva
4. **Local planner** - Nav2 local planner integracija
5. **MPC kontrola** - model predictive control za preciznije slijenje

## Licence

Apache 2.0

## Autor

Kresimir Hartl (kh239762@fsb.hr)
