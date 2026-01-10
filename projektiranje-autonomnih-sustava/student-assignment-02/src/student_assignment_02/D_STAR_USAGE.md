# D* Path Planner Node

## Pregled

D* (Dynamic A*) algoritam je inkrementalni algoritam za planiranje putanja koji efikasno ažurira putanju kada su promijenjeni troškovi grana (npr. kada robot otkrije nove prepreke tijekom izvršavanja).

## Ključne razlike od A* algoritma

### A* (Statičko planiranje)
- Planira cijelu putanju od početka do kraja
- Koristi g-vrijednost (cijena puta) i h-vrijednost (heurističku procjenu)
- Trebam ponovo planirati ako se mapa promijeni

### D* (Dinamičko planiranje)
- Planira putanju od **cilја prema robotu** (backward search)
- Čuva **rhs-vrijednost** (dorađena vrijednost) za inkrementalne ažuriranja
- Može efikasno ažurirati putanju bez punog replaniranja
- Idealan za robote koji istražuju nepoznate okoline

## Kako pokrenuti D* node

### 1. Kompajliranje

```bash
cd ~/ros2_workspace
colcon build --packages-select student_assignment_02
. install/setup.bash
```

### 2. Pokretanje noda

```bash
ros2 run student_assignment_02 d_star_path_planner
```

## ROS Topics

### Pretplate (Subscribers)

| Topic | Tip | Opis |
|-------|-----|------|
| `/map` | `nav_msgs/OccupancyGrid` | Karta okoline |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Ciljna pozicija iz RViza (2D Goal Pose) |

### Publikacije (Publishers)

| Topic | Tip | Opis |
|-------|-----|------|
| `/d_star_planned_path` | `nav_msgs/Path` | Planirana putanja |
| `/d_star_path_planning_visualization` | `visualization_msgs/MarkerArray` | Istraživane stanice (zelene) |
| `/d_star_planning_frontier` | `visualization_msgs/MarkerArray` | Čelna fronta pretraživanja (cijansko) |
| `/d_star_inflation_buffer_visualization` | `visualization_msgs/MarkerArray` | Zona zaštite oko prepreka (purpurna) |

## Parametri

```yaml
inflation_distance_m: 0.5  # Razmak od zidova (u metrima)
inflation_cost_threshold: 60  # Threshold za inflaciju
allow_diagonal: true  # Omogući dijagonalno kretanje
max_iterations: 50000  # Maksimalni broj iteracija
start_x: 0.0  # Početna X koordinata (ako nema base_link)
start_y: 0.0  # Početna Y koordinata (ako nema base_link)
goal_x: 5.0  # Zadana X ciljna koordinata
goal_y: 5.0  # Zadana Y ciljna koordinata
```

## Vizualizacija u RVizu

### 1. Otvorite RViz

```bash
ros2 run rviz2 rviz2
```

### 2. Dodajte markere

U RVizu:
1. **Add** → **By topic** → `/d_star_path_planning_visualization` (zelene točke - istraživane stanice)
2. **Add** → **By topic** → `/d_star_planning_frontier` (cijanske točke - čelna fronta)
3. **Add** → **By topic** → `/d_star_inflation_buffer_visualization` (purpurne točke - zona zaštite)
4. **Add** → **By topic** → `/d_star_planned_path` (crvena linija - planirana putanja)

### 3. Postavite cilj

1. Koristite **2D Goal Pose** tool iz RViza
2. Kliknite na mapu gdje želite da robot ode
3. Node će automatski planirati putanju

## Arhitektura D* algoritma

### Glavne strukture podataka

```python
g[s]     # Procjena cijene puta od cilja do čvora s
rhs[s]   # Dorađena vrijednost (rhs < g znači da trebamo replaniranje)
open_set # Prioritetni red čvorova koji trebaju biti obrađeni
```

### Ključna funkcija (Key)

```
key(s) = (min(g[s], rhs[s]) + h(s), min(g[s], rhs[s]))
```

Gdje je:
- `h(s)` = heurističke procjena od s do robota
- Prvi element = prioritet za backward pretraživanje
- Drugi element = tiebreaker

### Algoritam u pseudokodu

```
1. Inicijalizacija:
   - rhs[cilj] = 0
   - Ostali čvorovi: rhs[s] = g[s] = ∞

2. Glavni loop dok je open_set neprazan:
   a. Izvuci čvor s najmanjom vrijednosti
   b. Ako g[s] == rhs[s]: čvor je optimalno solviran
      - Za sve susjede: ažuriraj njihove rhs vrijednosti
   c. Ako g[s] > rhs[s]: pronašli smo bolji put
      - Postavi g[s] = rhs[s]
      - Ažuriraj susjede
   d. Ako g[s] < rhs[s]: put je postal gori
      - Postavi g[s] = ∞
      - Ažuriraj sve susjede

3. Rekonstrukcija putanje:
   - Prati put od robota prema cilju
   - Slijedi čvorove s minimalnom g vrijednosti
```

## Prednosti D* za autonomous robotiku

1. **Dinamičko okruženje**: Može detektirati nove prepreke i replanirati
2. **Efikasnost**: Inkrementalno ažurira samo promijenjene dijelove putanje
3. **Optimalnost**: Garantira optimalnu putanju kao i A*
4. **Fleksibilnost**: Koristi istu heuristiku kao A*, lako se može proširiti

## Primjer korištenja sa Stage simulatorom

```bash
# Terminal 1: Pokrenite Stage simulator
ros2 launch stage_ros2 stage.launch.py

# Terminal 2: Pokrenite mapiranje (ako trebate)
ros2 run gmapping slam_gmapping

# Terminal 3: Pokrenite D* node
ros2 run student_assignment_02 d_star_path_planner

# Terminal 4: Pokrenite RViz
ros2 run rviz2 rviz2 -d ~/.rviz/d_star_config.rviz

# Sada koristite RViz 2D Goal Pose tool za postavljanje ciljeva
```

## Napomena o performansama

- Za male karte (< 100x100): Jako brz
- Za velike karte (> 500x500): Može biti spora sa svim markerima
  - Preporuka: Prikazati samo svaku N-tu stanicu za vizualizaciju
  - Ili smanjiti `inflation_distance_m` parameter

## Debugging

Za detaljnije logove:

```bash
ros2 run student_assignment_02 d_star_path_planner --ros-args --log-level debug
```

Očekivani log izlaz:
```
[INFO] [d_star_path_planner]: D* Path Planner Node: Started
[INFO] [d_star_path_planner]: Mapa primljena: 1000x1000, rezolucija: 0.050 m/stanica
[INFO] [d_star_path_planner]: D* Planiranje putanje od (10, 10) do (100, 100)
[INFO] [d_star_path_planner]: D* zavšio u 523 iteracija, istraživao 245 stanica, dužina putanje: 127 stanica
```

## Prednosti D* nad ostalim algoritmima

| Algoritam | Prednosti | Nedostaci |
|-----------|----------|----------|
| **A*** | Brz, optimalan, često korišten | Trebam ponovo planirati sa novim preprekama |
| **D*** | Dinamičan, inkrementalan, optimalan | Kompleksniji za implementaciju |
| **Theta*** | Glatke putanje (any-angle) | Sporiji od A* |
| **RRT** | Brz za visoko-dimenzionalne prostore | Ne garantira optimalnost |

## Literatura

- Sven Koenig, Maxim Likhachev: "Incremental A*", 2002
- Originalni D* algoritam: Anthony Stentz, 1994
- Praktična primjena u ROS-u

## Autora

Implementacija D* algoritma za ROS2 projekt autonomnih sustava.
