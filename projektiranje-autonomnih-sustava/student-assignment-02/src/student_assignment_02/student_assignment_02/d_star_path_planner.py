#!/usr/bin/env python3
"""
D* Path Planner Node
Koristi D* algoritam za dinamičko planiranje putanje na 2D mapi
Podržava pronalaženje prepreka tijekom izvršavanja
Vizualizira pretraživanje prostora i planiranu putanju u RViz-u
Podrživavas dinamiki goal pose iz RViza (2D Goal Pose)
Koristi base_link za početnu točku (pozicija robota)
Dodao: Inflation buffer od 0.2m oko zidova za sigurnu putanju
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
import numpy as np
from heapq import heappush, heappop
from typing import List, Tuple, Optional, Dict, Set
import math
from collections import defaultdict


class DStarPathPlanner(Node):
    """ROS2 čvor za D* planiranje putanje s inflation bufferom"""
    
    def __init__(self):
        super().__init__('d_star_path_planner')
        
        # TF Buffer i Listener za base_link transformaciju
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS profil
        qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe na mapu
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )
        
        # Subscribe na goal pose iz RViza (2D Goal Pose tool)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10  # Regular QoS za goal pose
        )
        
        # Publisher za putanju
        self.path_publisher = self.create_publisher(
            Path,
            '/d_star_planned_path',
            qos
        )
        
        # Publisher za vizualizaciju pretraživanja
        self.visualization_publisher = self.create_publisher(
            MarkerArray,
            '/d_star_path_planning_visualization',
            qos
        )
        
        # Publisher za čelnu frontu pretraživanja (opened čvorovi)
        self.frontier_publisher = self.create_publisher(
            MarkerArray,
            '/d_star_planning_frontier',
            qos
        )
        
        # Publisher za inflation buffer vizualizaciju
        self.inflation_buffer_publisher = self.create_publisher(
            MarkerArray,
            '/d_star_inflation_buffer_visualization',
            qos
        )
        
        self.map_data = None
        self.map_metadata = None
        self.marker_id_counter = 0
        self.goal_received = False
        
        # Parametri planiranja
        self.declare_parameter('inflation_radius', 1)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('max_iterations', 50000)
        self.declare_parameter('search_radius', -1)  # -1 = bez ograničenja
        self.declare_parameter('inflation_distance_m', 0.5)
        self.declare_parameter('inflation_cost_threshold', 60)
        
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.allow_diagonal = self.get_parameter('allow_diagonal').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.search_radius = self.get_parameter('search_radius').value
        self.inflation_distance_m = self.get_parameter('inflation_distance_m').value
        self.inflation_cost_threshold = self.get_parameter('inflation_cost_threshold').value
        
        # Trenutni goal
        self.current_goal_x = self.get_parameter('goal_x').value
        self.current_goal_y = self.get_parameter('goal_y').value
        self.current_start_x = self.get_parameter('start_x').value
        self.current_start_y = self.get_parameter('start_y').value
        
        # Mapa s inflacijom (kreširana nakon primanja mape)
        self.inflated_map = None
        
        # Zastarica za prvo planiranje
        self.first_plan = True
        
        self.get_logger().info('D* Path Planner Node: Started')
        self.get_logger().info(f'Max iterations: {self.max_iterations}')
        self.get_logger().info(f'Inflation distance: {self.inflation_distance_m}m (0.2m = 20cm)')
        self.get_logger().info('Slusa na /goal_pose za dinamicki goal (RViz 2D Goal Pose)')
        self.get_logger().info('Koristi base_link za početnu točku (poziciju robota)')
    
    def get_robot_position(self) -> Tuple[float, float]:
        """
        Proba pronaći base_link poziciju iz TF tree-a
        Ako ne može, koristi parametar start_x, start_y
        """
        try:
            # Proba dobiti transformaciju od map-a do base_link-a
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            self.get_logger().debug(f'base_link pozicija: ({robot_x:.2f}, {robot_y:.2f})')
            return (robot_x, robot_y)
            
        except Exception as e:
            # Ako ne može dobiti transformaciju, koristi parametre
            self.get_logger().warn(
                f'Ne mogu dobiti base_link transformaciju: {e}. '
                f'Koristim parametre start_x={self.current_start_x}, start_y={self.current_start_y}'
            )
            return (self.current_start_x, self.current_start_y)
    
    def goal_pose_callback(self, msg: PoseStamped):
        """
        Primanje goal pose iz RViza (2D Goal Pose tool)
        """
        self.current_goal_x = msg.pose.position.x
        self.current_goal_y = msg.pose.position.y
        self.goal_received = True
        
        self.get_logger().info(
            f'Nova goal pose primljena iz RViza: '
            f'({self.current_goal_x:.2f}, {self.current_goal_y:.2f})'
        )
        
        # Ako je mapa dostupna, planiraj putanju odmah
        if self.map_data is not None:
            self.plan_and_publish()
    
    def map_callback(self, msg: OccupancyGrid):
        """
        Primanje mape i planiranje putanje
        """
        self.map_data = msg.data
        self.map_metadata = msg.info
        
        # Kreiraj infliranu mapu
        self.inflated_map = self.create_inflated_map()
        
        self.get_logger().info(
            f'Mapa primljena: {msg.info.width}x{msg.info.height}, '
            f'rezolucija: {msg.info.resolution:.3f} m/stanica'
        )
        self.get_logger().info(
            f'Inflation buffer: {self.inflation_distance_m}m '
            f'({self._get_inflation_cells()} stanica)'
        )
        
        # Ako je goal postavljen, planiraj putanju
        if self.goal_received:
            self.plan_and_publish()
    
    def _get_inflation_cells(self) -> int:
        """
        Izračuna broj stanica za inflation buffer
        """
        if not self.map_metadata:
            return 0
        cells = int(self.inflation_distance_m / self.map_metadata.resolution)
        return max(cells, 1)  # Minimalno 1 stanica
    
    def create_inflated_map(self) -> List[int]:
        """
        Kreiraj inflirane mape - dodaj buffer oko prepreka
        Algoritam: Distance transform - označi sve stanice blizu prepreka
        """
        if not self.map_metadata or not self.map_data:
            return self.map_data
        
        width = self.map_metadata.width
        height = self.map_metadata.height
        resolution = self.map_metadata.resolution
        
        # Broj stanica za inflation
        inflation_cells = self._get_inflation_cells()
        
        # Kopiraj originalnu mapu
        inflated = list(self.map_data)
        
        # Pretraži sve stanice
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                
                # Ako je stanica slobodna
                if inflated[idx] < self.inflation_cost_threshold:
                    # Provjeri je li blizu prepreke
                    min_dist = self._min_distance_to_obstacle(x, y, inflation_cells)
                    
                    # Ako je blizu prepreke, označi je kao "opasnu"
                    if min_dist < inflation_cells:
                        inflated[idx] = 60 + int((inflation_cells - min_dist) * 10)
        
        self.get_logger().info(
            f'Inflirana mapa kreirana - buffer oko prepreka: {inflation_cells} stanica'
        )
        return inflated
    
    def _min_distance_to_obstacle(self, x: int, y: int, max_dist: int) -> float:
        """
        Izračuna minimalnu distancu od točke do prepreke
        Koristi BFS za efikasnost
        """
        if not self.map_metadata or not self.map_data:
            return max_dist
        
        width = self.map_metadata.width
        height = self.map_metadata.height
        
        # BFS queue
        from collections import deque
        queue = deque([(x, y, 0)])
        visited = set([(x, y)])
        
        while queue:
            cx, cy, dist = queue.popleft()
            
            # Ako je to prepreka
            idx = cy * width + cx
            if self.map_data[idx] >= self.inflation_cost_threshold:
                return dist
            
            # Ako je distanca premala, nema smisla nastaviti
            if dist >= max_dist:
                return max_dist
            
            # Dodaj susjede (4-povezanost)
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))
        
        return max_dist
    
    def plan_and_publish(self):
        """
        Planiraj putanju i objavi je
        """
        # Proba dobiti poziciju robota iz base_link-a
        start_x, start_y = self.get_robot_position()
        
        # Goal se koristi iz RViza ili parametara
        goal_x = self.current_goal_x
        goal_y = self.current_goal_y
        
        # Pretvori world koordinate u grid koordinate
        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        self.get_logger().info(
            f'D* Planiranje putanje od {start_grid} (world: {start_x:.2f}, {start_y:.2f}) '
            f'do {goal_grid} (world: {goal_x:.2f}, {goal_y:.2f})'
        )
        
        # Planiraj putanju
        path, explored_cells, frontier = self.plan_path_dstar(start_grid, goal_grid)
        
        if path:
            self.publish_path(path)
            self.get_logger().info(f'Putanja pronađena! Dužina: {len(path)} stanica')
        else:
            self.get_logger().warn('Putanja nije pronađena!')
        
        # Vizualiziraj pretraživanje
        self.visualize_search(explored_cells, frontier)
        self.visualize_inflation_buffer()
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Pretvori world koordinate u grid koordinate
        """
        if not self.map_metadata:
            return (0, 0)
        
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        resolution = self.map_metadata.resolution
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        # Klamp u granice
        grid_x = max(0, min(grid_x, self.map_metadata.width - 1))
        grid_y = max(0, min(grid_y, self.map_metadata.height - 1))
        
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Pretvori grid koordinate u world koordinate
        """
        if not self.map_metadata:
            return (0.0, 0.0)
        
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        resolution = self.map_metadata.resolution
        
        world_x = origin_x + (grid_x + 0.5) * resolution
        world_y = origin_y + (grid_y + 0.5) * resolution
        
        return (world_x, world_y)
    
    def is_valid_cell(self, x: int, y: int, use_inflation: bool = True) -> bool:
        """
        Provjeri je li stanica valjana (slobodna)
        """
        if not self.map_metadata:
            return False
        
        # Provjeri granice
        if x < 0 or x >= self.map_metadata.width or y < 0 or y >= self.map_metadata.height:
            return False
        
        # Pretvori u index
        index = y * self.map_metadata.width + x
        
        if index < 0 or index >= len(self.map_data):
            return False
        
        # Koristi inflirane mape ako su dostupne
        if use_inflation and self.inflated_map:
            cell_value = self.inflated_map[index]
        else:
            cell_value = self.map_data[index]
        
        # Threshold: < 50 = slobodno, >= 60 = inflation buffer, >= 100 = prepreka
        return cell_value < 50
    
    def get_neighbors(self, cell: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Dobij sve valjane susjede stanice
        """
        x, y = cell
        neighbors = []
        
        # 4-povezanost (gore, dolje, lijevo, desno)
        four_neighbors = [
            (x + 1, y), (x - 1, y),  # Desno, lijevo
            (x, y + 1), (x, y - 1)   # Gore, dolje
        ]
        
        # Dodaj dijagonalne ako je omogućeno
        if self.allow_diagonal:
            four_neighbors.extend([
                (x + 1, y + 1), (x + 1, y - 1),
                (x - 1, y + 1), (x - 1, y - 1)
            ])
        
        for nx, ny in four_neighbors:
            if self.is_valid_cell(nx, ny, use_inflation=True):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def heuristic(self, cell: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """
        Heuristička funkcija (Euklidska distanca)
        """
        dx = cell[0] - goal[0]
        dy = cell[1] - goal[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def get_cost(self, cell1: Tuple[int, int], cell2: Tuple[int, int]) -> float:
        """
        Trošak kretanja između dvije stanice
        """
        dx = cell2[0] - cell1[0]
        dy = cell2[1] - cell1[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def plan_path_dstar(
        self, 
        start: Tuple[int, int], 
        goal: Tuple[int, int]
    ) -> Tuple[Optional[List[Tuple[int, int]]], List[Tuple[int, int]], List[Tuple[int, int]]]:
        """
        D* algoritam za planiranje putanje
        
        D* (Dynamic A*) je inkrementalni algoritam koji može efikasno
        ažurirati putanju kada su promijenjeni troškovi grana.
        
        Koristi:
        - Backward search od cilja prema početku
        - g(s) = duljina optimalnog puta od cilja do čvora s
        - h(s) = heuristička procjena od s do robota
        - rhs(s) = minimalna procjena vrijednosti od s
        
        Returns:
            (path, explored_cells, frontier)
        """
        
        # Provjeri validnost starta i cilja
        if not self.is_valid_cell(start[0], start[1], use_inflation=True):
            self.get_logger().error('Start nije valjana stanica!')
            return None, [], []
        
        if not self.is_valid_cell(goal[0], goal[1], use_inflation=True):
            self.get_logger().error('Cilj nije valjana stanica!')
            return None, [], []
        
        # Inicijalizacija D* struktura podataka
        g = defaultdict(lambda: float('inf'))
        rhs = defaultdict(lambda: float('inf'))
        open_set = []  # Priority queue: (key, node)
        
        # Cilj ima rhs = 0
        rhs[goal] = 0
        
        # Ključna funkcija za D*
        def calc_key(node: Tuple[int, int]) -> Tuple[float, float]:
            min_val = min(g[node], rhs[node])
            k1 = min_val + self.heuristic(node, start)
            k2 = min_val
            return (k1, k2)
        
        # Dodaj cilj u open set
        heappush(open_set, (calc_key(goal), goal))
        
        # Za vizualizaciju
        explored = []
        frontier_cells = []
        
        iteration = 0
        max_iter = self.max_iterations
        
        while open_set and iteration < max_iter:
            iteration += 1
            
            # Izvuci čvor s najmanjom vrijednosti
            key, current = heappop(open_set)
            frontier_cells.append(current)
            
            # Ako je čvor konsistentan, preskoči ga
            if g[current] == rhs[current]:
                # Čvor je optimalno solviran - razmotri sve njegove susjede
                for neighbor in self.get_neighbors(current):
                    # Ažuriraj rhs vrijednost susjednog čvora
                    new_rhs = g[current] + self.get_cost(current, neighbor)
                    if new_rhs < rhs[neighbor]:
                        rhs[neighbor] = new_rhs
                        # Ako sused nije cilj, dodaj ga u open set
                        if neighbor != goal:
                            heappush(open_set, (calc_key(neighbor), neighbor))
            else:
                # Čvor je inkonsistentan - radi ga
                g_old = g[current]
                g[current] = rhs[current]
                
                # Ako se g vrijednost smanjila, ažuriraj susjede
                if g_old > rhs[current]:
                    # Pretražili smo ovaj čvor
                    explored.append(current)
                    
                    # Ažuriraj sve susjede
                    for neighbor in self.get_neighbors(current):
                        # Ažuriraj rhs vrijednost
                        new_rhs = g[current] + self.get_cost(current, neighbor)
                        if new_rhs < rhs[neighbor]:
                            rhs[neighbor] = new_rhs
                            heappush(open_set, (calc_key(neighbor), neighbor))
                else:
                    # Povećanje g vrijednosti - trebam provjeriti sve susjede i sadašnji čvor
                    explored.append(current)
                    
                    # Ažuriraj sadašnji čvor
                    new_rhs = float('inf')
                    for neighbor in self.get_neighbors(current):
                        new_rhs = min(new_rhs, g[neighbor] + self.get_cost(neighbor, current))
                    if new_rhs < float('inf'):
                        rhs[current] = new_rhs
                        heappush(open_set, (calc_key(current), current))
                    
                    # Ažuriraj sve susjede
                    for neighbor in self.get_neighbors(current):
                        new_rhs = g[current] + self.get_cost(current, neighbor)
                        if new_rhs < rhs[neighbor]:
                            rhs[neighbor] = new_rhs
                            heappush(open_set, (calc_key(neighbor), neighbor))
            
            # Provjeri je li dosegao start
            if g[start] != float('inf'):
                # Možemo rekonstruirati putanju
                if rhs[start] == g[start]:
                    # Pronašli smo optimalnu putanju
                    path = []
                    current = start
                    
                    while current != goal:
                        path.append(current)
                        
                        # Pronađi sljedећi čvor prema najmanjoj g vrijednosti
                        best_neighbor = None
                        best_cost = float('inf')
                        
                        for neighbor in self.get_neighbors(current):
                            cost = g[neighbor] + self.get_cost(neighbor, current)
                            if cost < best_cost:
                                best_cost = cost
                                best_neighbor = neighbor
                        
                        if best_neighbor is None:
                            break
                        
                        current = best_neighbor
                    
                    path.append(goal)
                    
                    self.get_logger().info(
                        f'D* završio u {iteration} iteracija, '
                        f'istraživao {len(explored)} stanica, '
                        f'dužina putanje: {len(path)}'
                    )
                    return path, explored, frontier_cells
        
        self.get_logger().warn(
            f'Nema putanje! Iteracija: {iteration}/{max_iter}, '
            f'istraživao {len(explored)} stanica'
        )
        return None, explored, frontier_cells
    
    def publish_path(self, path: List[Tuple[int, int]]):
        """
        Objavi pronađenu putanju
        """
        ros_path = Path()
        ros_path.header.frame_id = 'map'
        ros_path.header.stamp = self.get_clock().now().to_msg()
        
        for grid_pos in path:
            world_x, world_y = self.grid_to_world(grid_pos[0], grid_pos[1])
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            ros_path.poses.append(pose)
        
        self.path_publisher.publish(ros_path)
    
    def visualize_search(
        self, 
        explored: List[Tuple[int, int]], 
        frontier: List[Tuple[int, int]]
    ):
        """
        Vizualiziraj pretraživanje u RViz-u
        """
        
        # Vizualizacija istraživanih stanica
        explored_markers = MarkerArray()
        
        for idx, (gx, gy) in enumerate(explored):
            if idx % 5 != 0:  # Prikaži svaku 5. stanicu
                continue
            
            world_x, world_y = self.grid_to_world(gx, gy)
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'd_star_explored_cells'
            marker.id = idx // 5
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.0
            
            marker.scale.x = self.map_metadata.resolution * 0.8
            marker.scale.y = self.map_metadata.resolution * 0.8
            marker.scale.z = self.map_metadata.resolution * 0.8
            
            # D* koristi brojačé boje (zelena)
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 0.2
            marker.color.a = 0.3
            
            explored_markers.markers.append(marker)
        
        self.visualization_publisher.publish(explored_markers)
        
        # Vizualizacija čelne fronte
        frontier_markers = MarkerArray()
        
        for idx, (gx, gy) in enumerate(frontier[-1000:]):  # Samo zadnjih 1000
            world_x, world_y = self.grid_to_world(gx, gy)
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'd_star_frontier'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.0
            
            marker.scale.x = self.map_metadata.resolution * 0.6
            marker.scale.y = self.map_metadata.resolution * 0.6
            marker.scale.z = self.map_metadata.resolution * 0.6
            
            # D* čelna fronta koristi cijansku boju
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.5
            
            frontier_markers.markers.append(marker)
        
        self.frontier_publisher.publish(frontier_markers)
    
    def visualize_inflation_buffer(self):
        """
        Vizualiziraj inflation buffer zone
        """
        if not self.inflated_map or not self.map_metadata:
            return
        
        buffer_markers = MarkerArray()
        width = self.map_metadata.width
        height = self.map_metadata.height
        
        marker_id = 0
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                
                # Prikaži samo buffer zone (60-99)
                if 50 <= self.inflated_map[idx] < 100:
                    world_x, world_y = self.grid_to_world(x, y)
                    
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'd_star_inflation_buffer'
                    marker.id = marker_id
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    
                    marker.pose.position.x = world_x
                    marker.pose.position.y = world_y
                    marker.pose.position.z = 0.0
                    
                    res = self.map_metadata.resolution
                    marker.scale.x = res
                    marker.scale.y = res
                    marker.scale.z = 0.01
                    
                    # Razlika boja ovisno o distanci do prepreke (purpurna za D*)
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 0.2
                    
                    buffer_markers.markers.append(marker)
                    marker_id += 1
                    
                    # Prikaži samo svakih 10 markera za performanse
                    if marker_id > 500:
                        break
            if marker_id > 500:
                break
        
        self.inflation_buffer_publisher.publish(buffer_markers)
        self.get_logger().debug(f'Inflation buffer vizualizacija: {marker_id} markera')


def main(args=None):
    rclpy.init(args=args)
    node = DStarPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
