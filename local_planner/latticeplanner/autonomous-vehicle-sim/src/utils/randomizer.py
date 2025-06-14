import random
import numpy as np
from typing import List, Tuple, Dict, Any
from dataclasses import dataclass

@dataclass
class RandomObstacle:
    """Random obstacle definition"""
    x: float
    y: float
    width: float
    height: float
    obstacle_type: str
    
@dataclass
class ParkingZone:
    """Parking zone definition"""
    x: float
    y: float
    width: float
    height: float
    angle: float = 0.0
    occupied: bool = False

class ComplexEnvironmentGenerator:
    """Complex random environment generator"""
    
    def __init__(self, grid_config: Dict[str, Any]):
        self.grid_config = grid_config
        self.world_width = grid_config['width'] * grid_config['resolution']
        self.world_height = grid_config['height'] * grid_config['resolution']
        self.origin_x = grid_config['origin_x']
        self.origin_y = grid_config['origin_y']
        
        # World bounds
        self.world_x_min = self.origin_x
        self.world_x_max = self.origin_x + self.world_width
        self.world_y_min = self.origin_y
        self.world_y_max = self.origin_y + self.world_height
        
        # City zones
        self.road_network = []
        self.building_zones = []
        self.parking_zones = []
        self.obstacles = []
        
        # Road parameters - DAHA BÜYÜK YOLLAR VE ALANLAR
        self.grid_spacing_x = 90   # Daha büyük aralık (80 -> 90)
        self.grid_spacing_y = 80   # Daha büyük aralık (70 -> 80)
        self.road_width_main = 28  # Daha geniş ana yollar (25 -> 28)
        self.road_width_secondary = 20  # Daha geniş yan yollar (18 -> 20)
        
        print(f"🌍 World size: {self.world_width:.0f} x {self.world_height:.0f} meters")
        
    def generate_complex_city(self) -> Dict[str, Any]:
        """Generate a complex random city layout"""
        print("🏗️ Generating complex city...")
        
        # Create road network
        road_network = self._create_road_network()
        
        # Generate buildings
        buildings = self._generate_buildings(40, 70)
        
        # Generate obstacles
        obstacles = self._generate_obstacles(20, 35)
        
        # Create parking zones
        parking_zones = self._create_parking_zones(12, 20)
        
        # Generate dynamic obstacles
        dynamic_obstacles = self._generate_dynamic_obstacles(5, 10)
        
        city_data = {
            'roads': road_network,
            'buildings': buildings,
            'obstacles': obstacles,
            'parking_zones': parking_zones,
            'dynamic_obstacles': dynamic_obstacles,
            'safe_zones': self._calculate_safe_zones()
        }
        
        print(f"✅ Generated city with {len(road_network)} roads, {len(buildings)} buildings")
        return city_data
    
    def _create_road_network(self) -> List[Dict[str, Any]]:
        """Create proper road network"""
        roads = []
        
        # Horizontal roads - DAHA BÜYÜK ARALIKLARDA
        y_positions = list(range(int(self.world_y_min + 60), int(self.world_y_max - 60), self.grid_spacing_y))
        for i, y in enumerate(y_positions):
            width = self.road_width_main if i % 2 == 0 else self.road_width_secondary
            roads.append({
                'type': 'horizontal_main' if i % 2 == 0 else 'horizontal_secondary',
                'y': float(y),
                'x_start': self.world_x_min + 40,  # Daha geniş başlangıç (30 -> 40)
                'x_end': self.world_x_max - 40,    # Daha geniş bitiş (30 -> 40)
                'width': width,
                'lanes': 4 if i % 2 == 0 else 3   # Daha fazla şerit
            })
        
        # Vertical roads - DAHA BÜYÜK ARALIKLARDA
        x_positions = list(range(int(self.world_x_min + 60), int(self.world_x_max - 60), self.grid_spacing_x))
        for i, x in enumerate(x_positions):
            width = self.road_width_main if i % 2 == 0 else self.road_width_secondary
            roads.append({
                'type': 'vertical_main' if i % 2 == 0 else 'vertical_secondary',
                'x': float(x),
                'y_start': self.world_y_min + 40,  # Daha geniş başlangıç (30 -> 40)
                'y_end': self.world_y_max - 40,    # Daha geniş bitiş (30 -> 40)
                'width': width,
                'lanes': 4 if i % 2 == 0 else 3   # Daha fazla şerit
            })
        
        self.road_network = roads
        self.horizontal_roads = [r for r in roads if r['type'].startswith('horizontal')]
        self.vertical_roads = [r for r in roads if r['type'].startswith('vertical')]
        
        return roads
    
    def _generate_buildings(self, min_count: int, max_count: int) -> List[RandomObstacle]:
        """Generate LARGER buildings"""
        buildings = []
        count = random.randint(min_count, max_count)
        
        for _ in range(count):
            for _ in range(30):  # Max attempts
                width = random.uniform(18, 45)   # Daha büyük binalar (15-40 -> 18-45)
                height = random.uniform(15, 40)  # Daha büyük binalar (12-35 -> 15-40)
                x = random.uniform(self.world_x_min + width/2 + 40, 
                                self.world_x_max - width/2 - 40)
                y = random.uniform(self.world_y_min + height/2 + 40, 
                                self.world_y_max - height/2 - 40)
                
                if not self._conflicts_with_roads(x, y, width, height):
                    buildings.append(RandomObstacle(
                        x=x, y=y, width=width, height=height,
                        obstacle_type='building'
                    ))
                    break
        
        return buildings
    
    def _generate_obstacles(self, min_count: int, max_count: int) -> List[RandomObstacle]:
        """Generate more obstacles for larger map"""
        obstacles = []
        count = random.randint(min_count + 10, max_count + 15)  # Daha fazla engel
        
        obstacle_types = ['parked_car', 'barrier', 'tree', 'construction', 'pole']
        
        for _ in range(count):
            obstacle_type = random.choice(obstacle_types)
            
            if obstacle_type == 'parked_car':
                width, height = random.uniform(5, 7), random.uniform(2.5, 3.5)
            elif obstacle_type == 'barrier':
                width, height = random.uniform(6, 14), random.uniform(1.5, 2.8)  # Biraz daha büyük
            elif obstacle_type == 'tree':
                width, height = random.uniform(3, 7), random.uniform(3, 7)  # Biraz daha büyük
            elif obstacle_type == 'construction':
                width, height = random.uniform(10, 22), random.uniform(8, 15)  # Biraz daha büyük
            else:  # pole
                width, height = random.uniform(1, 2.5), random.uniform(1, 2.5)  # Biraz daha büyük
            
            # Place some obstacles on roads (35% chance - biraz daha fazla)
            if random.random() < 0.35 and self.road_network:
                position = self._get_road_obstacle_position(width, height)
            else:
                position = self._get_safe_obstacle_position(width, height)
            
            if position:
                x, y = position
                obstacles.append(RandomObstacle(
                    x=x, y=y, width=width, height=height,
                    obstacle_type=obstacle_type
                ))
        
        return obstacles
    
    def _get_road_obstacle_position(self, width: float, height: float) -> Tuple[float, float]:
        """Get position on road for obstacle"""
        road = random.choice(self.road_network)
        
        if road['type'].startswith('horizontal'):
            x = random.uniform(road['x_start'] + width/2 + 10, 
                             road['x_end'] - width/2 - 10)
            y = road['y'] + random.choice([-road['width']/3, road['width']/3])
        else:  # vertical
            x = road['x'] + random.choice([-road['width']/3, road['width']/3])
            y = random.uniform(road['y_start'] + height/2 + 10, 
                             road['y_end'] - height/2 - 10)
        
        return x, y
    
    def _get_safe_obstacle_position(self, width: float, height: float) -> Tuple[float, float]:
        """Get safe position for obstacle"""
        for _ in range(20):
            x = random.uniform(self.world_x_min + width/2 + 15, 
                             self.world_x_max - width/2 - 15)
            y = random.uniform(self.world_y_min + height/2 + 15, 
                             self.world_y_max - height/2 - 15)
            
            if not self._conflicts_with_roads(x, y, width, height):
                return x, y
        return None
    
    def _create_parking_zones(self, min_count: int, max_count: int) -> List[ParkingZone]:
        """Create more parking zones for larger map"""
        parking_zones = []
        count = random.randint(min_count + 5, max_count + 8)  # Daha fazla park alanı
        
        for _ in range(count):
            for _ in range(15):  # Max attempts
                width = random.uniform(20, 35)  # Daha büyük park alanları (18-30 -> 20-35)
                height = random.uniform(12, 20) # Daha büyük park alanları (10-16 -> 12-20)
                x = random.uniform(self.world_x_min + width/2 + 50, 
                                self.world_x_max - width/2 - 50)
                y = random.uniform(self.world_y_min + height/2 + 50, 
                                self.world_y_max - height/2 - 50)
                
                if not self._conflicts_with_roads(x, y, width, height):
                    parking_zones.append(ParkingZone(
                        x=x, y=y, width=width, height=height
                    ))
                    break
        
        return parking_zones
    
    def _generate_dynamic_obstacles(self, min_count: int, max_count: int) -> List[Dict[str, Any]]:
        """Generate more dynamic obstacles for larger map"""
        dynamic_obstacles = []
        count = random.randint(min_count + 3, max_count + 5)  # Daha fazla dinamik engel
        
        obstacle_types = ['pedestrian', 'cyclist', 'slow_vehicle']
        
        for _ in range(count):
            obstacle_type = random.choice(obstacle_types)
            
            # SLOW speeds - aynı yavaş hızlar
            if obstacle_type == 'pedestrian':
                width, height = 0.6, 0.6
                speed = random.uniform(0.3, 0.8)
            elif obstacle_type == 'cyclist':
                width, height = 0.8, 1.6
                speed = random.uniform(1.0, 2.5)
            else:  # slow_vehicle
                width, height = 4.0, 1.8
                speed = random.uniform(2.0, 4.0)
            
            x = random.uniform(self.world_x_min + 40, self.world_x_max - 40)
            y = random.uniform(self.world_y_min + 40, self.world_y_max - 40)
            
            angle = random.uniform(0, 2 * np.pi)
            vx = speed * np.cos(angle)
            vy = speed * np.sin(angle)
            
            dynamic_obstacles.append({
                'type': obstacle_type,
                'x': x, 'y': y,
                'width': width, 'height': height,
                'vx': vx, 'vy': vy,
                'speed': speed,
                'behavior': 'random_walk',
                'initial_pos': (x, y)
            })
        
        return dynamic_obstacles
    
    def _conflicts_with_roads(self, x: float, y: float, width: float, height: float) -> bool:
        """Check if position conflicts with roads"""
        for road in self.road_network:
            if road['type'].startswith('horizontal'):
                if (x - width/2 < road['x_end'] and x + width/2 > road['x_start'] and
                    y - height/2 < road['y'] + road['width']/2 and 
                    y + height/2 > road['y'] - road['width']/2):
                    return True
            else:  # vertical
                if (x - width/2 < road['x'] + road['width']/2 and 
                    x + width/2 > road['x'] - road['width']/2 and
                    y - height/2 < road['y_end'] and y + height/2 > road['y_start']):
                    return True
        return False
    
    def _calculate_safe_zones(self) -> List[Dict[str, float]]:
        """Calculate safe zones for navigation"""
        safe_zones = []
        
        for road in self.road_network:
            if road['type'].startswith('horizontal'):
                safe_zones.append({
                    'type': 'road_horizontal',
                    'x_min': road['x_start'],
                    'x_max': road['x_end'],
                    'y_min': road['y'] - road['width']/2,
                    'y_max': road['y'] + road['width']/2,
                    'center_y': road['y']
                })
            else:
                safe_zones.append({
                    'type': 'road_vertical',
                    'x_min': road['x'] - road['width']/2,
                    'x_max': road['x'] + road['width']/2,
                    'y_min': road['y_start'],
                    'y_max': road['y_end'],
                    'center_x': road['x']
                })
        
        return safe_zones
    
    def get_random_start_position(self) -> Tuple[float, float, float]:
        """Get random start position on road"""
        safe_zones = self._calculate_safe_zones()
        
        if not safe_zones:
            return 0.0, 0.0, 0.0
        
        zone = random.choice(safe_zones)
        
        if zone['type'] == 'road_horizontal':
            x = random.uniform(zone['x_min'] + 15, zone['x_max'] - 15)
            y = zone['center_y']
            heading = random.choice([0.0, np.pi])
        else:
            x = zone['center_x']
            y = random.uniform(zone['y_min'] + 15, zone['y_max'] - 15)
            heading = random.choice([np.pi/2, -np.pi/2])
        
        return x, y, heading
    
    def get_random_goal_with_parking(self, start_pos: Tuple[float, float]) -> Tuple[np.ndarray, ParkingZone]:
        """Get random goal with parking - DAHA UZAK HEDEFLER"""
        min_distance = 120.0  # Daha uzak minimum mesafe (100 -> 120)
        
        # Try existing parking zones
        for parking_zone in self.parking_zones:
            distance = np.linalg.norm([parking_zone.x - start_pos[0], 
                                     parking_zone.y - start_pos[1]])
            if distance > min_distance:
                goal = np.array([parking_zone.x + random.uniform(-18, 18), 
                               parking_zone.y + random.uniform(-18, 18), 0.0])
                return goal, parking_zone
        
        # Create new parking zone - DAHA BÜYÜK
        safe_zones = self._calculate_safe_zones()
        if safe_zones:
            zone = random.choice(safe_zones)
            if zone['type'] == 'road_horizontal':
                center_x = (zone['x_min'] + zone['x_max']) / 2
                center_y = zone['center_y']
            else:
                center_x = zone['center_x'] 
                center_y = (zone['y_min'] + zone['y_max']) / 2
            
            parking_zone = ParkingZone(
                x=center_x + random.uniform(-25, 25),
                y=center_y + random.uniform(-25, 25),
                width=25.0, height=15.0  # Daha büyük park alanı (20x12 -> 25x15)
            )
            
            goal = np.array([center_x, center_y, 0.0])
            return goal, parking_zone
        
        # Fallback - DAHA UZAK
        goal = np.array([180.0, 0.0, 0.0])  # Daha uzak (150 -> 180)
        parking_zone = ParkingZone(x=190.0, y=20.0, width=25.0, height=15.0)
        return goal, parking_zone

def create_random_scenario() -> Dict[str, Any]:
    """Create random scenario with LARGER content"""
    # DAHA BÜYÜK Grid configuration
    grid_config = {
        'width': 1000,      # Daha büyük (800 -> 1000)
        'height': 800,      # Daha büyük (600 -> 800)
        'resolution': 0.4,  # Biraz daha ince (0.5 -> 0.4)
        'origin_x': -200.0, # Aynı
        'origin_y': -160.0  # Biraz daha geniş (-150 -> -160)
    }
    
    env_gen = ComplexEnvironmentGenerator(grid_config)
    city_data = env_gen.generate_complex_city()
    
    start_pos = env_gen.get_random_start_position()
    goal, parking_zone = env_gen.get_random_goal_with_parking(start_pos[:2])
    
    scenario = {
        'grid_config': grid_config,
        'city_data': city_data,
        'start_position': start_pos,
        'goal_position': goal,
        'parking_zone': parking_zone,
        'environment_generator': env_gen
    }
    
    print(f"🎯 LARGER content scenario:")
    print(f"   🌍 World: {grid_config['width'] * grid_config['resolution']:.0f}x{grid_config['height'] * grid_config['resolution']:.0f}m")
    
    return scenario