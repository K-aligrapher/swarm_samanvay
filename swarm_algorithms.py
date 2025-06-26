"""
Swarm Drone Algorithms - Core Implementation
This module contains the main algorithms for drone swarm coordination,
including leader-follower formation, collision avoidance, and search patterns.
Designed for ESP32-based miniature drones communicating via MQTT.
"""

import numpy as np
import math
from typing import List, Tuple, Dict, Optional
from enum import Enum
from dataclasses import dataclass
import time
import threading
import queue

class DroneState(Enum):
    """Enumeration of possible drone states"""
    IDLE = "idle"
    LEADING = "leading"
    FOLLOWING = "following"
    SEARCHING = "searching"
    MAPPING = "mapping"
    AVOIDING = "avoiding"
    LANDING = "landing"
    EMERGENCY = "emergency"

@dataclass
class Position:
    """3D Position class with utility methods"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def distance_to(self, other: 'Position') -> float:
        """Calculate Euclidean distance to another position"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)
    
    def __add__(self, other: 'Position') -> 'Position':
        return Position(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: 'Position') -> 'Position':
        return Position(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar: float) -> 'Position':
        return Position(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def normalize(self) -> 'Position':
        """Return normalized position vector"""
        mag = math.sqrt(self.x**2 + self.y**2 + self.z**2)
        if mag == 0:
            return Position(0, 0, 0)
        return Position(self.x/mag, self.y/mag, self.z/mag)
    
    def magnitude(self) -> float:
        """Return magnitude of position vector"""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

@dataclass
class Velocity:
    """3D Velocity class"""
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    
    def magnitude(self) -> float:
        return math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
    
    def limit(self, max_speed: float) -> 'Velocity':
        """Limit velocity to maximum speed"""
        mag = self.magnitude()
        if mag > max_speed:
            factor = max_speed / mag
            return Velocity(self.vx * factor, self.vy * factor, self.vz * factor)
        return self

class Drone:
    """Individual drone class with state management and control algorithms"""
    
    def __init__(self, drone_id: int, initial_position: Position = None, is_leader: bool = False):
        self.id = drone_id
        self.is_leader = is_leader
        self.state = DroneState.IDLE
        
        # Position and movement
        self.position = initial_position or Position(0, 0, 0)
        self.velocity = Velocity(0, 0, 0)
        self.target_position = Position(0, 0, 0)
        self.desired_position = Position(0, 0, 0)
        
        # Physical constraints (for ESP32 mini drones)
        self.max_speed = 5.0  # m/s
        self.max_acceleration = 2.0  # m/s²
        self.min_altitude = 5.0  # m
        self.max_altitude = 50.0  # m
        
        # Formation parameters
        self.formation_distance = 15.0  # Desired distance from leader
        self.formation_angle = 0.0  # Angle in formation
        self.leader_reference = None  # Reference to leader drone
        
        # Collision avoidance
        self.collision_radius = 8.0  # Minimum safe distance
        self.avoidance_range = 12.0  # Detection range for other drones
        
        # Search and mapping
        self.search_radius = 20.0  # Coverage radius for searching
        self.mapped_area = []  # List of mapped positions
        self.search_pattern = "spiral"  # Search pattern type
        self.search_center = Position(0, 0, 0)
        
        # Communication
        self.last_communication = time.time()
        self.communication_timeout = 5.0  # seconds
        
        # Battery simulation (for ESP32 power management)
        self.battery_level = 100.0  # percentage
        self.battery_drain_rate = 0.1  # percent per second
        
    def update_physics(self, dt: float):
        """Update drone physics and position"""
        # Update battery
        self.battery_level -= self.battery_drain_rate * dt
        self.battery_level = max(0, self.battery_level)
        
        # Emergency landing if battery critical
        if self.battery_level < 10:
            self.state = DroneState.EMERGENCY
            self.target_position.z = 0
        
        # Update position based on velocity
        self.position.x += self.velocity.vx * dt
        self.position.y += self.velocity.vy * dt
        self.position.z += self.velocity.vz * dt
        
        # Enforce altitude constraints
        self.position.z = max(self.min_altitude, min(self.max_altitude, self.position.z))
        
        # Update communication timestamp
        self.last_communication = time.time()
    
    def calculate_leader_following_force(self, leader_drone: 'Drone') -> Position:
        """Calculate force for following leader in formation"""
        if not leader_drone:
            return Position(0, 0, 0)
        
        # Calculate desired position relative to leader
        leader_pos = leader_drone.position
        leader_vel = leader_drone.velocity
        
        # Predict leader's future position
        prediction_time = 1.0  # seconds
        predicted_leader_pos = Position(
            leader_pos.x + leader_vel.vx * prediction_time,
            leader_pos.y + leader_vel.vy * prediction_time,
            leader_pos.z + leader_vel.vz * prediction_time
        )
        
        # Calculate formation position
        formation_offset = Position(
            self.formation_distance * math.cos(self.formation_angle),
            self.formation_distance * math.sin(self.formation_angle),
            0  # Maintain same altitude as leader
        )
        
        desired_pos = predicted_leader_pos + formation_offset
        
        # Calculate force towards desired position
        error = desired_pos - self.position
        force_magnitude = min(error.magnitude() * 0.5, self.max_acceleration)
        
        if error.magnitude() > 0:
            return error.normalize() * force_magnitude
        return Position(0, 0, 0)
    
    def calculate_collision_avoidance_force(self, nearby_drones: List['Drone']) -> Position:
        """Calculate collision avoidance force using artificial potential fields"""
        avoidance_force = Position(0, 0, 0)
        
        for other_drone in nearby_drones:
            if other_drone.id == self.id:
                continue
                
            distance = self.position.distance_to(other_drone.position)
            
            if distance < self.avoidance_range and distance > 0:
                # Calculate repulsive force
                direction = self.position - other_drone.position
                direction = direction.normalize()
                
                # Stronger force when closer
                force_magnitude = (self.avoidance_range - distance) / self.avoidance_range
                force_magnitude = force_magnitude**2 * self.max_acceleration * 2
                
                avoidance_force = avoidance_force + (direction * force_magnitude)
        
        # Limit maximum avoidance force
        if avoidance_force.magnitude() > self.max_acceleration:
            avoidance_force = avoidance_force.normalize() * self.max_acceleration
        
        return avoidance_force
    
    def calculate_search_force(self) -> Position:
        """Calculate force for search pattern movement"""
        current_time = time.time()
        
        if self.search_pattern == "spiral":
            # Spiral search pattern
            t = current_time * 0.1  # Time scaling
            radius = min(t * 2, self.search_radius)
            angle = t * 2
            
            target_x = self.search_center.x + radius * math.cos(angle)
            target_y = self.search_center.y + radius * math.sin(angle)
            target_z = self.search_center.z
            
            target = Position(target_x, target_y, target_z)
            
        elif self.search_pattern == "lawnmower":
            # Lawnmower search pattern
            strip_width = 20.0
            t = current_time * 0.05
            
            strip_number = int(t) % 4
            progress = t - int(t)
            
            if strip_number % 2 == 0:
                target_x = self.search_center.x + (progress - 0.5) * self.search_radius * 2
            else:
                target_x = self.search_center.x + (0.5 - progress) * self.search_radius * 2
            
            target_y = self.search_center.y + (strip_number - 1.5) * strip_width
            target_z = self.search_center.z
            
            target = Position(target_x, target_y, target_z)
        
        else:  # Random walk
            if not hasattr(self, '_random_target') or self.position.distance_to(self._random_target) < 5:
                angle = np.random.uniform(0, 2 * math.pi)
                distance = np.random.uniform(10, self.search_radius)
                self._random_target = Position(
                    self.search_center.x + distance * math.cos(angle),
                    self.search_center.y + distance * math.sin(angle),
                    self.search_center.z
                )
            target = self._random_target
        
        # Calculate force towards search target
        error = target - self.position
        force_magnitude = min(error.magnitude() * 0.3, self.max_acceleration * 0.5)
        
        if error.magnitude() > 0:
            return error.normalize() * force_magnitude
        return Position(0, 0, 0)
    
    def calculate_mapping_force(self) -> Position:
        """Calculate force for systematic area mapping"""
        # Grid-based mapping pattern
        grid_size = 25.0
        current_time = time.time()
        
        # Determine current grid cell
        grid_x = int((current_time * 0.1) % 5) - 2
        grid_y = int((current_time * 0.02) % 5) - 2
        
        target_x = self.search_center.x + grid_x * grid_size
        target_y = self.search_center.y + grid_y * grid_size
        target_z = self.search_center.z
        
        target = Position(target_x, target_y, target_z)
        
        # Add mapped point if close to target
        if self.position.distance_to(target) < 5:
            self.mapped_area.append((target.x, target.y, target.z))
        
        # Calculate force towards mapping target
        error = target - self.position
        force_magnitude = min(error.magnitude() * 0.4, self.max_acceleration * 0.6)
        
        if error.magnitude() > 0:
            return error.normalize() * force_magnitude
        return Position(0, 0, 0)
    
    def update_control(self, dt: float, nearby_drones: List['Drone'], leader_drone: 'Drone' = None):
        """Main control update function combining all forces"""
        total_force = Position(0, 0, 0)
        
        # State-based behavior
        if self.state == DroneState.FOLLOWING and leader_drone:
            following_force = self.calculate_leader_following_force(leader_drone)
            total_force = total_force + following_force
            
        elif self.state == DroneState.SEARCHING:
            search_force = self.calculate_search_force()
            total_force = total_force + search_force
            
        elif self.state == DroneState.MAPPING:
            mapping_force = self.calculate_mapping_force()
            total_force = total_force + mapping_force
            
        elif self.state == DroneState.LEADING:
            # Leader follows predetermined path or user commands
            if hasattr(self, 'waypoints') and self.waypoints:
                target = self.waypoints[0]
                error = target - self.position
                if error.magnitude() < 5:
                    self.waypoints.pop(0)
                else:
                    force_magnitude = min(error.magnitude() * 0.3, self.max_acceleration * 0.5)
                    total_force = error.normalize() * force_magnitude
        
        # Always apply collision avoidance
        avoidance_force = self.calculate_collision_avoidance_force(nearby_drones)
        total_force = total_force + avoidance_force
        
        # Convert force to velocity
        self.velocity.vx += total_force.x * dt
        self.velocity.vy += total_force.y * dt
        self.velocity.vz += total_force.z * dt
        
        # Apply velocity limits
        current_vel = Velocity(self.velocity.vx, self.velocity.vy, self.velocity.vz)
        limited_vel = current_vel.limit(self.max_speed)
        self.velocity = limited_vel
        
        # Apply damping to prevent oscillations
        damping = 0.9
        self.velocity.vx *= damping
        self.velocity.vy *= damping
        self.velocity.vz *= damping

class SwarmController:
    """Main swarm controller managing multiple drones"""
    
    def __init__(self, num_drones: int = 5):
        self.drones = []
        self.num_drones = num_drones
        self.leader_drone = None
        
        # Mission parameters
        self.mission_area_center = Position(0, 0, 30)
        self.mission_area_radius = 100.0
        self.formation_type = "V-formation"
        
        # Communication simulation
        self.communication_range = 50.0
        self.message_queue = queue.Queue()
        
        # Performance metrics
        self.formation_error_history = []
        self.collision_events = []
        self.coverage_map = {}
        
        self.initialize_swarm()
    
    def initialize_swarm(self):
        """Initialize the drone swarm with default positions"""
        # Create drones
        for i in range(self.num_drones):
            is_leader = (i == 0)  # First drone is leader
            
            # Set initial positions in a line formation
            initial_pos = Position(
                i * 10.0 - (self.num_drones - 1) * 5.0,
                0.0,
                25.0 + np.random.uniform(-2, 2)
            )
            
            drone = Drone(i, initial_pos, is_leader)
            
            if is_leader:
                drone.state = DroneState.LEADING
                self.leader_drone = drone
                # Set initial waypoints for leader
                drone.waypoints = [
                    Position(50, 0, 30),
                    Position(50, 50, 30),
                    Position(0, 50, 30),
                    Position(-50, 50, 30),
                    Position(-50, 0, 30),
                    Position(0, 0, 30)
                ]
            else:
                drone.state = DroneState.FOLLOWING
                drone.leader_reference = self.leader_drone
                # Set formation angles for followers
                drone.formation_angle = (i - 1) * (math.pi / 3) - math.pi/2
            
            self.drones.append(drone)
    
    def get_nearby_drones(self, drone: Drone) -> List[Drone]:
        """Get list of drones within communication range"""
        nearby = []
        for other_drone in self.drones:
            if other_drone.id != drone.id:
                distance = drone.position.distance_to(other_drone.position)
                if distance <= self.communication_range:
                    nearby.append(other_drone)
        return nearby
    
    def update_swarm(self, dt: float):
        """Update entire swarm state"""
        # Update each drone
        for drone in self.drones:
            nearby_drones = self.get_nearby_drones(drone)
            drone.update_control(dt, nearby_drones, self.leader_drone)
            drone.update_physics(dt)
        
        # Update performance metrics
        self.update_performance_metrics()
        
        # Handle emergencies
        self.handle_emergencies()
    
    def update_performance_metrics(self):
        """Update swarm performance metrics"""
        if self.leader_drone:
            # Calculate formation error
            formation_errors = []
            for drone in self.drones:
                if not drone.is_leader:
                    distance_to_leader = drone.position.distance_to(self.leader_drone.position)
                    error = abs(distance_to_leader - drone.formation_distance)
                    formation_errors.append(error)
            
            if formation_errors:
                avg_formation_error = np.mean(formation_errors)
                self.formation_error_history.append(avg_formation_error)
        
        # Check for potential collisions
        for i, drone1 in enumerate(self.drones):
            for j, drone2 in enumerate(self.drones[i+1:], i+1):
                distance = drone1.position.distance_to(drone2.position)
                if distance < drone1.collision_radius:
                    collision_event = {
                        'time': time.time(),
                        'drone1': drone1.id,
                        'drone2': drone2.id,
                        'distance': distance
                    }
                    self.collision_events.append(collision_event)
    
    def handle_emergencies(self):
        """Handle emergency situations"""
        for drone in self.drones:
            # Battery emergency
            if drone.battery_level < 15:
                drone.state = DroneState.EMERGENCY
            
            # Communication timeout
            if time.time() - drone.last_communication > drone.communication_timeout:
                # Switch to autonomous mode
                if drone.state == DroneState.FOLLOWING:
                    drone.state = DroneState.SEARCHING
    
    def set_mission_mode(self, mode: str, area_center: Position = None, area_radius: float = None):
        """Set mission mode for the swarm"""
        if area_center:
            self.mission_area_center = area_center
        if area_radius:
            self.mission_area_radius = area_radius
        
        if mode == "search_and_rescue":
            for drone in self.drones:
                drone.state = DroneState.SEARCHING
                drone.search_center = self.mission_area_center
                drone.search_radius = self.mission_area_radius / self.num_drones
                # Assign different search patterns to different drones
                patterns = ["spiral", "lawnmower", "random"]
                drone.search_pattern = patterns[drone.id % len(patterns)]
        
        elif mode == "area_mapping":
            for drone in self.drones:
                drone.state = DroneState.MAPPING
                drone.search_center = self.mission_area_center
                # Divide area among drones
                angle_offset = (drone.id * 2 * math.pi) / self.num_drones
                offset_distance = 30
                drone.search_center = Position(
                    self.mission_area_center.x + offset_distance * math.cos(angle_offset),
                    self.mission_area_center.y + offset_distance * math.sin(angle_offset),
                    self.mission_area_center.z
                )
        
        elif mode == "formation_flight":
            if self.leader_drone:
                self.leader_drone.state = DroneState.LEADING
                for drone in self.drones:
                    if not drone.is_leader:
                        drone.state = DroneState.FOLLOWING
    
    def get_swarm_status(self) -> Dict:
        """Get comprehensive swarm status"""
        status = {
            'num_drones': len(self.drones),
            'leader_id': self.leader_drone.id if self.leader_drone else None,
            'drones': [],
            'performance': {
                'avg_formation_error': np.mean(self.formation_error_history) if self.formation_error_history else 0,
                'collision_events': len(self.collision_events),
                'coverage_percentage': len(self.coverage_map) / 100.0  # Simplified calculation
            }
        }
        
        for drone in self.drones:
            drone_status = {
                'id': drone.id,
                'state': drone.state.value,
                'position': {'x': drone.position.x, 'y': drone.position.y, 'z': drone.position.z},
                'battery': drone.battery_level,
                'is_leader': drone.is_leader,
                'mapped_points': len(drone.mapped_area)
            }
            status['drones'].append(drone_status)
        
        return status
    
    def get_mapped_areas(self) -> List[Tuple[float, float, float]]:
        """Get all mapped areas from all drones"""
        all_mapped = []
        for drone in self.drones:
            all_mapped.extend(drone.mapped_area)
        return all_mapped
    
    def emergency_land_all(self):
        """Emergency landing procedure for all drones"""
        for drone in self.drones:
            drone.state = DroneState.LANDING
            drone.target_position = Position(drone.position.x, drone.position.y, 0)
    
    def change_formation(self, formation_type: str):
        """Change formation type"""
        self.formation_type = formation_type
        
        formation_configs = {
            "V-formation": [0, -math.pi/4, math.pi/4, -math.pi/2, math.pi/2],
            "line": [0, -math.pi/2, math.pi/2, -math.pi, math.pi],
            "diamond": [0, math.pi/2, math.pi, -math.pi/2, 0],
            "circle": [i * 2 * math.pi / self.num_drones for i in range(self.num_drones)]
        }
        
        if formation_type in formation_configs:
            angles = formation_configs[formation_type]
            for i, drone in enumerate(self.drones):
                if not drone.is_leader and i-1 < len(angles)-1:
                    drone.formation_angle = angles[i]

# ESP32 Integration Classes
class ESP32Interface:
    """Interface for ESP32 drone communication"""
    
    def __init__(self, drone_id: int, mqtt_broker: str = "localhost"):
        self.drone_id = drone_id
        self.mqtt_broker = mqtt_broker
        self.connected = False
        
    def connect_mqtt(self):
        """Connect to MQTT broker (simulated)"""
        # In real implementation, use paho-mqtt
        self.connected = True
        print(f"ESP32 Drone {self.drone_id} connected to MQTT broker")
    
    def send_command(self, command: Dict):
        """Send command to ESP32 via MQTT"""
        if self.connected:
            # Simulate MQTT publish
            topic = f"swarm/drone/{self.drone_id}/command"
            print(f"Publishing to {topic}: {command}")
    
    def get_telemetry(self) -> Dict:
        """Get telemetry data from ESP32"""
        # Simulate sensor data
        return {
            'battery': np.random.uniform(20, 100),
            'altitude': np.random.uniform(10, 50),
            'gps_lat': 40.7589 + np.random.uniform(-0.001, 0.001),
            'gps_lon': -73.9851 + np.random.uniform(-0.001, 0.001),
            'orientation': {
                'roll': np.random.uniform(-10, 10),
                'pitch': np.random.uniform(-10, 10),
                'yaw': np.random.uniform(0, 360)
            }
        }

# Testing and validation functions
def validate_algorithms():
    """Validate swarm algorithms with test scenarios"""
    print("Validating swarm algorithms...")
    
    # Test 1: Formation maintenance
    swarm = SwarmController(5)
    dt = 0.1
    test_duration = 10.0
    steps = int(test_duration / dt)
    
    print("Test 1: Formation maintenance")
    for step in range(steps):
        swarm.update_swarm(dt)
        
        if step % 50 == 0:
            status = swarm.get_swarm_status()
            print(f"  Step {step}: Formation error = {status['performance']['avg_formation_error']:.2f}m")
    
    # Test 2: Collision avoidance
    print("\nTest 2: Collision avoidance")
    # Place drones close together
    for i, drone in enumerate(swarm.drones):
        drone.position = Position(i * 3, 0, 25)  # Close spacing
    
    min_distances = []
    for step in range(steps):
        swarm.update_swarm(dt)
        
        # Calculate minimum distance between any two drones
        min_dist = float('inf')
        for i, drone1 in enumerate(swarm.drones):
            for j, drone2 in enumerate(swarm.drones[i+1:], i+1):
                dist = drone1.position.distance_to(drone2.position)
                min_dist = min(min_dist, dist)
        min_distances.append(min_dist)
    
    print(f"  Minimum separation: {min(min_distances):.2f}m")
    print(f"  Average separation: {np.mean(min_distances):.2f}m")
    
    # Test 3: Search pattern efficiency
    print("\nTest 3: Search pattern coverage")
    swarm.set_mission_mode("search_and_rescue", Position(0, 0, 30), 50)
    
    covered_area = set()
    for step in range(steps * 2):  # Longer test
        swarm.update_swarm(dt)
        
        # Record coverage
        for drone in swarm.drones:
            grid_x = int(drone.position.x / 5) * 5
            grid_y = int(drone.position.y / 5) * 5
            covered_area.add((grid_x, grid_y))
    
    coverage_percentage = len(covered_area) / (20 * 20) * 100  # 100x100 area, 5x5 grid
    print(f"  Area coverage: {coverage_percentage:.1f}%")
    
    print("Algorithm validation completed!")

if __name__ == "__main__":
    # Run validation when module is executed directly
    validate_algorithms()