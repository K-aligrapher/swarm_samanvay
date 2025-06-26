#!/usr/bin/env python3
"""
Swarm Drone System - Complete Demo
This script demonstrates the complete swarm drone system with all features.
"""

import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import json
from datetime import datetime

# Import our custom modules
try:
    from swarm_algorithms import SwarmController, Position, DroneState
    from swarm_visualization import SwarmVisualization, TerrainMapper, SwarmControlUI, RealWorldIntegration
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure all required files are in the same directory")
    sys.exit(1)

class SwarmDemo:
    """Complete demonstration of the swarm drone system"""
    
    def __init__(self):
        self.swarm = None
        self.visualization = None
        self.terrain_mapper = None
        
    def demo_algorithms(self):
        """Demonstrate core algorithms without GUI"""
        print("=== Swarm Drone Algorithm Demo ===")
        print("Testing leader-follower and collision avoidance algorithms...")
        
        # Create swarm
        self.swarm = SwarmController(5)
        
        # Set initial positions for testing
        positions = [
            Position(0, 0, 30),      # Leader
            Position(-20, -15, 25),  # Follower 1
            Position(20, -15, 25),   # Follower 2
            Position(-20, 15, 35),   # Follower 3
            Position(20, 15, 35),    # Follower 4
        ]
        
        for i, pos in enumerate(positions):
            self.swarm.drones[i].position = pos
            
        print(f"Initial positions:")
        for i, drone in enumerate(self.swarm.drones):
            pos = drone.position
            print(f"  Drone {i}: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) - {'Leader' if drone.is_leader else 'Follower'}")
        
        # Run simulation
        print("\nRunning simulation...")
        dt = 0.1
        steps = 200
        
        # Store positions for analysis
        position_history = {i: [] for i in range(5)}
        
        for step in range(steps):
            self.swarm.update_swarm(dt)
            
            # Record positions
            for i, drone in enumerate(self.swarm.drones):
                position_history[i].append([drone.position.x, drone.position.y, drone.position.z])
            
            # Print progress every 50 steps
            if step % 50 == 0:
                print(f"  Step {step}/{steps}")
                leader = self.swarm.drones[0]
                print(f"    Leader position: ({leader.position.x:.1f}, {leader.position.y:.1f}, {leader.position.z:.1f})")
                
                # Check formation maintenance
                formation_distances = []
                for i in range(1, len(self.swarm.drones)):
                    dist = self.swarm.drones[i].position.distance_to(leader.position)
                    formation_distances.append(dist)
                
                avg_formation_dist = np.mean(formation_distances)
                print(f"    Average formation distance: {avg_formation_dist:.1f}m")
        
        print("\nAlgorithm test completed!")
        
        # Analyze results
        self.analyze_performance(position_history)
        
        return position_history
    
    def analyze_performance(self, position_history):
        """Analyze swarm performance metrics"""
        print("\n=== Performance Analysis ===")
        
        # Formation maintenance analysis
        leader_positions = np.array(position_history[0])
        formation_errors = []
        
        for step in range(len(leader_positions)):
            step_errors = []
            for drone_id in range(1, 5):
                follower_pos = np.array(position_history[drone_id][step])
                leader_pos = leader_positions[step]
                distance = np.linalg.norm(follower_pos - leader_pos)
                step_errors.append(abs(distance - 15.0))  # Target formation distance
            formation_errors.append(np.mean(step_errors))
        
        avg_formation_error = np.mean(formation_errors)
        max_formation_error = np.max(formation_errors)
        
        print(f"Formation Maintenance:")
        print(f"  Average formation error: {avg_formation_error:.2f}m")
        print(f"  Maximum formation error: {max_formation_error:.2f}m")
        
        # Collision analysis
        min_distances = []
        for step in range(len(leader_positions)):
            step_distances = []
            for i in range(5):
                for j in range(i+1, 5):
                    pos_i = np.array(position_history[i][step])
                    pos_j = np.array(position_history[j][step])
                    distance = np.linalg.norm(pos_i - pos_j)
                    step_distances.append(distance)
            min_distances.append(min(step_distances))
        
        min_separation = min(min_distances)
        avg_separation = np.mean(min_distances)
        
        print(f"Collision Avoidance:")
        print(f"  Minimum separation: {min_separation:.2f}m")
        print(f"  Average minimum separation: {avg_separation:.2f}m")
        print(f"  Collision threshold: 8.0m")
        
        if min_separation > 8.0:
            print("  ✓ No collisions detected")
        else:
            print("  ⚠ Potential collision detected")
    
    def demo_3d_visualization(self, position_history=None):
        """Demonstrate 3D visualization"""
        print("\n=== 3D Visualization Demo ===")
        
        if not position_history:
            print("Running quick simulation for visualization...")
            position_history = self.demo_algorithms()
        
        # Create basic 3D plot
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectories
        colors = ['red', 'blue', 'green', 'orange', 'purple']
        labels = ['Leader', 'Follower 1', 'Follower 2', 'Follower 3', 'Follower 4']
        
        for drone_id in range(5):
            trajectory = np.array(position_history[drone_id])
            ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 
                   color=colors[drone_id], label=labels[drone_id], linewidth=2)
            
            # Mark start and end positions
            ax.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], 
                      color=colors[drone_id], s=100, marker='o', alpha=0.7)
            ax.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], 
                      color=colors[drone_id], s=100, marker='s', alpha=0.7)
        
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_zlabel('Z (meters)')
        ax.set_title('Swarm Drone Trajectories')
        ax.legend()
        
        # Set equal aspect ratio
        max_range = 100
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([0, 60])
        
        plt.tight_layout()
        plt.show()
        
        print("3D visualization displayed. Close the window to continue.")
    
    def demo_terrain_mapping(self):
        """Demonstrate terrain mapping capabilities"""
        print("\n=== Terrain Mapping Demo ===")
        
        # Create terrain mapper
        self.terrain_mapper = TerrainMapper()
        
        # Generate sample terrain
        print("Generating sample terrain data...")
        lat_bounds = (40.0, 40.05)  # Small area around NYC
        lon_bounds = (-74.1, -74.05)
        
        elevation_grid, lats, lons = self.terrain_mapper.get_elevation_data(
            lat_bounds, lon_bounds, resolution=30
        )
        
        # Create terrain visualization
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Convert to meter coordinates for display
        center_lat, center_lon = np.mean(lats), np.mean(lons)
        X_meters = np.zeros_like(elevation_grid)
        Y_meters = np.zeros_like(elevation_grid)
        
        for i in range(elevation_grid.shape[0]):
            for j in range(elevation_grid.shape[1]):
                x_m, y_m = self.terrain_mapper.coords_to_meters(
                    lats[i], lons[j], center_lat, center_lon
                )
                X_meters[i, j] = x_m
                Y_meters[i, j] = y_m
        
        # Plot terrain surface
        surf = ax.plot_surface(X_meters, Y_meters, elevation_grid, 
                              cmap='terrain', alpha=0.8)
        
        # Add sample drone flight path over terrain
        flight_path_x = np.linspace(-2000, 2000, 50)
        flight_path_y = np.linspace(-2000, 2000, 50)
        flight_path_z = np.full_like(flight_path_x, 250)  # Constant altitude
        
        ax.plot(flight_path_x, flight_path_y, flight_path_z, 
               'r-', linewidth=3, label='Drone Flight Path')
        
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_zlabel('Altitude (meters)')
        ax.set_title('3D Terrain Mapping')
        ax.legend()
        
        plt.colorbar(surf, ax=ax, shrink=0.5, aspect=5)
        plt.show()
        
        print("Terrain mapping visualization displayed.")
    
    def demo_search_and_rescue(self):
        """Demonstrate search and rescue mission"""
        print("\n=== Search and Rescue Demo ===")
        
        # Create swarm for search mission
        self.swarm = SwarmController(5)
        
        # Set search area parameters
        search_area = {
            'center': Position(0, 0, 30),
            'radius': 80,
            'target_locations': [
                Position(45, -30, 0),   # Target 1
                Position(-25, 40, 0),   # Target 2  
                Position(30, 35, 0),    # Target 3
            ]
        }
        
        print(f"Search area: {search_area['radius']}m radius")
        print(f"Number of targets: {len(search_area['target_locations'])}")
        
        # Initialize search pattern
        search_positions = self.generate_search_pattern(search_area)
        
        # Set drones to search state
        for drone in self.swarm.drones:
            drone.state = DroneState.SEARCHING
        
        # Run search simulation
        dt = 0.1
        max_steps = 500
        found_targets = []
        detection_range = 15.0  # meters
        
        position_history = {i: [] for i in range(5)}
        
        print("Starting search mission...")
        for step in range(max_steps):
            self.swarm.update_swarm(dt)
            
            # Record positions
            for i, drone in enumerate(self.swarm.drones):
                position_history[i].append([drone.position.x, drone.position.y, drone.position.z])
            
            # Check for target detection
            for i, drone in enumerate(self.swarm.drones):
                for j, target in enumerate(search_area['target_locations']):
                    if j not in found_targets:
                        distance = drone.position.distance_to(target)
                        if distance <= detection_range:
                            found_targets.append(j)
                            print(f"  Target {j+1} found by Drone {i} at step {step}")
            
            # Progress update
            if step % 100 == 0:
                print(f"  Step {step}/{max_steps} - Targets found: {len(found_targets)}")
            
            # Stop if all targets found
            if len(found_targets) == len(search_area['target_locations']):
                print(f"All targets found! Mission completed in {step} steps.")
                break
        
        # Visualize search mission
        self.visualize_search_mission(position_history, search_area, found_targets)
        
        return position_history, search_area, found_targets
    
    def generate_search_pattern(self, search_area):
        """Generate systematic search pattern for drones"""
        center = search_area['center']
        radius = search_area['radius']
        
        # Create spiral search pattern
        angles = np.linspace(0, 4*np.pi, 100)
        radii = np.linspace(10, radius, 100)
        
        search_positions = []
        for angle, r in zip(angles, radii):
            x = center.x + r * np.cos(angle)
            y = center.y + r * np.sin(angle)
            z = center.z
            search_positions.append(Position(x, y, z))
        
        return search_positions
    
    def visualize_search_mission(self, position_history, search_area, found_targets):
        """Visualize search and rescue mission results"""
        fig = plt.figure(figsize=(15, 10))
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(121, projection='3d')
        
        colors = ['red', 'blue', 'green', 'orange', 'purple']
        for drone_id in range(5):
            trajectory = np.array(position_history[drone_id])
            ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 
                    color=colors[drone_id], label=f'Drone {drone_id}', alpha=0.7)
            
            # Mark final position
            ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], 
                       color=colors[drone_id], s=100, marker='o')
        
        # Mark targets
        for i, target in enumerate(search_area['target_locations']):
            marker = 'X' if i in found_targets else 'o'
            color = 'green' if i in found_targets else 'red'
            ax1.scatter(target.x, target.y, target.z, 
                       color=color, s=200, marker=marker, 
                       label=f'Target {i+1} {"(Found)" if i in found_targets else "(Missed)"}')
        
        # Draw search area boundary
        theta = np.linspace(0, 2*np.pi, 100)
        circle_x = search_area['center'].x + search_area['radius'] * np.cos(theta)
        circle_y = search_area['center'].y + search_area['radius'] * np.sin(theta)
        circle_z = np.full_like(circle_x, search_area['center'].z)
        ax1.plot(circle_x, circle_y, circle_z, 'k--', alpha=0.5, label='Search Boundary')
        
        ax1.set_xlabel('X (meters)')
        ax1.set_ylabel('Y (meters)')
        ax1.set_zlabel('Z (meters)')
        ax1.set_title('Search and Rescue Mission - 3D View')
        ax1.legend()
        
        # 2D overhead view
        ax2 = fig.add_subplot(122)
        
        for drone_id in range(5):
            trajectory = np.array(position_history[drone_id])
            ax2.plot(trajectory[:, 0], trajectory[:, 1], 
                    color=colors[drone_id], label=f'Drone {drone_id}', alpha=0.7)
            ax2.scatter(trajectory[-1, 0], trajectory[-1, 1], 
                       color=colors[drone_id], s=50, marker='o')
        
        # Mark targets
        for i, target in enumerate(search_area['target_locations']):
            marker = 'X' if i in found_targets else 'o'
            color = 'green' if i in found_targets else 'red'
            ax2.scatter(target.x, target.y, 
                       color=color, s=200, marker=marker, 
                       label=f'Target {i+1} {"(Found)" if i in found_targets else "(Missed)"}')
        
        # Draw search area circle
        circle = plt.Circle((search_area['center'].x, search_area['center'].y), 
                           search_area['radius'], fill=False, linestyle='--', alpha=0.5)
        ax2.add_patch(circle)
        
        ax2.set_xlabel('X (meters)')
        ax2.set_ylabel('Y (meters)')
        ax2.set_title('Search and Rescue Mission - Overhead View')
        ax2.legend()
        ax2.axis('equal')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def demo_formation_flight(self):
        """Demonstrate different formation flight patterns"""
        print("\n=== Formation Flight Demo ===")
        
        formations = ['V-Formation', 'Line Formation', 'Diamond Formation', 'Circle Formation']
        
        for formation_type in formations:
            print(f"\nTesting {formation_type}...")
            
            # Create new swarm for each formation
            swarm = SwarmController(5)
            
            # Set formation-specific positions
            if formation_type == 'V-Formation':
                target_positions = [
                    Position(0, 0, 30),      # Leader at front
                    Position(-15, -15, 30),  # Left wing
                    Position(15, -15, 30),   # Right wing
                    Position(-25, -25, 30),  # Left rear
                    Position(25, -25, 30),   # Right rear
                ]
            elif formation_type == 'Line Formation':
                target_positions = [
                    Position(0, 0, 30),      # Leader
                    Position(-20, 0, 30),    # Left
                    Position(20, 0, 30),     # Right
                    Position(-40, 0, 30),    # Far left
                    Position(40, 0, 30),     # Far right
                ]
            elif formation_type == 'Diamond Formation':
                target_positions = [
                    Position(0, 0, 30),      # Center leader
                    Position(0, 20, 30),     # Front
                    Position(0, -20, 30),    # Rear
                    Position(-20, 0, 30),    # Left
                    Position(20, 0, 30),     # Right
                ]
            else:  # Circle Formation
                angles = np.linspace(0, 2*np.pi, 6)[:-1]  # 5 positions
                radius = 25
                target_positions = []
                for angle in angles:
                    x = radius * np.cos(angle)
                    y = radius * np.sin(angle)
                    target_positions.append(Position(x, y, 30))
            
            # Set target positions for drones
            for i, pos in enumerate(target_positions):
                swarm.drones[i].target_position = pos
                swarm.drones[i].state = DroneState.FOLLOWING
            
            # Run formation simulation
            position_history = {i: [] for i in range(5)}
            dt = 0.1
            steps = 150
            
            for step in range(steps):
                swarm.update_swarm(dt)
                
                for i, drone in enumerate(swarm.drones):
                    position_history[i].append([drone.position.x, drone.position.y, drone.position.z])
            
            # Visualize this formation
            self.visualize_formation(position_history, formation_type, target_positions)
    
    def visualize_formation(self, position_history, formation_name, target_positions):
        """Visualize formation flight pattern"""
        fig = plt.figure(figsize=(12, 6))
        
        # 3D view
        ax1 = fig.add_subplot(121, projection='3d')
        colors = ['red', 'blue', 'green', 'orange', 'purple']
        
        for drone_id in range(5):
            trajectory = np.array(position_history[drone_id])
            ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 
                    color=colors[drone_id], alpha=0.7)
            ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], 
                       color=colors[drone_id], s=100, marker='o')
            
            # Mark target position
            target = target_positions[drone_id]
            ax1.scatter(target.x, target.y, target.z, 
                       color=colors[drone_id], s=50, marker='x', alpha=0.7)
        
        ax1.set_xlabel('X (meters)')
        ax1.set_ylabel('Y (meters)')
        ax1.set_zlabel('Z (meters)')
        ax1.set_title(f'{formation_name} - 3D View')
        
        # 2D overhead view
        ax2 = fig.add_subplot(122)
        
        for drone_id in range(5):
            trajectory = np.array(position_history[drone_id])
            ax2.plot(trajectory[:, 0], trajectory[:, 1], 
                    color=colors[drone_id], label=f'Drone {drone_id}', alpha=0.7)
            ax2.scatter(trajectory[-1, 0], trajectory[-1, 1], 
                       color=colors[drone_id], s=100, marker='o')
            
            # Mark target position
            target = target_positions[drone_id]
            ax2.scatter(target.x, target.y, 
                       color=colors[drone_id], s=50, marker='x', alpha=0.7)
        
        ax2.set_xlabel('X (meters)')
        ax2.set_ylabel('Y (meters)')
        ax2.set_title(f'{formation_name} - Overhead View')
        ax2.legend()
        ax2.axis('equal')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def demo_mqtt_communication(self):
        """Demonstrate MQTT communication simulation"""
        print("\n=== MQTT Communication Demo ===")
        
        from swarm_visualization import MQTTSimulator, ESP32DroneInterface
        
        # Create MQTT simulator and drone interfaces
        mqtt_sim = MQTTSimulator()
        drone_interfaces = [ESP32DroneInterface(i) for i in range(5)]
        
        print("Simulating MQTT communication between drones...")
        
        # Simulate position updates
        for i, interface in enumerate(drone_interfaces):
            pos = Position(i*10, i*5, 25 + i*2)
            mqtt_sim.publish_position(i, pos)
            mqtt_sim.publish_status(i, DroneState.FOLLOWING)
        
        # Get and display messages
        messages = mqtt_sim.get_messages()
        
        print(f"Generated {len(messages)} MQTT messages:")
        for msg in messages:
            print(f"  Topic: {msg['topic']}")
            print(f"  Payload: {msg['payload']}")
            print()
        
        # Simulate sensor data collection
        print("Collecting sensor data from ESP32 drones...")
        for i, interface in enumerate(drone_interfaces):
            sensor_data = interface.get_sensor_data()
            print(f"Drone {i} sensors: Battery={sensor_data['battery']:.1f}%, "
                  f"GPS Quality={sensor_data['gps_quality']}, "
                  f"Altitude={sensor_data['altitude']:.1f}m")
    
    def demo_real_world_integration(self):
        """Demonstrate real-world integration features"""
        print("\n=== Real-World Integration Demo ===")
        
        # Create integration handler
        integration = RealWorldIntegration()
        
        # Create sample mission data
        self.swarm = SwarmController(5)
        
        # Set some sample positions
        positions = [
            Position(10, 20, 30),
            Position(-15, 25, 35),
            Position(25, -10, 28),
            Position(-20, -15, 32),
            Position(15, 30, 25),
        ]
        
        for i, pos in enumerate(positions):
            self.swarm.drones[i].position = pos
        
        # Generate mapped areas
        mapped_areas = []
        for i in range(50):
            x = np.random.uniform(-50, 50)
            y = np.random.uniform(-50, 50)
            mapped_areas.append((x, y))
        
        # Create mission map
        print("Creating interactive mission map...")
        try:
            integration.create_mission_map(positions, mapped_areas, "demo_mission_map.html")
            print("Mission map saved as 'demo_mission_map.html'")
        except Exception as e:
            print(f"Error creating map: {e}")
        
        # Export mission data
        print("Exporting mission data...")
        try:
            integration.export_mission_data(self.swarm, "demo_mission_data.json")
            print("Mission data saved as 'demo_mission_data.json'")
        except Exception as e:
            print(f"Error exporting data: {e}")
    
    def create_demo_video(self):
        """Create a demo video of swarm operation"""
        print("\n=== Creating Demo Video ===")
        
        # Create swarm and terrain
        self.swarm = SwarmController(5)
        self.terrain_mapper = TerrainMapper()
        
        # Generate terrain
        terrain_data = self.terrain_mapper.get_elevation_data(
            (40.0, 40.02), (-74.1, -74.08), resolution=25
        )
        
        # Create visualization
        self.visualization = SwarmVisualization(self.swarm, self.terrain_mapper)
        self.visualization.setup_3d_plot(terrain_data)
        
        print("Starting animation recording...")
        
        # Start animation
        anim = self.visualization.start_animation()
        
        # Save as video
        try:
            self.visualization.save_animation_as_video("swarm_demo_video.mp4")
            print("Demo video saved as 'swarm_demo_video.mp4'")
        except Exception as e:
            print(f"Error creating video: {e}")
            print("Make sure ffmpeg is installed for video creation")
    
    def run_full_demo(self):
        """Run the complete demonstration"""
        print("="*60)
        print("SWARM DRONE SYSTEM - COMPLETE DEMONSTRATION")
        print("="*60)
        print(f"Demo started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print()
        
        try:
            # Core algorithm testing
            position_history = self.demo_algorithms()
            
            # 3D visualization
            self.demo_3d_visualization(position_history)
            
            # Terrain mapping
            self.demo_terrain_mapping()
            
            # Search and rescue mission
            self.demo_search_and_rescue()
            
            # Formation flight patterns
            self.demo_formation_flight()
            
            # MQTT communication
            self.demo_mqtt_communication()
            
            # Real-world integration
            self.demo_real_world_integration()
            
            print("\n" + "="*60)
            print("DEMONSTRATION COMPLETED SUCCESSFULLY")
            print("="*60)
            print("\nFiles generated:")
            print("- demo_mission_map.html (Interactive map)")
            print("- demo_mission_data.json (Mission data)")
            print("\nTo run the full GUI application, execute:")
            print("python swarm_visualization.py")
            
        except Exception as e:
            print(f"\nDemo error: {str(e)}")
            print("Please check that all dependencies are installed:")
            print("pip install numpy matplotlib folium")

def main():
    """Main function to run demonstrations"""
    print("Swarm Drone System Demo")
    print("Choose demo mode:")
    print("1. Full Demo (all features)")
    print("2. Algorithm Testing Only")
    print("3. 3D Visualization Only")
    print("4. Search and Rescue Demo")
    print("5. Formation Flight Demo")
    print("6. GUI Application")
    print("7. Create Demo Video")
    
    try:
        choice = input("\nEnter choice (1-7): ").strip()
        
        demo = SwarmDemo()
        
        if choice == '1':
            demo.run_full_demo()
        elif choice == '2':
            demo.demo_algorithms()
        elif choice == '3':
            demo.demo_3d_visualization()
        elif choice == '4':
            demo.demo_search_and_rescue()
        elif choice == '5':
            demo.demo_formation_flight()
        elif choice == '6':
            print("Starting GUI application...")
            app = SwarmControlUI()
            app.run()
        elif choice == '7':
            demo.create_demo_video()
        else:
            print("Invalid choice. Running full demo...")
            demo.run_full_demo()
            
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user.")
    except Exception as e:
        print(f"\nError running demo: {str(e)}")

if __name__ == "__main__":
    main()