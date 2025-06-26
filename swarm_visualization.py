import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.patches import Circle
import requests
import folium
from folium import plugins
import base64
from io import BytesIO
import PIL.Image as Image
from typing import List, Tuple, Dict, Optional
import json
import threading
import time
from dataclasses import dataclass
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib.backends.backend_tkagg as tkagg
from matplotlib.figure import Figure

# Import our algorithm components
from swarm_algorithms import SwarmController, Position, DroneState

class TerrainMapper:
    def __init__(self):
        self.elevation_data = {}
        self.terrain_bounds = None
        
    def get_elevation_data(self, lat_bounds: Tuple[float, float], lon_bounds: Tuple[float, float], 
                          resolution: int = 50) -> np.ndarray:
        """
        Simulate elevation data for terrain mapping
        In real implementation, this would fetch from elevation APIs like:
        - Open Elevation API
        - Google Elevation API
        - USGS Elevation Point Query Service
        """
        min_lat, max_lat = lat_bounds
        min_lon, max_lon = lon_bounds
        
        # Create coordinate grids
        lats = np.linspace(min_lat, max_lat, resolution)
        lons = np.linspace(min_lon, max_lon, resolution)
        
        # Simulate realistic terrain with multiple features
        elevation_grid = np.zeros((resolution, resolution))
        
        for i, lat in enumerate(lats):
            for j, lon in enumerate(lons):
                # Simulate mountainous terrain with valleys
                elevation = (
                    100 * np.sin(lat * 0.1) * np.cos(lon * 0.1) +
                    50 * np.sin(lat * 0.2) * np.sin(lon * 0.15) +
                    30 * np.random.normal(0, 1) +
                    200  # Base elevation
                )
                elevation_grid[i, j] = max(0, elevation)  # No negative elevations
        
        self.terrain_bounds = (min_lat, max_lat, min_lon, max_lon)
        return elevation_grid, lats, lons
    
    def coords_to_meters(self, lat: float, lon: float, center_lat: float, center_lon: float) -> Tuple[float, float]:
        """Convert lat/lon to local meter coordinates"""
        # Approximate conversion (not accurate for large distances)
        lat_to_m = 111000  # meters per degree latitude
        lon_to_m = 111000 * np.cos(np.radians(center_lat))  # meters per degree longitude
        
        x = (lon - center_lon) * lon_to_m
        y = (lat - center_lat) * lat_to_m
        
        return x, y

class SwarmVisualization:
    def __init__(self, swarm_controller: SwarmController, terrain_mapper: TerrainMapper):
        self.swarm = swarm_controller
        self.terrain = terrain_mapper
        self.fig = None
        self.ax = None
        self.drone_plots = []
        self.trail_plots = []
        self.mapped_area_plot = None
        self.terrain_surface = None
        
        # Animation parameters
        self.animation = None
        self.dt = 0.1
        self.frame_count = 0
        self.max_frames = 1000
        
        # Trail tracking
        self.drone_trails = [[] for _ in range(len(swarm_controller.drones))]
        self.mapped_points = []
        
        # Video recording
        self.frames_buffer = []
        self.recording = False
        
    def setup_3d_plot(self, terrain_data: Optional[Tuple] = None):
        """Setup the 3D matplotlib plot"""
        self.fig = Figure(figsize=(15, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Setup terrain if provided
        if terrain_data:
            elevation_grid, lats, lons = terrain_data
            # Convert to meter coordinates for visualization
            center_lat, center_lon = np.mean(lats), np.mean(lons)
            
            X, Y = np.meshgrid(lons, lats)
            X_meters = np.zeros_like(X)
            Y_meters = np.zeros_like(Y)
            
            for i in range(X.shape[0]):
                for j in range(X.shape[1]):
                    x_m, y_m = self.terrain.coords_to_meters(Y[i,j], X[i,j], center_lat, center_lon)
                    X_meters[i,j] = x_m
                    Y_meters[i,j] = y_m
            
            # Plot terrain surface
            self.terrain_surface = self.ax.plot_surface(
                X_meters, Y_meters, elevation_grid, 
                alpha=0.3, cmap='terrain', linewidth=0, antialiased=True
            )
        
        # Initialize drone plots
        colors = ['red', 'blue', 'green', 'orange', 'purple']
        for i, drone in enumerate(self.swarm.drones):
            color = colors[i % len(colors)]
            # Drone position marker
            drone_plot, = self.ax.plot([drone.position.x], [drone.position.y], [drone.position.z], 
                                     'o', color=color, markersize=10, 
                                     label=f'Drone {i} ({"Leader" if drone.is_leader else "Follower"})')
            self.drone_plots.append(drone_plot)
            
            # Trail line
            trail_plot, = self.ax.plot([], [], [], '-', color=color, alpha=0.6, linewidth=2)
            self.trail_plots.append(trail_plot)
        
        # Mapped area visualization
        self.mapped_area_plot, = self.ax.plot([], [], [], 'k.', markersize=2, alpha=0.5)
        
        # Set labels and title
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_zlabel('Z (meters)')
        self.ax.set_title('Swarm Drone System - 3D Visualization')
        self.ax.legend()
        
        # Set axis limits
        self.ax.set_xlim([-150, 150])
        self.ax.set_ylim([-150, 150])
        self.ax.set_zlim([0, 100])
        
    def update_animation(self, frame):
        """Animation update function"""
        # Update swarm
        self.swarm.update_swarm(self.dt)
        
        # Update drone positions and trails
        for i, (drone, drone_plot, trail_plot) in enumerate(zip(self.swarm.drones, self.drone_plots, self.trail_plots)):
            # Update position
            drone_plot.set_data_3d([drone.position.x], [drone.position.y], [drone.position.z])
            
            # Update trail
            self.drone_trails[i].append([drone.position.x, drone.position.y, drone.position.z])
            if len(self.drone_trails[i]) > 100:  # Limit trail length
                self.drone_trails[i] = self.drone_trails[i][-100:]
            
            if len(self.drone_trails[i]) > 1:
                trail_array = np.array(self.drone_trails[i])
                trail_plot.set_data_3d(trail_array[:, 0], trail_array[:, 1], trail_array[:, 2])
        
        # Update mapped areas
        mapped_points = self.swarm.get_mapped_areas()
        if mapped_points:
            mapped_array = np.array(mapped_points)
            self.mapped_area_plot.set_data_3d(mapped_array[:, 0], mapped_array[:, 1], mapped_array[:, 2])
        
        # Store frame for video if recording
        if self.recording:
            self.frames_buffer.append(self.fig)
        
        self.frame_count += 1
        
        return self.drone_plots + self.trail_plots + [self.mapped_area_plot]
    
    def start_animation(self):
        """Start the 3D animation"""
        self.animation = animation.FuncAnimation(
            self.fig, self.update_animation, frames=self.max_frames,
            interval=50, blit=False, repeat=True
        )
        return self.animation
    
    def save_animation_as_video(self, filename: str = "swarm_simulation.mp4"):
        """Save animation as video file"""
        if self.animation:
            Writer = animation.writers['ffmpeg']
            writer = Writer(fps=20, metadata=dict(artist='SwarmDroneSystem'), bitrate=1800)
            self.animation.save(filename, writer=writer)
            print(f"Animation saved as {filename}")

class SwarmControlUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Swarm Drone Control System")
        self.root.geometry("1400x900")
        
        # Initialize components
        self.swarm = SwarmController(5)
        self.terrain_mapper = TerrainMapper()
        self.visualization = None
        
        # UI Components
        self.setup_ui()
        
        # Simulation state
        self.simulation_running = False
        self.simulation_thread = None
        
    def setup_ui(self):
        """Setup the user interface"""
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Control panel
        control_frame = ttk.LabelFrame(main_frame, text="Mission Control")
        control_frame.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))
        
        # Mission type selection
        mission_frame = ttk.Frame(control_frame)
        mission_frame.pack(side=tk.LEFT, padx=10, pady=5)
        
        ttk.Label(mission_frame, text="Mission Type:").pack(side=tk.LEFT)
        self.mission_type = ttk.Combobox(mission_frame, values=["Search and Rescue", "Area Mapping", "Formation Flight"])
        self.mission_type.set("Search and Rescue")
        self.mission_type.pack(side=tk.LEFT, padx=5)
        
        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(side=tk.LEFT, padx=20)
        
        self.start_button = ttk.Button(button_frame, text="Start Mission", command=self.start_mission)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = ttk.Button(button_frame, text="Stop Mission", command=self.stop_mission, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        self.record_button = ttk.Button(button_frame, text="Record Video", command=self.toggle_recording)
        self.record_button.pack(side=tk.LEFT, padx=5)
        
        # Terrain selection
        terrain_frame = ttk.Frame(control_frame)
        terrain_frame.pack(side=tk.LEFT, padx=20)
        
        ttk.Label(terrain_frame, text="Terrain:").pack(side=tk.LEFT)
        self.terrain_type = ttk.Combobox(terrain_frame, values=["Mountainous", "Flat", "Coastal", "Urban"])
        self.terrain_type.set("Mountainous")
        self.terrain_type.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(terrain_frame, text="Load Terrain", command=self.load_terrain).pack(side=tk.LEFT, padx=5)
        
        # Status panel
        status_frame = ttk.LabelFrame(main_frame, text="System Status")
        status_frame.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))
        
        self.status_text = tk.Text(status_frame, height=8, width=50)
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        status_scroll = ttk.Scrollbar(status_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        status_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.status_text.config(yscrollcommand=status_scroll.set)
        
        # Drone status panel
        drone_frame = ttk.LabelFrame(status_frame, text="Drone Status")
        drone_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        
        self.drone_status_vars = []
        for i in range(5):
            var = tk.StringVar(value=f"Drone {i}: IDLE")
            self.drone_status_vars.append(var)
            ttk.Label(drone_frame, textvariable=var).pack(anchor=tk.W, padx=5, pady=2)
        
        # Visualization frame
        viz_frame = ttk.LabelFrame(main_frame, text="3D Visualization")
        viz_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        
        # Matplotlib canvas will be added here
        self.canvas_frame = viz_frame
        
    def log_message(self, message: str):
        """Add message to status log"""
        self.status_text.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n")
        self.status_text.see(tk.END)
        self.root.update()
        
    def update_drone_status(self):
        """Update drone status display"""
        for i, drone in enumerate(self.swarm.drones):
            status = f"Drone {i}: {drone.state.value.upper()} - Pos: ({drone.position.x:.1f}, {drone.position.y:.1f}, {drone.position.z:.1f})"
            self.drone_status_vars[i].set(status)
        self.root.update()
        
    def load_terrain(self):
        """Load terrain data based on selection"""
        terrain_type = self.terrain_type.get()
        self.log_message(f"Loading {terrain_type} terrain...")
        
        # Simulate different terrain types with different coordinate bounds
        terrain_bounds = {
            "Mountainous": (40.0, 40.1, -74.1, -74.0),  # Example coordinates
            "Flat": (39.0, 39.1, -75.1, -75.0),
            "Coastal": (36.0, 36.1, -76.1, -76.0),
            "Urban": (41.0, 41.1, -73.1, -73.0)
        }
        
        bounds = terrain_bounds.get(terrain_type, (40.0, 40.1, -74.1, -74.0))
        lat_bounds = (bounds[0], bounds[1])
        lon_bounds = (bounds[2], bounds[3])
        
        try:
            terrain_data = self.terrain_mapper.get_elevation_data(lat_bounds, lon_bounds)
            self.log_message(f"{terrain_type} terrain loaded successfully")
            
            # Setup visualization with terrain
            self.setup_visualization(terrain_data)
            
        except Exception as e:
            self.log_message(f"Error loading terrain: {str(e)}")
    
    def setup_visualization(self, terrain_data=None):
        """Setup the 3D visualization"""
        try:
            # Clear existing canvas
            for widget in self.canvas_frame.winfo_children():
                widget.destroy()
            
            # Create visualization
            self.visualization = SwarmVisualization(self.swarm, self.terrain_mapper)
            self.visualization.setup_3d_plot(terrain_data)
            
            # Embed matplotlib in tkinter
            canvas = tkagg.FigureCanvasTkAgg(self.visualization.fig, self.canvas_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            
            # Add toolbar
            toolbar = tkagg.NavigationToolbar2Tk(canvas, self.canvas_frame)
            toolbar.update()
            
            self.log_message("3D visualization initialized")
            
        except Exception as e:
            self.log_message(f"Error setting up visualization: {str(e)}")
    
    def start_mission(self):
        """Start the drone mission"""
        if self.simulation_running:
            return
            
        mission_type = self.mission_type.get()
        self.log_message(f"Starting {mission_type} mission...")
        
        # Configure mission parameters
        if mission_type == "Search and Rescue":
            for drone in self.swarm.drones:
                if not drone.is_leader:
                    drone.state = DroneState.FOLLOWING
                else:
                    drone.state = DroneState.LEADING
                    
        elif mission_type == "Area Mapping":
            for drone in self.swarm.drones:
                drone.state = DroneState.MAPPING
                
        # Start simulation
        self.simulation_running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        
        # Setup visualization if not already done
        if self.visualization is None:
            self.setup_visualization()
        
        # Start animation
        try:
            self.visualization.start_animation()
            self.log_message("Mission started successfully")
            
            # Start status update thread
            self.simulation_thread = threading.Thread(target=self.simulation_loop, daemon=True)
            self.simulation_thread.start()
            
        except Exception as e:
            self.log_message(f"Error starting mission: {str(e)}")
            self.stop_mission()
    
    def stop_mission(self):
        """Stop the drone mission"""
        self.simulation_running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        
        if self.visualization and self.visualization.animation:
            self.visualization.animation.event_source.stop()
            
        self.log_message("Mission stopped")
    
    def toggle_recording(self):
        """Toggle video recording"""
        if self.visualization:
            if not self.visualization.recording:
                self.visualization.recording = True
                self.record_button.config(text="Stop Recording")
                self.log_message("Started recording video...")
            else:
                self.visualization.recording = False
                self.record_button.config(text="Record Video")
                self.save_recorded_video()
    
    def save_recorded_video(self):
        """Save recorded video"""
        if self.visualization:
            try:
                filename = filedialog.asksaveasfilename(
                    defaultextension=".mp4",
                    filetypes=[("MP4 files", "*.mp4"), ("All files", "*.*")]
                )
                if filename:
                    self.visualization.save_animation_as_video(filename)
                    self.log_message(f"Video saved as {filename}")
            except Exception as e:
                self.log_message(f"Error saving video: {str(e)}")
    
    def simulation_loop(self):
        """Background simulation loop for status updates"""
        while self.simulation_running:
            try:
                self.update_drone_status()
                time.sleep(0.5)  # Update every 500ms
            except Exception as e:
                self.log_message(f"Simulation error: {str(e)}")
                break
    
    def run(self):
        """Start the GUI application"""
        self.log_message("Swarm Drone Control System initialized")
        self.log_message("Load terrain and start mission to begin")
        self.root.mainloop()

class MQTTSimulator:
    """
    Simulate MQTT communication between drones
    In real implementation, this would use paho-mqtt or similar library
    """
    def __init__(self):
        self.broker_host = "localhost"
        self.broker_port = 1883
        self.topics = {
            "position": "swarm/drone/{}/position",
            "status": "swarm/drone/{}/status",
            "command": "swarm/drone/{}/command",
            "formation": "swarm/formation",
            "mission": "swarm/mission"
        }
        self.message_queue = []
        
    def publish_position(self, drone_id: int, position: Position):
        """Simulate publishing drone position"""
        message = {
            "drone_id": drone_id,
            "timestamp": time.time(),
            "x": position.x,
            "y": position.y,
            "z": position.z
        }
        topic = self.topics["position"].format(drone_id)
        self.message_queue.append({"topic": topic, "payload": json.dumps(message)})
        
    def publish_status(self, drone_id: int, status: DroneState):
        """Simulate publishing drone status"""
        message = {
            "drone_id": drone_id,
            "timestamp": time.time(),
            "status": status.value
        }
        topic = self.topics["status"].format(drone_id)
        self.message_queue.append({"topic": topic, "payload": json.dumps(message)})
        
    def get_messages(self) -> List[Dict]:
        """Get all queued messages"""
        messages = self.message_queue.copy()
        self.message_queue.clear()
        return messages

class ESP32DroneInterface:
    """
    Interface class that would handle communication with actual ESP32 drones
    This simulates the interface you would use with real hardware
    """
    def __init__(self, drone_id: int):
        self.drone_id = drone_id
        self.mqtt_client = MQTTSimulator()
        self.last_command_time = time.time()
        
    def send_movement_command(self, target_position: Position, velocity: float):
        """Send movement command to ESP32 drone"""
        command = {
            "type": "move",
            "target_x": target_position.x,
            "target_y": target_position.y,
            "target_z": target_position.z,
            "velocity": velocity,
            "timestamp": time.time()
        }
        
        # In real implementation, this would send via MQTT or serial
        print(f"ESP32 Drone {self.drone_id}: Move to ({target_position.x:.2f}, {target_position.y:.2f}, {target_position.z:.2f})")
        
    def send_formation_command(self, formation_type: str, parameters: Dict):
        """Send formation flying command"""
        command = {
            "type": "formation",
            "formation_type": formation_type,
            "parameters": parameters,
            "timestamp": time.time()
        }
        
        print(f"ESP32 Drone {self.drone_id}: Formation command - {formation_type}")
    
    def get_sensor_data(self) -> Dict:
        """Get sensor data from ESP32 (simulated)"""
        return {
            "battery": np.random.uniform(20, 100),
            "gps_quality": np.random.randint(3, 10),
            "altitude": np.random.uniform(10, 50),
            "temperature": np.random.uniform(15, 35),
            "timestamp": time.time()
        }

class RealWorldIntegration:
    """
    Class to handle integration with real-world mapping services
    """
    def __init__(self):
        self.map_center = (40.7589, -73.9851)  # Default to NYC
        
    def create_mission_map(self, drone_positions: List[Position], mapped_areas: List[Tuple], 
                          save_path: str = "mission_map.html"):
        """Create an interactive map showing drone positions and mapped areas"""
        try:
            # Create folium map
            mission_map = folium.Map(
                location=self.map_center,
                zoom_start=15,
                tiles='OpenStreetMap'
            )
            
            # Add drone positions
            colors = ['red', 'blue', 'green', 'orange', 'purple']
            for i, pos in enumerate(drone_positions):
                # Convert drone coordinates to lat/lon (simplified conversion)
                lat = self.map_center[0] + pos.y / 111000
                lon = self.map_center[1] + pos.x / (111000 * np.cos(np.radians(self.map_center[0])))
                
                folium.Marker(
                    [lat, lon],
                    popup=f'Drone {i}',
                    icon=folium.Icon(color=colors[i % len(colors)], icon='plane')
                ).add_to(mission_map)
            
            # Add mapped areas as heat map
            if mapped_areas:
                heat_data = []
                for point in mapped_areas:
                    lat = self.map_center[0] + point[1] / 111000
                    lon = self.map_center[1] + point[0] / (111000 * np.cos(np.radians(self.map_center[0])))
                    heat_data.append([lat, lon])
                
                plugins.HeatMap(heat_data).add_to(mission_map)
            
            # Save map
            mission_map.save(save_path)
            print(f"Mission map saved as {save_path}")
            
        except Exception as e:
            print(f"Error creating mission map: {str(e)}")
    
    def export_mission_data(self, swarm: SwarmController, filename: str = "mission_data.json"):
        """Export mission data for analysis"""
        mission_data = {
            "timestamp": time.time(),
            "num_drones": len(swarm.drones),
            "drones": []
        }
        
        for drone in swarm.drones:
            drone_data = {
                "id": drone.id,
                "is_leader": drone.is_leader,
                "final_position": {
                    "x": drone.position.x,
                    "y": drone.position.y,
                    "z": drone.position.z
                },
                "state": drone.state.value,
                "mapped_points": len(drone.mapped_area)
            }
            mission_data["drones"].append(drone_data)
        
        with open(filename, 'w') as f:
            json.dump(mission_data, f, indent=2)
        
        print(f"Mission data exported to {filename}")

# Main application entry point
def main():
    """Main function to run the swarm drone system"""
    print("Initializing Swarm Drone System...")
    
    # Create and run the GUI application
    app = SwarmControlUI()
    
    try:
        app.run()
    except KeyboardInterrupt:
        print("\nShutting down system...")
    except Exception as e:
        print(f"System error: {str(e)}")

if __name__ == "__main__":
    main()