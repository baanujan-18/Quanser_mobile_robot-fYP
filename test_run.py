import time
import math
from qbot_sim import QBot2Sim

# Define your maze waypoints [x, z] based on the Gazebo map
# The valid corners of the new open maze are: (-17, -17), (17, -17), (17, 17), (-17, 17)
MAZE_WAYPOINTS = [
    [17.0, -17.0],  # Point 1 (Server Room)
    [17.0, 17.0],   # Point 2 (Main Lobby)
    [-17.0, 17.0],  # Point 3 (High Voltage Lab)
    [-17.0, -17.0]  # Back to start
]

def run_smart_patrol():
    print("Connecting to QUANSER QBot 2 Simulation...")
    robot = QBot2Sim()
    robot.reset()
    time.sleep(2)
    
    robot.log("QUANSER QBot 2 SMART PYTHON PATROL INITIATED")
    
    current_wp_idx = 0
    
    try:
        while True:
            # 1. Get current robot state natively streaming from simulation
            pos = robot.get_position() # Returns [x, y, z]
            target = MAZE_WAYPOINTS[current_wp_idx]
            
            # 2. Check for Obstacles (Collision Avoidance using real-time LiDAR!)
            front_dist = robot.get_lidar_front() 
            # I have upgraded `move_to` to natively use A* so it won't crash into walls!
            # We reduce the collision trigger threshold to 0.3m as a pure failsafe.
            if front_dist < 0.3:
                robot.log(f"CRITICAL: Obstacle Detected at {front_dist:.2f}m. Emergency Stop!")
                robot.stop()
                # Instead of a blind pivot trap, we tell the engine to recalculate its A* path!
                robot.move_to(target[0], target[1])
                time.sleep(0.5)
                continue

            # 3. Calculate distance to next waypoint
            # Three.js 3D space uses Z-axis for forward/backward instead of Y
            dist_to_target = math.sqrt((target[0] - pos[0])**2 + (target[1] - pos[2])**2)
            
            # 4. Navigation Logic
            if dist_to_target > 0.4:
                # Command JS physics engine to seamlessly navigate to point using A*
                robot.move_to(target[0], target[1])
            else:
                robot.log(f"Reached Waypoint {current_wp_idx}!")
                current_wp_idx = (current_wp_idx + 1) % len(MAZE_WAYPOINTS)
                robot.stop()
                time.sleep(1.0) # Pause at waypoint
            
            time.sleep(0.1) # Frequency of the control loop

    except KeyboardInterrupt:
        robot.stand()
        robot.log("Patrol Terminated by User.")
        print("Mission Stopped.")

if __name__ == "__main__":
    run_smart_patrol()
