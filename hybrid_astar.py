import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# --- HYBRID A* CHASSIS PARAMETERS ---
L = 0.33                     # Wheelbase (meters)
V_ACTIONS = [0.5, -0.5]      # Forward and Reverse gears (m/s)
DT = 0.5                     # Shorter time step for finer control
MAX_STEER = math.radians(30) # 30 degrees
clearance = 0.20             # Safety buffer

# Steering angles: [-30, -15, 0, 15, 30]
steering_actions = np.linspace(-MAX_STEER, MAX_STEER, 5)

# --- COST PENALTIES (The secret sauce of Hybrid A*) ---
PENALTY_REVERSE = 2.0        # Multiplier to heavily penalize driving backward
PENALTY_STEER = 0.1          # Slight penalty for turning the wheel (prefers straight lines)
PENALTY_GEAR_SHIFT = 5.0     # Heavy penalty for switching between forward and reverse

class Node:
    def __init__(self, x, y, theta, g, parent, v, phi):
        self.x = x
        self.y = y
        self.theta = theta
        self.g = g
        self.parent = parent
        self.v = v      
        self.phi = phi  
        self.f = 0

    def __lt__(self, other):
        return self.f < other.f

def is_obstacle(x, y):
    """Algebraic Obstacle Map"""
    if x <= 0 or x >= 10 or y <= 0 or y >= 10: return True
    # Circle Obstacle
    if (x - 5)**2 + (y - 5)**2 <= (1.5 + clearance)**2: return True
    # Rectangle Obstacle (Extended to force a difficult parking maneuver)
    if (2 - clearance) <= x <= (3 + clearance) and (0 - clearance) <= y <= (7 + clearance): return True
    return False

def get_child_nodes(current):
    children = []
    
    # Try all combinations of Forward/Reverse and Steering Angles
    for v in V_ACTIONS:
        for phi in steering_actions:
            
            # Kinematics
            if abs(phi) < 1e-5:
                x_new = current.x + v * math.cos(current.theta) * DT
                y_new = current.y + v * math.sin(current.theta) * DT
                theta_new = current.theta
            else:
                R = L / math.tan(phi)
                w = v / R
                x_new = current.x + R * (math.sin(current.theta + w * DT) - math.sin(current.theta))
                y_new = current.y - R * (math.cos(current.theta + w * DT) - math.cos(current.theta))
                theta_new = (current.theta + w * DT) % (2 * math.pi)
                
            if not is_obstacle(x_new, y_new):
                # Calculate Hybrid A* Costs
                distance = abs(v * DT)
                
                # 1. Penalize reversing
                if v < 0: distance *= PENALTY_REVERSE
                
                # 2. Penalize steering (encourages straight driving)
                distance += abs(phi) * PENALTY_STEER
                
                # 3. Penalize shifting gears (Forward to Reverse)
                if current.v != 0 and current.v != v:
                    distance += PENALTY_GEAR_SHIFT
                
                cost = current.g + distance
                children.append(Node(x_new, y_new, theta_new, cost, current, v, phi))
                
    return children

def run_hybrid_astar(start_pose, goal_pose):
    print("Running Hybrid A* with Forward/Reverse Actions...")
    start_node = Node(start_pose[0], start_pose[1], start_pose[2], 0.0, None, 0.0, 0.0)
    
    open_list = []
    heapq.heappush(open_list, start_node)
    visited = {}
    explored_points = []
    
    while open_list:
        current = heapq.heappop(open_list)
        explored_points.append((current.x, current.y))
        
        # HYBRID GOAL CHECK: Must be within 0.5m AND facing the correct angle (within 20 degrees)
        dist_to_goal = math.hypot(current.x - goal_pose[0], current.y - goal_pose[1])
        angle_to_goal = abs(current.theta - goal_pose[2])
        if angle_to_goal > math.pi: angle_to_goal = 2*math.pi - angle_to_goal # Normalize
        
        if dist_to_goal < 0.5 and angle_to_goal < math.radians(20):
            print("Goal Reached and Aligned!")
            path = []
            while current.parent is not None:
                path.append(current)
                current = current.parent
            return path[::-1], explored_points
            
        # Grid hashing
        idx = (int(round(current.x/0.1)), int(round(current.y/0.1)), int(round(current.theta/(math.pi/18))))
        if idx in visited and visited[idx] <= current.g:
            continue
        visited[idx] = current.g
        
        for child in get_child_nodes(current):
            # Hybrid Heuristic: Euclidean distance + Angle alignment penalty
            h = math.hypot(child.x - goal_pose[0], child.y - goal_pose[1])
            
            # Add a slight heuristic pull towards the final goal angle
            angle_diff = abs(child.theta - goal_pose[2])
            if angle_diff > math.pi: angle_diff = 2*math.pi - angle_diff
            h += angle_diff * 0.5 
            
            child.f = child.g + h
            heapq.heappush(open_list, child)
            
    print("Path not found!")
    return None, explored_points

def plot_hybrid_path(path, explored, start, goal):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, 10); ax.set_ylim(0, 10); ax.set_aspect('equal')
    
    # Draw obstacles
    ax.add_patch(patches.Circle((5, 5), 1.5, color='red', alpha=0.5))
    ax.add_patch(patches.Rectangle((2, 0), 1, 7, color='red', alpha=0.5))
    
    if explored:
        ex, ey = zip(*explored[::10]) 
        ax.scatter(ex, ey, s=1, color='lightgray', label="Explored Space")
        
    if path:
        # We split the plot into blue (forward) and orange (reverse) segments
        prev_x, prev_y = start[0], start[1]
        for node in path:
            color = 'blue' if node.v > 0 else 'orange'
            ax.plot([prev_x, node.x], [prev_y, node.y], color=color, linewidth=2.5)
            prev_x, prev_y = node.x, node.y
            
        # Add legend markers manually
        ax.plot([], [], color='blue', linewidth=2, label="Forward Gear")
        ax.plot([], [], color='orange', linewidth=2, label="Reverse Gear")

    # Plot Start and Goal with directional arrows to show orientation
    ax.arrow(start[0], start[1], 0.4*math.cos(start[2]), 0.4*math.sin(start[2]), head_width=0.2, color='green', label="Start Pose")
    ax.arrow(goal[0], goal[1], 0.4*math.cos(goal[2]), 0.4*math.sin(goal[2]), head_width=0.2, color='purple', label="Goal Pose")
    
    plt.legend(); plt.grid(True)
    plt.title("Hybrid A* (Forward/Reverse & Cost Penalties)")
    plt.show()

if __name__ == '__main__':
    # Start at bottom left, facing UP
    START = (1.0, 1.0, math.radians(90))
    
    # To test reversing, we place the goal right next to the wall, 
    # facing backwards (270 degrees). This forces a parallel park maneuver!
    GOAL = (1.5, 7.5, math.radians(270))
    
    path, explored = run_hybrid_astar(START, GOAL)
    
    if path:
        print("\n--- Autonomous Driving Commands ---")
        for node in path:
            gear = "Forward" if node.v > 0 else "Reverse"
            print(f"[{gear}] Speed: {node.v:>4.1f} | Steer: {math.degrees(node.phi):>5.1f} deg | Pose: ({node.x:.1f}, {node.y:.1f})")
            
    plot_hybrid_path(path, explored, START, GOAL)