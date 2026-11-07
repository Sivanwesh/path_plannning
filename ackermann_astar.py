import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# --- ACKERMANN CHASSIS PARAMETERS ---
L = 0.33               # Wheelbase in meters (distance between front and rear axles)
V_CONST = 0.5          # Constant driving speed in m/s
DT = 1.0               # Time step for numerical integration
MAX_STEER = math.radians(30) # Max steering angle (30 degrees)
clearance = 0.20       # Safety buffer (made slightly larger for a car-like body)

# 5 discrete steering actions: [-30, -15, 0, 15, 30] degrees
steering_actions = np.linspace(-MAX_STEER, MAX_STEER, 5)

class Node:
    def __init__(self, x, y, theta, g, parent, v, phi):
        self.x = x
        self.y = y
        self.theta = theta
        self.g = g
        self.parent = parent
        self.v = v      # Linear velocity used
        self.phi = phi  # Steering angle used (instead of wheel RPMs)
        self.h = 0
        self.f = 0

    def __lt__(self, other):
        return self.f < other.f

def is_obstacle(x, y):
    """Algebraic half-planes for obstacle collision."""
    if x <= 0 or x >= 10 or y <= 0 or y >= 10:
        return True
    # Circle Obstacle
    if (x - 5)**2 + (y - 5)**2 <= (1.5 + clearance)**2:
        return True
    # Rectangle Obstacle
    if (2 - clearance) <= x <= (3 + clearance) and (0 - clearance) <= y <= (6 + clearance):
        return True
    return False

def get_child_nodes(current):
    """Ackermann Kinematics (Bicycle Model)"""
    children = []
    for phi in steering_actions:
        
        # If driving straight (avoid division by zero)
        if abs(phi) < 1e-5: 
            x_new = current.x + V_CONST * math.cos(current.theta) * DT
            y_new = current.y + V_CONST * math.sin(current.theta) * DT
            theta_new = current.theta
            
        # If steering/turning
        else: 
            R = L / math.tan(phi) # Turning radius
            w = V_CONST / R       # Angular velocity
            
            # Exact circular arc integration
            x_new = current.x + R * (math.sin(current.theta + w * DT) - math.sin(current.theta))
            y_new = current.y - R * (math.cos(current.theta + w * DT) - math.cos(current.theta))
            theta_new = (current.theta + w * DT) % (2 * math.pi)
            
        if not is_obstacle(x_new, y_new):
            cost = current.g + (V_CONST * DT)
            children.append(Node(x_new, y_new, theta_new, cost, current, V_CONST, phi))
            
    return children

def astar(start_pose, goal_pose):
    print("Starting Ackermann A* Planning... (Cars take wider turns, give it a second!)")
    start_node = Node(start_pose[0], start_pose[1], start_pose[2], 0.0, None, 0.0, 0.0)
    start_node.h = math.hypot(start_node.x - goal_pose[0], start_node.y - goal_pose[1])
    start_node.f = start_node.g + start_node.h
    
    open_list = []
    heapq.heappush(open_list, start_node)
    
    visited = {}
    explored_points = []
    
    while open_list:
        current = heapq.heappop(open_list)
        explored_points.append((current.x, current.y))
        
        # INCREASED THRESHOLD: Cars cannot pivot on a dime, so hitting an exact (x,y) 
        # point is harder. We relax the goal radius to 0.5 meters.
        if math.hypot(current.x - goal_pose[0], current.y - goal_pose[1]) < 0.5:
            print("Goal Reached!")
            path = []
            while current.parent is not None:
                path.append(current)
                current = current.parent
            return path[::-1], explored_points
            
        # Grid hashing: 0.1m spatial, ~10 degrees angular (finer angular bins for car curves)
        idx = (int(round(current.x/0.1)), int(round(current.y/0.1)), int(round(current.theta/(math.pi/18))))
        if idx in visited and visited[idx] <= current.g:
            continue
        visited[idx] = current.g
        
        for child in get_child_nodes(current):
            child.h = math.hypot(child.x - goal_pose[0], child.y - goal_pose[1])
            child.f = child.g + child.h
            heapq.heappush(open_list, child)
            
    print("Path not found!")
    return None, explored_points

def plot_path(path, explored, start, goal):
    fig, ax = plt.subplots(figsize=(8, 8))
    
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect('equal') # <--- Forces a perfect circle!
    
    # Draw obstacles
    ax.add_patch(patches.Circle((5, 5), 1.5, color='red', alpha=0.5))
    ax.add_patch(patches.Rectangle((2, 0), 1, 6, color='red', alpha=0.5))
    
    # Plot search tree
    if explored:
        ex, ey = zip(*explored[::5]) 
        ax.scatter(ex, ey, s=2, color='lightgray', label="Explored Nodes")
        
    # Plot final trajectory
    if path:
        px = [start[0]] + [n.x for n in path]
        py = [start[1]] + [n.y for n in path]
        ax.plot(px, py, color='blue', linewidth=2, label="Ackermann Path")
        
    ax.plot(start[0], start[1], 'go', ms=8, label="Start")
    ax.plot(goal[0], goal[1], 'bo', ms=8, label="Goal")
    plt.legend()
    plt.grid(True)
    plt.title("Ackermann A* Path Planning (Bicycle Kinematics)")
    plt.show()

if __name__ == '__main__':
    # Start at (1,1) facing North (90 degrees)
    START = (1.0, 1.0, math.radians(90))
    GOAL = (7.5, 7.5, math.radians(270))
    
    path, explored = astar(START, GOAL)
    
    if path:
        print("\n--- Ackermann Steering Commands ---")
        for node in path:
            print(f"Speed: {node.v:.1f} m/s | Steer Angle: {math.degrees(node.phi):4.1f} deg  ->  Pose: ({node.x:.2f}, {node.y:.2f})")
            
    plot_path(path, explored, START, GOAL)