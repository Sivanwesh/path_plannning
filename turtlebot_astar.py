import math
import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# --- TURTLEBOT3 BURGER PARAMETERS (From GitHub) ---
r = 0.033  # Wheel radius in meters
L = 0.160  # Wheelbase in meters
dt = 1.0   # Time step for numerical integration
clearance = 0.15 # Robot radius + buffer

# Allowed wheel RPMs (Left, Right) - Exactly as defined in the repo
actions = [(50, 50), (100, 100), (50, 100), (100, 50), (0, 50), (50, 0), (0, 100), (100, 0)]

class Node:
    def __init__(self, x, y, theta, g, parent, ul, ur):
        self.x = x
        self.y = y
        self.theta = theta
        self.g = g
        self.parent = parent
        self.ul = ul  # Left wheel RPM used
        self.ur = ur  # Right wheel RPM used
        # For A* heuristic
        self.h = 0
        self.f = 0

    def __lt__(self, other):
        return self.f < other.f

def is_obstacle(x, y):
    """Algebraic half-planes matching the Github Gazebo world."""
    # Boundary limits (10x10 map)
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
    """Differential Drive Kinematics (Generating arcs from RPMs)"""
    children = []
    for ul, ur in actions:
        # Convert RPM to Rad/s
        wl = (2 * math.pi * ul) / 60.0
        wr = (2 * math.pi * ur) / 60.0
        
        # Calculate linear (v) and angular (w) velocities
        v = (r / 2.0) * (wl + wr)
        w = (r / L) * (wr - wl)
        
        # Integrate to find new pose over time dt
        x_new = current.x + v * math.cos(current.theta) * dt
        y_new = current.y + v * math.sin(current.theta) * dt
        theta_new = (current.theta + w * dt) % (2 * math.pi)
        
        # Check collisions
        if not is_obstacle(x_new, y_new):
            cost = current.g + (v * dt)
            children.append(Node(x_new, y_new, theta_new, cost, current, ul, ur))
            
    return children

def astar(start_pose, goal_pose):
    print("Starting TurtleBot A* Planning...")
    start_node = Node(start_pose[0], start_pose[1], start_pose[2], 0.0, None, 0, 0)
    start_node.h = math.hypot(start_node.x - goal_pose[0], start_node.y - goal_pose[1])
    start_node.f = start_node.g + start_node.h
    
    open_list = []
    heapq.heappush(open_list, start_node)
    
    # 3D Visited Matrix to handle continuous floats (x, y, theta)
    visited = {}
    explored_points = []
    
    while open_list:
        current = heapq.heappop(open_list)
        explored_points.append((current.x, current.y))
        
        # Goal check threshold (0.2 meters)
        if math.hypot(current.x - goal_pose[0], current.y - goal_pose[1]) < 0.2:
            print("Goal Reached!")
            path = []
            while current.parent is not None:
                path.append(current)
                current = current.parent
            return path[::-1], explored_points
            
        # Grid hashing for visited check (Spatial 0.1m, Angular ~30 deg)
        idx = (int(round(current.x/0.1)), int(round(current.y/0.1)), int(round(current.theta/(math.pi/6))))
        if idx in visited and visited[idx] <= current.g:
            continue
        visited[idx] = current.g
        
        # Generate valid wheel movements
        for child in get_child_nodes(current):
            # h(n) = Euclidean distance
            child.h = math.hypot(child.x - goal_pose[0], child.y - goal_pose[1])
            child.f = child.g + child.h
            heapq.heappush(open_list, child)
            
    print("Path not found!")
    return None, explored_points

def plot_path(path, explored, start, goal):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect('equal')  # <--- ADD THIS LINE
    
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
        ax.plot(px, py, color='blue', linewidth=2, label="TurtleBot Path")
        
    ax.plot(start[0], start[1], 'go', ms=8, label="Start")
    ax.plot(goal[0], goal[1], 'bo', ms=8, label="Goal")
    plt.legend()
    plt.grid(True)
    plt.title("GitHub TurtleBot A* (Differential Drive)")
    plt.show()

if __name__ == '__main__':
    # Start at (1,1) facing North (90 degrees)
    START = (1.0, 1.0, math.radians(90))
    GOAL = (8.0, 8.0)
    
    path, explored = astar(START, GOAL)
    
    if path:
        print("\n--- TurtleBot Wheel Commands (RPM_Left, RPM_Right) ---")
        for node in path:
            print(f"RPM_L: {node.ul:3} | RPM_R: {node.ur:3}  ->  Pose: ({node.x:.2f}, {node.y:.2f})")
            
    plot_path(path, explored, START, GOAL)