import math
import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# --- OPTIMIZED CONTINUOUS A* ROBOT ---
step_size = 0.4    # Slightly larger step size for faster traversal
clearance = 0.15   # Robot radius + buffer

# 16 directions offer smooth angles while keeping performance high
num_directions = 16
directions = []
for i in range(num_directions):
    angle = i * (2 * math.pi / num_directions)
    directions.append((math.cos(angle), math.sin(angle)))

class Node:
    def __init__(self, x, y, g, parent):
        self.x = x
        self.y = y
        self.g = g
        self.parent = parent
        self.h = 0
        self.f = 0

    def __lt__(self, other):
        return self.f < other.f

def is_obstacle(x, y):
    """Algebraic half-planes matching the map bounds and obstacles."""
    if x <= 0 or x >= 10 or y <= 0 or y >= 10:
        return True
    # Circle Obstacle
    if (x - 5)**2 + (y - 5)**2 <= (1.5 + clearance)**2:
        return True
    # Rectangle Obstacle
    if (2 - clearance) <= x <= (3 + clearance) and (0 - clearance) <= y <= (6 + clearance):
        return True
    return False

def astar(start_pose, goal_pose):
    print("Running A* Pathfinding...")
    start_node = Node(start_pose[0], start_pose[1], 0.0, None)
    start_node.h = math.hypot(start_node.x - goal_pose[0], start_node.y - goal_pose[1])
    start_node.f = start_node.g + start_node.h
    
    open_list = []
    heapq.heappush(open_list, start_node)
    
    visited = set()
    explored_points = []
    
    while open_list:
        current = heapq.heappop(open_list)
        explored_points.append((current.x, current.y))
        
        # Goal check threshold
        if math.hypot(current.x - goal_pose[0], current.y - goal_pose[1]) < 0.5:
            print("Goal Reached Successfully!")
            path = []
            while current.parent is not None:
                path.append(current)
                current = current.parent
            return path[::-1], explored_points
            
        # Grid quantization for visited set (0.1m resolution)
        idx = (int(round(current.x / 0.1)), int(round(current.y / 0.1)))
        if idx in visited:
            continue
        visited.add(idx)
        
        for dx, dy in directions:
            x_new = current.x + step_size * dx
            y_new = current.y + step_size * dy
            
            if not is_obstacle(x_new, y_new):
                cost = current.g + step_size
                child = Node(x_new, y_new, cost, current)
                child.h = math.hypot(child.x - goal_pose[0], child.y - goal_pose[1])
                child.f = child.g + child.h
                heapq.heappush(open_list, child)
            
    print("Path not found!")
    return None, explored_points

def smooth_path(path):
    """Post-processing shortcut to merge linear steps into clean straight lines."""
    if not path:
        return []
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i:
            start_node = path[i]
            end_node = path[j]
            
            collision = False
            steps = 15
            for s in range(1, steps):
                t = s / float(steps)
                cx = start_node.x * (1 - t) + end_node.x * t
                cy = start_node.y * (1 - t) + end_node.y * t
                if is_obstacle(cx, cy):
                    collision = True
                    break
            if not collision:
                break
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed

def plot_path(smoothed_path, explored, start, goal):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect('equal')
    
    # Draw obstacles
    ax.add_patch(patches.Circle((5, 5), 1.5, color='red', alpha=0.5))
    ax.add_patch(patches.Rectangle((2, 0), 1, 6, color='red', alpha=0.5))
    
    if explored:
        ex, ey = zip(*explored[::2]) 
        ax.scatter(ex, ey, s=2, color='lightgray', label="Explored Nodes")
        
    if smoothed_path:
        px = [start[0]] + [n.x for n in smoothed_path]
        py = [start[1]] + [n.y for n in smoothed_path]
        ax.plot(px, py, color='blue', linewidth=2.5, label="Optimized Straight-Line Path")
        
    ax.plot(start[0], start[1], 'go', ms=8, label="Start")
    ax.plot(goal[0], goal[1], 'bo', ms=8, label="Goal")
    plt.legend()
    plt.grid(True)
    plt.title("Fast Continuous Slope Pathfinding")
    plt.show()

if __name__ == '__main__':
    START = (1.0, 1.0)
    GOAL = (8.0, 8.0)
    
    path, explored = astar(START, GOAL)
    
    if path:
        smooth = smooth_path(path)
        print("\n--- Clean Straight-Line Waypoints ---")
        for node in smooth:
            print(f"Waypoint: ({node.x:.2f}, {node.y:.2f})")
        plot_path(smooth, explored, START, GOAL)