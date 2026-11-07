import heapq
import math
import matplotlib.pyplot as plt

# --- 1. MAP GRID & A* ALGORITHM ---
# 0 = Free Space, 1 = Obstacle
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [1, 0, 0, 0, 0]
]

HEADING_DELTAS = {
    0: (-1, 0),  # North
    1: (0, 1),   # East
    2: (1, 0),   # South
    3: (0, -1)   # West
}

def astar(start_x, start_y, start_heading, goal_x, goal_y):
    open_list = [(0, (start_x, start_y, start_heading), [], [(start_x, start_y)])]
    visited = set()

    while open_list:
        cost, (x, y, heading), actions, coord_path = heapq.heappop(open_list)
        
        if (x, y) == (goal_x, goal_y):
            return actions, coord_path
            
        if (x, y, heading) in visited: continue
        visited.add((x, y, heading))

        # FORWARD
        dx, dy = HEADING_DELTAS[heading]
        nx, ny = x + dx, y + dy
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 0:
            h = math.hypot(nx - goal_x, ny - goal_y)
            heapq.heappush(open_list, (cost + 1 + h, (nx, ny, heading), actions + ["FORWARD"], coord_path + [(nx, ny)]))

        # TURN RIGHT
        new_heading_R = (heading + 1) % 4
        h = math.hypot(x - goal_x, y - goal_y)
        heapq.heappush(open_list, (cost + 1 + h, (x, y, new_heading_R), actions + ["TURN_RIGHT"], coord_path))

        # TURN LEFT
        new_heading_L = (heading - 1) % 4
        h = math.hypot(x - goal_x, y - goal_y)
        heapq.heappush(open_list, (cost + 1 + h, (x, y, new_heading_L), actions + ["TURN_LEFT"], coord_path))

    return None, None

# --- 2. EXECUTE A* ---
START_X, START_Y = 0, 0
INITIAL_HEADING = 2 # 2 = South
GOAL_X, GOAL_Y = 4, 4

path_actions, path_coords = astar(START_X, START_Y, INITIAL_HEADING, GOAL_X, GOAL_Y)

if path_actions:
    print("\n=== SUCCESS: COPY AND PASTE THIS INTO YOUR ARDUINO setup() FUNCTION ===")
    for action in path_actions:
        if action == "FORWARD":
            print("  {220, -240, 1000},   // FORWARD")
        elif action == "TURN_RIGHT":
            print("  {220, 220, 490},  // TURN_RIGHT")
        elif action == "TURN_LEFT":
            print("  {-220, -220, 565},  // TURN_LEFT")
        elif action == "BACKWARD":
            print("  {-220, 220, 1000}, // BACKWARD")
            
    print("  stopMotors();")
    print("=========================================================================\n")
    
    # --- VISUAL SIMULATION ---
    plt.figure(figsize=(6, 6))
    plt.imshow(grid, cmap='binary', origin='upper')
    plt.plot(START_Y, START_X, 'go', markersize=12, label='Start (0,0)')
    plt.plot(GOAL_Y, GOAL_X, 'ro', markersize=12, label='Goal (4,4)')
    plot_y = [c[0] for c in path_coords]
    plot_x = [c[1] for c in path_coords]
    plt.plot(plot_x, plot_y, color='blue', linestyle='--', linewidth=2, label='Planned Path')
    plt.grid(True, which='both', color='gray', linestyle='-', linewidth=0.5)
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))
    plt.title("Robot Car Pathfinding Simulation")
    plt.show()
else:
    print("No valid path found.")