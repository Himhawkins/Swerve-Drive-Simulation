import pygame
import sys
import math
import numpy as np

# --- Custom Modules ---
import robot_config as cfg
import sim_grid as simulation_grid
from planner import Planner
import d2 as  dynamics_robot 

class HolonomicPurePursuit:
    def __init__(self, path, cost_map, lookahead_dist=50.0, max_speed=4.0, goal_tolerance=5):
        self.lookahead_dist = lookahead_dist
        self.max_speed = max_speed
        self.goal_tolerance = goal_tolerance
        self.last_index = 0
        self.path = path
        self.cost_map = cost_map
        self.dynamic = True
        self.prev_d2 = (0, 0)
        self.prev_vx=0
        self.prev_vy=0

    def reset(self):
        self.last_index = 0

    def _distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def get_command(self, robot_pos):
        
        path = self.path
        dist_to_goal = self._distance(robot_pos, path[-1])
        if dist_to_goal < self.goal_tolerance:
            return (0.0, 0.0), True, self.path[self.last_index]

        # --- Pure Pursuit Target Selection (Same as before) ---
        target_point = None
        target_index = self.last_index
        for i in range(self.last_index, min(self.last_index + 200, len(path))):
            d = self._distance(robot_pos, path[i])
            if d > self.lookahead_dist:
                target_point = path[i]
                target_index = i
                break
        if target_point is None:
            target_point = path[-1]
            target_index = len(path) - 1
        self.last_index = target_index

        # --- Base Velocity Calculation ---
        dx = target_point[0] - robot_pos[0]
        dy = target_point[1] - robot_pos[1]
        angle = math.atan2(dy, dx)

        current_speed = self.max_speed
        if dist_to_goal < self.lookahead_dist:
            current_speed = self.max_speed * (dist_to_goal / self.lookahead_dist)
            current_speed = max(current_speed, 0.1) 

        vx = current_speed * math.cos(angle)
        vy = current_speed * math.sin(angle)

        # --- Improved Dynamic Avoidance ---
        if self.dynamic and dist_to_goal > self.lookahead_dist:
            # Pass the INTENDED velocity (vx, vy) to see if it leads to danger
            obs_vx, obs_vy = self.calculate_avoidance_velocity(robot_pos, (vx, vy), self.cost_map)
            
            # Add avoidance vector to pure pursuit vector
            vx += obs_vx
            vy += obs_vy
            
            # Re-normalize to max_speed to ensure we don't exceed physical limits
            total_speed = math.hypot(vx, vy)
            if total_speed > self.max_speed:
                scale = self.max_speed / total_speed
                vx *= scale
                vy *= scale

        alpha = 0.2
        
        # self.prev_vx and self.prev_vy must be stored in __init__
        smoothed_vx = (alpha * vx) + ((1 - alpha) * self.prev_vx)
        smoothed_vy = (alpha * vy) + ((1 - alpha) * self.prev_vy)
        
        self.prev_vx = smoothed_vx
        self.prev_vy = smoothed_vy

        return (vx, vy), False, target_point

    def calculate_avoidance_velocity(self, robot_pos, velocity, costmap, gain_obs=2.0, lookahead_time=1):
        # 1. Calculate the Lookahead Point (Where will I be in 0.8 seconds?)
        # We use the current command velocity to project forward
        vx, vy = velocity
        
        # Clamp lookahead to avoid checking way off the map if speed is huge
        speed = math.hypot(vx, vy)
        if speed < 0.1: return 0.0, 0.0 # Not moving, no avoidance needed
        
        # Check a point ahead of the robot
        scan_x = robot_pos[0] + vx * lookahead_time
        scan_y = robot_pos[1] + vy * lookahead_time
        
        x = int(scan_x)
        y = int(scan_y)

        # Bounds check
        rows, cols = costmap.shape
        if x < 0 or x >= rows or y < 0 or y >= cols:
            return 0.0, 0.0

        # 2. Check Cost at the LOOKAHEAD point, not current point
        center_cost = costmap[x, y]
        
        # If the future position is safe (low cost), don't intervene
        # You might need to tune this threshold based on your map values (e.g., > 0 or > 50)
        if center_cost <= 0: 
            return 0.0, 0.0

        # 3. Calculate Gradient (Direction to safety)
        half_w = int((cfg.PIXELS_PER_METER * cfg.CHASSIS_WIDTH_M) / 2)
        r = max(1, int(half_w)) # Ensure radius is at least 1

        x_right  = min(rows - 1, x + r)
        x_left   = max(0,        x - r)
        y_bottom = min(cols - 1, y + r)
        y_top    = max(0,        y - r)

        # Vectors point FROM obstacle TO safety (negative gradient)
        # If right neighbor has higher cost, we push LEFT (-1, 0)
        push_x = 0.0
        push_y = 0.0
        
        if costmap[x_right, y] > center_cost:  push_x -= (costmap[x_right, y] - center_cost)
        if costmap[x_left, y] > center_cost:   push_x += (costmap[x_left, y] - center_cost)
        if costmap[x, y_bottom] > center_cost: push_y -= (costmap[x, y_bottom] - center_cost)
        if costmap[x, y_top] > center_cost:    push_y += (costmap[x, y_top] - center_cost)

        # If we are in a flat high-cost zone (no gradient), push opposite to velocity
        if push_x == 0 and push_y == 0:
            push_x = -vx
            push_y = -vy

        # Normalize the avoidance vector
        mag = math.hypot(push_x, push_y)
        if mag > 0:
            push_x = (push_x / mag) * gain_obs
            push_y = (push_y / mag) * gain_obs

        # 4. Swerve Heuristic (Orthogonal perturbation)
        # If the force is directly opposing motion (deadlock), add a slight sideways nudge
        # to force the robot to pick a side (left or right)
        dot_prod = (vx * push_x) + (vy * push_y)
        if dot_prod < -0.8 * speed * gain_obs: # Vectors are opposing
            # Add a perpendicular component
            push_x += -0.5 * push_y
            push_y +=  0.5 * push_x
        if abs(push_x)+abs(push_y)<0.7:
            push_x=0
            push_y=0
        return push_x, push_y
# --- Main Execution ---
if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((cfg.GRID_WINDOW_WIDTH, cfg.GRID_WINDOW_HEIGHT))
    pygame.display.set_caption("Integrated: Click to Add Walls")
    clock = pygame.time.Clock()

    print("Generating Maze...")
    maze = simulation_grid.MazeGenerator(cfg.GRID_COLS, cfg.GRID_ROWS)
    start_px, goal_px = simulation_grid.generate_start_goal(cfg.GRID_COLS, cfg.GRID_ROWS)

    print("Planning Path...")
    planner = Planner(maze.walls, cfg.GRID_WINDOW_WIDTH, cfg.GRID_WINDOW_HEIGHT)
    path_points = planner.get_path(start_px, goal_px)
    cost_map = planner.cost_map 
    
    controller = HolonomicPurePursuit(path_points, cost_map)

    # Physics Robot Setup
    start_x_m = start_px[0] / cfg.PIXELS_PER_METER
    start_y_m = start_px[1] / cfg.PIXELS_PER_METER
    robot = dynamics_robot.DynamicRobot(start_x_m, start_y_m)

    running = True
    while running:
        dt_ms = clock.tick(60)
        dt = dt_ms / 1000.0
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # --- KEYBOARD CONTROLS ---
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    # Full Reset
                    print("Resetting Maze...")
                    maze = simulation_grid.MazeGenerator(cfg.GRID_COLS, cfg.GRID_ROWS)
                    start_px, goal_px = simulation_grid.generate_start_goal(cfg.GRID_COLS, cfg.GRID_ROWS)
                    planner = Planner(maze.walls, cfg.GRID_WINDOW_WIDTH, cfg.GRID_WINDOW_HEIGHT)
                    path_points = planner.get_path(start_px, goal_px)
                    cost_map = planner.cost_map
                    controller = HolonomicPurePursuit(path_points, cost_map)
                    robot.x, robot.y = start_px[0] / cfg.PIXELS_PER_METER, start_px[1] / cfg.PIXELS_PER_METER
                    robot.vx, robot.vy, robot.omega = 0, 0, 0
            
            # --- MOUSE CLICK: ADD DYNAMIC OBSTACLE ---
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: # Left Click
                    mx, my = pygame.mouse.get_pos()
                    print(f"Adding wall at ({mx}, {my})...")

                    # 1. Create a new wall (Small Rect to approximate a circle point)
                    # We use a Rect because Planner logic relies on Rects for drawing the mask
                    wall_size = 20
                    new_wall = pygame.Rect(mx - wall_size//2, my - wall_size//2, wall_size, wall_size)
                    maze.walls.append(new_wall)

                    # 2. Update Planner Cost Map (Heavy Calculation)
                    # This recalculates the entire potential field including the new wall
                    planner.cost_map = planner.create_potential_field(maze.walls)

                    # 3. Update Planner Visualization (The Heatmap)
                    planner.vis_surface = planner.create_heatmap_surface()

                    # 4. Update Controller with new Cost Map
                    # This ensures the robot 'sees' the new wall immediately
                    controller.cost_map = planner.cost_map
                    
                    print("Cost map updated.")

        # --- Control Logic ---
        robot_pixel_pos = (robot.x * cfg.PIXELS_PER_METER, robot.y * cfg.PIXELS_PER_METER)
        V, stop, target_point = controller.get_command(robot_pixel_pos)

        if stop:
            V = (0, 0)
            # print("Goal Reached!")

        robot.set_velocity_control(V[0], V[1], 0, dt)
        robot.update_physics(dt)

        # --- Drawing ---
        screen.fill(cfg.COLOR_BG)
        screen.blit(planner.vis_surface, (0, 0)) # Draw Heatmap
        
        for wall in maze.walls:
            pygame.draw.rect(screen, cfg.COLOR_WALL, wall)

        if len(path_points) > 1:
            pygame.draw.lines(screen, cfg.COLOR_PATH, False, path_points, 3)

        pygame.draw.circle(screen, cfg.COLOR_GOAL, goal_px, 10)
        robot.draw(screen)
        pygame.draw.circle(screen, (0, 255, 255), target_point, 5)

        pygame.display.flip()

    pygame.quit()
    sys.exit()