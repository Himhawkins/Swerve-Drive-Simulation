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

    def reset(self):
        self.last_index = 0

    def _distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def get_command(self, robot_pos):
        path = self.path
        dist_to_goal = self._distance(robot_pos, path[-1])
        if dist_to_goal < self.goal_tolerance:
            return (0.0, 0.0), True, self.path[self.last_index]

        target_point = None
        target_index = self.last_index

        # Limit search horizon for performance
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

        dx = target_point[0] - robot_pos[0]
        dy = target_point[1] - robot_pos[1]
        angle = math.atan2(dy, dx)

        current_speed = self.max_speed
        if dist_to_goal < self.lookahead_dist:
            current_speed = self.max_speed * (dist_to_goal / self.lookahead_dist)
            current_speed = max(current_speed, 0.1) 

        vx = current_speed * math.cos(angle)
        vy = current_speed * math.sin(angle)

        # Dynamic Avoidance using the UPDATED cost_map
        if self.dynamic and target_index < len(path) - 1:
            obs_vx, obs_vy = self.calculate_avoidance_velocity(robot_pos, target_point, self.cost_map)
            vx += obs_vx
            vy += obs_vy
            
            vx = max(-self.max_speed, min(self.max_speed, vx))
            vy = max(-self.max_speed, min(self.max_speed, vy))

        return (vx, vy), False, target_point
    
    def calculate_avoidance_velocity(self, robot_pos, target_point, costmap, gain_obs=0.7, d_gain=-0.08):
        x, y = robot_pos
        x = int(x)
        y = int(y)

        # Basic bounds check to prevent crash if robot leaves map
        rows, cols = costmap.shape
        if x < 0 or x >= rows or y < 0 or y >= cols:
            return 0.0, 0.0

        half_w = int((cfg.PIXELS_PER_METER * cfg.CHASSIS_WIDTH_M) / 2)
        center_cost = costmap[x, y]
        r = int(half_w)

        x_right  = min(rows - 1, x + r)
        x_left   = max(0,        x - r)
        y_bottom = min(cols - 1, y + r)
        y_top    = max(0,        y - r)

        push_from_right  = np.array([-1, 0]) * (costmap[x_right, y] - center_cost)
        push_from_left   = np.array([1, 0]) * (costmap[x_left, y] - center_cost)
        push_from_bottom = np.array([0, -1]) * (costmap[x, y_bottom] - center_cost)
        push_from_top    = np.array([0, 1]) * (costmap[x, y_top] - center_cost)

        vectors = []
        if costmap[x_right, y]  > center_cost: vectors.append(push_from_right)
        if costmap[x_left, y]   > center_cost: vectors.append(push_from_left)
        if costmap[x, y_bottom] > center_cost: vectors.append(push_from_bottom)
        if costmap[x, y_top]    > center_cost: vectors.append(push_from_top)

        if not vectors:
            return 0.0, 0.0 
            
        dx, dy = max(vectors, key=lambda v: v[0]**2 + v[1]**2)
        # dx = max(-10, min(10, dx))
        # dy = max(-10, min(10, dy))

        obs_vx = dx
        obs_vy = dy
        d2x, d2y = self.prev_d2
        d2x = obs_vx - d2x
        d2y = obs_vy - d2y
        self.prev_d2 = (obs_vx, obs_vy)

        magnitude = math.sqrt(obs_vx**2 + obs_vy**2)
        if magnitude > 0:
            obs_vx = (obs_vx / magnitude) * gain_obs + d_gain * d2x 
            obs_vy = (obs_vy / magnitude) * gain_obs + d_gain * d2y
        
        return obs_vx, obs_vy

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