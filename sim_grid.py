import pygame
import random
import sys
import numpy as np
import robot_config as config
from planner import Planner

pygame.init()
screen = pygame.display.set_mode((config.GRID_WINDOW_WIDTH, config.GRID_WINDOW_HEIGHT))
pygame.display.set_caption("Potential Field Pathfinding (Press 'V' to Toggle Heatmap)")
clock = pygame.time.Clock()

class MazeGenerator:
    def __init__(self, cols, rows):
        self.cols = cols
        self.rows = rows
        self.grid = []
        self.walls = [] 
        for r in range(rows):
            row = []
            for c in range(cols):
                row.append([False, True, True, True, True])
            self.grid.append(row)
        self.generate_maze()
        self.create_wall_rects()

    def generate_maze(self):
        stack = [(0, 0)]
        self.grid[0][0][0] = True 
        while stack:
            current_r, current_c = stack[-1]
            neighbors = []
            directions = [(-1, 0, 1, 3), (0, 1, 2, 4), (1, 0, 3, 1), (0, -1, 4, 2)]
            for dr, dc, cw, nw in directions:
                nr, nc = current_r + dr, current_c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols and not self.grid[nr][nc][0]:
                    neighbors.append((nr, nc, cw, nw))
            if neighbors:
                next_r, next_c, cw, nw = random.choice(neighbors)
                self.grid[current_r][current_c][cw] = False
                self.grid[next_r][next_c][nw] = False
                self.grid[next_r][next_c][0] = True
                stack.append((next_r, next_c))
            else:
                stack.pop()

    def create_wall_rects(self):
        self.walls = []
        for r in range(self.rows):
            for c in range(self.cols):
                x, y = c * config.CELL_SIZE, r * config.CELL_SIZE
                cell = self.grid[r][c]
                if cell[1]: self.walls.append(pygame.Rect(x, y, config.CELL_SIZE, config.WALL_THICKNESS))
                if cell[2]: self.walls.append(pygame.Rect(x + config.CELL_SIZE - config.WALL_THICKNESS, y, config.WALL_THICKNESS, config.CELL_SIZE))
                if cell[3]: self.walls.append(pygame.Rect(x, y + config.CELL_SIZE - config.WALL_THICKNESS, config.CELL_SIZE, config.WALL_THICKNESS))
                if cell[4]: self.walls.append(pygame.Rect(x, y, config.WALL_THICKNESS, config.CELL_SIZE))

def generate_start_goal(cols, rows):
    start_pos = (random.randint(0, cols-1), random.randint(0, rows-1))
    goal_pos = (random.randint(0, cols-1), random.randint(0, rows-1))
    while start_pos == goal_pos:
        goal_pos = (random.randint(0, cols-1), random.randint(0, rows-1))
    def get_center(c, r):
        return (c * config.CELL_SIZE + config.CELL_SIZE//2, r * config.CELL_SIZE + config.CELL_SIZE//2)
    return get_center(*start_pos), get_center(*goal_pos)

class Robot:
    def __init__(self, x, y):
        self.rect = pygame.Rect(0, 0, config.PLAYER_SIZE, config.PLAYER_SIZE)
        self.rect.center = (x, y)
        self.pos_x, self.pos_y = float(self.rect.x), float(self.rect.y)

    def move(self, dx, dy, walls):
        self.pos_x += dx
        self.rect.x = int(self.pos_x)
        for wall in walls:
            if self.rect.colliderect(wall):
                if dx > 0: self.rect.right = wall.left
                if dx < 0: self.rect.left = wall.right
                self.pos_x = self.rect.x
        self.pos_y += dy
        self.rect.y = int(self.pos_y)
        for wall in walls:
            if self.rect.colliderect(wall):
                if dy > 0: self.rect.bottom = wall.top
                if dy < 0: self.rect.top = wall.bottom
                self.pos_y = self.rect.y
    def draw(self, surface):
        pygame.draw.rect(surface, config.COLOR_ROBOT, self.rect)


if __name__ == "__main__":
    # --- Main Setup ---
    maze = MazeGenerator(config.GRID_COLS, config.GRID_ROWS)
    start_xy, goal_xy = generate_start_goal(config.GRID_COLS, config.GRID_ROWS)
    robot = Robot(*start_xy)
    planner = Planner(maze.walls, config.GRID_WINDOW_WIDTH, config.GRID_WINDOW_HEIGHT)
    current_path = planner.get_path(start_xy, goal_xy)

    show_heatmap = True # Toggle state

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            
            # Press Space to Generate New Maze
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    maze = MazeGenerator(config.GRID_COLS, config.GRID_ROWS)
                    start_xy, goal_xy = generate_start_goal(config.GRID_COLS, config.GRID_ROWS)
                    robot = Robot(*start_xy)
                    planner = Planner(maze.walls, config.GRID_WINDOW_WIDTH, config.GRID_WINDOW_HEIGHT)
                    current_path = planner.get_path(start_xy, goal_xy)
                
                # Press V to Toggle Heatmap
                if event.key == pygame.K_v:
                    show_heatmap = not show_heatmap

        keys = pygame.key.get_pressed()
        mx, my = 0, 0
        if keys[pygame.K_LEFT]:  mx = -config.PLAYER_SPEED
        if keys[pygame.K_RIGHT]: mx = config.PLAYER_SPEED
        if keys[pygame.K_UP]:    my = -config.PLAYER_SPEED
        if keys[pygame.K_DOWN]:  my = config.PLAYER_SPEED

        robot.move(mx, my, maze.walls)

        screen.fill(config.COLOR_BG)
        
        # 1. Draw Heatmap (if enabled)
        if show_heatmap:
            screen.blit(planner.vis_surface, (0, 0))

        # 2. Draw Walls
        for wall in maze.walls:
            pygame.draw.rect(screen, config.COLOR_WALL, wall)

        # 3. Draw Path
        if len(current_path) > 1:
            pygame.draw.lines(screen, config.COLOR_PATH, False, current_path, 3)

        pygame.draw.circle(screen, config.COLOR_START, start_xy, 10)
        pygame.draw.circle(screen, config.COLOR_GOAL, goal_xy, 10)
        robot.draw(screen)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()