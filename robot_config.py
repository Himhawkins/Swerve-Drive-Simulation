# robot_config.py
import math

# --- Existing Config ---
# Screen / Sim
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
FPS = 60

# Colors
BACKGROUND_COLOR = (30, 30, 30)
ROBOT_BODY_COLOR = (50, 100, 200)
WHEEL_COLOR = (200, 50, 50)
VECTOR_LINE_COLOR = (0, 255, 0)
FORCE_VECTOR_COLOR = (255, 255, 0)
TEXT_COLOR = (255, 255, 255)

# Physics Scaling
PIXELS_PER_METER = 60 # 1 meter = 50 pixels

# Robot Physical Properties
ROBOT_MASS = 30.0      # kg
ROBOT_MOI = 3    # Moment of Inertia
GRAVITY = 9.81    

# Friction / Drag
LINEAR_DRAG = 2.0      
ANGULAR_DRAG = 3.0    
FRICTION_MU = 0.1        

# Chassis Dimensions
CHASSIS_WIDTH_M = 0.6
CHASSIS_LENGTH_M = 0.6
WHEEL_OFFSET_X = CHASSIS_WIDTH_M / 2
WHEEL_OFFSET_Y = CHASSIS_LENGTH_M / 2

# Wheel / Drive Limits
MAX_SPEED = 10.0        # m/s
MAX_ROTATION_SPEED = 20 # rad/s
MAX_WHEEL_FORCE = 200.0 # Newtons

kP=5
kI=0.005
kD=0.5

# Plot Colors (FR, FL, BL, BR)
PLOT_COLORS = [(255, 0, 0), (0, 255, 0), (0, 255, 255), (255, 0, 255)]

# --- NEW MAZE SIMULATION CONFIG ---

# Grid Dimensions (renamed to avoid clash)
GRID_WINDOW_WIDTH = 1500 
GRID_WINDOW_HEIGHT = 1200
CELL_SIZE = 180   #100
WALL_THICKNESS = 20 #10

# Player / Robot Logic
PLAYER_SIZE = 15
PLAYER_SPEED = 3.0

# Calculated Grid dimensions
GRID_COLS = GRID_WINDOW_WIDTH // CELL_SIZE
GRID_ROWS = GRID_WINDOW_HEIGHT // CELL_SIZE

# Maze Colors
COLOR_BG = (20, 20, 30)
COLOR_WALL = (200, 200, 200)
COLOR_ROBOT = (0, 255, 100)
COLOR_START = (50, 50, 255)
COLOR_GOAL = (255, 50, 50)
COLOR_PATH = (255, 255, 0)

#Obstacle Avoidance
Dynamic_Avoidance=False 