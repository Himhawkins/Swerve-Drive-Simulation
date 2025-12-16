import pygame
import math
from collections import deque

# --- MOCK CONFIGURATION (To make the script standalone) ---
# class cfg:
#     SCREEN_WIDTH = 1200  # Wielded for plots
#     SCREEN_HEIGHT = 600
#     PIXELS_PER_METER = 100
#     FPS = 60
    
#     # Robot Specs
#     CHASSIS_WIDTH_M = 0.5
#     CHASSIS_LENGTH_M = 0.5
#     WHEEL_OFFSET_X = 0.25
#     WHEEL_OFFSET_Y = 0.25
    
#     # Physics Limits
#     MAX_SPEED = 3.0           # m/s
#     MAX_ROTATION_SPEED = 4.0  # rad/s
    
#     # Colors
#     BACKGROUND_COLOR = (30, 30, 30)
#     ROBOT_BODY_COLOR = (100, 100, 255)
#     WHEEL_COLOR = (200, 200, 200)
#     TEXT_COLOR = (255, 255, 255)
    
import robot_config as cfg
   

# --- PLOTTING UTILITY ---
class RealTimePlotter:
    def __init__(self, x, y, w, h, title, y_range=(-1, 1), labels=None):
        self.rect = pygame.Rect(x, y, w, h)
        self.title = title
        self.y_min, self.y_max = y_range
        self.data_queues = [] 
        self.labels = labels if labels else []
        self.max_points = w  # One pixel per point horizontally
        self.font = pygame.font.SysFont("Consolas", 12)

    def add_series(self):
        """Adds a new data line to track"""
        self.data_queues.append(deque(maxlen=self.max_points))

    def update(self, values):
        """Values is a list of floats corresponding to series"""
        if not isinstance(values, list): values = [values]
        
        for i, val in enumerate(values):
            if i < len(self.data_queues):
                self.data_queues[i].append(val)

    def draw(self, surface):
        # Draw Background and Border
        pygame.draw.rect(surface, (50, 50, 50), self.rect)
        pygame.draw.rect(surface, (200, 200, 200), self.rect, 1)
        
        # Draw Title
        title_surf = self.font.render(self.title, True, (200, 200, 200))
        surface.blit(title_surf, (self.rect.x + 5, self.rect.y + 5))
        
        # Draw Zero Line
        if self.y_min < 0 < self.y_max:
            zero_ratio = (0 - self.y_min) / (self.y_max - self.y_min)
            zero_y = self.rect.bottom - (zero_ratio * self.rect.height)
            pygame.draw.line(surface, (100, 100, 100), (self.rect.left, zero_y), (self.rect.right, zero_y))

        # Draw Lines
        for i, queue in enumerate(self.data_queues):
            if len(queue) < 2: continue
            
            color = cfg.PLOT_COLORS[i % len(cfg.PLOT_COLORS)]
            points = []
            for x_idx, val in enumerate(queue):
                # Normalize Y
                clamped_val = max(self.y_min, min(self.y_max, val))
                ratio = (clamped_val - self.y_min) / (self.y_max - self.y_min)
                
                px = self.rect.right - (len(queue) - x_idx)
                py = self.rect.bottom - (ratio * self.rect.height)
                points.append((px, py))
            
            if len(points) > 1:
                pygame.draw.lines(surface, color, False, points, 2)
                
                # Draw Label/Legend text for this line
                if self.labels and i < len(self.labels):
                    label = self.labels[i]
                    val_text = f"{label}: {queue[-1]:.2f}"
                    txt_surf = self.font.render(val_text, True, color)
                    surface.blit(txt_surf, (self.rect.x + 5, self.rect.y + 20 + (i * 12)))

# --- ROBOT MODULES (Unchanged Logic, added getters) ---
class KinematicModule:
    def __init__(self, x_offset, y_offset):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.angle = 0.0
        self.speed = 0.0

    def calculate_target_state(self, vx, vy, omega):
        tangent_vx = -omega * self.y_offset
        tangent_vy = omega * self.x_offset
        final_vx = vx + tangent_vx
        final_vy = vy + tangent_vy
        
        self.speed = math.hypot(final_vx, final_vy)
        if self.speed > 0.01:
            self.angle = math.atan2(final_vy, final_vx)

    def draw(self, surface, robot_pos, robot_heading):
        px_x = robot_pos[0] * cfg.PIXELS_PER_METER
        px_y = robot_pos[1] * cfg.PIXELS_PER_METER
        cos_h, sin_h = math.cos(robot_heading), math.sin(robot_heading)
        rot_x = self.x_offset * cos_h - self.y_offset * sin_h
        rot_y = self.x_offset * sin_h + self.y_offset * cos_h
        screen_x = px_x + rot_x * cfg.PIXELS_PER_METER
        screen_y = px_y + rot_y * cfg.PIXELS_PER_METER
        
        wheel_surf = pygame.Surface((10, 20), pygame.SRCALPHA)
        wheel_surf.fill(cfg.WHEEL_COLOR)
        pygame.draw.line(wheel_surf, (50,50,50), (5, 10), (5, 0), 2) # Direction line
        
        total_angle_deg = -math.degrees(self.angle + robot_heading) - 90
        rotated_wheel = pygame.transform.rotate(wheel_surf, total_angle_deg)
        rect = rotated_wheel.get_rect(center=(screen_x, screen_y))
        surface.blit(rotated_wheel, rect)

class KinematicRobot:
    def __init__(self, start_x, start_y):
        self.x = start_x
        self.y = start_y
        self.heading = 0.0
        # Order: FR, FL, BL, BR
        self.modules = [
            KinematicModule(cfg.WHEEL_OFFSET_X, -cfg.WHEEL_OFFSET_Y), 
            KinematicModule(-cfg.WHEEL_OFFSET_X, -cfg.WHEEL_OFFSET_Y),
            KinematicModule(-cfg.WHEEL_OFFSET_X, cfg.WHEEL_OFFSET_Y), 
            KinematicModule(cfg.WHEEL_OFFSET_X, cfg.WHEEL_OFFSET_Y)
        ]

    def update(self, cmd_vx, cmd_vy, cmd_omega, dt):
        max_wheel_speed = 0.0
        for mod in self.modules:
            mod.calculate_target_state(cmd_vx, cmd_vy, cmd_omega)
            max_wheel_speed = max(max_wheel_speed, mod.speed)
            
        if max_wheel_speed > cfg.MAX_SPEED:
            scale = cfg.MAX_SPEED / max_wheel_speed
            for mod in self.modules: mod.speed *= scale

        act_vx, act_vy, act_omega = self._compute_forward_kinematics()

        # World Frame Integration
        world_vx = act_vx * math.cos(self.heading) - act_vy * math.sin(self.heading)
        world_vy = act_vx * math.sin(self.heading) + act_vy * math.cos(self.heading)
        
        self.x += world_vx * dt
        self.y += world_vy * dt
        self.heading += act_omega * dt

    def _compute_forward_kinematics(self):
        vx_sum, vy_sum, omega_sum = 0, 0, 0
        num_modules = len(self.modules)
        radius = math.hypot(cfg.WHEEL_OFFSET_X, cfg.WHEEL_OFFSET_Y)
        
        for mod in self.modules:
            wx = mod.speed * math.cos(mod.angle)
            wy = mod.speed * math.sin(mod.angle)
            vx_sum += wx
            vy_sum += wy
            tx = -mod.y_offset / radius
            ty = mod.x_offset / radius
            rotation_comp = (wx * tx) + (wy * ty)
            omega_sum += rotation_comp / radius
            
        return vx_sum / num_modules, vy_sum / num_modules, omega_sum / num_modules

    def draw(self, surface):
        px_x = self.x * cfg.PIXELS_PER_METER
        px_y = self.y * cfg.PIXELS_PER_METER
        w, l = cfg.CHASSIS_WIDTH_M / 2, cfg.CHASSIS_LENGTH_M / 2
        corners = []
        for lx, ly in [(w, -l), (-w, -l), (-w, l), (w, l)]:
            rx = lx * math.cos(self.heading) - ly * math.sin(self.heading)
            ry = lx * math.sin(self.heading) + ly * math.cos(self.heading)
            corners.append((px_x + rx * cfg.PIXELS_PER_METER, px_y + ry * cfg.PIXELS_PER_METER))
            
        pygame.draw.polygon(surface, cfg.ROBOT_BODY_COLOR, corners, 2)
        front_x = px_x + (w * 1.5 * cfg.PIXELS_PER_METER) * math.cos(self.heading)
        front_y = px_y + (w * 1.5 * cfg.PIXELS_PER_METER) * math.sin(self.heading)
        pygame.draw.line(surface, (255, 255, 0), (px_x, px_y), (front_x, front_y), 2)
        
        for mod in self.modules: mod.draw(surface, (self.x, self.y), self.heading)

# --- MAIN ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((cfg.SCREEN_WIDTH, cfg.SCREEN_HEIGHT))
    pygame.display.set_caption("Swerve Simulator - Press 'M' for Circle Mode")
    clock = pygame.time.Clock()
    
    # 1. Setup Robot
    # Start slightly to the left to leave room for plots
    robot = KinematicRobot(3.0, 3.0) 
    
    # 2. Setup Plotters (Right side of screen)
    plot_x = 700
    plot_w = 480
    
    # Plot 1: Robot Velocity (Vx, Vy)
    p_vel = RealTimePlotter(plot_x, 10, plot_w, 130, "Chassis Vel (m/s)", (-3.5, 3.5), ["Vx", "Vy"])
    p_vel.add_series(); p_vel.add_series() # Red, Green
    
    # Plot 2: Wheel Speeds (All 4 modules)
    p_wh_spd = RealTimePlotter(plot_x, 150, plot_w, 130, "Wheel Speeds (m/s)", (0, 3.5), ["FR", "FL", "BL", "BR"])
    for _ in range(4): p_wh_spd.add_series()
    
    # Plot 3: Steering Angles
    p_ang = RealTimePlotter(plot_x, 290, plot_w, 130, "Steer Angles (rad)", (-3.2, 3.2), ["FR", "FL", "BL", "BR"])
    for _ in range(4): p_ang.add_series()

    # Plot 4: Robot Omega
    p_rot = RealTimePlotter(plot_x, 430, plot_w, 130, "Robot Omega (rad/s)", (-4, 4), ["Omega"])
    p_rot.add_series()

    mode_circle = False
    circle_timer = 0.0
    
    running = True
    while running:
        dt_ms = clock.tick(cfg.FPS)
        dt = dt_ms / 1000.0 
        
        screen.fill(cfg.BACKGROUND_COLOR)
        
        # --- Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_m:
                    mode_circle = not mode_circle
                    circle_timer = 0.0 # Reset timer on entry
                    print(f"Mode changed: {'Circle' if mode_circle else 'Manual'}")

        # --- Command Generation ---
        vx, vy, omega = 0, 0, 0
        
        if mode_circle:
            # Circle Logic: 
            # We want the robot to translate in a circle while maintaining heading (Holonomic Circle)
            # or rotating. Let's do a Holonomic Circle (translating circle).
            circle_timer += dt
            radius = 1.0 # meter radius for the path
            speed = 1.5  # m/s
            
            # Parametric Circle: x = r*cos(t), y = r*sin(t)
            # Velocity is derivative: vx = -r*sin(t), vy = r*cos(t)
            # We scale t by speed so the robot actually moves at 'speed'
            freq = speed / radius
            
            vx = -speed * math.sin(circle_timer * freq)
            vy = speed * math.cos(circle_timer * freq)
            omega = 0.5 # Add a slow spin just to make the swerve modules work harder!
            
        else:
            # Manual Input
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w]: vy = -cfg.MAX_SPEED
            if keys[pygame.K_s]: vy = cfg.MAX_SPEED
            if keys[pygame.K_a]: vx = -cfg.MAX_SPEED
            if keys[pygame.K_d]: vx = cfg.MAX_SPEED
            if keys[pygame.K_q]: omega = -cfg.MAX_ROTATION_SPEED
            if keys[pygame.K_e]: omega = cfg.MAX_ROTATION_SPEED

        # --- Physics Update ---
        robot.update(vx, vy, omega, dt)
        
        # --- Data Plotting Update ---
        # 1. Chassis Velocity
        # Note: We plot the COMMANDED velocity here to see smoothness
        p_vel.update([vx, vy]) 
        
        # 2. Wheel Speeds
        speeds = [m.speed for m in robot.modules]
        p_wh_spd.update(speeds)
        
        # 3. Steering Angles
        angles = [m.angle for m in robot.modules]
        p_ang.update(angles)
        
        # 4. Omega
        p_rot.update([omega])

        # --- Rendering ---
        robot.draw(screen)
        
        # Draw Plots
        p_vel.draw(screen)
        p_wh_spd.draw(screen)
        p_ang.draw(screen)
        p_rot.draw(screen)
        
        # Overlay Text
        font = pygame.font.SysFont("Consolas", 18)
        mode_txt = "CIRCLE MODE (Auto)" if mode_circle else "MANUAL MODE (WASD+QE)"
        screen.blit(font.render(f"Mode: {mode_txt} (Press 'M' to toggle)", True, (255, 255, 0)), (10, 10))
        
        pygame.display.flip()
    pygame.quit()

if __name__ == "__main__":
    main()