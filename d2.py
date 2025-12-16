import pygame
import math
from collections import deque
import robot_config as cfg
# --- CONFIGURATION ---
#

# --- PLOTTING UTILITY ---
class RealTimePlotter:
    def __init__(self, x, y, w, h, title, y_range=(-1, 1), labels=None):
        self.rect = pygame.Rect(x, y, w, h)
        self.base_title = title
        self.y_min, self.y_max = y_range
        self.data_queues = [] 
        self.labels = labels if labels else []
        self.max_points = w // 2
        self.font = pygame.font.SysFont("Consolas", 12)
        
    def reset(self, new_title, new_y_range, new_labels):
        self.base_title = new_title
        self.y_min, self.y_max = new_y_range
        self.labels = new_labels
        self.data_queues = [deque(maxlen=self.max_points) for _ in new_labels]

    def update(self, values):
        if not isinstance(values, list): values = [values]
        for i, val in enumerate(values):
            if i < len(self.data_queues):
                self.data_queues[i].append(val)

    def draw(self, surface):
        pygame.draw.rect(surface, (40, 40, 40), self.rect)
        pygame.draw.rect(surface, (150, 150, 150), self.rect, 1)
        
        title_surf = self.font.render(self.base_title, True, (220, 220, 220))
        surface.blit(title_surf, (self.rect.x + 5, self.rect.y + 5))
        
        # Zero Line
        if self.y_min < 0 < self.y_max:
            zero_ratio = (0 - self.y_min) / (self.y_max - self.y_min)
            zero_y = self.rect.bottom - (zero_ratio * self.rect.height)
            pygame.draw.line(surface, (80, 80, 80), (self.rect.left, zero_y), (self.rect.right, zero_y))

        for i, queue in enumerate(self.data_queues):
            if len(queue) < 2: continue
            color = cfg.PLOT_COLORS[i % len(cfg.PLOT_COLORS)]
            points = []
            for x_idx, val in enumerate(queue):
                c_val = max(self.y_min, min(self.y_max, val))
                ratio = (c_val - self.y_min) / (self.y_max - self.y_min)
                px = self.rect.right - (len(queue) - x_idx) * 2
                py = self.rect.bottom - (ratio * self.rect.height)
                points.append((px, py))
            
            if len(points) > 1:
                pygame.draw.lines(surface, color, False, points, 2)
                if i < len(self.labels):
                    lbl = f"{self.labels[i]}: {queue[-1]:.2f}"
                    ts = self.font.render(lbl, True, color)
                    surface.blit(ts, (self.rect.x + 5, self.rect.y + 20 + i*12))

# --- DYNAMIC MODULE ---
import numpy as np
class DynamicModule:
    def __init__(self, x_offset, y_offset):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.angle = 0.0          
        self.force_mag = 0.0      
        self.pid_integral = 0.0
        self.pid_last_error = 0.0

    def set_command(self, target_angle, force):
        self.angle = target_angle
        # if np.random.uniform()<0.3:
        #     force+= np.random.normal(loc=0, scale=10)
        self.force_mag = max(-cfg.MAX_WHEEL_FORCE, min(force, cfg.MAX_WHEEL_FORCE))

    def get_force_vector_robot_frame(self):
        fx = self.force_mag * math.cos(self.angle)
        fy = self.force_mag * math.sin(self.angle)
        return fx, fy

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
        pygame.draw.line(wheel_surf, (255,255,255), (10, 20), (10, 0), 3)
        
        total_angle_deg = -math.degrees(self.angle + robot_heading) - 90
        rotated_wheel = pygame.transform.rotate(wheel_surf, total_angle_deg)
        rect = rotated_wheel.get_rect(center=(screen_x, screen_y))
        surface.blit(rotated_wheel, rect)
        
        if abs(self.force_mag) > 1.0:
            force_len = (self.force_mag / cfg.MAX_WHEEL_FORCE) * 40
            world_angle = self.angle + robot_heading
            end_x = screen_x + math.cos(world_angle) * force_len
            end_y = screen_y + math.sin(world_angle) * force_len
            pygame.draw.line(surface, cfg.FORCE_VECTOR_COLOR, (screen_x, screen_y), (end_x, end_y), 2)

# --- DYNAMIC ROBOT ---
class DynamicRobot:
    def __init__(self, start_x, start_y):
        self.x = start_x
        self.y = start_y
        self.heading = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.tar_v = 0
        self.path = deque(maxlen=320)
        self.modules = [
            DynamicModule(cfg.WHEEL_OFFSET_X, -cfg.WHEEL_OFFSET_Y),
            DynamicModule(-cfg.WHEEL_OFFSET_X, -cfg.WHEEL_OFFSET_Y),
            DynamicModule(-cfg.WHEEL_OFFSET_X, cfg.WHEEL_OFFSET_Y),
            DynamicModule(cfg.WHEEL_OFFSET_X, cfg.WHEEL_OFFSET_Y)
        ]

    def update_physics(self, dt):
        # 1. Sum Forces from Wheels (Robot Frame)
        net_fx, net_fy, net_torque = 0.0, 0.0, 0.0
        
        for mod in self.modules:
            fx, fy = mod.get_force_vector_robot_frame()
            net_fx += fx
            net_fy += fy
            torque = (mod.x_offset * fy) - (mod.y_offset * fx)
            net_torque += torque
            
        # 2. Transform Wheel Forces to World Frame
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        world_fx = net_fx * cos_h - net_fy * sin_h
        world_fy = net_fx * sin_h + net_fy * cos_h

        # 3. Calculate Velocities and Speed
        speed = math.hypot(self.vx, self.vy)
        
        # --- FRICTION PHYSICS ---
        drag_fx = -self.vx * cfg.LINEAR_DRAG * cfg.ROBOT_MASS
        drag_fy = -self.vy * cfg.LINEAR_DRAG * cfg.ROBOT_MASS
        friction_mag = cfg.FRICTION_MU * cfg.ROBOT_MASS * cfg.GRAVITY
        friction_fx, friction_fy = 0.0, 0.0
        
        STOP_THRESHOLD = 0.1
        applied_fx = world_fx + drag_fx
        applied_fy = world_fy + drag_fy
        applied_mag = math.hypot(applied_fx, applied_fy)
        
        # Define Torque Friction Limit (approximate lever arm)
        friction_torque_limit = friction_mag * 0.35 

        # --- LOGIC FIX START ---
        if speed < STOP_THRESHOLD:
            # Check Linear Stiction
            if applied_mag < friction_mag:
                # Force too weak to move linearly -> Linear Stiction Active
                self.vx = 0.0
                self.vy = 0.0
                
                # To prevent movement, Friction cancels Applied Force exactly
                friction_fx = -applied_fx
                friction_fy = -applied_fy
                
                # NOW Check Angular Stiction INDEPENDENTLY
                if abs(net_torque) < friction_torque_limit and abs(self.omega) < 0.1:
                    self.omega = 0.0
                    return # Only return if BOTH linear and angular are stuck
                
                # If we have torque, we DO NOT return. We continue to integrate angular acceleration.
            else:
                # Breaking Linear Static Friction (Starting to move)
                friction_fx = -(applied_fx / applied_mag) * friction_mag
                friction_fy = -(applied_fy / applied_mag) * friction_mag
        else:
            # Kinetic Friction: Opposes Velocity
            friction_fx = -(self.vx / speed) * friction_mag
            friction_fy = -(self.vy / speed) * friction_mag
        # --- LOGIC FIX END ---
        net_torque -= self.omega * cfg.ANGULAR_DRAG * cfg.ROBOT_MOI
        
        # 5. Integrate
        final_fx = world_fx + drag_fx + friction_fx
        final_fy = world_fy + drag_fy + friction_fy
        
        ax = final_fx / cfg.ROBOT_MASS
        ay = final_fy / cfg.ROBOT_MASS
        alpha = net_torque / cfg.ROBOT_MOI
        
        self.vx += ax * dt
        self.vy += ay * dt
        self.omega += alpha * dt
        
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.path.append((self.x, self.y))
        self.heading += self.omega * dt 
    
    def set_velocity_control(self, target_vx, target_vy, target_omega, dt):
        cos_h, sin_h = math.cos(self.heading),math.sin(self.heading)
        sin_h = math.sin(self.heading)
        current_r_vx = self.vx * cos_h + self.vy * sin_h
        current_r_vy = -self.vx * sin_h + self.vy * cos_h
        
        for mod in self.modules:
            tan_vx = -target_omega * mod.y_offset
            tan_vy = target_omega * mod.x_offset
            want_vx = target_vx + tan_vx
            want_vy = target_vy + tan_vy
            want_speed = math.hypot(want_vx, want_vy)
            
            cur_wheel_vx = current_r_vx - self.omega * mod.y_offset
            cur_wheel_vy = current_r_vy + self.omega * mod.x_offset

            if want_speed > 0.01:
                want_angle = math.atan2(want_vy, want_vx)
                dir_x, dir_y = math.cos(want_angle), math.sin(want_angle)
                current_speed_proj = (cur_wheel_vx * dir_x) + (cur_wheel_vy * dir_y)
                
                error = want_speed - current_speed_proj
                mod.pid_integral += error * dt
                derivative = (error - mod.pid_last_error) / dt if dt > 0 else 0
                mod.pid_last_error = error
                
                # Friction Feed-Forward
                friction_ff = (cfg.FRICTION_MU * cfg.ROBOT_MASS * cfg.GRAVITY) / 4.0
                output = (cfg.kP * error) + (cfg.kI * mod.pid_integral) + (cfg.kD * derivative) + friction_ff
                mod.set_command(want_angle, output)
            else:
                mod.set_command(mod.angle, 0)
                mod.pid_integral = 0

    def draw(self, surface):
        px_x = self.x * cfg.PIXELS_PER_METER
        px_y = self.y * cfg.PIXELS_PER_METER
        if len(self.path) > 1:
            # Convert metric coordinates to screen pixels
            pixel_points = []
            for px, py in self.path:
                screen_x = px * cfg.PIXELS_PER_METER
                screen_y = py * cfg.PIXELS_PER_METER
                pixel_points.append((screen_x, screen_y))
            
            # Draw a fading line (or just a solid one for simplicity)
            pygame.draw.lines(surface, (100, 255, 100), False, pixel_points, 2)
        corners = []
        w, l = cfg.CHASSIS_WIDTH_M / 2, cfg.CHASSIS_LENGTH_M / 2
        for lx, ly in [(w, -l), (-w, -l), (-w, l), (w, l)]:
            rx = lx * math.cos(self.heading) - ly * math.sin(self.heading)
            ry = lx * math.sin(self.heading) + ly * math.cos(self.heading)
            corners.append((px_x + rx * cfg.PIXELS_PER_METER, px_y + ry * cfg.PIXELS_PER_METER))
            
        pygame.draw.polygon(surface, cfg.ROBOT_BODY_COLOR, corners, 2)
        
        front_x = px_x + (w * 1.5) * cfg.PIXELS_PER_METER * math.cos(self.heading)
        front_y = px_y + (w * 1.5) * cfg.PIXELS_PER_METER * math.sin(self.heading)
        pygame.draw.line(surface, (255, 0, 0), (px_x, px_y), (front_x, front_y), 2)

        for mod in self.modules:
            mod.draw(surface, (self.x, self.y), self.heading)

# --- MAIN ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((cfg.SCREEN_WIDTH, cfg.SCREEN_HEIGHT))
    pygame.display.set_caption("Dynamic Swerve + Friction & Wheel Angle Plots")
    clock = pygame.time.Clock()
    
    robot = DynamicRobot(3.0, 3.0)
    
    # --- PLOT LAYOUT (Adjusted for 4 plots) ---
    plot_x = 700
    plot_w = 480
    plot_h = 130 # Slightly shorter to fit 4
    
    # 4 Plots arranged vertically
    p1 = RealTimePlotter(plot_x, 10, plot_w, plot_h, "")
    p2 = RealTimePlotter(plot_x, 150, plot_w, plot_h, "")
    p3 = RealTimePlotter(plot_x, 290, plot_w, plot_h, "Position Y (m)", (0, 6), ["Y Pos", "X Pos"])
    p4 = RealTimePlotter(plot_x, 430, plot_w, plot_h, "Wheel Angles (rad)", (-3.5, 3.5), ["FR", "FL", "BL", "BR"]) # NEW PLOT

    MODE_MANUAL = 0
    MODE_CIRCLE_VEL = 1
    MODE_CIRCLE_TORQUE = 2
    
    current_mode = MODE_MANUAL
    timer = 0.0

    # Initial Reset
    p1.reset("Robot Velocity (m/s)", (-4, 4), ["Vx", "Vy"])
    p2.reset("Wheel Torque Cmd (N)", (-50, 50), ["FR", "FL", "BL", "BR"])
    p3.reset("Robot Position (m)", (0, 6), ["Y Pos", "X Pos"])
    # p4 stays as wheel angles for all modes
    p4.reset("Wheel Angles (rad)", (-3.5, 3.5), ["FR", "FL", "BL", "BR"])
    running = True
    while running:
        dt_ms = clock.tick(cfg.FPS)
        dt = dt_ms / 1000.0
        
        screen.fill(cfg.BACKGROUND_COLOR)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.KEYDOWN:
                new_mode = None
                if event.key == pygame.K_v: new_mode = MODE_CIRCLE_VEL
                elif event.key == pygame.K_t: new_mode = MODE_CIRCLE_TORQUE
                elif event.key == pygame.K_m: new_mode = MODE_MANUAL
                
                if new_mode is not None and new_mode != current_mode:
                    current_mode = new_mode
                    timer = 0.0
                    # Reconfigure Velocity/Torque plots based on mode
                    if current_mode == MODE_CIRCLE_TORQUE:
                        p1.reset("Robot Velocity (m/s)", (-3, 3), ["Vx", "Vy"])
                        p2.reset("Wheel Torque Cmd (N)", (-40, 40), ["FR", "FL", "BL", "BR"])
                    elif current_mode == MODE_CIRCLE_VEL:
                        p1.reset("Velocity Error (m/s)", (-0.5, 0.5), ["Err Vx", "Err Vy"])
                        p2.reset("Wheel Torque Cmd (N)", (-40, 40), ["FR", "FL", "BL", "BR"])
                    else:
                        p1.reset("Robot Velocity (m/s)", (-4, 4), ["Vx", "Vy"])
                        p2.reset("Wheel Torque Cmd (N)", (-40, 40), ["FR", "FL", "BL", "BR"])
                    # P3 (Position) and P4 (Angles) stay the same

        # Control Logic
        req_vx, req_vy, req_omega = 0, 0, 0
        target_v_plot = [0, 0] 
        
        if current_mode == MODE_MANUAL:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w]: req_vy = -cfg.MAX_SPEED
            if keys[pygame.K_s]: req_vy = cfg.MAX_SPEED
            if keys[pygame.K_a]: req_vx = -cfg.MAX_SPEED
            if keys[pygame.K_d]: req_vx = cfg.MAX_SPEED
            if keys[pygame.K_q]: req_omega = -cfg.MAX_ROTATION_SPEED
            if keys[pygame.K_e]: req_omega = cfg.MAX_ROTATION_SPEED
            robot.set_velocity_control(req_vx, req_vy, req_omega, dt)
            
        elif current_mode == MODE_CIRCLE_VEL:
            timer += dt
            speed = 5.0
            req_vx = speed * math.cos(timer)
            req_vy = speed * math.sin(timer)
            target_v_plot = [req_vx, req_vy]
            robot.set_velocity_control(req_vx, req_vy, 0, dt)
            
        elif current_mode == MODE_CIRCLE_TORQUE:
            timer += dt
            force_angle = timer
            force_mag = 30.0 
            for mod in robot.modules:
                mod.set_command(force_angle, force_mag)

        # Physics Step
        robot.update_physics(dt)
        
        # --- DATA PLOTTING ---
        if current_mode == MODE_CIRCLE_VEL:
            p1.update([target_v_plot[0] - robot.vx, target_v_plot[1] - robot.vy])
        else:
            p1.update([robot.vx, robot.vy])
            
        p2.update([m.force_mag for m in robot.modules])
        p3.update([robot.y, robot.x])
        
        # Plot 4: Wheel Angles (New)
        p4.update([m.angle for m in robot.modules])
        
        robot.draw(screen)
        p1.draw(screen)
        p2.draw(screen)
        p3.draw(screen)
        p4.draw(screen) # Draw new plot
        
        font = pygame.font.SysFont("Consolas", 18)
        mode_str = ["MANUAL", "VELOCITY CIRCLE", "TORQUE CIRCLE"][current_mode]
        screen.blit(font.render(f"MODE: {mode_str}", True, (255, 255, 0)), (10, 10))
        
        pygame.display.flip()
    pygame.quit()

if __name__ == "__main__":
    main()