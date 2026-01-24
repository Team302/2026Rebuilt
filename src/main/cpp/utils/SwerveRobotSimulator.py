import pygame
import numpy as np
import math
from dataclasses import dataclass

# Initialize Pygame
pygame.init()

# Constants
FIELD_LENGTH_FEET = 54 + 3/12  # 54'3" in feet
FIELD_WIDTH_FEET = 26 + 3/12   # 26'3" in feet
FIELD_LENGTH_METERS = FIELD_LENGTH_FEET * 0.3048  # Convert to meters
FIELD_WIDTH_METERS = FIELD_WIDTH_FEET * 0.3048    # Convert to meters
TARGET_X_METERS = 4.625
TARGET_Y_METERS = 4.025

ROBOT_SIZE_INCHES = 27
ROBOT_SIZE_METERS = ROBOT_SIZE_INCHES * 0.0254  # Convert to meters

# Launcher position (in robot coordinates - back corner, 8 inches from each side)
# Robot frame: +X is forward, +Y is left, origin at robot center
# Back-left corner is at (-X, +Y)
LAUNCHER_OFFSET_X_INCHES = -(13.5 - 8)  # Back of robot: -13.5 + 8 inches = -5.5 inches (back)
LAUNCHER_OFFSET_Y_INCHES = (0) 
LAUNCHER_OFFSET_X_METERS = LAUNCHER_OFFSET_X_INCHES * 0.0254
LAUNCHER_OFFSET_Y_METERS = LAUNCHER_OFFSET_Y_INCHES * 0.0254

MAX_SPEED = 5.0  # m/s
ACCELERATION = 5.0  # m/s^2
DECELERATION_MULTIPLIER = 2.0  # Deceleration is this times faster than acceleration

# Calculate screen size based on field aspect ratio
# Use a comfortable viewing size
SCREEN_HEIGHT = 800
SCREEN_WIDTH = int(SCREEN_HEIGHT * (FIELD_LENGTH_METERS / FIELD_WIDTH_METERS))

# Calculate pixels per meter to fit field exactly to screen
PIXELS_PER_METER = SCREEN_WIDTH / FIELD_LENGTH_METERS

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
PURPLE = (255, 0, 255)
ORANGE = (255, 165, 0)
CYAN = (0, 255, 255)

# Virtual goal constants
VIRTUAL_GOAL_LOOKAHEAD_TIME = 1.0  # seconds - how far ahead to project the virtual goal
VIRTUAL_GOAL_SPEED_LIMIT = 0.25  # Lock speed to 25% of max when targeting


@dataclass
class RobotState:
    """Robot state class to track pose and velocity"""
    x: float = FIELD_LENGTH_METERS / 2  # meters (center of field)
    y: float = FIELD_WIDTH_METERS / 2   # meters (center of field)
    theta: float = 0.0  # radians (rotation, 0 is pointing right(+X), pi/2 is pointing up(+Y))
    vx: float = 0.0  # m/s (velocity in field frame - X direction (horizontal, right is positive))
    vy: float = 0.0  # m/s (velocity in field frame - Y direction (vertical, up is positive))
    omega: float = 0.0  # rad/s (angular velocity)


class SwerveRobotSimulator:
    def __init__(self):
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Swerve Drive Robot Simulator")
        self.clock = pygame.time.Clock()
        self.running = True
        self.dt = 0.02  # 50 Hz update rate
        
        # Load field image
        self.field_image = pygame.image.load("assets/field.png")
        # Scale field image to fit the screen exactly
        self.field_image = pygame.transform.scale(self.field_image, (SCREEN_WIDTH, SCREEN_HEIGHT))
        
        # Robot state
        self.robot = RobotState()
        
        # Control inputs
        self.forward_input = 0.0
        self.strafe_input = 0.0
        self.rotate_input = 0.0
        
        # Virtual goal targeting
        self.virtual_goal_active = False
        self.real_goal_x = TARGET_X_METERS
        self.real_goal_y = TARGET_Y_METERS
        self.virtual_goal_x = self.real_goal_x
        self.virtual_goal_y = self.real_goal_y
        self.locked_speed = 0.0
        
        # Font for displaying information
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
    def world_to_screen(self, x, y):
        """Convert world coordinates (meters) to screen coordinates (pixels)
        Bottom-left is (0,0) in world, forward (up) and left are positive"""
        screen_x = x * PIXELS_PER_METER
        screen_y = SCREEN_HEIGHT - (y * PIXELS_PER_METER)  # Flip Y axis
        return int(screen_x), int(screen_y)
    
    def handle_input(self):
        """Handle keyboard input"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    # Toggle virtual goal targeting
                    self.virtual_goal_active = not self.virtual_goal_active
                    if self.virtual_goal_active:
                        # Lock the current speed when activating
                        self.locked_speed = math.sqrt(self.robot.vx**2 + self.robot.vy**2)
                        print(f"Virtual Goal Active - Locked speed: {self.locked_speed:.2f} m/s")
                    else:
                        print("Virtual Goal Deactivated")
                elif event.key == pygame.K_r:
                    # Reset robot to center
                    self.robot.x = FIELD_LENGTH_METERS / 2
                    self.robot.y = FIELD_WIDTH_METERS / 2
                    self.robot.theta = 0.0
                    self.robot.vx = 0.0
                    self.robot.vy = 0.0
                    self.robot.omega = 0.0
                    self.virtual_goal_active = False
                elif event.key == pygame.K_g:
                    # Set real goal to current mouse position
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    self.real_goal_x = mouse_x / PIXELS_PER_METER
                    self.real_goal_y = (SCREEN_HEIGHT - mouse_y) / PIXELS_PER_METER
        
        # Get pressed keys for continuous movement
        keys = pygame.key.get_pressed()
        
        # Forward/Backward (Up/Down arrows)
        if keys[pygame.K_UP]:
            self.forward_input = 1.0
        elif keys[pygame.K_DOWN]:
            self.forward_input = -1.0
        else:
            self.forward_input = 0.0
        
        # Strafe Left/Right (Left/Right arrows)
        if keys[pygame.K_LEFT]:
            self.strafe_input = 1.0  # Left is positive
        elif keys[pygame.K_RIGHT]:
            self.strafe_input = -1.0
        else:
            self.strafe_input = 0.0
        
        # Rotation (Q/W keys)
        if keys[pygame.K_q]:
            self.rotate_input = 1.0  # Counter-clockwise
        elif keys[pygame.K_w]:
            self.rotate_input = -1.0  # Clockwise
        else:
            self.rotate_input = 0.0
    
    def update_robot(self):
        """Update robot state based on inputs (field-centric drive)"""
        # Apply acceleration to velocities (in field frame)
        # In field-centric drive, inputs are relative to the field, not the robot
        
        # Determine max speed based on virtual goal mode
        effective_max_speed = MAX_SPEED
        if self.virtual_goal_active:
            effective_max_speed = MAX_SPEED * VIRTUAL_GOAL_SPEED_LIMIT
        
        target_vx = self.forward_input * effective_max_speed  # Forward on field (X direction)
        target_vy = self.strafe_input * effective_max_speed   # Left on field (Y direction)
        
        # Determine if we're accelerating or decelerating for X
        is_accelerating_x = (abs(target_vx) > abs(self.robot.vx))
        accel_rate_x = ACCELERATION if is_accelerating_x else ACCELERATION * DECELERATION_MULTIPLIER
        
        # Accelerate towards target velocity (X)
        if abs(target_vx - self.robot.vx) > accel_rate_x * self.dt:
            self.robot.vx += np.sign(target_vx - self.robot.vx) * accel_rate_x * self.dt
        else:
            self.robot.vx = target_vx
        
        # Determine if we're accelerating or decelerating for Y
        is_accelerating_y = (abs(target_vy) > abs(self.robot.vy))
        accel_rate_y = ACCELERATION if is_accelerating_y else ACCELERATION * DECELERATION_MULTIPLIER
        
        # Accelerate towards target velocity (Y)
        if abs(target_vy - self.robot.vy) > accel_rate_y * self.dt:
            self.robot.vy += np.sign(target_vy - self.robot.vy) * accel_rate_y * self.dt
        else:
            self.robot.vy = target_vy
        
        # Update angular velocity (instant response for rotation)
        max_angular_velocity = 2 * math.pi  # 1 rotation per second
        self.robot.omega = self.rotate_input * max_angular_velocity
        
        # Update rotation
        self.robot.theta += self.robot.omega * self.dt
        
        # Update position directly using field-frame velocities
        new_x = self.robot.x + self.robot.vx * self.dt
        new_y = self.robot.y + self.robot.vy * self.dt
        
        # Boundary checking (keep robot within field bounds)
        half_robot = ROBOT_SIZE_METERS / 2
        self.robot.x = np.clip(new_x, half_robot, FIELD_LENGTH_METERS - half_robot)
        self.robot.y = np.clip(new_y, half_robot, FIELD_WIDTH_METERS - half_robot)
        
        # Stop velocity if we hit a boundary
        if new_x < half_robot or new_x > FIELD_LENGTH_METERS - half_robot:
            self.robot.vx = 0
        if new_y < half_robot or new_y > FIELD_WIDTH_METERS - half_robot:
            self.robot.vy = 0
        
        # Update virtual goal if active
        if self.virtual_goal_active:
            self.calculate_virtual_goal()
    
    def calculate_virtual_goal(self):
        """Calculate virtual goal position based on robot velocity and locked speed"""
        # The virtual goal should be offset from the real goal based on the robot's velocity
        # This compensates for the robot's movement when shooting
        # If not moving, virtual goal = real goal
        
        # If moving toward the goal, aim closer (subtract velocity offset)
        # If moving away from goal, aim farther (add velocity offset)
        # The projectile inherits the robot's velocity, so we compensate by aiming in the opposite direction
        self.virtual_goal_x = self.real_goal_x - self.robot.vx * VIRTUAL_GOAL_LOOKAHEAD_TIME
        self.virtual_goal_y = self.real_goal_y - self.robot.vy * VIRTUAL_GOAL_LOOKAHEAD_TIME
    
    def get_launcher_world_position(self):
        """Calculate the launcher's position in world coordinates (field frame)"""
        # Launcher position in robot frame
        launcher_x_robot = LAUNCHER_OFFSET_X_METERS
        launcher_y_robot = LAUNCHER_OFFSET_Y_METERS
        
        # Rotate launcher position by robot's rotation to get world position
        cos_theta = math.cos(self.robot.theta)
        sin_theta = math.sin(self.robot.theta)
        
        # Rotation transformation (robot frame to world frame)
        launcher_x_world = self.robot.x + (launcher_x_robot * cos_theta - launcher_y_robot * sin_theta)
        launcher_y_world = self.robot.y + (launcher_x_robot * sin_theta + launcher_y_robot * cos_theta)
        
        return launcher_x_world, launcher_y_world
    
    def calculate_launcher_to_target_distance(self, use_virtual=None):
        """Calculate distance from launcher to target in meters
        
        Args:
            use_virtual: If None, uses self.virtual_goal_active. Otherwise, override with boolean.
        
        Returns:
            float: Distance in meters
        """
        # Get launcher world position
        launcher_x, launcher_y = self.get_launcher_world_position()
        
        # Determine target
        if use_virtual is None:
            use_virtual = self.virtual_goal_active
            
        if use_virtual:
            target_x, target_y = self.virtual_goal_x, self.virtual_goal_y
        else:
            target_x, target_y = self.real_goal_x, self.real_goal_y
        
        # Calculate distance
        dx = target_x - launcher_x
        dy = target_y - launcher_y
        distance = math.sqrt(dx**2 + dy**2)
        
        return distance
    
    def calculate_launcher_to_target_angle(self, use_virtual=None):
        """Calculate angle from launcher to target in robot coordinate system
        
        Robot coordinate system: 0 degrees points forward (in direction of robot's front)
        Positive angles are counter-clockwise
        
        Args:
            use_virtual: If None, uses self.virtual_goal_active. Otherwise, override with boolean.
        
        Returns:
            float: Angle in degrees (0° = forward, 90° = left, -90° = right, 180° = backward)
        """
        # Get launcher world position
        launcher_x, launcher_y = self.get_launcher_world_position()
        
        # Determine target
        if use_virtual is None:
            use_virtual = self.virtual_goal_active
            
        if use_virtual:
            target_x, target_y = self.virtual_goal_x, self.virtual_goal_y
        else:
            target_x, target_y = self.real_goal_x, self.real_goal_y
        
        # Calculate vector from launcher to target in world frame
        dx_world = target_x - launcher_x
        dy_world = target_y - launcher_y
        
        # Calculate angle in world frame using standard atan2(y, x)
        # This gives angle from +X axis (pointing right on field)
        # In our field: +X is right, +Y is up
        angle_world = math.atan2(dy_world, dx_world)
        
        # Convert to robot frame by subtracting robot's rotation
        # Robot theta is measured from +X axis as well
        angle_robot = angle_world - self.robot.theta
        
        # Normalize angle to [-pi, pi]
        angle_robot = math.atan2(math.sin(angle_robot), math.cos(angle_robot))
        
        # Convert to degrees
        angle_degrees = math.degrees(angle_robot)
        
        return angle_degrees
    
    def draw_robot(self):
        """Draw the robot on the screen"""
        center_x, center_y = self.world_to_screen(self.robot.x, self.robot.y)
        
        # Draw robot as a square rotated by theta
        robot_size_pixels = ROBOT_SIZE_METERS * PIXELS_PER_METER
        half_size = robot_size_pixels / 2
        
        # Create robot rectangle points (before rotation)
        points = [
            (-half_size, -half_size),
            (half_size, -half_size),
            (half_size, half_size),
            (-half_size, half_size)
        ]
        
        # Rotate points by theta and translate to robot position
        cos_theta = math.cos(self.robot.theta)
        sin_theta = math.sin(self.robot.theta)
        
        rotated_points = []
        for px, py in points:
            rx = px * cos_theta - py * sin_theta
            ry = px * sin_theta + py * cos_theta
            rotated_points.append((center_x + rx, center_y - ry))  # Subtract ry because screen Y is flipped
        
        # Draw robot body
        pygame.draw.polygon(self.screen, BLUE, rotated_points, 0)
        pygame.draw.polygon(self.screen, WHITE, rotated_points, 2)
        
        # Draw launcher position (small circle in back corner)
        launcher_world_x, launcher_world_y = self.get_launcher_world_position()
        launcher_screen_x, launcher_screen_y = self.world_to_screen(launcher_world_x, launcher_world_y)
        pygame.draw.circle(self.screen, ORANGE, (launcher_screen_x, launcher_screen_y), 6)
        pygame.draw.circle(self.screen, BLACK, (launcher_screen_x, launcher_screen_y), 6, 2)
        
        # Draw front indicator (small line showing robot's forward direction)
        front_length = half_size * 0.8
        front_x = center_x + front_length * cos_theta
        front_y = center_y - front_length * sin_theta  # Negative because screen Y is flipped
        pygame.draw.line(self.screen, YELLOW, (center_x, center_y), (front_x, front_y), 3)
        
        # Draw center point
        pygame.draw.circle(self.screen, RED, (center_x, center_y), 5)
    
    def draw_velocity_vectors(self):
        """Draw velocity vectors from robot center (field-frame velocities)"""
        center_x, center_y = self.world_to_screen(self.robot.x, self.robot.y)
        
        # Velocities are already in field frame, no conversion needed
        field_vx = self.robot.vx
        field_vy = self.robot.vy
        
        # Scale factor for velocity vectors (pixels per m/s)
        vector_scale = 30
        
        # Calculate vector end points
        vec_end_x = center_x + field_vx * vector_scale
        vec_end_y = center_y - field_vy * vector_scale  # Negative because screen Y is flipped
        
        # Draw velocity vector
        if field_vx != 0 or field_vy != 0:
            pygame.draw.line(self.screen, GREEN, (center_x, center_y), (vec_end_x, vec_end_y), 3)
            # Draw arrowhead
            self.draw_arrow_head(center_x, center_y, vec_end_x, vec_end_y, GREEN)
    
    def draw_arrow_head(self, start_x, start_y, end_x, end_y, color):
        """Draw an arrow head at the end of a line"""
        arrow_length = 10
        arrow_angle = math.pi / 6  # 30 degrees
        
        # Calculate angle of the line
        dx = end_x - start_x
        dy = end_y - start_y
        angle = math.atan2(dy, dx)
        
        # Calculate arrow head points
        left_x = end_x - arrow_length * math.cos(angle - arrow_angle)
        left_y = end_y - arrow_length * math.sin(angle - arrow_angle)
        right_x = end_x - arrow_length * math.cos(angle + arrow_angle)
        right_y = end_y - arrow_length * math.sin(angle + arrow_angle)
        
        # Draw arrow head
        pygame.draw.polygon(self.screen, color, [(end_x, end_y), (left_x, left_y), (right_x, right_y)])
    
    def draw_goals(self):
        """Draw real and virtual goal markers"""
        # Draw real goal (red circle with crosshair)
        real_goal_screen = self.world_to_screen(self.real_goal_x, self.real_goal_y)
        pygame.draw.circle(self.screen, RED, real_goal_screen, 12, 3)
        pygame.draw.line(self.screen, RED, 
                        (real_goal_screen[0] - 15, real_goal_screen[1]), 
                        (real_goal_screen[0] + 15, real_goal_screen[1]), 2)
        pygame.draw.line(self.screen, RED, 
                        (real_goal_screen[0], real_goal_screen[1] - 15), 
                        (real_goal_screen[0], real_goal_screen[1] + 15), 2)
        
        # Draw virtual goal if active (purple/magenta circle)
        if self.virtual_goal_active:
            virtual_goal_screen = self.world_to_screen(self.virtual_goal_x, self.virtual_goal_y)
            pygame.draw.circle(self.screen, PURPLE, virtual_goal_screen, 12, 3)
            pygame.draw.line(self.screen, PURPLE, 
                            (virtual_goal_screen[0] - 15, virtual_goal_screen[1]), 
                            (virtual_goal_screen[0] + 15, virtual_goal_screen[1]), 2)
            pygame.draw.line(self.screen, PURPLE, 
                            (virtual_goal_screen[0], virtual_goal_screen[1] - 15), 
                            (virtual_goal_screen[0], virtual_goal_screen[1] + 15), 2)
            
            # Draw line from real goal to virtual goal
            pygame.draw.line(self.screen, CYAN, real_goal_screen, virtual_goal_screen, 2)
    
    def draw_aim_line(self):
        """Draw aim line from launcher to the target goal"""
        launcher_world_x, launcher_world_y = self.get_launcher_world_position()
        launcher_screen = self.world_to_screen(launcher_world_x, launcher_world_y)
        
        # Determine which goal to aim at
        if self.virtual_goal_active:
            target_x, target_y = self.virtual_goal_x, self.virtual_goal_y
            line_color = PURPLE
        else:
            target_x, target_y = self.real_goal_x, self.real_goal_y
            line_color = ORANGE
        
        target_screen = self.world_to_screen(target_x, target_y)
        
        # Draw dashed line from launcher to target
        dx = target_screen[0] - launcher_screen[0]
        dy = target_screen[1] - launcher_screen[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance > 0:
            num_dashes = int(distance / 20)  # Dash every 20 pixels
            for i in range(num_dashes):
                start_ratio = i / num_dashes
                end_ratio = (i + 0.5) / num_dashes
                start_x = launcher_screen[0] + dx * start_ratio
                start_y = launcher_screen[1] + dy * start_ratio
                end_x = launcher_screen[0] + dx * end_ratio
                end_y = launcher_screen[1] + dy * end_ratio
                pygame.draw.line(self.screen, line_color, (start_x, start_y), (end_x, end_y), 2)
    
    def draw_info(self):
        """Draw information text on screen"""
        # Position info on the right side of the screen
        info_x = SCREEN_WIDTH - 300
        info_y = 10
        line_height = 25
        
        # Calculate launcher targeting info
        distance_to_target = self.calculate_launcher_to_target_distance()
        angle_to_target = self.calculate_launcher_to_target_angle()
        distance_inches = distance_to_target * 39.3701  # Convert meters to inches
        
        # Robot state information
        texts = [
            f"Position: ({self.robot.x:.2f}m, {self.robot.y:.2f}m)",
            f"Rotation: {math.degrees(self.robot.theta):.1f}°",
            f"Velocity X (field forward): {self.robot.vx:.2f} m/s",
            f"Velocity Y (field left): {self.robot.vy:.2f} m/s",
            f"Angular Velocity: {math.degrees(self.robot.omega):.1f}°/s",
            f"Speed: {math.sqrt(self.robot.vx**2 + self.robot.vy**2):.2f} m/s",
            f"Drive Mode: Field-Centric",
            "",
            f"LAUNCHER INFO:",
            f"Distance to Target: {distance_to_target:.2f}m ({distance_inches:.1f}in)",
            f"Angle to Target: {angle_to_target:.1f}° (robot frame)",
        ]
        
        # Add virtual goal status
        if self.virtual_goal_active:
            max_allowed = MAX_SPEED * VIRTUAL_GOAL_SPEED_LIMIT
            texts.append("")
            texts.append(f"VIRTUAL GOAL ACTIVE (Max Speed: {max_allowed:.2f} m/s)")
            texts.append(f"Virtual Goal: ({self.virtual_goal_x:.2f}m, {self.virtual_goal_y:.2f}m)")
        
        texts.append("")
        texts.append(f"Real Goal: ({self.real_goal_x:.2f}m, {self.real_goal_y:.2f}m)")
        
        for i, text in enumerate(texts):
            if "VIRTUAL GOAL ACTIVE" in text:
                color = PURPLE
            elif "LAUNCHER INFO" in text:
                color = ORANGE
            else:
                color = WHITE
            surface = self.font.render(text, True, color)
            self.screen.blit(surface, (info_x, info_y + i * line_height))
        
        # Controls information
        controls_y = info_y + len(texts) * line_height + 20
        control_texts = [
            "Controls:",
            "  ↑↓ - Forward/Backward",
            "  ←→ - Strafe Left/Right",
            "  Q/W - Rotate CCW/CW",
            "  SPACE - Toggle Virtual Goal",
            "  G - Set Real Goal (at mouse)",
            "  R - Reset Robot",
            "  ESC - Exit"
        ]
        
        for i, text in enumerate(control_texts):
            surface = self.small_font.render(text, True, YELLOW)
            self.screen.blit(surface, (info_x, controls_y + i * 20))
    
    def run(self):
        """Main game loop"""
        while self.running:
            self.handle_input()
            self.update_robot()
            
            # Draw everything
            self.screen.fill(BLACK)
            
            # Draw field image
            self.screen.blit(self.field_image, (0, 0))
            
            # Draw goals and aim line
            self.draw_goals()
            self.draw_aim_line()
            
            # Draw robot and vectors
            self.draw_velocity_vectors()
            self.draw_robot()
            
            # Draw info overlay
            self.draw_info()
            
            pygame.display.flip()
            self.clock.tick(50)  # 50 FPS
        
        pygame.quit()


if __name__ == "__main__":
    simulator = SwerveRobotSimulator()
    simulator.run()


#  Look ahead time:
# The lookahead time represents how long you expect the projectile to be in flight before it reaches the goal.
# It's used to calculate how far your robot will move during that time, which determines the compensation needed.

# Virtual Goal Offset = Robot Velocity × Lookahead Time
# Examples with different lookahead times:
# Scenario: Moving forward at 1.25 m/s

# Lookahead Time = 0.5 seconds (fast projectile):

# Offset = 1.25 m/s × 0.5s = 0.625 meters
# Virtual goal is 0.625m closer than real goal
# Less compensation needed (projectile reaches quickly)
# Lookahead Time = 1.0 seconds (current setting):

# Offset = 1.25 m/s × 1.0s = 1.25 meters
# Virtual goal is 1.25m closer than real goal
# Medium compensation
# Lookahead Time = 2.0 seconds (slow/arcing projectile):

# Offset = 1.25 m/s × 2.0s = 2.5 meters
# Virtual goal is 2.5m closer than real goal
# More compensation needed (projectile takes longer to reach)
# Practical Usage:
# Shorter lookahead time = Less offset = Use for fast, direct shots
# Longer lookahead time = More offset = Use for slow, arcing shots (like lobbing game pieces)