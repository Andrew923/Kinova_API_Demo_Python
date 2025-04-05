"""
Drag-based MouseController with fixed coordinates and full rotation control
"""

import pygame
import math
from connect import RobotConnect
from arm_mover import ArmMover

class MouseController:
    def __init__(self, robot_connection):
        self.mover = ArmMover(robot_connection)
        pygame.init()
        
        # Window setup
        self.screen = pygame.display.set_mode((600, 600))
        pygame.display.set_caption("Arm Control")
        self.font = pygame.font.SysFont('Arial', 20)
        
        # Control parameters
        self.max_linear_speed = 1
        self.max_angular_speed = 1
        self.scroll_sensitivity = 0.5
        self.angular_scroll_sensitivity = 1
        self.gripper_speed = 1
        
        # Control state
        self.left_dragging = False
        self.right_dragging = False
        self.drag_start_pos = (0, 0)
        self.current_drag_pos = (0, 0)
        self.linear_scroll_value = 0
        self.angular_scroll_value = 0
        
        self.running = True

    def _calculate_velocity(self, start, current, max_speed):
        dy = start[0] - current[0]
        dx = start[1] - current[1]
        distance = math.hypot(dx, dy)
        
        if distance == 0:
            return (0, 0)
            
        direction = (dx/distance, dy/distance)
        speed = min(distance / 150, 1) * max_speed  # Normalized to window size
        return (direction[0] * speed, direction[1] * speed)

    def control_loop(self):
        """Main control loop"""
        # Move to initial position
        self.mover.object_tracking_position()

        clock = pygame.time.Clock()
        
        while self.running:
            self.screen.fill((240, 240, 240))
            
            # Handle events
            gripper = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # print(event.button)
                    if event.button == 1:  # Left click
                        self.left_dragging = True
                        self.drag_start_pos = event.pos
                        self.current_drag_pos = event.pos
                    elif event.button == 3:  # Right click
                        self.right_dragging = True
                        self.drag_start_pos = event.pos
                        self.current_drag_pos = event.pos
                    elif event.button == 6:
                        self.mover.move_gripper(self.gripper_speed)  # Open gripper
                        gripper = True
                    elif event.button == 7:
                        self.mover.move_gripper(-self.gripper_speed)  # Close gripper
                        gripper = True
                
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        self.left_dragging = False
                        self.mover.set_cartesian_velocity(0, 0, 0)
                    elif event.button == 3:
                        self.right_dragging = False
                        self.mover.set_cartesian_velocity(angular_x=0, angular_y=0, angular_z=0)
                    elif event.button == 6 or event.button == 7:
                        self.mover.move_gripper(0)
                
                elif event.type == pygame.MOUSEMOTION:
                    if self.left_dragging or self.right_dragging:
                        self.current_drag_pos = event.pos
                
                elif event.type == pygame.MOUSEWHEEL:
                    if self.left_dragging:
                        self.linear_scroll_value += event.y * self.scroll_sensitivity
                    elif self.right_dragging:
                        self.angular_scroll_value += event.y * self.angular_scroll_sensitivity

            # Handle gripper state
            if not gripper:
                self.mover.move_gripper(0)
            
            # Update velocities
            if self.left_dragging:
                # Draw translation visualization
                pygame.draw.circle(self.screen, (0, 150, 0), self.drag_start_pos, 8)
                pygame.draw.line(self.screen, (0, 0, 0), 
                               self.drag_start_pos, self.current_drag_pos, 3)
                
                # Linear velocity calculation
                vx, vy = self._calculate_velocity(
                    self.drag_start_pos, self.current_drag_pos, self.max_linear_speed
                )
                vz = self.linear_scroll_value  # Capture before reset
                self.mover.set_cartesian_velocity(
                    linear_x=vx,
                    linear_y=vy,
                    linear_z=vz
                )
                
                # Show Z-axis status
                text = self.font.render(f"Z-translate: {vz:.2f} m/s", True, (0, 0, 0))
                self.screen.blit(text, (10, 40))
                
            elif self.right_dragging:
                # Draw rotation visualization
                pygame.draw.circle(self.screen, (150, 0, 0), self.drag_start_pos, 8)
                pygame.draw.line(self.screen, (0, 0, 0), 
                               self.drag_start_pos, self.current_drag_pos, 3)
                
                # Angular velocity calculation
                rx, ry = self._calculate_velocity(
                    self.drag_start_pos, self.current_drag_pos, self.max_angular_speed
                )
                angular_z = self.angular_scroll_value  # Capture before reset
                self.mover.set_cartesian_velocity(
                    angular_x=ry,  # Pitch
                    angular_y=rx,  # Roll
                    angular_z=angular_z  # Yaw from scroll
                )
                
                # Show rotation status
                text = self.font.render(f"Z-rotate: {angular_z:.1f}°/s", True, (0, 0, 0))
                self.screen.blit(text, (10, 40))
            else:
                self.linear_scroll_value = 0
                self.angular_scroll_value = 0
                self.mover.set_cartesian_velocity()
            
            # Draw help text
            help_text = [
                "Left-drag: X/Y Translation (+ scroll for Z)",
                "Right-drag: X/Y Rotation (+ scroll for Z-rotation)",
                "Mouse4: Open gripper | Mouse5: Close gripper",
                "Click and drag in window to control",
                "Close window to quit"
            ]
            for i, text in enumerate(help_text):
                surface = self.font.render(text, True, (0, 0, 0))
                self.screen.blit(surface, (10, 570 - (len(help_text)-i-1)*20))
            
            pygame.display.flip()
            clock.tick(60)

    def run(self):
        print("Drag Control Active:")
        print("- Left-drag: X/Y translation + scroll for Z")
        print("- Right-drag: X/Y rotation + scroll for Z-rotation")
        
        self.control_loop()  # Run in main thread
        
        pygame.quit()
        self.mover.stop()

if __name__ == "__main__":
    rc = RobotConnect("192.168.2.10", 10000, ("acyu", "kinova"))
    rc.create_connection()
    controller = MouseController(rc)
    controller.run()
    rc.close_connection()