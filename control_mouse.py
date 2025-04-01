"""
MouseController with sliding window input filtering
"""

import pygame
import threading
from collections import deque
from connect import RobotConnect
from arm_mover import ArmMover

class MouseController:
    def __init__(self, robot_connection):
        self.mover = ArmMover(robot_connection)
        pygame.init()
        
        # Window setup
        self.screen = pygame.display.set_mode((300, 300))
        pygame.display.set_caption("Arm Control")
        self.font = pygame.font.SysFont('Arial', 20)
        
        # Control parameters
        self.linear_speed = 1
        self.angular_speed = 0.5
        
        # Sliding window parameters
        self.window_size = 5  # Number of samples to consider
        self.x_window = deque(maxlen=self.window_size)
        self.y_window = deque(maxlen=self.window_size)
        
        self.running = True
        self.left_down = False

    def _get_majority_value(self, window):
        """Returns the most common value (-1, 0, or 1) in the window"""
        if not window:
            return 0
        return max(set(window), key=window.count)

    def _process_movement(self, dx, dy):
        """Process movement with sliding window filtering"""
        # Convert to binary values
        x_val = 1 if dx > 5 else (-1 if dx < -5 else 0)
        y_val = 1 if dy > 5 else (-1 if dy < -5 else 0)
        
        # Update sliding windows
        self.x_window.append(x_val)
        self.y_window.append(y_val)
        
        # Get majority values
        final_x = self._get_majority_value(self.x_window)
        final_y = self._get_majority_value(self.y_window)
        
        return final_x, final_y

    def control_loop(self):
        """Main control thread"""
        clock = pygame.time.Clock()
        
        while self.running:
            # Clear screen and show mode
            self.screen.fill((240, 240, 240))
            mode = "ROTATION" if self.left_down else "TRANSLATION"
            text = self.font.render(f"Mode: {mode}", True, (0, 0, 0))
            self.screen.blit(text, (10, 10))
            pygame.display.flip()
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                
                elif event.type == pygame.MOUSEMOTION:
                    dx, dy = self._process_movement(*event.rel)
                    
                    if self.left_down:  # Rotation mode
                        self.mover.set_cartesian_velocity(
                            angular_x=dy * self.angular_speed,
                            angular_y=dx * self.angular_speed
                        )
                    else:  # Translation mode
                        self.mover.set_cartesian_velocity(
                            linear_x=dx * self.linear_speed,
                            linear_y=-dy * self.linear_speed
                        )
                
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self.left_down = True
                        self.x_window.clear()
                        self.y_window.clear()
                
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        self.left_down = False
                        self.mover.set_cartesian_velocity(angular_x=0, angular_y=0)
            
            clock.tick(60)

    def run(self):
        print("Control Active:\n"
              "- Mouse: X/Y translation\n"
              "- Left-click + Mouse: X/Y rotation\n"
              "Close window to quit")
        
        control_thread = threading.Thread(target=self.control_loop)
        control_thread.start()
        
        try:
            while self.running:
                pygame.time.wait(100)
        except KeyboardInterrupt:
            self.running = False
        finally:
            control_thread.join()
            pygame.quit()
            self.mover.stop()

if __name__ == "__main__":
    rc = RobotConnect("192.168.2.10", 10000, ("acyu", "kinova"))
    rc.create_connection()
    controller = MouseController(rc)
    controller.run()
    rc.close_connection()
