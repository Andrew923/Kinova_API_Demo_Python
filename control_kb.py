"""
Keyboard control for Kinova robotic arm with improved key mapping using Pygame
Spacebar toggles between linear/angular velocity modes
Arrow keys control gripper
"""

import sys
import pygame
from pygame.locals import *
from connect import RobotConnect
from arm_mover import ArmMover

class KeyboardController:
    def __init__(self, robot_connection):
        self.mover = ArmMover(robot_connection)
        self.running = True
        
        # Control parameters
        self.linear = 2
        self.angular = 1
        self.gripper_speed = 2   # gripper speed multiplier
        
        # Control states
        self.velocity_mode = 'linear'  # 'linear' or 'angular'
        self.key_states = {
            'w': False, 'a': False, 's': False, 'd': False,
            'q': False, 'e': False, 
            'up': False, 'down': False
        }
        
    def on_press(self, key):
        try:
            if key == 'space':
                # Toggle between linear and angular velocity modes
                self.velocity_mode = 'angular' if self.velocity_mode == 'linear' else 'linear'
                print(f"Switched to {self.velocity_mode} velocity mode")
            elif key == 'esc':
                self.running = False
            elif key == 'r':
                self.mover.object_tracking_position()
            elif key in ['up', 'down']:
                self.key_states[key] = True
                self._update_gripper()
            elif key in self.key_states:
                self.key_states[key] = True
                self._update_movement()
        except Exception as e:
            print(f"Key press error: {e}")

    def on_release(self, key):
        try:
            if key in ['up', 'down']:
                self.key_states[key] = False
                self._update_gripper()
            elif key in self.key_states:
                self.key_states[key] = False
                self._update_movement()
        except Exception as e:
            print(f"Key release error: {e}")

    def _update_movement(self):
        """Update movement based on current velocity mode"""
        if self.velocity_mode == 'angular':
            # Angular velocity mode (rotation)
            wx = (1 if self.key_states['a'] else 0) + (-1 if self.key_states['d'] else 0)
            wy = (1 if self.key_states['w'] else 0) + (-1 if self.key_states['s'] else 0)
            wz = (1 if self.key_states['q'] else 0) + (-1 if self.key_states['e'] else 0)
            self.mover.set_cartesian_velocity(angular_x=wx * self.angular, angular_y=wy * self.angular, angular_z=wz * self.angular)
        else:
            # Linear velocity mode (translation)
            vx = (1 if self.key_states['w'] else 0) + (-1 if self.key_states['s'] else 0)
            vy = (1 if self.key_states['a'] else 0) + (-1 if self.key_states['d'] else 0)
            vz = (1 if self.key_states['q'] else 0) + (-1 if self.key_states['e'] else 0)
            self.mover.set_cartesian_velocity(linear_x=vx * self.linear, linear_y=vy * self.linear, linear_z=vz * self.linear)

    def _update_gripper(self):
        """Update gripper based on arrow key states"""
        if self.key_states['up'] and self.key_states['down']:
            # Both pressed - stop gripper
            self.mover.move_gripper(0)
        elif self.key_states['up']:
            # Open gripper
            self.mover.move_gripper(self.gripper_speed)
        elif self.key_states['down']:
            # Close gripper
            self.mover.move_gripper(-self.gripper_speed)
        else:
            # No keys pressed - stop gripper
            self.mover.move_gripper(0)

    def run(self):
        print("Starting keyboard control...")
        print("WASD + QE: Linear velocity mode (translation)")
        print("Space: Toggle to angular velocity mode (rotation)")
        print("Up/Down arrows: Control gripper")
        print("R: Reset to tracking position")
        print("ESC: Quit")
        
        # Move to initial position
        self.mover.object_tracking_position()
        
        # Initialize Pygame
        pygame.init()
        screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Kinova Arm Controller")

        key_map = {
            pygame.K_w: 'w',
            pygame.K_a: 'a',
            pygame.K_s: 's',
            pygame.K_d: 'd',
            pygame.K_q: 'q',
            pygame.K_e: 'e',
            pygame.K_UP: 'up',
            pygame.K_DOWN: 'down',
            pygame.K_SPACE: 'space',
            pygame.K_ESCAPE: 'esc',
            pygame.K_r: 'r',
        }

        clock = pygame.time.Clock()

        while self.running:
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False
                elif event.type == KEYDOWN:
                    if event.key in key_map:
                        self.on_press(key_map[event.key])
                elif event.type == KEYUP:
                    if event.key in key_map:
                        self.on_release(key_map[event.key])
            
            clock.tick(60)

        # Cleanup
        pygame.quit()
        self.mover.stop()

if __name__ == "__main__":
    ip = "192.168.2.10"
    port = 10000
    credentials = ("acyu", "kinova")
    
    robot_connection = RobotConnect(ip, port, credentials)
    robot_connection.create_connection()
    
    try:
        controller = KeyboardController(robot_connection)
        controller.run()
    finally:
        robot_connection.close_connection()