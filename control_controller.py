"""
Xbox Controller Interface (Main Thread Version)
"""

import pygame
from connect import RobotConnect
from arm_mover import ArmMover

class XboxController:
    def __init__(self, robot_connection):
        self.mover = ArmMover(robot_connection)
        pygame.init()
        
        # Controller setup
        pygame.joystick.init()
        self.joystick = None
        self.running = True
        
        # Control parameters
        self.linear_sensitivity = 2
        self.angular_sensitivity = 1
        self.gripper_speed = 0.7
        self.deadzone = 0.1
        
        # Initialize controller
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Controller connected: {self.joystick.get_name()}")
        else:
            print("No controller found!")
            self.running = False

    def _apply_deadzone(self, value):
        return value if abs(value) > self.deadzone else 0

    def run(self):
        if not self.joystick:
            return

        print("Xbox Control Active:")
        print("Left Stick: X/Y Translation")
        print("Triggers: Z Translation (RT-up, LT-down)")
        print("Right Stick: X/Y Rotation")
        print("Bumpers: Z Rotation (RB-CW, LB-CCW)")
        print("A/B: Open/Close Gripper")
        print("Back: Exit")

        # Move to initial position
        self.mover.object_tracking_position()

        clock = pygame.time.Clock()
        
        while self.running:
            # Process events in main thread
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            
            # Get controller state
            axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
            buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

            # --- Translation Controls ---
            lx = -self._apply_deadzone(axes[1])
            ly = -self._apply_deadzone(axes[0])
            lt = (axes[4] + 1) / 2  # Left trigger [0, 1]
            rt = (axes[5] + 1) / 2  # Right trigger [0, 1]
            lz = rt - lt  # Combined Z axis

            # --- Rotation Controls ---
            rx = -self._apply_deadzone(axes[3])
            ry = self._apply_deadzone(axes[2])
            lb = buttons[4]
            rb = buttons[5]
            rz = rb - lb

            self.mover.set_cartesian_velocity(
                linear_x=lx * self.linear_sensitivity,
                linear_y=ly * self.linear_sensitivity,
                linear_z=lz * self.linear_sensitivity,
                angular_x=ry * self.angular_sensitivity,
                angular_y=rx * self.angular_sensitivity,
                angular_z=rz * self.angular_sensitivity
            )

            # --- Gripper Control ---
            if buttons[0]:  # A button
                self.mover.move_gripper(self.gripper_speed)
            elif buttons[1]:  # B button
                self.mover.move_gripper(-self.gripper_speed)
            else:
                self.mover.move_gripper(0)

            # Exit condition
            if buttons[6]:  # Back button
                self.running = False

            clock.tick(60)  # 60Hz update rate

        # Cleanup
        pygame.quit()
        self.mover.stop()

if __name__ == "__main__":
    rc = RobotConnect("192.168.2.10", 10000, ("acyu", "kinova"))
    rc.create_connection()
    controller = XboxController(rc)
    controller.run()
    rc.close_connection()