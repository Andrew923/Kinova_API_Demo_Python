import pygame
pygame.init()
clock = pygame.time.Clock()
keepPlaying = True

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller connected: {joystick.get_name()}")
else:
    print("No controller found!")
while keepPlaying:
    pygame.event.pump()  # Process event queue
            
    # Get all axis values (adjust indices based on your controller)
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    print("Axes:", axes)
    print("Buttons:", buttons)
    clock.tick(1)