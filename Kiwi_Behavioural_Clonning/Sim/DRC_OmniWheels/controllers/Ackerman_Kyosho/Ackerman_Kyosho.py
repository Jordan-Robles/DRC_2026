"""Kiwi_Control controller - RC Steering Test"""
from controller import Robot
import cv2


car = Robot()
keyboard = Keyboard()

TIME_STEP = int(driver.getBasicTimeStep())
keyboard.enable(TIME_STEP)

speed = 0
steering = 0

while driver.step() != -1:

    key = keyboard.getKey()

    # Forward
    if key == Keyboard.UP:
        speed += 2

    # Reverse
    elif key == Keyboard.DOWN:
        speed -= 2

    # Left
    elif key == Keyboard.LEFT:
        steering += 0.05

    # Right
    elif key == Keyboard.RIGHT:
        steering -= 0.05

    # Clamp steering
    steering = max(min(steering, 0.6), -0.6)

    # Apply controls
    driver.setCruisingSpeed(speed)
    driver.setSteeringAngle(steering)