"""Kiwi_Control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import cv2
import numpy as np

if __name__ == "__main__":

    # create the Robot instance.
    kiwi = Robot()
    
    # get the time step of the current world.
    #timestep = int(robot.getBasicTimeStep())
    timestep = 64
    
    camera = kiwi.getDevice('camera')
    camera.enable(timestep)
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    right_motor = kiwi.getMotor('Right_motor')
    left_motor = kiwi.getMotor('Left_motor')
    rear_motor = kiwi.getMotor('Rear_motor')
    
    right_motor.setPosition(float('inf'))
    left_motor.setPosition(float('inf'))
    rear_motor.setPosition(float('inf'))
    
    right_motor.setVelocity(0.0)
    left_motor.setVelocity(0.0)
    rear_motor.setVelocity(0.0)
    
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while kiwi.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        print(camera)
         # --- Camera ---
        image = camera.getImage()
        width = camera.getWidth()
        height = camera.getHeight()

        img = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
        img = img[:, :, :3]  # remove alpha

        cv2.imshow("camera", img)
        cv2.waitKey(1)
    
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        
        # Desired motion
        x = 1.0   # strafe right
        y = 1.0   # forward
        w = 1.0   # rotation
        
        #Kiwi kinematics
        v_rear = x + w
        v_left = -0.5 * x + 0.866 * y + w
        v_right = -0.5 * x - 0.866 * y + w
        
        # Apply to motors
        right_motor.setVelocity(v_right)
        left_motor.setVelocity(v_left)
        
        #v = 0.25 * 6
        rear_motor.setVelocity(v_rear)
        
    cv2.destroyAllWindows()
    
    # Enter here exit cleanup code.
    