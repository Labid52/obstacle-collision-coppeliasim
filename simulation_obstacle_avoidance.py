from coppeliasim_zmqremoteapi_client import *
import time
import math
import numpy as np
import random

DEG_TO_RAD = math.pi / 180

# Connecting via ZMQ Api at port 23000
client = RemoteAPIClient()
# Getting simulation handles
sim = client.getObject('sim')
sim.startSimulation()

# Get the handle of left and right motors
motorLeft = sim.getObject('./leftMotor')
motorRight = sim.getObject('./rightMotor')

# Get the handles for the ultrasonic sensors
ultrasonicSensors = []
for i in range(16):
    sensor_handle = sim.getObject(f'./ultrasonicSensor[{i}]')
    ultrasonicSensors.append(sensor_handle)

# Set the robot speed for linear motion
robotSpeed = 30 * DEG_TO_RAD  # Forward speed for the robot

# Constants for the Braitenberg-like algorithm
noDetectionDist = 0.5  # Max distance to consider an obstacle
maxDetectionDist = 0.2  # Minimum distance before we react
braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
v0 = 2  # Base speed

# Function to check for obstacles using ultrasonic sensors and visualize the active sensor
def check_for_obstacle():
    detect = [0] * 16
    for i in range(16):
        res, dist, _, _, _ = sim.readProximitySensor(ultrasonicSensors[i])  # Unpack five values, but we use only res and dist
        if res > 0 and dist < noDetectionDist:
            if dist < maxDetectionDist:
                dist = maxDetectionDist
            detect[i] = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
        else:
            detect[i] = 0
    return detect

# Function to calculate motor velocities using Braitenberg-like approach
def calculate_motor_speeds(detect):
    vLeft = v0
    vRight = v0
    for i in range(16):
        vLeft += braitenbergL[i] * detect[i]
        vRight += braitenbergR[i] * detect[i]
    return vLeft, vRight

# Main loop: move the robot, avoid obstacles, and keep navigating
while True:
    # Get sensor readings and calculate obstacle detection
    detect = check_for_obstacle()
    
    # Calculate motor velocities based on Braitenberg algorithm
    vLeft, vRight = calculate_motor_speeds(detect)
    
    # Apply the calculated velocities to the motors
    sim.setJointTargetVelocity(motorLeft, vLeft)
    sim.setJointTargetVelocity(motorRight, vRight)

    # Continue moving forward for a while
    time.sleep(0.1)

# Stop the robot when finished
sim.setJointTargetVelocity(motorLeft, 0)
sim.setJointTargetVelocity(motorRight, 0)

sim.stopSimulation()
