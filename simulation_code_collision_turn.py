from coppeliasim_zmqremoteapi_client import *
import time
import math
import random

DEG_TO_RAD = math.pi / 180

# Connecting via ZMQ API at port 23000
client = RemoteAPIClient()
# Getting simulation handles
sim = client.getObject('sim')
sim.startSimulation()

# Get the handle of left and right motors
motorLeft = sim.getObject('./leftMotor')
motorRight = sim.getObject('./rightMotor')

# Get the handle for the forward sensor (sensor 3)
ultrasonicSensor = sim.getObject('./ultrasonicSensor[3]')

# Increased robot speed for faster movement
robotSpeed = 50 * DEG_TO_RAD  # Increased speed
turnSpeed = 40 * DEG_TO_RAD  # Adjusted turn speed

# Define the threshold velocity to detect if the robot has stopped
stopVelocityThreshold = 0.1  # If the robot's velocity is below this threshold, it's considered stopped
collisionDistanceThreshold = 0.1  # Set the collision distance threshold for detecting collision

# Function to check the robot's current velocity
def get_robot_velocity():
    left_velocity = sim.getJointTargetVelocity(motorLeft)  # Get velocity of the left motor
    right_velocity = sim.getJointTargetVelocity(motorRight)  # Get velocity of the right motor
    return (left_velocity + right_velocity) / 2  # Average velocity of both motors

# Function to check if collision happened based on proximity sensor reading
def is_collision():
    res, dist, _, _, _ = sim.readProximitySensor(ultrasonicSensor)  # Get the sensor data
    if res == 1 and dist < collisionDistanceThreshold:  # If object detected and distance is small enough
        return True
    return False

# Move the robot forward
print('Robot is moving forward...')
sim.setJointTargetVelocity(motorLeft, robotSpeed)
sim.setJointTargetVelocity(motorRight, robotSpeed)

while True:
    current_velocity = get_robot_velocity()
    
    # Print velocity for debugging
    print(f"Current Velocity: {current_velocity}")
    
    if is_collision():  # Check if collision has happened
        print('Collision detected! Robot stopped.')

        # Stop the robot
        sim.setJointTargetVelocity(motorLeft, 0)
        sim.setJointTargetVelocity(motorRight, 0)
        time.sleep(1)  # Wait for 1 second before moving

        # Perform a random turn (between 150 and 210 degrees)
        random_turn_angle = random.uniform(150, 210)  # Random angle between 150 and 210 degrees
        print(f'Turning {random_turn_angle} degrees...')
        
        # Turn randomly by the chosen angle
        sim.setJointTargetVelocity(motorLeft, turnSpeed * (random_turn_angle / abs(random_turn_angle)))
        sim.setJointTargetVelocity(motorRight, -turnSpeed * (random_turn_angle / abs(random_turn_angle)))
        time.sleep(abs(random_turn_angle) / 180 * 2)  # Adjust sleep time based on turn angle

        # Move forward after turning
        print('Resuming movement...')
        sim.setJointTargetVelocity(motorLeft, robotSpeed)
        sim.setJointTargetVelocity(motorRight, robotSpeed)

    time.sleep(0.1)  # Small delay to reduce simulation load

sim.stopSimulation()
