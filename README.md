

# Mobile Robot - Obstacle Avoidance and Collision Detection

## Project Overview

This project involves programming a **mobile robot** to perform **collision detection** and **obstacle avoidance** using ultrasonic sensors. The robot uses two primary algorithms:

1. **Collision Detection with Random Turn**: The robot detects obstacles and, upon collision, stops, performs a random turn, and resumes movement.
2. **Obstacle Avoidance Using a Braitenberg-Like Approach**: The robot adjusts its motor speeds dynamically based on proximity sensor data to avoid obstacles.

### Key Concepts Covered:

* **Ultrasonic Sensor-Based Obstacle Detection**
* **Random Turn for Collision Handling**
* **Braitenberg-Like Algorithm for Dynamic Navigation**

The robot autonomously navigates and avoids obstacles by adjusting its motor speeds based on real-time sensor data.

## Files Overview

### 1. **simulation\_code\_collision\_turn.py**

This Python script integrates with the **CoppeliaSim** simulation environment to:

* Move the robot forward.
* Stop the robot when a collision is detected using ultrasonic sensors.
* Perform a **random turn** after a collision.
* Resume movement after the turn.

Key features include:

* Collision detection with an ultrasonic sensor.
* Random turn functionality based on collision.
* Movement control with adjustable speeds.

### 2. **simulation\_obstacle\_avoidance.py**

This Python script uses the **Braitenberg-Like Approach** to continuously move the robot while avoiding obstacles:

* The robot’s behavior is influenced by real-time data from **16 ultrasonic sensors**.
* The script adjusts motor speeds dynamically based on sensor readings, enabling obstacle avoidance.

Key features include:

* Continuous movement while avoiding obstacles.
* Braitenberg-like sensor-to-motor coupling for reactive behavior.

### 3. **CoppeliaSim Scene File (`pioneer_p3dx_wall.ttt`)**

The **CoppeliaSim `.ttt` scene file** is required to run the simulation with the robot in the virtual environment. This file should be loaded in **CoppeliaSim** before running the Python scripts.

---

## Installation Instructions

### Prerequisites

1. **CoppeliaSim**: Download and install **CoppeliaSim** for robot simulation.
2. **Python 3.x**: Ensure Python is installed.
3. **Required Python Libraries**:

   * **CoppeliaSim ZMQ Remote API**: For communication between Python and CoppeliaSim.
   * **NumPy**: Install it using `pip install numpy`.
   * **Time & Math Libraries**: These are part of Python's standard library.

### Steps for Running the Simulation

1. **Set Up CoppeliaSim**:

   * Open **CoppeliaSim** and **load the `pioneer_p3dx_wall.ttt` scene** file. This file contains the robot model and the environment setup.

     * In CoppeliaSim, go to **File** → **Open Scene** and select the `pioneer_p3dx_wall.ttt` file.

2. **Clone or Download the Repository**:

   * Clone the repository or download the files to your local machine.

3. **Run the Python Script**:

   * Open a terminal, navigate to the project directory, and run either `simulation_code_collision_turn.py` or `simulation_obstacle_avoidance.py`.

   ```bash
   python simulation_code_collision_turn.py
   ```

4. **Observe the Robot's Behavior**:

   * The robot should start moving, detecting obstacles, and performing random turns or avoiding obstacles based on the algorithm implemented.

---

## Code Explanation

### 1. **Collision Detection and Random Turn** (`simulation_code_collision_turn.py`)

* **Initialization**: The robot starts by moving forward.
* **Collision Detection**: The robot uses an ultrasonic sensor to detect obstacles. If an obstacle is detected within a threshold, the robot stops and performs a random turn.
* **Random Turn**: The robot turns randomly between 150° and 210° after a collision and resumes forward movement.

### 2. **Obstacle Avoidance Using Braitenberg Algorithm** (`simulation_obstacle_avoidance.py`)

* **Initialization**: The robot starts moving with a predefined speed.
* **Obstacle Detection**: The script checks 16 ultrasonic sensors for obstacles. The closer the obstacle, the stronger the sensor's reading.
* **Motor Speed Adjustment**: The motor speeds are calculated based on the Braitenberg algorithm, which adjusts motor speeds dynamically to avoid obstacles.

### Key Functions:

* `check_for_obstacle()`: Reads ultrasonic sensor values to detect obstacles.
* `calculate_motor_speeds()`: Uses the Braitenberg-like logic to calculate left and right motor speeds based on sensor input.

---

## Testing and Evaluation

### **Collision Detection and Random Turn**:

* Test the robot's ability to detect obstacles, stop, perform a random turn, and resume movement.

### **Obstacle Avoidance**:

* Test the robot's ability to navigate dynamically while avoiding obstacles using the Braitenberg-like approach.

---

## Conclusion

This project demonstrates **reactive behavior** in a mobile robot using real-time data from ultrasonic sensors to avoid obstacles and navigate autonomously. It utilizes both a **collision detection system with random turns** and a **Braitenberg-like algorithm** for continuous movement and obstacle avoidance.



