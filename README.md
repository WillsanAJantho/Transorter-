# TRANSORTER
Transorter is a system based robot based on arduino for it's main controller and python for it's computer vision system. This robot aims to create an efficient warehouse for the industry offering maximum space utilization.

# Thoeries Applied 
  1. Robot Kinematics & dynamics (3 Link Planar Arm)
  2. Control System (PID)
  3. Machine Learning for Robotics (Computer Vision)

# Main Components
  1. 3d print Filament
  2. Limit switch
  3. Poistional servo 180deg 7kg
  4. Webcam
  5. Rotary Encoder
  6. Continuous servo 20kg
  7. Power Supply
  8. Electromagnet (as the end effector)
  9. custom made Items


# Working Procedure
  1. Camera determines item coordinate
  2. The machine learning algorithm determines the output poistion
  3. microcontroller calculates the angle of movement for each link to reach the initial position
  4. microcontroller moves the links according to the calculation
  5. When the robot arm reaches the arm the end effector is activated to pick up the item
  6. microcontroller calculates the angle of movement for each link to reach the output poisiton
  7. microcontroller moves the links according to the calculation
  8. When the output position is reached the end effector disengages and the robot returns to it's initial position

Watch working Video:
https://drive.google.com/file/d/1aI5gfKcouDieq0nfYjQXC8FpY32KMXMp/view?usp=sharing


