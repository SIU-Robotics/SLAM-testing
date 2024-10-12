## How to create a LIDAR mapping and localization using SLAM (Simultaneous Localization and Mapping) techniques.
## ROS Autonomous Mode: Random Walk Script
### Used GMapping package and TurtleBot3 for simulation
#### Steps:
1. Create and build your ROS package.
2. Set up the workspace and dependencies.
3. Create and make the random_walk.py script executable.
4. Run the ROS Master (roscore).
5. Launch the random_walk.py script using rosrun to enable autonomous movement.


### Create a Catkin Workspace
```bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# Source the ROS Environment
# After building the workspace, make sure to source your ROS environment:
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# To avoid sourcing every time, add the following lines to your .bashrc file:
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Initialize rosdep
# rosdep helps you install system dependencies for your ROS packages:
sudo rosdep init
rosdep update


###  Creating a ROS Package
```bash
# Navigate to Your Catkin Workspace:
cd ~/catkin_ws/src

# Create a new ROS package named my_robot_package (you can choose a different name) with dependencies on rospy and geometry_msgs:
catkin_create_pkg my_robot_package rospy geometry_msgs

# Create a Scripts Directory in Your Package:
# Inside the newly created package, create a scripts directory to hold your Python scripts:
cd ~/catkin_ws/src/my_robot_package
mkdir scripts

### Creating the random_walk.py Script

# Create and Edit the Script:
# Open a text editor to create a new file named random_walk.py:
nano ~/catkin_ws/src/my_robot_package/scripts/random_walk.py

# Make the Script Executable:
# Once the script is created, make it executable:
chmod +x ~/catkin_ws/src/my_robot_package/scripts/random_walk.py

# Ensure Python 3 Compatibility:
# Make sure the first line of your random_walk.py script reads:
# #!/usr/bin/env python3

### Building the Workspace

# Navigate to the Workspace Root:
cd ~/catkin_ws

# Build the Workspace:
# Build the catkin workspace to ensure everything is linked properly:
catkin_make

# Source the Workspace:
# After building, source the workspace again to ensure the environment is updated:
source devel/setup.bash

### Running the Autonomous Mode Script

# Start ROS Master:
# Before running any ROS nodes, you need to start the ROS Master (roscore):
roscore

# Open a new terminal and proceed with the next steps while roscore is running.

# Run the Random Walk Script:
# In a new terminal, source the workspace and run the Python script:
source ~/catkin_ws/devel/setup.bash
rosrun my_robot_package random_walk.py

# This command will launch the autonomous mode where the robot will start moving based on the logic defined in the random_walk.py script. (**Logic can be edited in future as per need**)

# Optional: Visualize in RViz

# Launch RViz:
# To visualize the robot's movement:
rosrun rviz rviz

# Configure RViz:
# In RViz, you can add necessary topics like /cmd_vel and robot models to visualize the robot's movement based on velocity commands.
