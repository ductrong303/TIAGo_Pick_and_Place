# ROS TIAGo Pick and Place

This project implements a **ROS-based pick-and-place system** for the [PAL Robotics TIAGo robot](https://pal-robotics.com/robots/tiago/). The robot detects an object using an **ArUco marker**, plans a collision-free path using **MoveIt!**, and executes a pick-and-place operation in simulation.  

## Features
- **ArUco marker detection** for object localization  
- **ROS Action Server/Client** architecture for asynchronous control  
- **MoveIt! integration** for motion planning and execution  
- **Gripper control** for picking and placing objects  
- **Simulation testing** in a Construct-based TIAGo environment  

## Technology Stack
- [ROS Noetic](http://wiki.ros.org/noetic)  
- [MoveIt!](https://moveit.ros.org/) for motion planning  
- [OpenCV ArUco](https://docs.opencv.org/3.4/d5/dae/tutorial_aruco_detection.html) for marker detection  
- [RViz](http://wiki.ros.org/rviz) for visualization  

## Project Structure
```
pick_and_place/
├── action/
│   └── MoveArm.action        # Custom action definition
├── scripts/
│   ├── main.py               # Action client (object detection & goal sending)
│   └── pnp_server.py         # Action server (motion planning & execution)
├── CMakeLists.txt
└── package.xml
```

## Setup & Installation
1. Install **ROS Noetic** and dependencies:  
   ```bash
   sudo apt-get install ros-noetic-moveit ros-noetic-aruco-detect
   ```
2. Clone the package into your catkin workspace:  
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/yourusername/ros-tiago-pnp.git
   cd ~/catkin_ws
   catkin_make
   ```
3. Source your workspace:  
   ```bash
   source devel/setup.bash
   ```

## Usage
1. **Start the simulation** (The Construct.ai TIAGo course environment or Gazebo).  
2. **Run the server**:  
   ```bash
   rosrun pick_and_place pnp_server.py
   ```
3. **Run the client**:  
   ```bash
   rosrun pick_and_place main.py
   ```

## How It Works
1. **Marker detection** – The client detects an ArUco marker using the `/aruco_single/pose` topic.  
2. **Goal computation** – Computes entry & pick poses for the TCP.  
3. **Action call** – Sends a goal (PoseStamped) to the action server.  
4. **Motion planning** – Server uses MoveIt! to plan a collision-free path.  
5. **Execution** – Robot moves to entry point → pick → raise → place.  

## Simulation Example
- The simulation is tested with a **tabletop scenario** containing three objects.  
- **PlanningScene** in MoveIt! ensures collision-free motion (see RViz visualization).  

## Troubleshooting
- **MoveIt Commander missing on TIAGo**:  
  Installing `moveit_commander` on the real TIAGo (ROS Noetic) caused issues. Assistance from PAL Robotics is required.  
- **ArUco detection**:  
  Real robot could not detect markers due to potential TF timestamp issues.  

## Future Work
- Enable **real robot testing** with functional ArUco detection.  
- Add **navigation** to reposition TIAGo before picking objects.  
- Improve **robustness** for multi-object manipulation.  

## Author
**Duc Trong Nguyen** – M.Sc. Electrical Engineering and Information Technology  
Supervisor: **Prof. Dr.-Eng. Karl Kleinmann**  
