AURO


This project implements an autonomous robot system using ROS2 to simulate and evaluate item detection, navigation, and delivery in dynamic environments. The solution supports single and multi-robot configurations, random seed variations, and zone deactivations, allowing for flexible and scalable testing scenarios.


Setup Instructions

        1-) Clone the Repository        
           * Clone the project files into the src folder of your ROS2 workspace.

        2-) Build the Workspace
           * To prepare the workspace, navigate to your ROS2 workspace folder. Use colcon to build the workspace and ensure it is properly sourced before running the simulation.

        3-) Launch the Simulation
           * To start the system, use the following command:
           ‘ros2 launch solution solution_nav2_launch.py’

Running the Scenarios

Below are the available scenarios and the corresponding commands to run them:
        1-) Single Robot
           * Launch the simulation with one robot:
           ‘ros2 launch solution solution_nav2_launch.py’

        2-) Two Robots
           * Launch the simulation with two robots:
           ‘ros2 launch solution solution_nav2_launch.py num_robots:=2’
        
        3-) Three Robots
           * Launch the simulation with three robots:
           ‘ros2 launch solution solution_nav2_launch.py num_robots:=3’
        
        4-) Random Seed Variation
           * Run the simulation with a specific random seed to vary item placement:
           ‘ros2 launch solution solution_nav2_launch.py random_seed:=3’

        5-) Single Zone Deactivation
           * Deactivate the bottom-left zone:
           ‘ros2 launch solution solution_nav2_launch.py zone_bottom_left:=false’

        6-) Two Zone Deactivation with Two Robots
           * Deactivate the bottom-left and top-left zones while using two robots:
           ‘ros2 launch solution solution_nav2_launch.py zone_bottom_left:=false zone_top_left:=false num_robots:=2’
        
        7-) Odometry Source Variation
           * Change the odometry source to world-based:
           ‘ros2 launch solution solution_nav2_launch.py odometry:=world’