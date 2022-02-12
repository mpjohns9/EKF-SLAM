# THE NUTURTLE_CONTROL PACKAGE
**Author: Marshall Johnson**  

This package contains nodes that are useful to both simulating and 
controlling real-life motion of a turtlebot. 

# Nodes
- turtle_interface: enables control of turtlebot using `geometry_msgs/Twist` via the `cmd_vel` topic  
- odometry: publishes odometry messages and transform  
- circle: publishes `cmd_vel` commands that cause robot to drive in a circle of specified radius and speed  

# Usage Instructions
- The preceding nodes can be run using the command `roslaunch nuturtle_control start_robot.launch`. 
- Additionally, the following arguments are availble to be modified:  
    - cmd_src:  
        - `circle` to start circle node  
        - `teleop` to start turtlebot3 turtlebot3_teleop_key  
    - robot:  
        - `nusim` to start simulator (default)
        - `localhost` to run directly from turtlebot3
        - `<turtlebotname>` to run on specific turtlebot

# Demos
## Forward/Backward

**Real-world:** https://youtu.be/OZtiVYpRCfk

**Simulation:** https://youtu.be/zCq7UXUxPhI  

The final position was recorded as (0, 0.0015, 0)

## Rotation Only

**Real-world:** https://youtu.be/bfPDctbZdXg

**Simulation:** https://youtu.be/L8e5fMKe1tY  

The final position was recorded as (0, 0, 0)


## Circle Path

**Real-world:** https://youtu.be/vQjXq2rNg9s

**Simulation:** https://youtu.be/8U03lX5KjsE  

The final position was recorded as (0.015, -0.07, 0)


## Attempt at Worse Result

**Real-world:** https://youtu.be/dL4bgiMu9FU

**Simulation:** https://youtu.be/hBRwvl_i8C0  

The final position was recorded as (0, 0.032, 0)


