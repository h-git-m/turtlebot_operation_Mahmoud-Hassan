# turtlebot_operation_Mahmoud-Hassan


https://github.com/user-attachments/assets/ba11922e-978e-4856-819b-9de2efcd055f


## 1. Step-by-Step Setup Instructions
1. Add `obstacle_direction_controller` and `obstacle_direction_interfaces` packages to the `src` folder of your ROS2 workspace.
2. Open a terminal and navigate to your workspace root (e.g., from a nested folder, go back two levels).
3. Source your shell configuration to load ROS2 environment variables.
4. Build only the added packages.
5. Source the newly built workspace so ROS2 can find the package's executables.
6. Verify the package's executables are available.
7. In the 3D simulator, launch `turtlebot3_world.launch` to load the robot environment.
8. Run the obstacle_direction_controller node to start the obstacle avoidance mode of the robot. Afterwards, the robot will start moving in the turtlebot3_world while avoiding obstacles collision using LiDAR readings.
9. Open a new terminal and repeat steps 2, 3, and 5 to set up the environment there too.
10. To manuallly override the running obstacle avoidance control, send a service call to the newly opened terminal in step 9 (see Testing section below).
11. Open a third terminal, source the environment, and run `rqt_graph` to visualize the running ROS2 nodes/topics, if needed.

## 2. Commands Used and What They Do
| Command | Description |
|---|---|
| `cd ../..` | Moves up two directory levels, back to the workspace root. |
| `source ~/.bashrc` | Reloads shell configuration, ensuring ROS2 environment variables are set. |
| `colcon build --packages-select obstacle_direction_controller obstacle_direction_interfaces` | Builds only these packages instead of the whole workspace. |
| `source install/setup.bash` | Loads the newly built package into the current terminal's environment so ROS2 can find it. |
| `ros2 pkg executables obstacle_direction_controller` | Lists all runnable executables (nodes) inside the `obstacle_direction_controller` package. |
| `ros2 run obstacle_direction_controller A3_control` | Runs the obstacle_direction_controller node to start the obstacle avoidance mode of the robot by sending `Twist` messages to `/cmd_vel` topic|
| `ros2 service call /set_direction obstacle_direction_interfaces/srv/SetDirection "{direction: 'reverse'}"` | Sends a request using obstacle_direction_interfaces to manually override the current control and change the state to reverse. Another 3 manual override requests are available: 'forward', 'left', and 'right'. |
| `ros2 run rqt_graph rqt_graph` | Opens a graphical tool showing active nodes and topic connections in the ROS2 system. |

## 3. How to Test the Nodes
1. Launch the simulator with `turtlebot3_world.launch` to load the robot.
2. In Terminal 1: run `ros2 run obstacle_direction_controller A3_control` and observe the obstacle avoidance control implemented.
3. In Terminal 2 (after sourcing the workspace again): run `ros2 service call /set_direction obstacle_direction_interfaces/srv/SetDirection "{direction: 'reverse'}"` to change the state of the robot to revese direction instantly. On the other hand, replacing 'reverse' with 'forward' changes the state of the robot to forward direction instantly.
4. In the same terminal, run `ros2 service call /set_direction obstacle_direction_interfaces/srv/SetDirection "{direction: 'left'}"` to change the state of the robot and turn instantly to the left. This will keep running for 2 seconds with triple the normal angular velocity to make the change observable in the simulation. Same applies to 'right' request.
5. Watch the robot in the simulator to confirm its movement matches the requested manual override.
6. In Terminal 3 (after sourcing): run `ros2 run rqt_graph rqt_graph` to visually confirm all connected topics and nodes.

## 4. Expected Output
1. **Terminal 1 (`A3_control` node):**
   - On startup, logs `Direction-Aware Controller Started`.
   - Continuously logs LiDAR sector readings each scan cycle, e.g.:
     
[INFO] [direction_aware_obstacle_avoidance_controller]: F:1.85m | L:2.10m | R:0.95m
 [INFO] [direction_aware_obstacle_avoidance_controller]: ACTION: FORWARD

  - When an obstacle is detected within `0.50m` in front, the state switches and logs a warning, e.g.:
    
[WARN] OBSTACLE: Front 0.42m <= 0.50m, switching to TURN state
 [WARN] ROTATE LEFT until front path is free

  - When a manual override service call is received, logs the request and result, e.g.:
    
[INFO] DIRECTION CHANGE REQUEST: LEFT -> success=True

2. **Terminal 2 (service calls):**
   - Each `ros2 service call` returns a response confirming the override, e.g.:
     
requester: making request: obstacle_direction_interfaces.srv.SetDirection_Request(direction='left')

 response:
 obstacle_direction_interfaces.srv.SetDirection_Response(success=True, message='Switched instantly to LEFT direction')
 - An invalid direction (e.g. `'up'`) returns `success=False` with a message listing the accepted values.

3. **Simulator (Gazebo/turtlebot3_world):**
   - The robot moves forward and autonomously steers away from walls/obstacles using LiDAR.
   - After a `'left'` or `'right'` service call, the robot visibly rotates in place at triple angular velocity for ~2 seconds before autonomy resumes — enough duration to clearly observe the turn.
   - After a `'reverse'` call, the robot immediately backs up and rotates toward the safer (more open) side.
   - After a `'forward'` call, the robot immediately resumes straight-line movement.

4. **Terminal 3 (`rqt_graph`):**
   - A graph window opens showing:
     - `/direction_aware_obstacle_avoidance_controller` node connected to `/scan` (subscriber) and `/cmd_vel` (publisher).
     - The `/set_direction` service link between the calling terminal and the controller node.
   - Confirms the node is correctly wired to the simulator's topics/services with no disconnected nodes.
