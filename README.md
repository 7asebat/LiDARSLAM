# Team Members
| Name | Section | BN |
| --- | --- | --- |
| Ahmed Nasser Ahmed | 1 | 8 |
| Ahmed Hisham Eid | 1 | 9 |
| Abdelrahman Ahmed Mohamed Farid | 1 | 33 |
| Youssef Walid Hassan | 2 | 34 |

# Launch Instructions

1. Install ROS melodic
2. Initialize catkin workspace
3. Place code inside `src/` directory
4. Run `catkin_make`
5. Run `roslaunch vehicle_sim_launcher corner.launch gpu:=true` to launch gazebo simulation
6. Run `rosrun slam_code main.py` in another terminal to run the code

# File Structure

Inside `vehicle_sim/worlds` we can find the gazebo worlds used
Inside `vehicle_sim/launcher/vehicle_sim_launcher/launch/corner.launch` we can find the `corner.launch` file used
Inside `slam_code/main.py` we can find the main python script for the project.

# Code structure

`main.py`:
  - The main loop consists of:
    - Subscribers
    - Publishers
    - The ROS loop which executes once every 1 second and is responsible for calling `normalize_and_publish_map()`
  - Global variables
    - particle: particle class which contains the car position
  - `scan_matching_callback`
    - Callback for scan matching
    - Responsible for updating the robot position globally
  - `transformed_lidar_reading_callback`
    - Callback for the lidar readings
    - Responsible for:
      - Parsing the lidar PC2 into an np array
      - Transforming lidar points correctly into `world` and `map` point of views
      - Checking for out of bound particles
      - Filtering out car particles
      - Casting rays using `cast_ray_and_update_map`
  - `cast_ray_and_update_map`
    - Casts rays using different methods:
      - naive: the usual unit vector casting method
      - bresenham: the bresenham algorithm which is more accurate
      - skimage_line_2d: only used for 2d ray casting, uses `skimage.draw.line` method
    - Updates map accordingly by log odds likelihood method
  - `normalize_and_publish_map`
    - Responsible for mapping the grid array into probabilities and publishing this on `/map_pc2` topic
    - Uses vectorized ways to quickly map into PC2 format
`bresenham3d.py`:
  - Contains 3d bresenham algorithm
  - Copied from [geeksforgeeks](https://www.geeksforgeeks.org/bresenhams-algorithm-for-3-d-line-drawing/)