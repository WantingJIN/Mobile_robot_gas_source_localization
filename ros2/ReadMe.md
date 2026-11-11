Build with **ROS2 Foxy**

1. add the `export LD_LIBRARY_PATH=/home/wanting/Softwares/libtorch-cxx11-abi-shared-with-deps-1.13.1+cu117/libtorch/lib:$LD_LIBRARY_PATH` to `~/.zshrc` to add the library to the run path

2. First build the interfaces package: `colcon build --packages-select ste_interfaces`

3. build the package at the ros2 level with `colcon build --symlink-install`

4. run the node with `ros2 run ste_package controller_node`

5. Remember to set the same ID on both side of laptop and robot, currently the ID is set on laptop side as 7

   `export ROS_DOMAIN_ID=7`

6. ros2 action send_goal /robot_mission ste_interfaces/action/NavigateSample \ "{  goal_positions: {    header: {frame_id: 'map'},    poses: [      {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}},      {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}},      {position: {x: 3.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}    ]  } }" --feedback

7. Domain range x [-10,3] y[-2.5, 2.5]

8. wind is in local frame

9. The anemometer coordinate has 90 degree (clockwise) difference compare to the robot coordinate 
