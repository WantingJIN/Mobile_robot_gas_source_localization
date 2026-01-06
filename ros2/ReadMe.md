Build with **ROS2 Foxy**

1. add the `export LD_LIBRARY_PATH=/home/wanting/Softwares/libtorch-cxx11-abi-shared-with-deps-1.13.1+cu117/libtorch/lib:$LD_LIBRARY_PATH` to `~/.zshrc` to add the library to the run path

2. First build the interfaces package: `colcon build --packages-select ste_interfaces`

3. build the package at the ros2 level with `colcon build --symlink-install`

4. run the node with `ros2 run ste_package controller_node`

5. Remember to set the same ID on both side of laptop and robot, currently the ID is set on laptop side as 60

   `export ROS_DOMAIN_ID=60`
