# Multi-robot for GSL

Setup:

1. To setup a new marker set, let the robot face towards the outlet side (to initialize the markerset heading as 0, and we use -heading + 90 as the robot heading)

2. Reduce the frame rate of the motion capture system to 20 to reduce the communication bandwidth with multiple robot

3. To check the localization data from motion capture system with `roslaunch wtcortex_bridge wtcortex_bridge.launch`, change the Data Multicast Port to 5900 (Cortex-> setting-> System-> Data Multicast Port), and change it back to 1001 for STE_K4_client and crazyfiles

   Note: I also tried to change the value in pp 58 of cortex.cpp, but it could not work

Khepera:

Khepera 106, 102, 103 are tested (with new sensor board) and ready to be used.

Use `bash launch_STE_distributed_debug.sh` to start the experiments with multi-robot (change the robot id specified in it)

NOTE: before the experiment, check the performance with the single robot first, I changed most `double` variables to `float` to optimize the calculation with robots. However, I am not sure the influence on the estimation precision. Maybe for PDF calculation, the `float` is still required to guarantee the precision.

Full pipeline:

1. First run ./launch_STE_DDPM.sh to generate the offline gas dispersion model run by laptop
2. Run bash launch_STE_distributed.sh to launch the STE on robot by reading the offline gas dispersion model

**TODOs**:

- [ ] The communication between robots are done (test with 106 and 102, didn't check with three robots); The send and received belief maps are consistent (kind of, with a bit different at 10^-5 level); However, the merged pdf are different, not sure if it is because of the float precision, still need to check
- [ ] The obstacle avoidance still needs to be improved, the robot keeps blocking each other
- [ ] The test with single robot with the new on-board STE needs to be tested first

NOTE:

1. for communication between robots: consider multiple-thread if we need to listen to the socket while moving 



Paper Idea:

Multi-robot system for GSL

1. Sensor difference -> Relative measurement -> Belief merging instead of sharing the measurements
2. Sense-in-motion -> Region allocation
3. Replan (re-allocate the region and robot replan the trajectory) with the KLD difference with the last belief map used for planning

**Region allocation consider the region growing or Lloyd’s Algorithm (geometry balanced Voronoi) with the seed as the centroid of the Belief clusters (with DBscan).** 

Reason: In order for the relative measurement to converge, all the robots need to explore the environmental spatially sufficiently. Otherwise, if we split the region with one robot stays close to source and one robot always far from the source, the merge belief map with not work, as the robot stay far from the source will give the wrong estimation. Now we tackled this issue with region growing with the seeds as the three highest belief cell, however, this strategy can be too greedy/ risky, as the other clusters with small beliefs are negelected.

Path planning can try belief clustering scanning/ centroid of the belief clustering/ cumulative IPP



   

