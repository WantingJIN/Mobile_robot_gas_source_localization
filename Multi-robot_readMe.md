# Multi-robot for GSL

Setup:

1.  To setup a new marker set, let the robot face towards the outlet side (to initialize the markerset heading as 0, and we use -heading + 90 as the robot heading)
2. Reduce the frame rate of the motion capture system to 20 to reduce the communication bandwidth with multiple robot



Khepera:

Use `bash launch_STE_distributed_debug.sh` to start the experiments with multi-robot (change the robot id specified in it)

NOTE: before the experiment, check the performance with the single robot first, I changed most `double` variables to `float` to optimize the calculation with robots. However, I am not sure the influence on the estimation. Maybe for PDF calculation, the `float` is still required to guarantee the precision.

Full pipeline:

1. First run ./launch_STE_DDPM.sh to generate the offline gas dispersion model run by laptop
2. Run bash launch_STE_distributed.sh to launch the STE on robot by reading the offline gas dispersion model 

The communication between different robot is not setup yet (we can use the module lower ID (client) <-> higher ID (server) to gurantee peer to peer communication)



Progress: 

Now the communication between robots are done (test with 106 and 102, didn't check with three robots); The send and received belief maps are consistent (kind of, with a bit different at 10^-5 level); However, the merged pdf are different, not sure if it is because of the float precision, still need to check





NOTE:

1. for communication between robots: consider multiple-thread if we need to listen to the socket while moving 
