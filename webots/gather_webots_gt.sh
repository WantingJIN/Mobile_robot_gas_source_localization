#!/bin/bash

now=$(date +"%T")
echo "Start time : $now"

webots="/home/wjin/Softwares/webots-R2021b/webots/webots"
webots_folder="../webots"
map_nbr_filename="/home/wjin/Workspace/continuous_STE/webots/data/map_nbr.txt"


#{0..8}
for m in 0
do
	echo $m > $map_nbr_filename

	# wind map and obstacle list
	cp /home/wjin/Workspace/continuous_STE/openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m".txt data/plugin_parameters/wind_map.txt
	cp /home/wjin/Workspace/continuous_STE/openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m"_obstacles.txt data/obstacle_list.txt
	cp /home/wjin/Workspace/continuous_STE/Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment

	#for k in {1..3}
    for k in 1 #{1..10}
	do
		$webots '--minimize' '--mode=fast'  $webots_folder'/worlds/WT_ground_truth.wbt' # >wb_output.txt
		now=$(date +"%T")
		echo "map $m  k $k  end time $now"
	done
done


now=$(date +"%T")
echo "End time : $now"
