#!/bin/bash

now=$(date +"%T")
echo "Start time : $now"

webots="/home/wjin/Softwares/webots-R2021b/webots/webots"
webots_folder="../webots"
map_nbr_filename="/home/wjin/Workspace/continuous_STE/webots/data/map_nbr.txt"

#delete last unfinished job
result_folder_name="/home/wjin/Workspace/continuous_STE/Results/current/"  # the folder to store the result for experiment that not finish
if [ -d "$result_folder_name" ]; then
    echo "Folder exists. Delete"
    rm -r "$result_folder_name"
    echo "Folder deleted successfully."
else
    echo "Folder does not exist."
fi
#{0..8}
for m in 0
do
	echo $m > $map_nbr_filename

	# wind map and obstacle list
	cp /home/wjin/Workspace/continuous_STE/openfoam/test_maps/map"$m"/map"$m"_wind1/"$m".txt data/plugin_parameters/wind_map.txt
	cp /home/wjin/Workspace/continuous_STE/openfoam/test_maps/map"$m"/map"$m"_wind1/"$m"_obstacles.txt data/obstacle_list.txt
	cp /home/wjin/Workspace/continuous_STE/Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment

	#for k in {1..3}
    for k in {1..8}
	do
		$webots '--minimize' '--mode=fast'  $webots_folder'/worlds/WT_test_maps_with_Gaussian_plume.wbt' # >wb_output.txt
		now=$(date +"%T")
		echo "map $m  k $k  end time $now"
	done
done


now=$(date +"%T")
echo "End time : $now"
