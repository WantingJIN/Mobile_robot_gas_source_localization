#!/bin/bash

now=$(date +"%T")
echo "Start time : $now"

webots="/home/wjin/Softwares/webots-R2021b/webots/webots"
webots_folder="../webots"
map_nbr_filename="/home/wjin/Workspace/continuous_STE/webots/data/map_nbr.txt"
source_pos_filename="/home/wjin/Workspace/continuous_STE/webots/data/source_pos.txt"
sampling_prd_filename="/home/wjin/Workspace/continuous_STE/webots/data/simulation_parameters/sampling_period.txt"
MCMC_exhaustive_filename="/home/wjin/Workspace/continuous_STE/webots/data/simulation_parameters/MCMC_or_exhaustive_eval.txt"
# use environment 0: obstacle free enviroment
p=10
echo $p > $sampling_prd_filename
#use exhaustive search
s=0
echo $s > $MCMC_exhaustive_filename

nbr_source_pos=10
source_pos_x=12.7
source_pos_y_min=0.5
source_pos_y_max=3.5
source_pos_y_step=$(awk "BEGIN { print ($source_pos_y_max - $source_pos_y_min) / $nbr_source_pos }")

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
for m in 2
do
	echo $m > $map_nbr_filename

	# wind map and obstacle list
	cp /home/wjin/Workspace/continuous_STE/openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m".txt data/plugin_parameters/wind_map.txt
	cp /home/wjin/Workspace/continuous_STE/openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m"_obstacles.txt data/obstacle_list.txt
	cp /home/wjin/Workspace/continuous_STE/Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment

	#for k in {1..3}
    for k in 1 2 3
	do
		source_pos_y=$(awk "BEGIN { print $source_pos_y_min + $k * $source_pos_y_step }")
		echo $source_pos_x $source_pos_y > $source_pos_filename
		$webots '--minimize' '--mode=fast'  $webots_folder'/worlds/WT_test_maps.wbt' # >wb_output.txt
		now=$(date +"%T")
		echo "map $m  k $k  end time $now"
	done
done


now=$(date +"%T")
echo "End time : $now"
