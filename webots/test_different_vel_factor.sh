#!/bin/bash

# To run from webots folder

now=$(date +"%T")
echo "Start time : $now"

webots=$WEBOTS_HOME/webots
webots_folder="../webots"
map_nbr_filename="/home/wjin/Workspace/continuous_STE/webots/data/map_nbr.txt"
vel_factor_filename="/home/wjin/Workspace/continuous_STE/webots/data/simulation_parameters/vel_factor.txt" 
source_pos_filename="/home/wjin/Workspace/continuous_STE/webots/data/source_pos.txt"

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

# save previous velocity value
tmp=$(cat "$vel_factor_filename") 

# the maximal speed of Khepera is 0.8m/s, so that the maximal velocity factor is 4
for p in 2 4  #0.01 0.05 0.1 0.2 0.3 0.4 #0.5 0.6 0.7 0.8 #  if constant. percent of max speed
do	
	echo "velocity multiplier is $p"
	echo $p > $vel_factor_filename
	for m in 1 2 10
	do
            echo $m > $map_nbr_filename
		cp /home/wjin/Workspace/continuous_STE/openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m".txt data/plugin_parameters/wind_map.txt
		cp /home/wjin/Workspace/continuous_STE/openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m"_obstacles.txt data/obstacle_list.txt
		cp /home/wjin/Workspace/continuous_STE/Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment
		# wind map and obstacle list
		for ((k=1; k<=$nbr_source_pos; k++))
		do
			source_pos_y=$(awk "BEGIN { print $source_pos_y_min + $k * $source_pos_y_step }")
			echo $source_pos_x $source_pos_y > $source_pos_filename
			$webots '--minimize' '--mode=fast'  $webots_folder'/worlds/WT_test_maps.wbt' '--no-rendering' # >wb_output.txt
			now=$(date +"%T")
			echo "map $m  p $p k $k  end time $now"
		done
	done
done

# write original period value back
echo $tmp > $vel_factor_filename

now=$(date +"%T")
echo "End time : $now"
