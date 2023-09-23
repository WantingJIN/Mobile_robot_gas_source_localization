#!/bin/bash

now=$(date +"%T")
echo "Start time : $now"

webots="/usr/local/bin/webots"
webots_folder="../webots"
map_nbr_filename="/home/durancea/plume_model_learning/webots/data/map_nbr.txt"
cluster_entropy_ratio_filename="/home/durancea/plume_model_learning/webots/data/simulation_parameters/cluster_entropy_ratio.txt"
source_pos_filename="/home/durancea/plume_model_learning/webots/data/source_pos.txt"

nbr_source_pos=8
source_pos_x=12.7
source_pos_y_min=0.5
source_pos_y_max=3.5
source_pos_y_step=$(awk "BEGIN { print ($source_pos_y_max - $source_pos_y_min) / $nbr_source_pos }")

#delete last unfinished job
result_folder_name="/home/durancea/plume_model_learning/Results/current/"  # the folder to store the result for experiment that not finish
if [ -d "$result_folder_name" ]; then
    echo "Folder exists. Delete"
    rm -r "$result_folder_name"
    echo "Folder deleted successfully."
else
    echo "Folder does not exist."
fi

#{0..8}
for r in 1 2.5 5 10 20;
do
	echo "cluster entropy ratio is $r"
	echo $r > $cluster_entropy_ratio_filename
    for m in {1..8}
    do
		echo $m > $map_nbr_filename

		# wind map and obstacle list
		cp /home/durancea/plume_model_learning/openfoam/test_maps/map"$m"/map"$m"_wind1/"$m".txt data/plugin_parameters/wind_map.txt
		cp /home/durancea/plume_model_learning/openfoam/test_maps/map"$m"/map"$m"_wind1/"$m"_obstacles.txt data/obstacle_list.txt
		cp /home/durancea/plume_model_learning/Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment
		#for k in {1..3}
		for k in {1..8}
		do
			source_pos_y=$(awk "BEGIN { print $source_pos_y_min + $k * $source_pos_y_step }")
			echo $source_pos_x $source_pos_y > $source_pos_filename
			$webots '--minimize' '--mode=fast'  $webots_folder'/worlds/WT_test_maps.wbt' # >wb_output.txt
			now=$(date +"%T")
			echo "map $m  k $k  end time $now"
		done
	done
done


now=$(date +"%T")
echo "End time : $now"
