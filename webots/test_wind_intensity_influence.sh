#!/bin/bash

# To run from webots folder

now=$(date +"%T")
echo "Start time : $now"

webots="/usr/local/bin/webots"
webots_folder="../webots"
map_nbr_filename="/home/durancea/plume_model_learning/webots/data/map_nbr.txt"
# use map 0: obstacle free enviroment
m=1
echo $m > $map_nbr_filename
cp /home/durancea/plume_model_learning/openfoam/test_maps/map"$m"/map"$m"_wind1/"$m".txt data/plugin_parameters/wind_map.txt
cp /home/durancea/plume_model_learning/openfoam/test_maps/map"$m"/map"$m"_wind1/"$m"_obstacles.txt data/obstacle_list.txt
cp /home/durancea/plume_model_learning/Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment
vel_factor_filename="/home/durancea/plume_model_learning/webots/data/simulation_parameters/vel_factor.txt"
wind_factor_filename="/home/durancea/plume_model_learning/webots/data/simulation_parameters/wind_factor.txt"
source_pos_filename="/home/durancea/plume_model_learning/webots/data/source_pos.txt"

nbr_source_pos=1 #6
source_pos_x=12.7
source_pos_y_min=0.5
source_pos_y_max=3.5
source_pos_y_step=$(awk "BEGIN { print ($source_pos_y_max - $source_pos_y_min) / $nbr_source_pos }")

# save previous period value
tmp_vel=$(cat "$vel_factor_filename") 
tmp_wind=$(cat "$wind_factor_filename") 
echo 0.4 > $vel_factor_filename

#delete last unfinished job
result_folder_name="/home/durancea/plume_model_learning/Results/current/"  # the folder to store the result for experiment that not finish
if [ -d "$result_folder_name" ]; then
    echo "Folder exists. Delete"
    rm -r "$result_folder_name"
    echo "Folder deleted successfully."
else
    echo "Folder does not exist."
fi


for p in 0.75 1.0 #0.1 0.5 0.75 1.0 1.25 1.5 2.0 2.5 
do
	echo "wind multiplier is $p"
	echo $p > $wind_factor_filename

	# wind map and obstacle list
	for ((k=1; k<=$nbr_source_pos; k++))
	do
		source_pos_y=$(awk "BEGIN { print $source_pos_y_min + $k * $source_pos_y_step }")
		echo $source_pos_x $source_pos_y > $source_pos_filename
		$webots '--minimize' '--mode=fast'  $webots_folder'/worlds/WT_test_maps.wbt' # >wb_output.txt
		now=$(date +"%T")
		echo "map $m  p $p k $k  end time $now"
	done

done

# write original period value back
echo $tmp_vel > $vel_factor_filename
echo $tmp_wind > $wind_factor_filename

now=$(date +"%T")
echo "End time : $now"
