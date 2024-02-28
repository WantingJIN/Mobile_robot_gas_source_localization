#!/bin/bash

# To run from webots folder

now=$(date +"%T")
echo "Start time : $now"

webots="/usr/local/bin/webots"
webots_folder="../webots"
map_nbr_filename="/home/durancea/plume_model_learning/webots/data/map_nbr.txt"
# use map 0: obstacle free enviroment
m=0
echo $m > $map_nbr_filename
cp ../openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m".txt data/plugin_parameters/wind_map.txt
cp ../openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m"_obstacles.txt data/obstacle_list.txt
cp ../Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment
sampling_prd_filename="../webots/data/simulation_parameters/sampling_period.txt"
source_pos_filename="../webots/data/source_pos.txt"

nbr_source_pos=6
source_pos_x=12.7
source_pos_y_min=0.5
source_pos_y_max=3.5
source_pos_y_step=$(awk "BEGIN { print ($source_pos_y_max - $source_pos_y_min) / $nbr_source_pos }")

#delete last unfinished job
result_folder_name="../Results/current/"  # the folder to store the result for experiment that not finish
if [ -d "$result_folder_name" ]; then
    echo "Folder exists. Delete"
    rm -r "$result_folder_name"
    echo "Folder deleted successfully."
else
    echo "Folder does not exist."
fi

# save previous period value
tmp=$(cat "$sampling_prd_filename") 

for p in 1 10 30 60 90  #10 15 20 25 30 35 40; 
do
	echo "frequency is 1/$p"
	echo $p > $sampling_prd_filename

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
echo $tmp > $sampling_prd_filename

now=$(date +"%T")
echo "End time : $now"
