#!/bin/bash

now=$(date +"%T")
echo "Start time : $now"

webots=$WEBOTS_HOME/webots
webots_folder="../webots"
map_nbr_filename="../webots/data/map_nbr.txt"


#{0..8}
for m in 10
do
	echo $m > $map_nbr_filename

	# wind map and obstacle list
	cp ../openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m".txt data/plugin_parameters/wind_map.txt
	cp ../openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m"_obstacles.txt data/obstacle_list.txt
	cp ../Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment

	#for k in {1..3}
      for k in {1}
	do
		$webots '--minimize' '--mode=fast'  $webots_folder'/worlds/WT_test_maps_with_visu_map10.wbt' # >wb_output.txt
		now=$(date +"%T")
		echo "map $m  k $k  end time $now"
	done
done


now=$(date +"%T")
echo "End time : $now"
