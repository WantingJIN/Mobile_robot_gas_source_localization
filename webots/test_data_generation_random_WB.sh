#!/bin/bash

webots="/usr/local/bin/webots"
wb_folder=`pwd`
wind_map_folder=$wb_folder/../openfoam/test_maps
cd $wind_map_folder

# for every wind map
for d in {1..9}
do
    echo "map${d}"
    # make sure this wind map was not used before
    #if [ ! -d "../../Results/training_set_map1/$d" ]
	#then
		# create a new folder with the same name
		results_folder=../../Results/test_maps/map${d}
		mkdir -p $results_folder
		# copy the txt file for odor plugin
		cp map${d}/map${d}_wind1/${d}.txt $wb_folder/data/plugin_parameters/wind_map.txt
		cp map${d}/map${d}_wind1/${d}_obstacles.txt $wb_folder/data/obstacle_list.txt

		# then, 20 random release rate [36, 53]
		for i in 1
		do
			rr=53
			#rr=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); rr=rand()*17+36; print rr; }')
			# write in the plugin file
			echo "$rr" > $wb_folder/data/plugin_parameters/releaseAmount.txt
			# create a folder for this release rate
			# mkdir ../../Results/training_set_map1/$d/rr$rr
			# run the simulation 20 times (with random source positions)
			for j in {1..20}
			do
				# run webots
				cd $wb_folder
				$webots 'worlds/WT_static_sensor_network.wbt' '--minimize' '--mode=fast' '--no-rendering'
				cd $wind_map_folder

				# if a log file was created successfully
				count=`ls -1 $wb_folder/log/*.csv 2>/dev/null | wc -l`
				if [ $count != 0 ]
				then
					# then copy the result in the folder we created
					cp $wb_folder/log/*.csv $results_folder
					# and remove the results for the next run
					rm $wb_folder/log/*.csv
				fi
			done
		done
		# remove the wind map for the next round
		rm $wb_folder/data/plugin_parameters/wind_map.txt
	#fi
done
