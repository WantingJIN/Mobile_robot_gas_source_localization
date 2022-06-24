#!/bin/bash

wb_folder=`pwd`
wind_map_folder=$wb_folder/../openfoam/training_random/training_random_wind1.0_angle0/
cd $wind_map_folder
webots="/home/wjin/Softwares/webots-R2021b/webots/webots"

iter_start = $1
iter_end = $2
mkdir -p $wb_folder/log
# for every wind map
for d in {$iter_start..$iter_end}
do
    echo "$d"
    # make sure this wind map was not used before
    #if [ ! -d "../../Results/training_set_map1/$d" ]
	#then
		# create a new folder with the same name
		results_folder=../../../Results2/training_random/$d
		mkdir -p $results_folder
		# copy the txt file for odor plugin
		cp $d/$d.txt $wb_folder/data/plugin_parameters/wind_map.txt
		cp $d/${d}_obstacles.txt $wb_folder/data/obstacle_list.txt

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
