#!/bin/bash

now=$(date +"%T")
echo "Start time : $now"

webots="/usr/local/bin/webots"
webots_folder="../webots"
map_nbr_filename="/home/durancea/plume_model_learning/webots/data/map_nbr.txt"
source_pos_filename="/home/durancea/plume_model_learning/webots/data/source_pos.txt"
sampling_prd_filename="/home/durancea/plume_model_learning/webots/data/simulation_parameters/sampling_period.txt"
MCMC_exhaustive_filename="/home/durancea/plume_model_learning/webots/data/simulation_parameters/MCMC_or_exhaustive_eval.txt"
vel_factor_filename="/home/durancea/plume_model_learning/webots/data/simulation_parameters/vel_factor.txt" 
rr_filename="/home/durancea/plume_model_learning/webots/data/plugin_parameters/releaseAmount.txt"
s=0
echo $s > $MCMC_exhaustive_filename

nbr_source_pos=10
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

v=1 # 1 for default controller
# save previous velocity value
tmp=$(cat "$vel_factor_filename") 
echo $v > $vel_factor_filename
echo $vel_factor_filename

# release amount
for r in 53 #30 53 80 120 200
	do
		for m in 0 1 3 5 10 #0 1  
		do
			echo $r > $rr_filename
			echo $m > $map_nbr_filename
			# wind map and obstacle list
			cp /home/durancea/plume_model_learning/openfoam/test_maps/map"$m"/map"$m"_wind1/"$m".txt data/plugin_parameters/wind_map.txt
			cp /home/durancea/plume_model_learning/openfoam/test_maps/map"$m"/map"$m"_wind1/"$m"_obstacles.txt data/obstacle_list.txt
			cp /home/durancea/plume_model_learning/Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment

			#for k in {1..3} 1
			for ((k=1; k<=$nbr_source_pos; k++)) 
			do
				source_pos_y=$(awk "BEGIN { print $source_pos_y_min + $k * $source_pos_y_step }")
				echo $source_pos_x $source_pos_y > $source_pos_filename
				$webots '--minimize' '--mode=fast'  $webots_folder'/worlds/WT_test_maps.wbt' '--no-rendering' # &> wb_output.txt # >wb_output.txt
				now=$(date +"%T")
				echo "map $m  k $k  release amount $r end time $now"
			done
		done
done

# write original velocity value back
echo $tmp > $vel_factor_filename

r=53
echo "$r" > $rr_filename

now=$(date +"%T")
echo "End time : $now"
