#!/bin/bash

# Limpiar Webots al presionar Ctrl+C
# Al presionar Ctrl+C, ejecutar 'cleanup'
trap cleanup SIGINT

cleanup() {
    echo "🛑 Ctrl+C detectado. Matando Webots y saliendo..."
    pkill -f webots  # Mata cualquier proceso Webots (ajustable si querés algo más fino)
    exit 1
}


now=$(date +"%T")
echo "Start time : $now"

webots=$WEBOTS_HOME/webots
webots_folder="../webots"
map_nbr_filename="../webots/data/map_nbr.txt"
source_pos_filename="../webots/data/source_pos.txt"
sampling_prd_filename="../webots/data/simulation_parameters/sampling_period.txt"
MCMC_exhaustive_filename="../webots/data/simulation_parameters/MCMC_or_exhaustive_eval.txt"
vel_factor_filename="../webots/data/simulation_parameters/vel_factor.txt" 
rr_filename="../webots/data/plugin_parameters/releaseAmount.txt"
config_file="../webots/data/config.txt"
s=0
echo $s > $MCMC_exhaustive_filename

nbr_source_pos=30
# source_pos_x=12.7   # default: 12.7
source_pos_y_min=0.5
source_pos_y_max=3.5
source_pos_y_step=$(awk "BEGIN { print ($source_pos_y_max - $source_pos_y_min) / $nbr_source_pos }")

source_pos_x_min=10
source_pos_x_max=12.7
source_pos_x_step=$(awk "BEGIN { print ($source_pos_x_max - $source_pos_x_min) / $nbr_source_pos }")

#delete last unfinished job
result_folder_name="../Results/current/"  # the folder to store the result for experiment that not finish
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
R=100 # gas sensor resistance
sigma=0.4 #noise level for the STE likelihood

# release amount
for r in 53 #53 #200 #10 30 53 80 200 #10 53 200 #53 #10 53 200 # 10 120 200 30 53 80 # default release amount 53
do
	echo $r > $rr_filename
	for m in 9 #9 #5 #1 2 3 5 10 #0 1  
	do
		echo $m > $map_nbr_filename
		# for likelihood in 2000 3000
		# for sigma in 0.43 0.45 0.48
	      # for R in 100 1500
		# for thres in 100 300
		# for lambda in 0.5 0.7 0.9
		# for planningStrategy in "INFORMATIVE_KLD"
		# for scoreSigma in 0.4 0.5 0.7
		# for scoreFun in "RATIO" "LOG" "SIGMOID"
		# for infoFeature in "VARIANCE" "MEAN" "PDF"
		# for weighting_entropy_ratio in 10
	      # do
			# sed -i "s/^R0 *= *.*/R0 = $R/" "$config_file"
			# sed -i "s/^sigma *= *.*/sigma = $sigma/" "$config_file"
			# sed -i "s/^likelihood_error *= *.*/likelihood_error = $likelihood/" "$config_file"
			# sed -i "s/^gasHitThres *= *.*/gasHitThres = $thres/" "$config_file"
			# sed -i "s/^lambda *= *.*/lambda = $lambda/" "$config_file"
			# sed -i "s/^cluster_entropy_ratio *= *.*/cluster_entropy_ratio = $cluster_entropy_ratio/" "$config_file"
			# sed -i "s/^planningStrategy *= *.*/planningStrategy = $planningStrategy/" "$config_file"
			# sed -i "s/^scoreSigma *= *.*/scoreSigma = $scoreSigma/" "$config_file"
			# sed -i "s/^scoreFun *= *.*/scoreFun = $scoreFun/" "$config_file"
			# sed -i "s/^infoFeature *= *.*/infoFeature = $infoFeature/" "$config_file"
			# sed -i "s/^weighting_entropy_ratio *= *.*/weighting_entropy_ratio = $weighting_entropy_ratio/" "$config_file"
			# wind map and obstacle list
			cp ../openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m".txt data/plugin_parameters/wind_map.txt
			cp ../openfoam/test_maps/map"$m"/map"$m"_wind0.75/"$m"_obstacles.txt data/obstacle_list.txt
			cp ../Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment

			#for k in {1..3} 1
			for ((k=1; k<=$nbr_source_pos; k++)) 
			do
				if [ "$k" -eq 1 ]; then
					echo "Map: $m"
				fi

				source_pos_y=$(awk "BEGIN { print $source_pos_y_min + $k * $source_pos_y_step }")
				source_pos_x=$(awk "BEGIN { print $source_pos_x_min + $k * $source_pos_x_step }")
				echo $source_pos_x $source_pos_y > $source_pos_filename
				#$webots '--minimize' '--mode=fast'  $webots_folder'/worlds/WT_test_maps.wbt' &> wb_output.txt # >wb_output.txt
				timeout 100s $webots '--minimize' '--mode=fast'  "$webots_folder/worlds/WT_test_maps.wbt" &> wb_output.txt
				exit_code=$?

				if [ "$exit_code" -eq 124 ]; then
					echo "⏱❌ Simulación $k se excedió del tiempo límite de 60seg. Saltando..."
					mkdir -p "$result_folder_name/timeout_$k"
					cp wb_output.txt "$result_folder_name/timeout_$k/webots_log.txt"
				else
					echo 
				fi

				now=$(date +"%T")
				echo "map $m  k $k  release amount $r R $R end time $now"
			done
	      # done	
	done
done

# write original velocity value back
echo $tmp > $vel_factor_filename

r=53
echo "$r" > $rr_filename

now=$(date +"%T")
echo "End time : $now"
