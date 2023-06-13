#!/bin/bash

wb_folder=`pwd`
project_folder=/home/wjin/Workspace/continuous_STE
# The number of core you plan to use for the parallel dataset generation
core=8

# for a chunk of wind map 
iter_start=20
iter_end=40
task_for_each_core=$((iter_start/core+iter_end/core))
echo $task_for_each_core

for d in 1 2 3 4
do 
	echo "The process you are on: $d"
	cp -r $project_folder/webots $project_folder/webots_test_$d
	cd $project_folder/webots_test_$d
	start=$((iter_start+d*task_for_each_core-task_for_each_core))
	end=$((iter_start+d*task_for_each_core))
	if [ $d -eq 1 ]
	then
		tmux new-session -d '.$project_folder/training_data_generation_random_WB_test.sh start end'
	else if [ $d\2 -eq 0 ] 
    	then  
		tmux split-window -v '.$project_folder/training_data_generation_random_WB_test.sh start end'
		else 
		tmux split-window -h '.$project_folder/training_data_generation_random_WB_test.sh start end'
		fi
	fi
done

tmux new-window 'mutt'
tmux -2 attach-session -d