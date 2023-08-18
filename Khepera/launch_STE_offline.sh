#!/bin/bash


# robot's ID
# ID=103

# set the map number
map=2
read -r source_x source_y <<<"12.7 2"
workingMode=0

echo $workingMode > "/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/working_mode.txt"
echo $map > "/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/map_nbr.txt"
echo $source_x $source_y > "/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/source_position.txt"
# environment map
cp /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/Env_map/test_map"$map".environment /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/test_map.environment
cp /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/Obstacle_lists/"$map"_obstacles.txt /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/obstacle_list.txt
#delete last unfinished job
result_folder_name="/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/log_STE_multi/"  # the folder to store the result for experiment that not finish
if [ -d "$result_folder_name" ]; then
    echo "Folder exists. Delete"
    rm -r "$result_folder_name"
    echo "Folder deleted successfully."
else
    echo "Folder does not exist."
fi
# start the server
export PYTHONPATH=/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/STE_PC_server
./STE_PC_server/STE_PC_server | tee output.txt

# start the client
# k4put STE_K4_client/STE_K4_client +$ID
# ssh -t root@192.168.1.$ID 'cd rahbar; ./STE_K4_client'

# gather log files
# current_time=$(date +"%Y-%m-%d_%H-%M")
# mkdir Results/$current_time
# mv -r /home/rahbar/Projects/DDPM_K4/log_STE_multi Results/$"current_time"/pc
# k4get log_STE_multi +$ID
# mv -r /home/rahbar/Projects/DDPM_K4/k4-192.168.1."$ID"/log_STE_multi Results/$"current_time"/k4
