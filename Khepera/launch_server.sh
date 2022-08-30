#!/bin/bash


# robot's ID
# ID=103

# set the map number
map=11
echo $map > "/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/map_nbr.txt"
# environment map
cp /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/Env_map/test_map"$map".environment /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/test_map.environment
# k4put /home/rahbar/Projects/DDPM_K4/Data/test_map.environment +$ID
# obstacle list
cp /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/Obstacle_lists/"$map"_obstacles.txt /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/obstacle_list.txt
# k4put /home/rahbar/Projects/DDPM_K4/Data/obstacle_list.txt +$ID

# start the server
export PYTHONPATH=/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/STE_PC_server
./STE_PC_server/STE_PC_server

# start the client
# k4put STE_K4_client/STE_K4_client +$ID
# ssh -t root@192.168.141.$ID 'cd rahbar; ./STE_K4_client'

# gather log files
# current_time=$(date +"%Y-%m-%d_%H-%M")
# mkdir Results/$current_time
# mv -r /home/rahbar/Projects/DDPM_K4/log_STE_multi Results/$"current_time"/pc
# k4get log_STE_multi +$ID
# mv -r /home/rahbar/Projects/DDPM_K4/k4-192.168.141."$ID"/log_STE_multi Results/$"current_time"/k4
