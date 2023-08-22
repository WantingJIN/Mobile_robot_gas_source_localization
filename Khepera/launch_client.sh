#!/bin/bash


# robot's ID
ID=106

# set the map number
map=10
echo "+++ Map number $map +++"
# echo $map > "/home/rahbar/Projects/DDPM_K4/Data/params/map_nbr.txt"
# environment map
# cp /home/rahbar/Projects/DDPM_K4/Data/Env_map/test_map"$map".environment /home/rahbar/Projects/DDPM_K4/Data/test_map.environment
k4put /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/test_map.environment +$ID
# obstacle list
# cp /home/rahbar/Projects/DDPM_K4/Data/Obstacle_lists/"$map"_obstacles.txt /home/rahbar/Projects/DDPM_K4/Data/obstacle_list.txt
k4put /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/obstacle_list.txt +$ID
k4put /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/source_position.txt +$ID

# start the server
# export PYTHONPATH=/home/rahbar/Projects/DDPM_K4/STE_PC_server
# ./STE_PC_server/STE_PC_server &

# start the client
k4put STE_K4_client/STE_K4_client +$ID
ssh -t root@192.168.1.$ID 'cd wanting; rm -r log_STE_multi; ./STE_K4_client'

# gather log files
# current_time=$(date +"%Y-%m-%d_%H-%M")
# mkdir Results/$current_time
# mv -r /home/rahbar/Projects/DDPM_K4/log_STE_multi Results/$"current_time"/pc
# k4get log_STE_multi +$ID
# mv -r /home/rahbar/Projects/DDPM_K4/k4-192.168.1."$ID"/log_STE_multi Results/$"current_time"/k4
