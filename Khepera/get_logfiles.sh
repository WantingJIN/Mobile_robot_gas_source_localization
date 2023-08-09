#!/bin/bash


# robot's ID
ID=106

# set the map number
# map=0
# echo "+++ Map number $map +++"
# echo $map > "/home/rahbar/Projects/DDPM_K4/Data/params/map_nbr.txt"
# environment map
# cp /home/rahbar/Projects/DDPM_K4/Data/Env_map/test_map"$map".environment /home/rahbar/Projects/DDPM_K4/Data/test_map.environment
# k4put /home/rahbar/Projects/DDPM_K4/Data/test_map.environment +$ID
# obstacle list
# cp /home/rahbar/Projects/DDPM_K4/Data/Obstacle_lists/"$map"_obstacles.txt /home/rahbar/Projects/DDPM_K4/Data/obstacle_list.txt
# k4put /home/rahbar/Projects/DDPM_K4/Data/obstacle_list.txt +$ID

# start the server
# export PYTHONPATH=/home/rahbar/Projects/DDPM_K4/STE_PC_server
# ./STE_PC_server/STE_PC_server &

# start the client
# k4put STE_K4_client/STE_K4_client +$ID
# ssh -t root@192.168.1.$ID 'cd rahbar; ./STE_K4_client'

# gather log files
current_time=$(date +"%Y-%m-%d_%H-%M")
mkdir Results_0808/$current_time
cp /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/source_position.txt log_STE_multi/
mv /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/log_STE_multi Results_0808/$current_time/pc
k4get log_STE_multi +$ID
mv /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/k4-192.168.1."$ID"/log_STE_multi Results_0808/$current_time/k4
