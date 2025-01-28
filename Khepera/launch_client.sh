#!/bin/bash


# robot's ID
ID=106

# set the map number
map=9
echo "+++ Map number $map +++"
# environment map
k4put $STE_DDPM_ROOT/Khepera/Data/test_map.environment +$ID
# obstacle list
k4put $STE_DDPM_ROOT/Khepera/Data/obstacle_list.txt +$ID
k4put $STE_DDPM_ROOT/Khepera/Data/source_position.txt +$ID

# start the server
# export PYTHONPATH=/home/rahbar/Projects/DDPM_K4/STE_PC_server
# ./STE_PC_server/STE_PC_server &

# start the client
k4put STE_K4_client/STE_K4_client +$ID
ssh -t root@192.168.1.$ID 'cd wanting; rm -r log_STE_multi; ./STE_K4_client'

# gather log files
current_date=$(date +"%Y-%m-%d")
current_time=$(date +"%H-%M")
mkdir -p Results_$current_date/$current_time
cp $STE_DDPM_ROOT/Khepera/Data/source_position.txt log_STE_multi/
mv $STE_DDPM_ROOT/Khepera/log_STE_multi Results_$current_date/$current_time/pc
k4get log_STE_multi +$ID
mv $STE_DDPM_ROOT/Khepera/k4-192.168.1."$ID"/log_STE_multi Results_$current_date/$current_time/k4
