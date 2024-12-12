#!/bin/bash

# robot's ID
# ID=103

# set the map number
map=9
read -r source_x source_y <<<"12.7 1"
config_file="Data/config.txt"
workMode=OFFLINE

sed -i "s/^workMode *= *.*/workMode = $workMode/" "$config_file"
echo $map > $STE_DDPM_ROOT/Khepera/Data/params/map_nbr.txt
echo $source_x $source_y > $STE_DDPM_ROOT/Khepera/Data/source_position.txt
# environment map
cp $STE_DDPM_ROOT/Khepera/Data/Env_map/test_map"$map".environment $STE_DDPM_ROOT/Khepera/Data/test_map.environment
cp $STE_DDPM_ROOT/Khepera/Data/Obstacle_lists/"$map"_obstacles.txt $STE_DDPM_ROOT/Khepera/Data/obstacle_list.txt
#delete last unfinished job
result_folder_name=$STE_DDPM_ROOT/Khepera/log_STE_multi/ 
if [ -d "$result_folder_name" ]; then
    echo "Folder exists. Delete"
    rm -r "$result_folder_name"
    echo "Folder deleted successfully."
else
    echo "Folder does not exist."
fi
./STE_PC_server/STE_PC_server
