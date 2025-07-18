#!/bin/bash

# robot's ID
# ID=103

# set the map number
map=10
# save experiment configuration
releaseRate=MIDDLE
read -r source_x source_y <<<"12.7 2"

#config file
config_file="Data/config.txt"
workMode="EXPERIMENT"

echo $map > $STE_DDPM_ROOT/Khepera/Data/params/map_nbr.txt
sed -i "s/^workMode *= *.*/workMode = $workMode/" "$config_file"
sed -i "s/^releaseRate *= *.*/releaseRate = $releaseRate/" "$config_file"
echo $source_x $source_y > $STE_DDPM_ROOT/Khepera/Data/source_position.txt
# environment map
cp $STE_DDPM_ROOT/Khepera/Data/Env_map/test_map"$map".environment $STE_DDPM_ROOT/Khepera/Data/test_map.environment

#delete last unfinished job
result_folder_name=$STE_DDPM_ROOT/Khepera/log_STE_multi/  # the folder to store the result for experiment that not finish
if [ -d "$result_folder_name" ]; then
    echo "Folder exists. Delete"
    rm -r "$result_folder_name"
    echo "Folder deleted successfully."
else
    echo "Folder does not exist."
fi
cp $STE_DDPM_ROOT/Khepera/Data/Obstacle_lists/"$map"_obstacles.txt $STE_DDPM_ROOT/Khepera/Data/obstacle_list.txt

./STE_PC_server/STE_PC_server


