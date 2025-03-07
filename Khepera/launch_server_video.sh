#!/bin/bash

# robot's ID
# ID=103
config_file="../Khepera/Data/config.txt"
# set the map number
map=9
workMode="VIDEO"
experimental_logname="Results_RAL/Results_RAL_BoardD/2/19-00"
read -r source_x source_y <<<"12.7 1"

echo $map > "/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/map_nbr.txt"
echo $source_x $source_y > "/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/source_position.txt"

sed -i "s/^workMode *= *.*/workMode = $workMode/" "$config_file"
sed -i "s|^experimental_logname *= *.*|experimental_logname = $experimental_logname|" "$config_file"
# environment map
cp /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/Env_map/test_map"$map".environment /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/test_map.environment

#delete last unfinished job
result_folder_name="/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/log_STE_multi/"  # the folder to store the result for experiment that not finish
if [ -d "$result_folder_name" ]; then
    echo "Folder exists. Delete"
    rm -r "$result_folder_name"
    echo "Folder deleted successfully."
else
    echo "Folder does not exist."
fi

cp /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/Obstacle_lists/"$map"_obstacles.txt /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/obstacle_list.txt

export PYTHONPATH=/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/STE_PC_server
./STE_PC_server/STE_PC_server

