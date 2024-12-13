#!/bin/bash

# robot's ID
# ID=103

# set the map number
map=9
# save experiment configuration
releaseRate=MIDDLE
read -r source_x source_y <<<"12.7 3"

#config file
config_file="Data/config.txt"
workMode=EXPERIMENT

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


# MarginY=0.2
# MarginX=0.05
# MINX=`echo $source_x - $MarginX | bc`
# MAXY=`echo $source_y + $MarginY | bc`
# MINY=`echo $source_y - $MarginY | bc`
# text1="12.6 	0.15"
# text2="12.6	0.15\n12.6    $MINY\n$MINX    $MINY\n$MINX    $MAXY\n12.6    $MAXY"
# sed -i -e :n -e "$!N;/\n.*\n/!{$!bn};  s/$text1/$text2/;P;D" /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/test_map.environment
# # add the source 
# # k4put /home/rahbar/Projects/DDPM_K4/Data/test_map.environment +$ID
# # obstacle list
# # k4put /home/rahbar/Projects/DDPM_K4/Data/obstacle_list.txt +$ID
# read -r obstacle_nbr<"/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/obstacle_list.txt"
# obstacle_nbr_new=`echo $obstacle_nbr + 1 |tr -d $'\r'| bc`
# #replace the first
# sed -i "1s/.*/$obstacle_nbr_new/" /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/obstacle_list.txt
# #insert as last line
# OffsetX=0.1
# OffsetY=0.05
# obsX=`echo $source_x + $OffsetX | bc`
# obsYMIN=`echo $source_y - $OffsetY | bc`
# obsYMAX=`echo $source_y + $OffsetY | bc`
# obstacle_source="$obsX $obsYMIN 13 $obsYMAX"
# echo $obstacle_source >> /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/obstacle_list.txt
#sed -i -e '$a>obstacle_source>' /home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/obstacle_list.txt
# start the client
# k4put STE_K4_client/STE_K4_client +$ID
# ssh -t root@192.168.1.$ID 'cd rahbar; ./STE_K4_client'

# gather log files
# current_time=$(date +"%Y-%m-%d_%H-%M")
# mkdir Results/$current_time
# mv -r /home/rahbar/Projects/DDPM_K4/log_STE_multi Results/$"current_time"/pc
# k4get log_STE_multi +$ID
# mv -r /home/rahbar/Projects/DDPM_K4/k4-192.168.1."$ID"/log_STE_multi Results/$"current_time"/k4
