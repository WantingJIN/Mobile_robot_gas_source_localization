#!/bin/bash


# robot's ID
# ID=103

# set the map number
map=0
read -r source_x source_y <<<"12.7 2"

echo $map > "/home/wjin/Workspace/continuous_STE/Khepera/Data/params/map_nbr.txt"
echo $source_x $source_y > "/home/wjin/Workspace/continuous_STE/Khepera/Data/source_position.txt"
# environment map
# add the source position to the environment map to avoid collision with source location
cp /home/wjin/Workspace/continuous_STE/Khepera/Data/Env_map/test_map"$map".environment /home/wjin/Workspace/continuous_STE/Khepera/Data/test_map.environment
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
cp /home/wjin/Workspace/continuous_STE/Khepera/Data/Obstacle_lists/"$map"_obstacles.txt //home/wjin/Workspace/continuous_STE/Khepera/Data/obstacle_list.txt
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
# start the server
export PYTHONPATH=/home/wjin/Workspace/continuous_STE/Khepera/STE_PC_server
./STE_offline/STE_offline

# start the client
# k4put STE_K4_client/STE_K4_client +$ID
# ssh -t root@192.168.1.$ID 'cd rahbar; ./STE_K4_client'

# gather log files
# current_time=$(date +"%Y-%m-%d_%H-%M")
# mkdir Results/$current_time
# mv -r /home/rahbar/Projects/DDPM_K4/log_STE_multi Results/$"current_time"/pc
# k4get log_STE_multi +$ID
# mv -r /home/rahbar/Projects/DDPM_K4/k4-192.168.1."$ID"/log_STE_multi Results/$"current_time"/k4
