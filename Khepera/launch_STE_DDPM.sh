#!/bin/bash

# robot's ID
# ID=103

# set the map number
map=1

echo $map > $STE_DDPM_ROOT/Khepera/Data/params/map_nbr.txt

# environment map
cp $STE_DDPM_ROOT/Khepera/Data/Env_map/test_map"$map".environment $STE_DDPM_ROOT/Khepera/Data/test_map.environment

cp $STE_DDPM_ROOT/Khepera/Data/Obstacle_lists/"$map"_obstacles.txt $STE_DDPM_ROOT/Khepera/Data/obstacle_list.txt

./STE_PC_DDPM/STE_PC_DDPM

