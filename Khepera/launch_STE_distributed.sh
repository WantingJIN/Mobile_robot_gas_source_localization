#!/bin/bash

# NOTE: did not test this script, it is a copy of launch_STE_distributed_debug.sh with some modifications

# robot's ID
ID=103

# set the map number
map=1
echo "+++ Map number $map +++"

cp $STE_DDPM_ROOT/Khepera/Data/Env_map/test_map"$map".environment $STE_DDPM_ROOT/Khepera/Data/test_map.environment
cp $STE_DDPM_ROOT/Khepera/Data/Obstacle_lists/"$map"_obstacles.txt $STE_DDPM_ROOT/Khepera/Data/obstacle_list.txt
cp $STE_DDPM_ROOT/Khepera/Data/DeepCFD_result/gas_concentration_maps_"$map".csv $STE_DDPM_ROOT/Khepera/Data/gas_concentration_maps.csv



# environment map
k4put $STE_DDPM_ROOT/Khepera/Data/test_map.environment +$ID
# obstacle list
k4put $STE_DDPM_ROOT/Khepera/Data/obstacle_list.txt +$ID
k4put $STE_DDPM_ROOT/Khepera/Data/source_position.txt +$ID
# gas concentration maps
k4put $STE_DDPM_ROOT/Khepera/Data/gas_concentration_maps.csv +$ID
# config file
k4put $STE_DDPM_ROOT/Khepera/Data/config.txt +$ID   

# start the server
# export PYTHONPATH=/home/rahbar/Projects/DDPM_K4/STE_PC_server
# ./STE_PC_server/STE_PC_server &

# start the client
k4put STE_K4_distributed/STE_K4_distributed +$ID
ssh -t root@192.168.1.$ID 'cd wanting; rm -r log_STE_multi; ./STE_K4_distributed'

# gather log files
current_date=$(date +"%Y-%m-%d")
current_time=$(date +"%H-%M")
mkdir -p Results_$current_date/$current_time
cp $STE_DDPM_ROOT/Khepera/Data/source_position.txt log_STE_multi/
mv $STE_DDPM_ROOT/Khepera/log_STE_multi Results_$current_date/$current_time/pc
k4get log_STE_multi +$ID
mv $STE_DDPM_ROOT/Khepera/k4-192.168.1."$ID"/log_STE_multi Results_$current_date/$current_time/k4



# ---------------- CONFIG ---------------- #
ID_list=(102 106)
map=1
session_name="robots_view"
config_file="$STE_DDPM_ROOT/Khepera/Data/config.txt"
binary_path="$STE_DDPM_ROOT/Khepera/STE_K4_distributed/STE_K4_distributed"
STE_DDPM_ROOT=${STE_DDPM_ROOT:-/your/path/to/project/root}  # Set default if not defined
cp $STE_DDPM_ROOT/Khepera/Data/Env_map/test_map"$map".environment $STE_DDPM_ROOT/Khepera/Data/test_map.environment
cp $STE_DDPM_ROOT/Khepera/Data/Obstacle_lists/"$map"_obstacles.txt $STE_DDPM_ROOT/Khepera/Data/obstacle_list.txt
cp $STE_DDPM_ROOT/Khepera/Data/DeepCFD_result/gas_concentration_maps_"$map".csv $STE_DDPM_ROOT/Khepera/Data/gas_concentration_maps.csv
# ---------------------------------------- #

# Update config.txt with ID_list
ID_list_str=$(IFS=,; echo "${ID_list[*]}")
sed -i -E "s/^ID_list *= *.*/ID_list=$ID_list_str/" "$config_file"
echo "✅ Updated config.txt with ID_list=$ID_list_str"

# Build the executable
cd "$STE_DDPM_ROOT/Khepera/STE_K4_distributed" || exit 1
make

cd "$STE_DDPM_ROOT/Khepera" || exit 1

# Upload config & binary to each robot
for ID in "${ID_list[@]}"; do
    echo "📤 Uploading to Robot $ID..."
    k4put "$config_file" +$ID
    k4put "$binary_path" +$ID
    k4put $STE_DDPM_ROOT/Khepera/Data/test_map.environment +$ID
    # obstacle list
    k4put $STE_DDPM_ROOT/Khepera/Data/obstacle_list.txt +$ID
    k4put $STE_DDPM_ROOT/Khepera/Data/source_position.txt +$ID
    # gas concentration maps
    k4put $STE_DDPM_ROOT/Khepera/Data/gas_concentration_maps.csv +$ID
    echo "✅ Uploaded to Robot $ID"
done

# Prepare result folders
current_date=$(date +"%Y-%m-%d")
current_time=$(date +"%H-%M")
results_path="Results_$current_date/$current_time"
mkdir -p "$results_path/k4"

# Kill any existing tmux session
tmux has-session -t $session_name 2>/dev/null && tmux kill-session -t $session_name

# Create tmux session
tmux new-session -d -s $session_name
tmux rename-window -t $session_name "server-and-robots"
tmux split-window -h -t $session_name  # Split server (left) and robots (right)

# Start server monitor in left pane
tmux select-pane -t 0
id_args="${ID_list[*]}"
tmux send-keys "echo '🚀 Starting server...'; bash server_monitor_stop.sh $id_args" C-m

# Create robot panes (split vertically for each additional robot)
robot_count=${#ID_list[@]}
if [ "$robot_count" -gt 1 ]; then
    for ((i = 1; i < robot_count; i++)); do
        tmux split-window -v -t $session_name
    done
fi

tmux select-layout -t $session_name tiled

# Launch each robot and gather logs
for ((i = 0; i < robot_count; i++)); do
    pane_index=$((i + 1))
    robot_id="${ID_list[$i]}"
    tmux select-pane -t $pane_index

    run_command="echo '🤖 Connecting to Robot $robot_id'; \
ssh root@192.168.1.$robot_id 'cd wanting; rm -r log_STE_multi; ./STE_K4_distributed'; \
echo '📥 Gathering logs from Robot $robot_id'; \
k4get log_STE_multi +$robot_id; \
mv \"$STE_DDPM_ROOT/Khepera/k4-192.168.1.$robot_id/log_STE_multi\" \"$results_path/k4/log_STE_multi_$robot_id\"; \
echo '✅ Logs saved for Robot $robot_id'"
tmux send-keys "$run_command" C-m

done

# Optionally collect PC-side logs (add source_position.txt if needed)
cp "$STE_DDPM_ROOT/Khepera/Data/source_position.txt" "$results_path/" 2>/dev/null

# Focus back to server pane
tmux select-pane -t 0

# Attach to tmux session
tmux attach-session -t $session_name
