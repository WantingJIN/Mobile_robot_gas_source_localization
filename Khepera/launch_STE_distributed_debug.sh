#!/bin/bash
# NOTE: for debugging purposes, without sending all the documents to the robots; with full experiment run with different maps, please use launch_STE_distributed.sh
# ---------------- CONFIG ---------------- #
ID_list=(102 105 103)
source_position="11 3"
session_name="robots_view"
config_file="$STE_DDPM_ROOT/Khepera/Data/config.txt"
source_position_file="$STE_DDPM_ROOT/Khepera/Data/source_position.txt"
binary_path="$STE_DDPM_ROOT/Khepera/STE_K4_distributed/STE_K4_distributed"
STE_DDPM_ROOT=${STE_DDPM_ROOT:-/your/path/to/project/root}  # Set default if not defined
# ---------------------------------------- #

# Update config.txt with ID_list
ID_list_str=$(IFS=,; echo "${ID_list[*]}")
sed -i -E "s/^ID_list *= *.*/ID_list = $ID_list_str/" "$config_file"
echo "✅ Updated config.txt with ID_list=$ID_list_str"

# Write source position to file
echo "$source_position" > "$source_position_file"
echo "✅ Updated source_position.txt with source_position=$source_position"

# Build the executable
cd "$STE_DDPM_ROOT/Khepera/STE_K4_distributed" || exit 1
make

cd "$STE_DDPM_ROOT/Khepera" || exit 1

# Upload config & binary to each robot
for ID in "${ID_list[@]}"; do
    echo "📤 Uploading to Robot $ID..."
    k4put "$config_file" +$ID
    k4put "$binary_path" +$ID
done

# Prepare result folders
current_date=$(date +"%Y-%m-%d")
current_time=$(date +"%H-%M")
results_path="Results_$current_date/$current_time"
mkdir -p "$results_path/k4"
mkdir -p "$results_path/terminal"

# Pre-create empty log files for each robot
for ID in "${ID_list[@]}"; do
    touch "$results_path/terminal/$ID.txt"
done


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
ssh root@192.168.1.$robot_id 'cd wanting; rm -r log_STE_multi; ./STE_K4_distributed 2>&1 | tee robot_output.log'; \
echo '📥 Gathering logs from Robot $robot_id'; \
k4get log_STE_multi +$robot_id; \
k4get robot_output.log +$robot_id; \
mv \"$STE_DDPM_ROOT/Khepera/k4-192.168.1.$robot_id/log_STE_multi\" \"$results_path/k4/log_STE_multi_$robot_id\"; \
mv \"$STE_DDPM_ROOT/Khepera/k4-192.168.1.$robot_id/robot_output.log\" \"$results_path/terminal/$robot_id.txt\"; \
echo '✅ Logs saved for Robot $robot_id'"
tmux send-keys "$run_command" C-m

done

# Optionally collect PC-side logs (add source_position.txt if needed)
cp "$STE_DDPM_ROOT/Khepera/Data/source_position.txt" "$results_path/" 2>/dev/null

# Focus back to server pane
tmux select-pane -t 0

# Attach to tmux session
tmux attach-session -t $session_name

