#!/bin/bash
# NOTE: for debugging purposes, without sending all the documents to the robots; with full experiment run with different maps, please use launch_STE_distributed.sh
# ---------------- CONFIG ---------------- #
ID_list=(103)
source_position="11 2"
session_name="robots_view"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STE_DDPM_ROOT=${STE_DDPM_ROOT:-$(dirname "$SCRIPT_DIR")}
config_file="$STE_DDPM_ROOT/Khepera/Data/config.yaml"
source_position_file="$STE_DDPM_ROOT/Khepera/Data/source_position.txt"
binary_path="$STE_DDPM_ROOT/Khepera/STE_K4_distributed/STE_K4_distributed"
target_sysroot="${SDKTARGETSYSROOT:-/opt/poky/1.8/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi}"
superlu_soname_lib="$target_sysroot/usr/lib/libsuperlu.so.5"
superlu_lib="$(readlink -f "$superlu_soname_lib" 2>/dev/null || true)"
[ -n "$superlu_lib" ] || superlu_lib="$superlu_soname_lib"
superlu_runtime_name="$(basename "$superlu_lib")"
blas_lib="$target_sysroot/usr/lib/libblas.so"
runtime_libs=("$superlu_lib" "$blas_lib")
# ---------------------------------------- #

#set the map number
map=9
echo "+++ Map number $map +++"

cp $STE_DDPM_ROOT/Khepera/Data/Env_map/test_map"$map".environment $STE_DDPM_ROOT/Khepera/Data/test_map.environment
cp $STE_DDPM_ROOT/Khepera/Data/Obstacle_lists/"$map"_obstacles.txt $STE_DDPM_ROOT/Khepera/Data/obstacle_list.txt
#cp $STE_DDPM_ROOT/Khepera/Data/DeepCFD_result/gas_concentration_maps_"$map".csv $STE_DDPM_ROOT/Khepera/Data/gas_concentration_maps.csv



# Check if map files need to be re-uploaded
config_map_nbr=$(grep -E "^[[:space:]]*map_nbr:" "$config_file" | sed -E "s/^[[:space:]]*map_nbr:[[:space:]]*([0-9]+).*/\1/")
if [ "$config_map_nbr" = "$map" ]; then
    upload_map_files=false
    echo "ℹ️  Map $map already in config.yaml, skipping test_map.environment and obstacle_list.txt upload"
else
    upload_map_files=true
fi

# Update config.yaml with ID_list and map_nbr
ID_list_str=$(IFS=,; echo "${ID_list[*]}")
sed -i -E "s/^([[:space:]]*)ID_list:[[:space:]]*.*/\1ID_list: [$ID_list_str]/" "$config_file"
sed -i -E "s/^([[:space:]]*)map_nbr:[[:space:]]*.*/\1map_nbr: $map/" "$config_file"
sed -i -E "s/^([[:space:]]*)number_of_robots:[[:space:]]*.*/\1number_of_robots: ${#ID_list[@]}/" "$config_file"
echo "✅ Updated config.yaml with ID_list=[$ID_list_str], map_nbr=$map, number_of_robots=${#ID_list[@]}"

# Write source position to file
echo "$source_position" > "$source_position_file"
echo "✅ Updated source_position.txt with source_position=$source_position"

# Build the executable
cd "$STE_DDPM_ROOT/Khepera/STE_K4_distributed" || exit 1
make

for runtime_lib in "${runtime_libs[@]}"; do
    if [ ! -f "$runtime_lib" ]; then
        echo "❌ Missing runtime library: $runtime_lib"
        exit 1
    fi
done

cd "$STE_DDPM_ROOT/Khepera" || exit 1




# Upload config & binary to each robot
for ID in "${ID_list[@]}"; do
    echo "📤 Uploading to Robot $ID..."
    k4put "$config_file" +$ID
    k4put "$binary_path" +$ID
    for runtime_lib in "${runtime_libs[@]}"; do
        k4put "$runtime_lib" +$ID
    done
    ssh root@192.168.1.$ID "cd wanting; ln -sf '$superlu_runtime_name' libsuperlu.so.5; ln -sf '$superlu_runtime_name' libsuperlu.so"
    # environment map and obstacle list (skip if map unchanged)
    if [ "$upload_map_files" = true ]; then
        k4put $STE_DDPM_ROOT/Khepera/Data/test_map.environment +$ID
        k4put $STE_DDPM_ROOT/Khepera/Data/obstacle_list.txt +$ID
    fi
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

# Kill any existing tmux session (skip if we're currently inside it)
current_tmux_session=""
[ -n "$TMUX" ] && current_tmux_session=$(tmux display-message -p '#S')
if tmux has-session -t "$session_name" 2>/dev/null && [ "$current_tmux_session" != "$session_name" ]; then
    tmux kill-session -t "$session_name"
fi

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
ssh root@192.168.1.$robot_id 'cd wanting; rm -rf log_STE_multi; export LD_LIBRARY_PATH=\$PWD:\${LD_LIBRARY_PATH:-}; stdbuf -oL -eL ./STE_K4_distributed 2>&1 | tee robot_output.log'; \
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
