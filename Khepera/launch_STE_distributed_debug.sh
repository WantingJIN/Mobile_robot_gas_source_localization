# #!/bin/bash

# # Robot list
# ID_list=(102 106)
# session_name="robots_view"

# # Step 1: Update config.txt
# config_file="$STE_DDPM_ROOT/Khepera/Data/config.txt"
# ID_list_str=$(IFS=,; echo "${ID_list[*]}")
# sed -i -E "s/^ID_list *= *.*/ID_list=$ID_list_str/" "$config_file"
# echo "Updated config.txt with ID_list=$ID_list_str"

# # Step 2: Upload config & binary to each robot
# cd $STE_DDPM_ROOT/Khepera/STE_K4_distributed
# make

# for ID in "${ID_list[@]}"; do
#     echo "Uploading to Robot $ID..."
#     k4put "$config_file" +$ID
#     k4put "$STE_DDPM_ROOT/Khepera/STE_K4_distributed/STE_K4_distributed" +$ID
# done

# # Step 3: Create a new tmux session (detached) for robot outputs
# session_name="robot_run"

# # Kill any existing session
# tmux has-session -t $session_name 2>/dev/null && tmux kill-session -t $session_name

# # Create session with a controller pane (left side)
# tmux new-session -d -s $session_name -n main bash  # start with a bash shell

# # Split vertically: right side will be for robot views
# tmux split-window -h -t $session_name:0 -p 70       # 70% right side for robot views

# # Select right pane (robot side)
# tmux select-pane -t $session_name:0.1

# # First robot gets first split
# ID=${ID_list[0]}
# tmux send-keys -t $session_name:0.1 "ssh root@192.168.1.$ID 'cd wanting; ./STE_K4_distributed'" C-m

# # Loop through remaining robots, stacking them vertically
# for i in "${!ID_list[@]}"; do
#     if [ $i -eq 0 ]; then continue; fi
#     ID=${ID_list[$i]}
#     tmux split-window -v -t $session_name:0.1 bash
#     tmux send-keys -t $session_name:0.$((i+1)) "ssh root@192.168.1.$ID 'cd wanting; ./STE_K4_distributed'" C-m
# done

# # Optional: tile the robot sub-panes
# tmux select-layout -t $session_name:0 tiled

# # Always return focus to left controller pane
# tmux select-pane -t $session_name:0.0

# # Attach the session so the user sees it
# tmux attach -t $session_name

# # Step 6: Show message
# echo ">>> Robot programs running in tmux session '$session_name'"
# echo ">>> You can attach with: tmux attach-session -t $session_name"
# echo ">>> Press Ctrl+C here to stop both robots."

# # Step 7: Define cleanup function
# cleanup() {
#     echo ">>> Interrupt detected. Stopping robot programs and motors..."
#     for ID in "${ID_list[@]}"; do
#         ssh root@192.168.1.$ID "pkill -f STE_K4_distributed; ~/home/motor_stop" &
#     done
#     wait
#     echo ">>> All robots stopped."
#     # Optional: Kill the tmux session
#     tmux kill-session -t $session_name
#     exit 1
# }

# # Step 8: Trap SIGINT and SIGQUIT
# trap cleanup SIGINT SIGQUIT

# # Step 9: Keep script alive while robots are running
# while true; do sleep 1; done


# ID_list=(102 106)
# session_name="robot_run"

# # === IF NOT INSIDE TMUX: launch tmux and run this script again inside ===
# if [ -z "$TMUX" ]; then
#     echo "Launching tmux session..."
#     tmux new-session -d -s "$session_name" "$0 inside_tmux"
#     tmux attach-session -t "$session_name"
#     exit
# fi

# # === INSIDE TMUX: now safe to create layout ===
# if [ "$1" == "inside_tmux" ]; then
#     echo ">>> Controller ready. Press Ctrl+C or Ctrl+\\ to stop all robots."
#     echo ">>> Robot outputs are in right panes."

#     # Trap for stopping robots
#     cleanup() {
#         echo ""
#         echo ">>> Interrupt received. Stopping all robots..."
#         for ID in "${ID_list[@]}"; do
#             ssh root@192.168.1.$ID "pkill -f STE_K4_distributed; ~/home/motor_stop" &
#         done
#         wait
#         echo ">>> Cleanup complete. Press Ctrl+D to exit."
#     }

#     trap cleanup SIGINT SIGQUIT

#     # Split right panel
#     tmux split-window -h -p 70

#     # First robot in right pane
#     ID=${ID_list[0]}
#     tmux send-keys -t 1 "ssh root@192.168.1.$ID 'cd wanting; ./STE_K4_distributed'" C-m

#     # Split vertically for other robots
#     for i in "${!ID_list[@]}"; do
#         if [ "$i" -eq 0 ]; then continue; fi
#         ID=${ID_list[$i]}
#         tmux split-window -v -t 1 bash
#         new_pane=$(tmux display-message -p "#{pane_id}")
#         tmux send-keys -t "$new_pane" "ssh root@192.168.1.$ID 'cd wanting; ./STE_K4_distributed'" C-m
#     done

#     tmux select-layout tiled
#     tmux select-pane -t 0  # focus controller pane

#     # Keep controller pane alive
#     while true; do sleep 1; done
# fi

#!/bin/bash

# # Start a new tmux session (or attach if already inside one)
# SESSION_NAME="multi_robot_view"
# tmux has-session -t $SESSION_NAME 2>/dev/null

# if [ $? != 0 ]; then
#     tmux new-session -d -s $SESSION_NAME
# fi

# # Rename window for clarity
# tmux rename-window -t $SESSION_NAME "server-and-clients"

# # Select the session
# tmux select-window -t $SESSION_NAME

# # Split window vertically: left (server), right
# tmux select-pane -t 0
# tmux split-window -h -t $SESSION_NAME

# # In left pane (pane 0): run server
# tmux select-pane -t 0
# tmux send-keys "echo 'Starting server...'" C-m

# # Split right pane (pane 1) into top and bottom
# tmux select-pane -t 1
# tmux split-window -v -t $SESSION_NAME

# # In top-right pane (pane 1): connect to first remote client
# tmux select-pane -t 1
# tmux send-keys "k4go +102" C-m

# # In bottom-right pane (pane 2): connect to second remote client
# tmux select-pane -t 2
# tmux send-keys "k4go +106" C-m

# # Focus back to the left (server) pane
# tmux select-pane -t 0

# # Attach to the session
# tmux attach-session -t $SESSION_NAME

#!/bin/bash

# ---------------- CONFIG ---------------- #
ID_list=(102 106)
session_name="robots_view"
config_file="$STE_DDPM_ROOT/Khepera/Data/config.txt"
binary_path="$STE_DDPM_ROOT/Khepera/STE_K4_distributed/STE_K4_distributed"
# ---------------------------------------- #

# Convert ID_list to comma-separated string and update config.txt
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
done

#prepare results folder
current_date=$(date +"%Y-%m-%d")
current_time=$(date +"%H-%M")  
results_path="Results_$current_date/$current_time"
mkdir -p "$results_path" 


# Kill any existing session
tmux has-session -t $session_name 2>/dev/null && tmux kill-session -t $session_name

# Create or attach to tmux session
tmux has-session -t $session_name 2>/dev/null
if [ $? != 0 ]; then
    tmux new-session -d -s $session_name
fi

tmux rename-window -t $session_name "server-and-robots"

# Split left (server) and right (robots)
tmux select-window -t $session_name
tmux select-pane -t 0
tmux split-window -h -t $session_name

# Run server in left pane
tmux select-pane -t 0
id_args="${ID_list[*]}"
tmux send-keys "echo '🚀 Starting server...'; bash server_monitor_stop.sh $id_args " C-m

# Focus on right pane
tmux select-pane -t 1

# Create robot panes (split vertically for each additional robot)
robot_count=${#ID_list[@]}
if [ "$robot_count" -gt 1 ]; then
    for ((i = 1; i < robot_count; i++)); do
        tmux split-window -v -t $session_name
    done
fi

# Layout the right side evenly
tmux select-layout -t $session_name tiled

# Assign each robot connection
for ((i = 0; i < robot_count; i++)); do
    pane_index=$((i + 1))  # since pane 0 is server
    tmux select-pane -t $pane_index
    robot_id="${ID_list[$i]}"
    tmux send-keys "echo '🤖 Connecting to Robot $robot_id'; ssh root@192.168.1.$robot_id 'cd wanting; ./STE_K4_distributed' " C-m
done

# Return focus to the server pane (left side)
tmux select-pane -t 0

# Attach session
tmux attach-session -t $session_name



