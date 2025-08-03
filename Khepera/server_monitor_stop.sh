#!/bin/bash
set -m

ID_list=("$@")

cleanup() {
    echo ""
    echo ">>> 🔴 Interrupt received. Stopping all robots..."
    for ID in "${ID_list[@]}"; do
        echo ">>> Stopping robot $ID..."
        (
            ssh root@192.168.1.$ID 'pkill -f STE_K4_distributed'
            ssh root@192.168.1.$ID './motor_stop'   
        ) &
    done
    wait
    echo ">>> ✅ Cleanup complete."
    exit 0
}

trap cleanup SIGINT SIGQUIT

echo ">>> 🟢 Server started. Press Ctrl+C or Ctrl+\ to terminate and stop robots."

while true; do
    sleep 1
done
