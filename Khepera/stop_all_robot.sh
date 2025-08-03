#!/bin/bash
set -m

ID_list=(102 106)

echo ">>> 🔴 Stopping all robots..."
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


