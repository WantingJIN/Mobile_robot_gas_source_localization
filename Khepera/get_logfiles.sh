#!/bin/bash


# robot's ID
ID=106

# gather log files
current_time=$(date +"%Y-%m-%d_%H-%M")
mkdir Results_1202/$current_time
cp $STE_DDPM_ROOT/Khepera/Data/source_position.txt log_STE_multi/
mv $STE_DDPM_ROOT/Khepera/log_STE_multi Results_0808/$current_time/pc
k4get log_STE_multi +$ID
mv $STE_DDPM_ROOT/Khepera/k4-192.168.1."$ID"/log_STE_multi Results_1202/$current_time/k4
