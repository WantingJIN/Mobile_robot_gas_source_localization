#!/bin/bash

#for all the maps
for n in ./map*
do
	cd $n
	echo "removing results in $n"
	./remove_all_wind_speeds.sh
	cd ..
done

