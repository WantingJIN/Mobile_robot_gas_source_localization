#!/bin/bash

#for all the maps
for n in 9 #{0..10}
do
	cd map$n
	echo $PWD
	#run for different wind speeds
	for w in 0.75
	do
		d=map${n}_wind$w
		echo $d
		#for wind speeds other than 0
		if [ $w != 0 ]
		then
			#if the folder does not exist (i.e. that wind speed was not executed)
			if [ ! -d "$d" ]
			then
				echo $d does not exist
				cp -r map${n}_wind0 $d
				#go in the subfolder
				cd $d
				#change the wind speed in the file U.orig in folder 0
				text1="internalField   uniform (-0.75 0 0);"
				text2="internalField   uniform (-$w 0 0);"
				sed -i -e :n -e "$!N;/\n.*\n/!{$!bn};  s/$text1/$text2/;P;D" 0/U.orig
				./Allrun_parallel.sh
                #clean logs
				rm log.*
				rm -r 5/
				cd ..
			else
				echo $d already exists! Nothing to be done here.
			fi
		else
			#for folder 0, check if it was already executed
			if [ ! -d "$d/VTK" ]
			then
				#go in the subfolder
				cd $d
				./Allrun_parallel.sh
				cd ..
			fi
		fi
	done
	cd ..
done

