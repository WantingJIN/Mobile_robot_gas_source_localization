#!/bin/bash

#for all wind speeds
for w in map*_wind*
do
	#make sure we don't remove wind0
	read num1 num2 <<<${w//[^0-9]/ }
	if (( ${#num2} == 1 )) && (( $num2 == 0 ))
	then
		#we keep wind 0
		echo "keeping $w"
		cd $w
		./Allremove.sh
		cd ..
 	else
 		#we remove any other wind speeds
 		echo "removing $w"
 		rm -r $w
 	fi
done

