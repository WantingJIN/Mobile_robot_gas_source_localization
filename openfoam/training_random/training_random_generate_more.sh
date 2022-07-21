#!/bin/bash

cd training_random_wind1.0_angle0

# ________________________
# 		|			|
# 		|			|
# 		|			|
# 		|			|
# 		|___________|

# for as many times as we need
for i in {121..130}
do
	# if folder i exists, we skip
	if [ -d $i ]
	then
		continue
	fi

	# copy the initial folder
	cp -r 0/ $i/

	# generate random values for MINX, MINY, MAXX, MAXY (1 rectangle)
	dX1=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX1=1+rand()*4; print dX1; }')
	dY1=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY1=1+rand()*2; print dY1; }')
	MINX1=$(awk -v d="$dX1" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(8-d); print MINX1; }')
	# stuck to the upper wall or lower
	upORdown=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); upORdown=rand(); print upORdown; }')
	if (( $(echo "$upORdown > 0.5" |bc -l) )); then
		MINY1=`echo 4 - $dY1 | bc` 	#up
	else
		MINY1=0 					#down
	fi
	MAXX1=`echo $MINX1 + $dX1 | bc`
	MAXY1=`echo $MINY1 + $dY1 | bc`

	# write the values in topoSetDict
	text1="boxes ((11.5 0 0) (12 1.5 2));"
	text2="boxes (($MINX1 $MINY1 0) ($MAXX1 $MAXY1 1));"
	sed -i -e :n -e "$!N;/\n.*\n/!{$!bn};  s/$text1/$text2/;P;D" $i/system/topoSetDict

	# run the simulation
	cd $i
	./Allrun_parallel.sh
	cd ..
done
#________________________________


# _________________________		
# 		|			|			
# 		|			|			
# 		|			|			_____________
# 		|			|			|			|
# 		|___________|			|			|
# 								|			|
# 								|			|
# 						________|___________|_______

# for as many times as we need
for i in {131..140}
do
	# if folder i exists, we skip
	if [ -d $i ]
	then
		continue
	fi

	# copy the initial folder
	cp -r 0/ $i/

	# generate random values for MINX, MINY, MAXX, MAXY (2 rectangles)
	dX1=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX1=1+rand()*2; print dX1; }')
	dY1=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY1=1+rand()*1.5; print dY1; }')
	MINX1=$(awk -v d="$dX1" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(5-d); print MINX1; }')
	# stuck to the upper wall or lower
	upORdown=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); upORdown=rand(); print upORdown; }')
	if (( $(echo "$upORdown > 0.5" |bc -l) )); then
		MINY1=`echo 4 - $dY1 | bc` 	#up
	else
		MINY1=0 					#down
	fi
	MAXX1=`echo $MINX1 + $dX1 | bc`
	MAXY1=`echo $MINY1 + $dY1 | bc`

	dX2=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX2=1+rand()*2; print dX2; }')
	dY2=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY2=1+rand()*1.5; print dY2; }')
	MINX2=$(awk -v d="$dX2" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX2=7.75+rand()*(5-d); print MINX2; }')
	# stuck to the opposite wall as the other one
	if (( $(echo "$upORdown > 0.5" |bc -l) )); then
		MINY2=0
	else
		MINY2=`echo 4 - $dY2 | bc`
	fi
	MAXX2=`echo $MINX2 + $dX2 | bc`
	MAXY2=`echo $MINY2 + $dY2 | bc`

	# write the values in topoSetDict
	text1="boxes ((11.5 0 0) (12 1.5 2));"
	text2="boxes (($MINX1 $MINY1 0) ($MAXX1 $MAXY1 1) ($MINX2 $MINY2 0) ($MAXX2 $MAXY2 1));"
	sed -i -e :n -e "$!N;/\n.*\n/!{$!bn};  s/$text1/$text2/;P;D" $i/system/topoSetDict

	# run the simulation
	cd $i
	./Allrun_parallel.sh
	cd ..
done
