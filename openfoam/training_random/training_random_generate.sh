#!/bin/bash

cd training_random_wind1.0_angle0

# ___________
# |			|
# |			|
# |			|
# |			|
# |_________|

# for as many times as we need
for i in {0..20}
do
	# if folder i exists, we skip
	if [ -d $i ]
	then
		continue
	fi

	# copy the initial folder
	cp -r 0/ $i/

	# generate random values for MINX, MINY, MAXX, MAXY (1 rectangle)
	dX1=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX1=1+rand()*2; print dX1; }')
	dY1=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY1=1+rand()*1; print dY1; }')
	MINX1=$(awk -v d="$dX1" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(8-d); print MINX1; }')
	MINY1=$(awk -v d="$dY1" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
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

# ___________		_____________
# |			|		|			|
# |			|		|			|
# |			|		|			|
# |			|		|			|
# |_________|		|___________|

# for as many times as we need
for i in {21..40}
do
	# if folder i exists, we skip
	if [ -d $i ]
	then
		continue
	fi

	# copy the initial folder
	cp -r 0/ $i/

	# generate random values for MINX, MINY, MAXX, MAXY (2 rectangles)
	dX1=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX1=1+rand()*1; print dX1; }')
	dY1=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY1=1+rand()*1; print dY1; }')
	MINX1=$(awk -v d="$dX1" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(8-d); print MINX1; }')
	MINY1=$(awk -v d="$dY1" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
	MAXX1=`echo $MINX1 + $dX1 | bc`
	MAXY1=`echo $MINY1 + $dY1 | bc`

	dX2=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX2=1+rand()*2; print dX2; }')
	dY2=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY2=1+rand()*2; print dY2; }')
	MINX2=$(awk -v d="$dX2" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX2=4.75+rand()*(8-d); print MINX2; }')
	MINY2=$(awk -v d="$dY2" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY2=0+rand()*(4-d); print MINY2; }')
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