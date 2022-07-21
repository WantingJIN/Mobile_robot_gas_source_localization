#!/bin/bash

cd training_random_wind1.0_angle0
# ____
# |	 |
# |	 |
# |	 |
# |	 |______
# |_________|

# for as many times as we need
for i in {41..80}
do
	# if folder i exists, we skip
	if [ -d $i ]
	then
		continue
	fi

	# copy the initial folder
	cp -r 0/ $i/

	# determine the shape of the L-shape randomly
	s=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); s=rand()*4; print s; }')

	# determine the size of the L shape on x, y and thickness
	dX=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX=1+rand()*2; print dX; }')
	dY=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY=1+rand()*1; print dY; }')
	dL=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dL=0.5+rand()*0.5; print dL; }')

	if (( $(echo "$s < 1" |bc -l) )); then
		# 				_____(MAXX2,MAXY2)
		# 				|	|
		# 				|	|
		# 				|	|
		# 				|	|_____(MAXX1,MAXY1)
		# (MINX, MINY)	|_________|

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(8-d); print MINX1; }')
		MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		MAXX1=`echo $MINX1 + $dX | bc`
		MAXY1=`echo $MINY1 + $dL | bc`
		
		MINX2=$MINX1
		MINY2=$MINY1
		MAXX2=`echo $MINX2 + $dL | bc`
		MAXY2=`echo $MINY2 + $dY | bc`

	elif (( $(echo "$s < 2" |bc -l) )); then
		# 					  ____(MAXX2,MAXY2)
		# 					  |	  |
		# 					  |	  |
		# 					  |	  |
		# 				______|	  |(MAXX1,MAXY1)
		# (MINX1,MINY1)	|_____.___|
		#					(MINX2,MINY2)

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(8-d); print MINX1; }')
		MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		MAXX1=`echo $MINX1 + $dX | bc`
		MAXY1=`echo $MINY1 + $dL | bc`

		MINX2=`echo $MINX1 + $dX - $dL | bc`
		MINY2=`echo $MINY1 + 0 | bc`
		MAXX2=`echo $MINX2 + $dL | bc`
		MAXY2=`echo $MINY2 + $dY | bc`

	elif (( $(echo "$s < 3" |bc -l) )); then
		#					(MAXX1,MAXY1)
		#				____.______(MAXX2,MAXY2)
		#  (MINX2,MINY2)|.	_______|
		# 				|	|
		# 				|	|
		# 				|	|
		# (MINX1,MINY1)	|___|

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(8-d); print MINX1; }')
		MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		MAXX1=`echo $MINX1 + $dL | bc`
		MAXY1=`echo $MINY1 + $dY | bc`

		MINX2=`echo $MINX1 + 0 | bc`
		MINY2=`echo $MINY1 + $dY - $dL | bc`
		MAXX2=`echo $MINX2 + $dX | bc`
		MAXY2=`echo $MINY2 + $dL | bc`

	else
		# 		____________(MAXX,MAXY)
		# 		|.______    |
		# (MINX2,MINY2)	|	|
		# 		  		|	|
		# 		  		|	|	
		# (MINX1,MINY1)	|___|

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MAXX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MAXX1=4.75+d+rand()*(8-d); print MAXX1; }')
		MAXY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MAXY1=0+d+rand()*(4-d); print MAXY1; }')
		MINX1=`echo $MAXX1 - $dL | bc`
		MINY1=`echo $MAXY1 - $dY | bc`

		MAXX2=$MAXX1
		MAXY2=$MAXY1
		MINX2=`echo $MAXX2 - $dX | bc`
		MINY2=`echo $MAXY2 - $dL | bc`

	fi

	# write the values in topoSetDict
	text1="boxes ((11.5 0 0) (12 1.5 2));"
	text2="boxes (($MINX1 $MINY1 0) ($MAXX1 $MAXY1 1) ($MINX2 $MINY2 0) ($MAXX2 $MAXY2 1));"
	sed -i -e :n -e "$!N;/\n.*\n/!{$!bn};  s/$text1/$text2/;P;D" $i/system/topoSetDict

	# run the simulation
	cd $i
	./Allrun_parallel.sh
	cd ..
done
#________________________________
