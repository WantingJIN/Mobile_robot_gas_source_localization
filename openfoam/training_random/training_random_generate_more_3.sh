#!/bin/bash

cd training_random_wind0.75_angle0

# _________________________		
# 		|	|			
# 		|	|			
# 		|	|_______			_____________
# 		|			|			|			|
# 		|___________|			|_______	|
# 										|	|
# 										|	|
# 						________________|___|_______

# for as many times as we need
for i in {161..180}
do
	# if folder i exists, we skip
	if [ -d $i ]
	then
		continue
	fi

	# copy the initial folder
	cp -r 0/ $i/

	######### 1 ##########
	# determine the shape of the L-shape randomly
	s=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); s=rand()*4; print s; }')

	# determine the size of the L shape on x, y and thickness
	dX=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX=1+rand()*1; print dX; }')
	dY=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY=1+rand()*1; print dY; }')
	dL=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dL=0.5+rand()*0.5; print dL; }')

	# stuck to the upper wall or lower
	upORdown=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); upORdown=rand()*3; print upORdown; }')

	if (( $(echo "$s < 1" |bc -l) )); then
		# 				_____(MAXX2,MAXY2)
		# 				|	|
		# 				|	|
		# 				|	|
		# 				|	|_____(MAXX1,MAXY1)
		# (MINX, MINY)	|_________|

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=7.75+rand()*(5-d); print MINX1; }')
		# stuck to the wall or not
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
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
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=7.75+rand()*(5-d); print MINX1; }')
		# stuck to the wall or not
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
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
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=7.75+rand()*(5-d); print MINX1; }')
		# stuck to the wall or not
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
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
		MAXX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MAXX1=7.75+d+rand()*(5-d); print MAXX1; }')
		# stuck to the wall or not
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MAXY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MAXY1=0+d+rand()*(4-d); print MAXY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MAXY1=4
		else
			MAXY1=$dY
		fi
		MINX1=`echo $MAXX1 - $dL | bc`
		MINY1=`echo $MAXY1 - $dY | bc`

		MAXX2=$MAXX1
		MAXY2=$MAXY1
		MINX2=`echo $MAXX2 - $dX | bc`
		MINY2=`echo $MAXY2 - $dL | bc`

	fi

	MINX3=$MINX1
	MINY3=$MINY1
	MAXX3=$MAXX1
	MAXY3=$MAXY1
	MINX4=$MINX2
	MINY4=$MINY2
	MAXX4=$MAXX2
	MAXY4=$MAXY2

	######### 2 ##########
	# determine the shape of the L-shape randomly
	s=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); s=rand()*4; print s; }')

	# determine the size of the L shape on x, y and thickness
	dX=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX=1+rand()*2; print dX; }')
	dY=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY=1+rand()*1; print dY; }')
	dL=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dL=0.5+rand()*0.5; print dL; }')

	# stuck to the upper wall or lower
	upORdown=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); upORdown=rand()*3; print upORdown; }')

	if (( $(echo "$s < 1" |bc -l) )); then
		# 				_____(MAXX2,MAXY2)
		# 				|	|
		# 				|	|
		# 				|	|
		# 				|	|_____(MAXX1,MAXY1)
		# (MINX, MINY)	|_________|

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(5-d); print MINX1; }')
		# stuck to the wall or not
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
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
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(5-d); print MINX1; }')
		# stuck to the wall or not
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
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
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(5-d); print MINX1; }')
		# stuck to the wall or not
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
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
		MAXX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MAXX1=4.75+d+rand()*(5-d); print MAXX1; }')
		# stuck to the wall or not
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MAXY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MAXY1=0+d+rand()*(4-d); print MAXY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MAXY1=4
		else
			MAXY1=$dY
		fi
		MINX1=`echo $MAXX1 - $dL | bc`
		MINY1=`echo $MAXY1 - $dY | bc`

		MAXX2=$MAXX1
		MAXY2=$MAXY1
		MINX2=`echo $MAXX2 - $dX | bc`
		MINY2=`echo $MAXY2 - $dL | bc`

	fi
	#######################

	# write the values in topoSetDict
	text1="boxes ((11.5 0 0) (12 1.5 2));"
	text2="boxes (($MINX1 $MINY1 0) ($MAXX1 $MAXY1 0.5) ($MINX2 $MINY2 0) ($MAXX2 $MAXY2 0.5) ($MINX3 $MINY3 0) ($MAXX3 $MAXY3 0.5) ($MINX4 $MINY4 0) ($MAXX4 $MAXY4 0.5));"
	sed -i -e :n -e "$!N;/\n.*\n/!{$!bn};  s/$text1/$text2/;P;D" $i/system/topoSetDict

	# run the simulation
	cd $i
	./Allrun_parallel.sh
	cd ..
done
#________________________________




# 						________________________________
#									|	|		|	|
#  		______ 		_____ 			|	|		|	|
# 		|	 |		|	|			|	|_______|	|
# 		|	 |		|	|			|_______________|
# 		|	 |		|	|			
# 		|	 |______|	|
# ______|_______________|_______


# for as many times as we need
for i in {181..200}
do
	# if folder i exists, we skip
	if [ -d $i ]
	then
		continue
	fi

	# copy the initial folder
	cp -r 0/ $i/

	######### 1 ##########
	# determine the shape of the L-shape randomly
	s=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); s=rand()*4; print s; }')

	# determine the size of the L shape on x, y and thickness
	dX=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX=1+rand()*1; print dX; }')
	dY=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY=1+rand()*1; print dY; }')
	dL=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dL=0.5+rand()*0.5; print dL; }')

	# stuck to the upper wall or lower
	upORdown=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); upORdown=rand()*3; print upORdown; }')

	if (( $(echo "$s < 1" |bc -l) )); then
		#				(MAXX2,MAXY2)	(MAXX3,MAXY3)
		# 				____.		____.
		# 				|	|		|	|
		# 				|	|		|	|
		# 				|	|		|	|
		# 				|	|_______|  .|(MAXX1,MAXY1)
		#(MINX12,MINY12)|___________.___|
		#							(MINX3, MINY3)

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=7.75+rand()*(5-d); print MINX1; }')
		# stuck to the wall or not
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
		MAXX1=`echo $MINX1 + $dX | bc`
		MAXY1=`echo $MINY1 + $dL | bc`
		
		MINX2=$MINX1
		MINY2=$MINY1
		MAXX2=`echo $MINX2 + $dL | bc`
		MAXY2=`echo $MINY2 + $dY | bc`

		MINX3=`echo $MINX1 + $dX - $dL | bc`
		MINY3=$MINY1
		MAXX3=`echo $MINX3 + $dL | bc`
		MAXY3=`echo $MINY3 + $dY | bc`

	elif (( $(echo "$s < 2" |bc -l) )); then
		# 				__________(MAXX23,MAXY23)
		# 				|_____ 	  |
		# 	(MINX3,MINY3)	  |	  |
		# 					  |	  |
		# 				______|	  |(MAXX1,MAXY1)
		# 				|_____.___|
		#  (MINX1,MINY1)	(MINX2,MINY2)

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=7.75+rand()*(5-d); print MINX1; }')
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
		MAXX1=`echo $MINX1 + $dX | bc`
		MAXY1=`echo $MINY1 + $dL | bc`

		MINX2=`echo $MINX1 + $dX - $dL | bc`
		MINY2=`echo $MINY1 + 0 | bc`
		MAXX2=`echo $MINX2 + $dL | bc`
		MAXY2=`echo $MINY2 + $dY | bc`

		MINX3=`echo $MINX1 + 0 | bc`
		MINY3=`echo $MINY1 + $dY - $dL | bc`
		MAXX3=`echo $MINX3 + $dX | bc`
		MAXY3=`echo $MINY3 + $dL | bc`

	elif (( $(echo "$s < 3" |bc -l) )); then
		#					(MAXX2,MAXY2)
		#				____.______(MAXX3,MAXY3)
		#  				|.	_______|
		#  (MINX3,MINY3)|	|
		# 				|	|
		# 				|	|______(MAXX1,MAXY1)
		# 				|__________|
		# (MINX12,MINY12)

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=7.75+rand()*(5-d); print MINX1; }')
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
		MAXX1=`echo $MINX1 + $dX | bc`
		MAXY1=`echo $MINY1 + $dL | bc`

		MINX2=$MINX1
		MINY2=$MINY1
		MAXX2=`echo $MINX2 + $dL | bc`
		MAXY2=`echo $MINY2 + $dY | bc`

		MINX3=$MINX1
		MINY3=`echo $MINY1 + $dY - $dL | bc`
		MAXX3=`echo $MINX3 + $dX | bc`
		MAXY3=`echo $MINY3 + $dL | bc`

	else
		#					(MAXX1,MAXY1)
		# 				____.___________(MAXX23,MAXY23)
		# 				|	._______   	|
		# (MINX2,MINY2)	|	|		|	|
		# 		  		|	|		|	|
		# 		  		|	|		|	|	
		# (MINX1,MINY1)	|___|		|.__|
		#							(MINX3,MINY3)

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=7.75+rand()*(5-d); print MINX1; }')
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
		MAXX1=`echo $MINX1 + $dL | bc`
		MAXY1=`echo $MINY1 + $dY | bc`

		MINX2=$MINX1
		MINY2=`echo $MINY1 + $dY - $dL | bc`
		MAXX2=`echo $MINX2 + $dX | bc`
		MAXY2=`echo $MINY2 + $dL | bc`

		MINX3=`echo $MINX1 + $dX - $dL | bc`
		MINY3=$MINY1
		MAXX3=`echo $MINX3 + $dL | bc`
		MAXY3=`echo $MINY3 + $dY | bc`

	fi

	MINX4=$MINX1
	MINY4=$MINY1
	MAXX4=$MAXX1
	MAXY4=$MAXY1
	MINX5=$MINX2
	MINY5=$MINY2
	MAXX5=$MAXX2
	MAXY5=$MAXY2
	MINX6=$MINX3
	MINY6=$MINY3
	MAXX6=$MAXX3
	MAXY6=$MAXY3

	######### 2 ##########
	# determine the shape of the L-shape randomly
	s=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); s=rand()*4; print s; }')

	# determine the size of the L shape on x, y and thickness
	dX=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dX=1+rand()*1; print dX; }')
	dY=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dY=1+rand()*1; print dY; }')
	dL=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); dL=0.5+rand()*0.5; print dL; }')

	# stuck to the upper wall or lower
	upORdown=$(awk -v seed="$RANDOM" 'BEGIN { srand(seed); upORdown=rand()*3; print upORdown; }')

	if (( $(echo "$s < 1" |bc -l) )); then
		#				(MAXX2,MAXY2)	(MAXX3,MAXY3)
		# 				____.		____.
		# 				|	|		|	|
		# 				|	|		|	|
		# 				|	|		|	|
		# 				|	|_______|  .|(MAXX1,MAXY1)
		#(MINX12,MINY12)|___________.___|
		#							(MINX3, MINY3)

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(5-d); print MINX1; }')
		# stuck to the wall or not
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
		MAXX1=`echo $MINX1 + $dX | bc`
		MAXY1=`echo $MINY1 + $dL | bc`
		
		MINX2=$MINX1
		MINY2=$MINY1
		MAXX2=`echo $MINX2 + $dL | bc`
		MAXY2=`echo $MINY2 + $dY | bc`

		MINX3=`echo $MINX1 + $dX - $dL | bc`
		MINY3=$MINY1
		MAXX3=`echo $MINX3 + $dL | bc`
		MAXY3=`echo $MINY3 + $dY | bc`

	elif (( $(echo "$s < 2" |bc -l) )); then
		# 				__________(MAXX23,MAXY23)
		# 				|_____ 	  |
		# 	(MINX3,MINY3)	  |	  |
		# 					  |	  |
		# 				______|	  |(MAXX1,MAXY1)
		# 				|_____.___|
		#  (MINX1,MINY1)	(MINX2,MINY2)

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(5-d); print MINX1; }')
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
		MAXX1=`echo $MINX1 + $dX | bc`
		MAXY1=`echo $MINY1 + $dL | bc`

		MINX2=`echo $MINX1 + $dX - $dL | bc`
		MINY2=`echo $MINY1 + 0 | bc`
		MAXX2=`echo $MINX2 + $dL | bc`
		MAXY2=`echo $MINY2 + $dY | bc`

		MINX3=`echo $MINX1 + 0 | bc`
		MINY3=`echo $MINY1 + $dY - $dL | bc`
		MAXX3=`echo $MINX3 + $dX | bc`
		MAXY3=`echo $MINY3 + $dL | bc`

	elif (( $(echo "$s < 3" |bc -l) )); then
		#					(MAXX2,MAXY2)
		#				____.______(MAXX3,MAXY3)
		#  				|.	_______|
		#  (MINX3,MINY3)|	|
		# 				|	|
		# 				|	|______(MAXX1,MAXY1)
		# 				|__________|
		# (MINX12,MINY12)

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(5-d); print MINX1; }')
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
		MAXX1=`echo $MINX1 + $dX | bc`
		MAXY1=`echo $MINY1 + $dL | bc`

		MINX2=$MINX1
		MINY2=$MINY1
		MAXX2=`echo $MINX2 + $dL | bc`
		MAXY2=`echo $MINY2 + $dY | bc`

		MINX3=$MINX1
		MINY3=`echo $MINY1 + $dY - $dL | bc`
		MAXX3=`echo $MINX3 + $dX | bc`
		MAXY3=`echo $MINY3 + $dL | bc`

	else
		#					(MAXX1,MAXY1)
		# 				____.___________(MAXX23,MAXY23)
		# 				|	._______   	|
		# (MINX2,MINY2)	|	|		|	|
		# 		  		|	|		|	|
		# 		  		|	|		|	|	
		# (MINX1,MINY1)	|___|		|.__|
		#							(MINX3,MINY3)

		# generate random values for MINX, MINY, MAXX, MAXY (1 L-shape)
		MINX1=$(awk -v d="$dX" -v seed="$RANDOM" 'BEGIN { srand(seed); MINX1=4.75+rand()*(5-d); print MINX1; }')
		if (( $(echo "$upORdown < 1" |bc -l) )); then
			MINY1=$(awk -v d="$dY" -v seed="$RANDOM" 'BEGIN { srand(seed); MINY1=0+rand()*(4-d); print MINY1; }')
		elif (( $(echo "$upORdown < 2" |bc -l) )); then
			MINY1=`echo 4 - $dY | bc`
		else
			MINY1=0
		fi
		MAXX1=`echo $MINX1 + $dL | bc`
		MAXY1=`echo $MINY1 + $dY | bc`

		MINX2=$MINX1
		MINY2=`echo $MINY1 + $dY - $dL | bc`
		MAXX2=`echo $MINX2 + $dX | bc`
		MAXY2=`echo $MINY2 + $dL | bc`

		MINX3=`echo $MINX1 + $dX - $dL | bc`
		MINY3=$MINY1
		MAXX3=`echo $MINX3 + $dL | bc`
		MAXY3=`echo $MINY3 + $dY | bc`

	fi
	#######################

	# write the values in topoSetDict
	text1="boxes ((11.5 0 0) (12 1.5 2));"
	text2="boxes (($MINX1 $MINY1 0) ($MAXX1 $MAXY1 0.5) ($MINX2 $MINY2 0) ($MAXX2 $MAXY2 0.5) ($MINX3 $MINY3 0) ($MAXX3 $MAXY3 0.5) ($MINX4 $MINY4 0) ($MAXX4 $MAXY4 0.5) ($MINX5 $MINY5 0) ($MAXX5 $MAXY5 0.5) ($MINX6 $MINY6 0) ($MAXX6 $MAXY6 0.5));"
	sed -i -e :n -e "$!N;/\n.*\n/!{$!bn};  s/$text1/$text2/;P;D" $i/system/topoSetDict

	# run the simulation
	cd $i
	./Allrun_parallel.sh
	cd ..
done
#________________________________
