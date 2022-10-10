#!/bin/sh

#get the number of the current map
name="$(basename -- $PWD)"
read n <<<${name//[^0-9]/ }
echo $n 

#define wind values
for w in 0 0.1
do
	d=map${n}_wind$w
	if [ $w != 0 ]
	then
		cp -r map${n}_wind0 $d
	fi
	#go in the subfolder
	cd $d
	./Allremove.sh
	#change the wind speed
	text1="internalField   uniform (-0.75 0 0);\n"
	text2="internalField   uniform (-$w 0 0);\n"
	sed -i -e :n -e "$!N;/\n.*\n/!{$!bn};  s/$text1/$text2/;P;D" 0/U
	./Allrun.sh
	cd ..
done

