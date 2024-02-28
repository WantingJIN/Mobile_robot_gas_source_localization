#!/bin/sh

#get the number of the current map
name="$(basename -- $PWD)"
read n <<<${name//[^0-9]/ }
echo $n 

#define wind values
for w in 0
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
	text1="value           uniform (0 0 0);\n    }\n    outlet"
	text2="value           uniform (-$w 0 0);\n    }\n    outlet"
	sed -i -e :n -e "$!N;/\n.*\n/!{$!bn};  s/$text1/$text2/;P;D" 0/U
	./Allrun.sh
	cd ..
done

