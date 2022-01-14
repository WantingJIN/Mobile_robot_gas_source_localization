CC=gcc
INCLUDE=-I"$(WEBOTS_HOME)/include/controller/c"
COMPILER_OPTS= $(CC) -g -Wall -Werror -c $(INCLUDE) -lgsl -lgslcblas -o

libk3_webots.a: khepera3_drive.o khepera3.o khepera3_infrared.o khepera3_ultrasound.o odometry_track.o commandline.o utilities.o relativepositionboard.o globalpose.o
	ar rcs $@ $^

khepera3_drive.o: khepera3_drive.c
	$(COMPILER_OPTS) $@ $^

khepera3.o: khepera3.c
	$(COMPILER_OPTS) $@ $^

khepera3_infrared.o: khepera3_infrared.c
	$(COMPILER_OPTS) $@ $^

khepera3_ultrasound.o: khepera3_ultrasound.c
	$(COMPILER_OPTS) $@ $^

odometry_track.o: odometry_track.c
	$(COMPILER_OPTS) $@ $^
	
utilities.o: utilities.c
	$(COMPILER_OPTS) $@ $^

relativepositionboard.o: relativepositionboard.c
	$(COMPILER_OPTS) $@ $^

commandline.o: commandline.c
	$(COMPILER_OPTS) $@ $^

globalpose.o: globalpose.c
	$(COMPILER_OPTS) $@ $^


clean:
	rm -f *.o *.a *~

	


