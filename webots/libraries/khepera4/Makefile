CC=gcc
INCLUDE=-I"$(WEBOTS_HOME)/include/controller/c"
COMPILER_OPTS= $(CC) -g -Wall -fPIC -Werror -c $(INCLUDE) -lgsl -lgslcblas -o

libk4_webots.a: khepera4_drive.o khepera4.o khepera4_infrared.o khepera4_ultrasound.o odometry_track.o commandline.o utilities.o relativepositionboard.o globalpose.o
	ar rcs $@ $^

khepera4_drive.o: khepera4_drive.c
	$(COMPILER_OPTS) $@ $^

khepera4.o: khepera4.c
	$(COMPILER_OPTS) $@ $^

khepera4_infrared.o: khepera4_infrared.c
	$(COMPILER_OPTS) $@ $^

khepera4_ultrasound.o: khepera4_ultrasound.c
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

	


