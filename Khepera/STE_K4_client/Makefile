#CC			:= arm-angstrom-linux-gnueabi-g++
CC			:= arm-angstrom-linux-gnueabi-g++-4.6

CFLAGS_ADD := -std=c++0x -pthread
#-std=c++11

#
#Need to add an export line in .bashrc like
#export K4_ROOT=/home/Roro/Documents/Epfl/master_project/code/khepera4.git

MODULES	:= khepera4 commandline odometry_track odometry_goto i2cal i2c_stream nmea odorboard windsensorboard cortex

#Y_MODULES	:= /home/disal/khepera4toolbox/Modules/
#MY_MODULES_FOLDER := /Modules

TARGET	:= STE_K4_client

#INCS_ADD	:= -I /home/disal/projects/Faezeh/Eigen

include Makefile.include

# ---------------------------------------------------#

# original Makefile :
# MODULES	:= khepera3 commandline i2cal odometry_track
# MY_MODULES			:= odorsensor windsensor
# MY_MODULES_FOLDER	:= ../../Modules
#
# TARGET	:= odortracking_zigzag
#
# include $(K3_ROOT)/Programs/Makefile.include
