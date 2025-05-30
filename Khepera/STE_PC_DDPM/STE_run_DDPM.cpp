/*
* File:          controller_STE_PC.cpp
* Date:          May 2026
* Description:   The goal is to calculate the results of the DDPM and used for the K4 controller later
* Author:        Wanting Jin
* Modifications: 26/01/2019
*/

//custom libraries
#include <fstream>
//C libraries
#include <assert.h>
#include <math.h>
#include <stdio.h> //printf
#include <stdlib.h> //srand
#include <sys/stat.h> // mkdir
#include <sys/time.h>
#include <time.h> //time
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib> // for getenv
#include "PlumeModel.h"
#include "Position.h"
#include "constant_and_helper.h"
#include "CellMeasurements.h"
#include "GasDispersionMap.h"


using namespace std;

const char *workspacePath = getenv("STE_DDPM_ROOT");

DeepCFDPlumeModel2D plume_model;


// =========================== MAIN ============================== //

int main(int argc, char **argv){
      // read the environmental map
	FILE* sim_param_file;
	int ok = 0;
	int map_nbr = 0;
	string map_file_path = string(workspacePath) + "/Khepera/Data/params/map_nbr.txt";
	// check if the file exists
	if (access(map_file_path.c_str(), F_OK) != -1)
	{
		sim_param_file = fopen(map_file_path.c_str(), "r");
		ok = fscanf(sim_param_file, "%d", &map_nbr);
		fclose(sim_param_file);
	}
	// Precompute gas concentration maps for all possible source locations, resolution 20x20
	printf("loading the model and initializing the gas concentration maps...\n");
	plume_model.initialize_potential_gas_maps();
	// save the gas concentration maps to a file 
	string gas_concentration_map_logfile_name = string(workspacePath)+ "/Khepera/Data/DeepCFD_result/gas_concentration_maps_" + std::to_string(map_nbr) + ".csv";
	// if the file already exists, delete it
	if (access(gas_concentration_map_logfile_name.c_str(), F_OK) != -1)
	{
		remove(gas_concentration_map_logfile_name.c_str());
	}
	// FILE * gas_concentration_map_logfile = fopen(gas_concentration_map_logfile_name.c_str(), "w");
	// fclose(gas_concentration_map_logfile);
	plume_model.log_gas_concentration_map(gas_concentration_map_logfile_name.c_str(), 'a');
	printf("The gas concentration maps are saved!\n");
	return 0;
}

