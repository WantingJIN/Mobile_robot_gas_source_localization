#include "SimulationParameters.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include <unistd.h>

eScenario parse_scenario(const char* str) {
	if (strcmp(str, "NO_COM_NO_COOR") == 0) return NO_COM_NO_COOR;
	if (strcmp(str, "COM_NO_COOR") == 0) return COM_NO_COOR;
	if (strcmp(str, "COM_COOR") == 0) return COM_COOR;
	fprintf(stderr, "Unknown scenario: %s\n", str);
	exit(1);
}
eEvaluationMethod parse_evaluation_method(const char* str) {
	if (strcmp(str, "MCMC") == 0) return MCMC;
	if (strcmp(str, "EXHAUSTIVE") == 0) return EXHAUSTIVE;
	fprintf(stderr, "Unknown evaluation method: %s\n", str);
	exit(1);
}
eProcessingMode parse_processing_mode(const char* str) {
    if (strcmp(str, "NO_PROCESSING") == 0) return NO_PROCESSING;
    if (strcmp(str, "KERNEL_MAPPING") == 0) return KERNEL_MAPPING;
    if (strcmp(str, "DECONVOLUTION") == 0) return DECONVOLUTION;
    if (strcmp(str, "MAPPING_AND_DECONVOLUTION") == 0) return MAPPING_AND_DECONVOLUTION;
    fprintf(stderr, "Unknown processing mode: %s\n", str);
    exit(1);
}
ePlanningStrategy parse_planning_strategy(const char* str) {
	if (strcmp(str, "GREEDY") == 0) return GREEDY;
	if (strcmp(str, "BALANCE_MAXIMAL_EXPECTED") == 0) return BALANCE_MAXIMAL_EXPECTED;
	if (strcmp(str, "RANDOM") == 0) return RANDOM;
	if (strcmp(str, "LAWNMOWER") == 0) return LAWNMOWER;
	if (strcmp(str, "BELIEF_CLUSTERING") == 0) return BELIEF_CLUSTERING;
	if (strcmp(str, "INFORMATIVE_KLD") == 0) return INFORMATIVE_KLD;
	if (strcmp(str, "INFO_CLUSTERING") == 0) return INFO_CLUSTERING;
	fprintf(stderr, "Unknown planning strategy: %s\n", str);
	exit(1);
}

eGasFeature parse_gas_feature(const char* str) {
	if (strcmp(str, "ABSOLUTE_VALUE") == 0) return ABSOLUTE_VALUE;
	if (strcmp(str, "GASHIT") == 0) return GASHIT;
	if (strcmp(str, "RELATIVE_VALUE") == 0) return RELATIVE_VALUE;
	if (strcmp(str, "PDF") == 0) return PDF;
	fprintf(stderr, "Unknown gas feature: %s\n", str);
	exit(1);
}
eVisitedList parse_visited_list(const char* str) {
	if (strcmp(str, "visitedList_ON") == 0) return visitedList_ON;
	if (strcmp(str, "visitedList_OFF") == 0) return visitedList_OFF;
	fprintf(stderr, "Unknown visited list: %s\n", str);
	exit(1);
}
eInfoFeature parse_info_feature(const char* str) {
	if (strcmp(str, "VARIANCE") == 0) return VARIANCE;
	if (strcmp(str, "MEAN") == 0) return MEAN;
	fprintf(stderr, "Unknown info feature: %s\n", str);
	exit(1);
}
eRelativeMethod parse_relative_method(const char* str) {
	if (strcmp(str, "NORMALIZATION") == 0) return NORMALIZATION;
	if (strcmp(str, "GAUSSIAN_CDF") == 0) return GAUSSIAN_CDF;
	if (strcmp(str, "M_DISTANCE") == 0) return M_DISTANCE;
	if (strcmp(str, "EMPIRICAL_CDF") == 0) return EMPIRICAL_CDF;
	fprintf(stderr, "Unknown relative method: %s\n", str);
	exit(1);
}
eWorkingMode parse_work_mode(const char* str) {
	if (strcmp(str, "OFFLINE") == 0) return OFFLINE;
	if (strcmp(str, "EXPERIMENT") == 0) return EXPERIMENT;
	if (strcmp(str, "VIDEO") == 0) return VIDEO;
	fprintf(stderr, "Unknown work mode: %s\n", str);
	exit(1);
}
eCalibrationMode parse_calibration_mode(const char* str) {
	if (strcmp(str, "OFF") == 0) return OFF;
	if (strcmp(str, "LINEAR") == 0) return LINEAR;
	if (strcmp(str, "NOLINEAR") == 0) return NOLINEAR;
	fprintf(stderr, "Unknown calibration mode: %s\n", str);
	exit(1);
}
eGasHitMethod parse_gas_hit_method(const char* str) {
	if (strcmp(str, "FIXED_THRES") == 0) return FIXED_THRES;
	if (strcmp(str, "MIN_MAX_THRES") == 0) return MIN_MAX_THRES;
	if (strcmp(str, "SLIDING_WINDOW_THRES") == 0) return SLIDING_WINDOW_THRES;
	if (strcmp(str, "FIXED_THRES_AND_GAUSSIAN_CDF") == 0) return FIXED_THRES_AND_GAUSSIAN_CDF;
	fprintf(stderr, "Unknown gas hit method: %s\n", str);
	exit(1);
}

eReleaseRate parse_release_rate(const char* str) {
	if (strcmp(str, "LOW") == 0) return LOW;
	if (strcmp(str, "MIDDLE") == 0) return MIDDLE;
	if (strcmp(str, "HIGH") == 0) return HIGH;
	fprintf(stderr, "Unknown release rate: %s\n", str);
	exit(1);
}

SimulationParameters::SimulationParameters(){
	// default values
	// simulation parameters
	likelihood_error = 30;
	global_local_search_ratio = 0.5; 
	MCMC_iteration_num = 1000; 
	MCMC_or_exhaustive_eval = EXHAUSTIVE; 
	number_of_robots = 1; 
	scenario = NO_COM_NO_COOR; 
	map_nbr = 0; 
	//samplingMode = CONTINOUS_SAMPLING; 
      processingMode = NO_PROCESSING; 
	//gasSensor = PID;
	planningStrategy = BELIEF_CLUSTERING; 
	gasFeature = GASHIT; 
	visitedList = visitedList_ON;
	infoFeature = VARIANCE; 
	//sampling_period = 1; 
	cluster_entropy_ratio = 10; 
	//vel_factor = 10; 
	sampling_dist_thresh = 0.1; 
	relativeMethod = EMPIRICAL_CDF; 
	gasHitMethod = SLIDING_WINDOW_THRES; 
    calibrationMode = OFF;
	workingMode = EXPERIMENT;
	measurement_error = 0;
	sigma = 0.45;
	releaseRate = LOW;
	//simuMode = ONLINE; 
	//R0 = 100;
	//gas_sensor_noise_var = 40000;
}


void SimulationParameters::load_simulation_parameters_from_config(const char *filename){
	// read parameters from file
	FILE *file = fopen(filename, "r");
	if (!file) {
		fprintf(stderr, "Error opening config file\n");
		exit(1);
	}

	char key[256], value[256];
	while (fscanf(file, "%255s = %255s", key, value) == 2) {
		if (strcmp(key, "likelihood_error") == 0) {
			likelihood_error = atof(value);
		} else if (strcmp(key, "global_local_search_ratio") == 0) {
			global_local_search_ratio = atof(value);
		} else if (strcmp(key, "MCMC_or_exhaustive_eval") == 0) {
			MCMC_or_exhaustive_eval = parse_evaluation_method(value);
		} else if (strcmp(key, "number_of_robots") == 0) {
			number_of_robots = atoi(value);
		} else if (strcmp(key, "scenario") == 0) {
			scenario = parse_scenario(value);
		} else if (strcmp(key, "map_nbr") == 0) {
			map_nbr = atoi(value);
		} else if (strcmp(key, "MCMC_iteration_num") == 0) {
			MCMC_iteration_num = atoi(value);
		} else if (strcmp(key, "processingMode") == 0) {
			processingMode = parse_processing_mode(value);
		} else if (strcmp(key, "planningStrategy") == 0) {
			planningStrategy = parse_planning_strategy(value);
		} else if (strcmp(key, "gasFeature") == 0) {
			gasFeature = parse_gas_feature(value);
		} else if (strcmp(key, "visitedList") == 0) {
			visitedList = parse_visited_list(value);
		} else if (strcmp(key, "infoFeature") == 0) {
			infoFeature = parse_info_feature(value);
		} else if (strcmp(key, "cluster_entropy_ratio") == 0) {
			cluster_entropy_ratio = atof(value);
		} else if (strcmp(key, "relativeMethod") == 0) {
			relativeMethod = parse_relative_method(value);
		} else if (strcmp(key, "gasHitMethod") == 0) {
			gasHitMethod = parse_gas_hit_method(value);
		} else if (strcmp(key, "gasHitThres") == 0) {
			gas_hit_threshold = atoi(value);
		} else if (strcmp(key, "calibrationMode") == 0) {
			calibrationMode = parse_calibration_mode(value);
		} else if (strcmp(key, "workMode") == 0) {
			workingMode = parse_work_mode(value);
		} else if (strcmp(key, "measurement_error") == 0) {
			measurement_error = atoi(value);
		} else if (strcmp(key, "sigma") == 0) {
			sigma = atof(value);	
		} else if (strcmp(key, "sampling_dist_thresh") == 0) {
			sampling_dist_thresh = atof(value);	
		} else if (strcmp(key, "releaseRate") == 0) {
			releaseRate = parse_release_rate(value);
		} else if (strcmp(key, "experimental_logname") == 0) {
			strcpy(experimental_logname, value);
		} else {
			fprintf(stderr, "Unknown parameter: %s\n", key);
			exit(1);
		}
	}

    fclose(file);
}
void SimulationParameters::rewrite_parameters_from_file(){
	// read simulation parameters
	//read simulation parameters
	FILE* sim_param_file;
	int ok = 0;
	// string sim_param_file_path;

	// sim_param_file_path = string(workspacePath) + "/Khepera/Data/params/map_nbr.txt";
	// if(access(sim_param_file_path.c_str(), F_OK) != -1 ){
	// 	sim_param_file = fopen(sim_param_file_path.c_str(), "r");
	// 	ok = fscanf(sim_param_file, "%d", &map_nbr);
	// 	if(ok) printf("map number : %d\n", map_nbr);
	// 	fclose(sim_param_file);
	// }
	//FIXME - temporary solution: should modify the config file directly
      if (access("Data/params/map_nbr.txt", F_OK) != -1)
	{
		sim_param_file = fopen("Data/params/map_nbr.txt", "r");
		ok = fscanf(sim_param_file, "%d", &map_nbr);
		if (ok)
			printf("map : %d\n", map_nbr);
		fclose(sim_param_file);
	}
}

void SimulationParameters::log_simulation_parameters(const char *config_logfile_name){
	// write simulation parameters to file
	FILE *sim_param_file;
	sim_param_file = fopen(config_logfile_name, "a");
	fprintf(sim_param_file, "likelihood_error %f\n", likelihood_error);
	fprintf(sim_param_file, "global_local_search_ratio %f\n", global_local_search_ratio);
	fprintf(sim_param_file, "MCMC_iteration_num %d\n", MCMC_iteration_num);
	fprintf(sim_param_file, "MCMC_or_exhaustive_eval %d\n", MCMC_or_exhaustive_eval);
	fprintf(sim_param_file, "number_of_robots %d\n", number_of_robots);
	fprintf(sim_param_file, "scenario %d\n", scenario);
	fprintf(sim_param_file, "map_nbr %d\n", map_nbr);
	fprintf(sim_param_file, "data_processing_mode %d\n", processingMode);
	fprintf(sim_param_file, "planning_strategy %d\n", planningStrategy);
	fprintf(sim_param_file, "gas_feature %d\n", gasFeature);
	fprintf(sim_param_file, "visited_list %d\n", visitedList);
	fprintf(sim_param_file, "info_feature %d\n", infoFeature);
	fprintf(sim_param_file, "cluster_ratio %f\n", cluster_entropy_ratio);
	fprintf(sim_param_file, "relative_method %d\n", relativeMethod);
	fprintf(sim_param_file, "calibration_mode %d\n", calibrationMode);
	fprintf(sim_param_file, "gas_hit_method %d\n", gasHitMethod);
	fprintf(sim_param_file, "gas_hit_threshold %d\n", gas_hit_threshold);
	fprintf(sim_param_file, "working_mode %d\n", workingMode);
	fprintf(sim_param_file, "measurement_error %d\n", measurement_error);
	fprintf(sim_param_file, "sigma %f\n", sigma);
	fprintf(sim_param_file, "sampling_dist_thresh %f\n", sampling_dist_thresh);
	fprintf(sim_param_file, "release_rate %d\n", releaseRate);
	fclose(sim_param_file);
}


SimulationParameters::~SimulationParameters(){
}