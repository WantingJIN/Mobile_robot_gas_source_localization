#include "SimulationParameters.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

eEvaluationMethod parse_evaluation_method(const char *str)
{
	if (strcmp(str, "MCMC") == 0)
		return MCMC;
	if (strcmp(str, "EXHAUSTIVE") == 0)
		return EXHAUSTIVE;
	fprintf(stderr, "Unknown evaluation method: %s\n", str);
	exit(1);
}
eSamplingMode parse_sampling_mode(const char *str)
{
	if (strcmp(str, "CONTINOUS_SAMPLING") == 0)
		return CONTINOUS_SAMPLING;
	if (strcmp(str, "STOP_AND_GO_SAMPLING") == 0)
		return STOP_AND_GO_SAMPLING;
	if (strcmp(str, "SENSE_WITHOUT_STOP") == 0)
		return SENSE_WITHOUT_STOP;
	fprintf(stderr, "Unknown sampling mode: %s\n", str);
	exit(1);
}
eGasSensor parse_gas_sensor(const char *str)
{
	if (strcmp(str, "PID") == 0)
		return PID;
	if (strcmp(str, "MOX") == 0)
		return MOX;
	if (strcmp(str, "NOISY_MOX") == 0)
		return NOISY_MOX;
	if (strcmp(str, "SLOW_MOX") == 0)
		return SLOW_MOX;
	if (strcmp(str, "VERY_SLOW_MOX") == 0)
		return VERY_SLOW_MOX;
	if (strcmp(str, "NOISY_SLOW_MOX") == 0)
		return NOISY_SLOW_MOX;
	if (strcmp(str, "NOISY_VERY_SLOW_MOX") == 0)
		return NOISY_VERY_SLOW_MOX;
	fprintf(stderr, "Unknown gas sensor: %s\n", str);
	exit(1);
}
eProcessingMode parse_processing_mode(const char *str)
{
	if (strcmp(str, "NO_PROCESSING") == 0)
		return NO_PROCESSING;
	if (strcmp(str, "KERNEL_MAPPING") == 0)
		return KERNEL_MAPPING;
	if (strcmp(str, "DECONVOLUTION") == 0)
		return DECONVOLUTION;
	if (strcmp(str, "MAPPING_AND_DECONVOLUTION") == 0)
		return MAPPING_AND_DECONVOLUTION;
	fprintf(stderr, "Unknown processing mode: %s\n", str);
	exit(1);
}
ePlanningStrategy parse_planning_strategy(const char *str)
{
	if (strcmp(str, "COMBINED") == 0)
		return COMBINED;
	if (strcmp(str, "GREEDY") == 0)
		return GREEDY;
	if (strcmp(str, "INFO_CENTER") == 0)
		return INFO_CENTER;
	if (strcmp(str, "FARTHEST_CELL") == 0)
		return FARTHEST_CELL;
	if (strcmp(str, "BALANCE_MAXIMAL_EXPECTED") == 0)
		return BALANCE_MAXIMAL_EXPECTED;
	if (strcmp(str, "RANDOM") == 0)
		return RANDOM;
	if (strcmp(str, "LAWNMOWER") == 0)
		return LAWNMOWER;
	if (strcmp(str, "BELIEF_CLUSTERING") == 0)
		return BELIEF_CLUSTERING;
	if (strcmp(str, "INFORMATIVE_KLD") == 0)
		return INFORMATIVE_KLD;
	if (strcmp(str, "INFO_CLUSTERING") == 0)
		return INFO_CLUSTERING;
	if (strcmp(str, "WEIGHTED_CLUSTERING") == 0)
		return WEIGHTED_CLUSTERING;
	if (strcmp(str, "SUM_DIJKSTRA_FIX") == 0)
		return SUM_DIJKSTRA_FIX;
	if (strcmp(str, "SUM_DIJKSTRA_ADP") == 0)
		return SUM_DIJKSTRA_ADP;
	if (strcmp(str, "MULTI_DIJKSTRA") == 0)
		return MULTI_DIJKSTRA;
	fprintf(stderr, "Unknown planning strategy: %s\n", str);
	exit(1);
}
eGasFeature parse_gas_feature(const char *str)
{
	if (strcmp(str, "ABSOLUTE_VALUE") == 0)
		return ABSOLUTE_VALUE;
	if (strcmp(str, "GASHIT") == 0)
		return GASHIT;
	if (strcmp(str, "RELATIVE_VALUE") == 0)
		return RELATIVE_VALUE;
	fprintf(stderr, "Unknown gas feature: %s\n", str);
	exit(1);
}
eVisitedList parse_visited_list(const char *str)
{
	if (strcmp(str, "visitedList_ON") == 0)
		return visitedList_ON;
	if (strcmp(str, "visitedList_OFF") == 0)
		return visitedList_OFF;
	fprintf(stderr, "Unknown visited list: %s\n", str);
	exit(1);
}
eInfoFeature parse_info_feature(const char *str)
{
	if (strcmp(str, "VARIANCE") == 0)
		return VARIANCE;
	if (strcmp(str, "MEAN") == 0)
		return MEAN;
	if (strcmp(str, "PDF") == 0)
		return PDF;
	fprintf(stderr, "Unknown info feature: %s\n", str);
	exit(1);
}
eRelativeMethod parse_relative_method(const char *str)
{
	if (strcmp(str, "NORMALIZATION") == 0)
		return NORMALIZATION;
	if (strcmp(str, "Z_SCORE") == 0)
		return Z_SCORE;
	if (strcmp(str, "EMPIRICAL_CDF") == 0)
		return EMPIRICAL_CDF;
	if (strcmp(str, "EMPIRICAL_CDF_AND_SLOPE") == 0)
		return EMPIRICAL_CDF_AND_SLOPE;
	if (strcmp(str, "EMPIRICAL_CDF_NUM") == 0)
		return EMPIRICAL_CDF_NUM;
	fprintf(stderr, "Unknown relative method: %s\n", str);
	exit(1);
}
eSimuMode parse_simu_mode(const char *str)
{
	if (strcmp(str, "ONLINE") == 0)
		return ONLINE;
	if (strcmp(str, "OFFLINE") == 0)
		return OFFLINE;
	fprintf(stderr, "Unknown simulation mode: %s\n", str);
	exit(1);
}
eCalibrationMode parse_calibration_mode(const char *str)
{
	if (strcmp(str, "OFF") == 0)
		return OFF;
	if (strcmp(str, "LINEAR") == 0)
		return LINEAR;
	if (strcmp(str, "NOLINEAR") == 0)
		return NOLINEAR;
	fprintf(stderr, "Unknown calibration mode: %s\n", str);
	exit(1);
}
eGasHitMethod parse_gas_hit_method(const char *str)
{
	if (strcmp(str, "FIXED_THRES") == 0)
		return FIXED_THRES;
	if (strcmp(str, "FIXED_THRES_NUM") == 0)
		return FIXED_THRES_NUM;
	if (strcmp(str, "MIN_MAX_THRES") == 0)
		return MIN_MAX_THRES;
	if (strcmp(str, "SLIDING_WINDOW_THRES") == 0)
		return SLIDING_WINDOW_THRES;
	if (strcmp(str, "SLIDING_WINDOW_THRES_NUM") == 0)
		return SLIDING_WINDOW_THRES_NUM;
	fprintf(stderr, "Unknown gas hit method: %s\n", str);
	exit(1);
}

eMapCoverageStrategy parse_map_coverage_strategy(const char* str) {
	if (strcmp(str, "VORONOI") == 0) return VORONOI;
	if (strcmp(str, "LLOYD_D") == 0) return LLOYD_D;
	if (strcmp(str, "LLOYD_P") == 0) return LLOYD_P;
	if (strcmp(str, "REGION_GROWING_P") == 0) return REGION_GROWING_P;
	if (strcmp(str, "REGION_GROWING_D") == 0) return REGION_GROWING_D;
	if (strcmp(str, "NONE") == 0) return NONE;
	fprintf(stderr, "Unknown cluster method: %s\n", str);
	exit(1);
}

eMapCoverageSeed parse_map_coverage_seed(const char* str) {
	if (strcmp(str, "MAX_CELL") == 0) return MAX_CELL;
	if (strcmp(str, "ROBOT_CELL") == 0) return ROBOT_CELL;
	fprintf(stderr, "Unknown cluster method: %s\n", str);
	exit(1);
}

eStopCondition parse_stop_condition(const char* str) {
	if (strcmp(str, "ENTROPY_ONLY") == 0) return ENTROPY_ONLY;
	if (strcmp(str, "SPREAD_ONLY") == 0) return SPREAD_ONLY;
	if (strcmp(str, "ENTROPY_SPREAD") == 0) return ENTROPY_SPREAD;
	fprintf(stderr, "Unknown stop condition: %s\n", str);
	exit(1);
}


eScoreFun parse_score_fun(const char *str)
{
	if (strcmp(str, "RATIO") == 0)
		return RATIO;
	if (strcmp(str, "LOG") == 0)
		return LOG;
	if (strcmp(str, "SIGMOID") == 0)
		return SIGMOID;
	if (strcmp(str, "SUM") == 0)
		return SUM;
	if (strcmp(str, "EXP") == 0)
		return EXP;
	fprintf(stderr, "Unknown score function: %s\n", str);
	exit(1);
}


ePlumeModel parse_plume_model(const char* str) {
	if (strcmp(str, "GAUSSIAN_PLUME") == 0) return GAUSSIAN_PLUME;
	if (strcmp(str, "DDPM") == 0) return DDPM;
	if (strcmp(str, "FDM_PDE") == 0) return FDM_PDE;
	fprintf(stderr, "Unknown plume model: %s\n", str);
	exit(1);
}

eWindEstimationMethod parse_wind_estimation_method(const char* str) {
	if (strcmp(str, "GROUND_TRUTH") == 0) return GROUND_TRUTH;
	if (strcmp(str, "GAUSSIAN_KERNEL") == 0) return GAUSSIAN_KERNEL;
	if (strcmp(str, "BASEFLOW_CORRECTION") == 0) return BASEFLOW_CORRECTION;
	fprintf(stderr, "Unknown wind estimation method: %s\n", str);
	exit(1);
}

eWindInfo parse_wind_info(const char* str) {
	if (strcmp(str, "WIND_OFFLINE") == 0) return WIND_OFFLINE;
	if (strcmp(str, "WIND_ONLINE") == 0) return WIND_ONLINE;
	fprintf(stderr, "Unknown wind info: %s\n", str);
	exit(1);
}

eModelUncertainty parse_model_uncertainty(const char* str) {
	if (strcmp(str, "FIXED_UNCERTAINTY") == 0) return FIXED_UNCERTAINTY;
	if (strcmp(str, "DYNAMIC_UNCERTAINTY") == 0) return DYNAMIC_UNCERTAINTY;
	fprintf(stderr, "Unknown model uncertainty: %s\n", str);
	exit(1);
}

SimulationParameters::SimulationParameters(const char *filename){
	load_simulation_parameters_from_config(filename);
}

eTeamStrategy parse_team_strategy(const char* str) {
	if (strcmp(str, "INDIVIDUAL") == 0) return INDIVIDUAL;
	if (strcmp(str, "DISTRIBUTED") == 0) return DISTRIBUTED;
	fprintf(stderr, "Unknown team strategy: %s\n", str);
	exit(1);
}

void SimulationParameters::load_simulation_parameters_from_config(const char *filename)
{
	YAML::Node config;
	try {
		config = YAML::LoadFile(filename);
	} catch (const YAML::Exception& e) {
		fprintf(stderr, "Error loading config file '%s': %s\n", filename, e.what());
		exit(1);
	}

	// --- simulation ---
	{
		auto n = config["simulation"];
		simuMode    = parse_simu_mode(n["simuMode"].as<std::string>().c_str());
		map_nbr     = n["map_nbr"].as<int>();
		strcpy(experimental_logname, n["experimental_logname"].as<std::string>().c_str());
	}

	// --- robot ---
	{
		auto n = config["robot"];
		number_of_robots = n["number_of_robots"].as<int>();
		vel_factor       = n["vel_factor"].as<float>();
	}

	// --- sensing ---
	{
		auto n = config["sensing"];
		samplingMode    = parse_sampling_mode(n["samplingMode"].as<std::string>().c_str());
		processingMode  = parse_processing_mode(n["processingMode"].as<std::string>().c_str());
		gasSensor       = parse_gas_sensor(n["gasSensor"].as<std::string>().c_str());
		calibrationMode = parse_calibration_mode(n["calibrationMode"].as<std::string>().c_str());

		// sampling_period and sampling_dist_thresh are nested under the active sampling mode
		std::string samplingModeStr = n["samplingMode"].as<std::string>();
		if (n[samplingModeStr]) {
			auto s = n[samplingModeStr];
			sampling_period      = s["sampling_period"].as<int>();
			sampling_dist_thresh = s["sampling_dist_thresh"].as<float>();
		}

		// NOLINEAR sub-params
		if (calibrationMode == NOLINEAR && n["NOLINEAR"]) {
			R0 = n["NOLINEAR"]["R0"].as<float>();
		}
	}

	// --- gas_feature ---
	{
		auto n = config["gas_feature"];
		gasFeature = parse_gas_feature(n["gasFeature"].as<std::string>().c_str());

		// RELATIVE_VALUE sub-params
		if (gasFeature == RELATIVE_VALUE && n["RELATIVE_VALUE"]) {
			auto rv = n["RELATIVE_VALUE"];
			std::string rmStr = rv["relativeMethod"].as<std::string>();
			relativeMethod = parse_relative_method(rmStr.c_str());
			// likelihoodSigma is nested under the active relativeMethod
			if (rv[rmStr] && rv[rmStr]["likelihoodSigma"]) {
				likelihoodSigma = rv[rmStr]["likelihoodSigma"].as<float>();
			}
		}
		if (gasFeature == ABSOLUTE_VALUE && n["ABSOLUTE_VALUE"]) {
			auto av = n["ABSOLUTE_VALUE"];
			likelihoodSigma = av["likelihoodSigma"].as<float>();
		}

		// GASHIT sub-params
		if (gasFeature == GASHIT && n["GASHIT"]) {
			auto gh = n["GASHIT"];
			gasHitMethod = parse_gas_hit_method(gh["gasHitMethod"].as<std::string>().c_str());
			// threshold is shared by FIXED_THRES and FIXED_THRES_NUM
			if ((gasHitMethod == FIXED_THRES || gasHitMethod == FIXED_THRES_NUM) && gh["FIXED_THRES"]) {
				gas_hit_threshold = gh["FIXED_THRES"]["gasHitThres"].as<int>();
			}
			// lambda is shared by SLIDING_WINDOW_THRES and SLIDING_WINDOW_THRES_NUM
			if (gasHitMethod == SLIDING_WINDOW_THRES && gh["SLIDING_WINDOW_THRES"]) {
				lambda = gh["SLIDING_WINDOW_THRES"]["lambda"].as<float>();
			}
			if (gasHitMethod == SLIDING_WINDOW_THRES_NUM && gh["SLIDING_WINDOW_THRES_NUM"]) {
				lambda = gh["SLIDING_WINDOW_THRES_NUM"]["lambda"].as<float>();
			}
		}
	}

	// --- bayesian_estimation ---
	{
		auto n = config["bayesian_estimation"];
		BayesianModelingMethod = parse_evaluation_method(
			n["BayesianModelingMethod"].as<std::string>().c_str());

		// MCMC sub-params
		if (BayesianModelingMethod == MCMC && n["MCMC"]) {
			MCMC_iteration_num = n["MCMC"]["MCMC_iteration_num"].as<int>();
		}
	}

	// --- plume_model ---
	{
		auto n = config["plume_model"];
		plumeModel = parse_plume_model(n["plumeModel"].as<std::string>().c_str());

		// FDM_PDE sub-params
		if (plumeModel == FDM_PDE && n["FDM_PDE"]) {
			diff_coefficient = n["FDM_PDE"]["diffCoefficient"].as<float>();
		}
	}

	// --- wind ---
	{
		auto n = config["wind"];
		windInfo              = parse_wind_info(n["windInfo"].as<std::string>().c_str());
		windEstimationMethod  = parse_wind_estimation_method(
			n["windEstimationMethod"].as<std::string>().c_str());
		modelUncertainty      = parse_model_uncertainty(n["modelUncertainty"].as<std::string>().c_str());

		// DYNAMIC_UNCERTAINTY sub-params
		if (modelUncertainty == DYNAMIC_UNCERTAINTY && n["DYNAMIC_UNCERTAINTY"]) {
			wind_belief_weight = n["DYNAMIC_UNCERTAINTY"]["windBeliefWeight"].as<float>();
		}
	}

	// --- planning ---
	{
		auto n = config["planning"];
		planningStrategy      = parse_planning_strategy(n["planningStrategy"].as<std::string>().c_str());
		visitedList           = parse_visited_list(n["visitedList"].as<std::string>().c_str());
		infoFeature           = parse_info_feature(n["infoFeature"].as<std::string>().c_str());
		scoreFun              = parse_score_fun(n["scoreFun"].as<std::string>().c_str());
		scoreSigma            = n["scoreSigma"].as<float>();
		planning_step         = n["planning_step"].as<int>();
		global_local_search_ratio = n["global_local_search_ratio"].as<float>();

		// BELIEF_CLUSTERING / INFO_CLUSTERING sub-params
		if ((planningStrategy == BELIEF_CLUSTERING || planningStrategy == INFO_CLUSTERING)
		     && n["BELIEF_CLUSTERING"]) {
			cluster_entropy_ratio = n["BELIEF_CLUSTERING"]["cluster_entropy_ratio"].as<float>();
		}

		// SUM_DIJKSTRA_ADP / WEIGHTED_CLUSTERING sub-params
		if ((planningStrategy == SUM_DIJKSTRA_ADP || planningStrategy == WEIGHTED_CLUSTERING)
		     && n["SUM_DIJKSTRA_ADP"]) {
			weighting_entropy_ratio = n["SUM_DIJKSTRA_ADP"]["weighting_entropy_ratio"].as<float>();
		}
	}

	// --- team ---
	{
		auto n = config["team"];
		teamStrategy     = parse_team_strategy(n["teamStrategy"].as<std::string>().c_str());
		mapCoverageStrategy = parse_map_coverage_strategy(n["mapCoverageStrategy"].as<std::string>().c_str());
		mapCoverageSeed = parse_map_coverage_seed(n["mapCoverageSeed"].as<std::string>().c_str());
		epsilon_clusterize = n["epsilon_clusterize"].as<float>();
	}
	// --- termination ---
	{
		auto n = config["termination"];
		stopCondition = parse_stop_condition(n["stopCondition"].as<std::string>().c_str());
	}
}

void SimulationParameters::read_mapID_from_file()
{
	// read simulation parameters
	FILE *sim_param_file;
	int ok = 0;

	if (access("../../data/map_nbr.txt", F_OK) != -1)
	{
		sim_param_file = fopen("../../data/map_nbr.txt", "r");
		ok = fscanf(sim_param_file, "%d", &map_nbr);
		if (ok)
			printf("map ID : %d\n", map_nbr);
		fclose(sim_param_file);
	}
}

void SimulationParameters::log_simulation_parameters(const char *config_logfile_name)
{
	// write simulation parameters to file
	FILE *sim_param_file;
	sim_param_file = fopen(config_logfile_name, "a");
	fprintf(sim_param_file, "global_local_search_ratio %f\n", global_local_search_ratio);
	fprintf(sim_param_file, "MCMC_iteration_num %d\n", MCMC_iteration_num);
	fprintf(sim_param_file, "BayesianModelingMethod %d\n", BayesianModelingMethod);
	fprintf(sim_param_file, "number_of_robots %d\n", number_of_robots);
	fprintf(sim_param_file, "map_nbr %d\n", map_nbr);
	fprintf(sim_param_file, "sampling_mode %d\n", samplingMode);
	fprintf(sim_param_file, "data_processing_mode %d\n", processingMode);
	fprintf(sim_param_file, "gas_sensor_type %d\n", gasSensor);
	fprintf(sim_param_file, "planning_strategy %d\n", planningStrategy);
	fprintf(sim_param_file, "gas_feature %d\n", gasFeature);
	fprintf(sim_param_file, "visited_list %d\n", visitedList);
	fprintf(sim_param_file, "info_feature %d\n", infoFeature);
	fprintf(sim_param_file, "sampling_period %d\n", sampling_period);
	fprintf(sim_param_file, "cluster_ratio %f\n", cluster_entropy_ratio);
	fprintf(sim_param_file, "vel_factor %f\n", vel_factor);
	fprintf(sim_param_file, "sampling_dist_thresh %f\n", sampling_dist_thresh);
	fprintf(sim_param_file, "relative_method %d\n", relativeMethod);
	fprintf(sim_param_file, "simu_mode %d\n", simuMode);
	fprintf(sim_param_file, "calibration_mode %d\n", calibrationMode);
	fprintf(sim_param_file, "gas_hit_method %d\n", gasHitMethod);
	fprintf(sim_param_file, "gas_hit_threshold %d\n", gas_hit_threshold);
	fprintf(sim_param_file, "R0 %f\n", R0);
	fprintf(sim_param_file, "likelihood_sigma %f\n", likelihoodSigma);
	fprintf(sim_param_file, "lambda %f\n", lambda);
	fprintf(sim_param_file, "score_sigma %f\n", scoreSigma);
	fprintf(sim_param_file, "score_fun %d\n", scoreFun);
	fprintf(sim_param_file, "planning_step %d\n", planning_step);
	fprintf(sim_param_file, "weighting_entropy_ratio %f\n", weighting_entropy_ratio);
	fprintf(sim_param_file, "team_strategy %d\n", teamStrategy);
	fprintf(sim_param_file, "stop_condition %d\n", stopCondition);
	fprintf(sim_param_file, "update_goal_time %d\n", update_goal_time);
	fprintf(sim_param_file, "plume_model %d\n", plumeModel);
	fprintf(sim_param_file, "wind_info %d\n", windInfo);
	fprintf(sim_param_file, "wind_estimation_method %d\n", windEstimationMethod);
	fprintf(sim_param_file, "model_uncertainty %d\n", modelUncertainty);
	fprintf(sim_param_file, "diff_coefficient %f\n", diff_coefficient);
	fprintf(sim_param_file, "wind_belief_weight %f\n", wind_belief_weight);

	fprintf(sim_param_file, "cluster_method %d\n", mapCoverageStrategy);
	fprintf(sim_param_file, "cluster_seed %d\n", mapCoverageSeed);
	fprintf(sim_param_file, "epsilon_clusterize %f\n", epsilon_clusterize);
	fclose(sim_param_file);
}

SimulationParameters::~SimulationParameters()
{
}
