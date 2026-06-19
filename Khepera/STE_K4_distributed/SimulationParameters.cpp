#include "SimulationParameters.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cctype>
#include <initializer_list>
#include <map>
#include <sstream>
#include <string>
using std::vector;

#include <unistd.h>

eScenario parse_scenario(const char *str)
{
	if (strcmp(str, "NO_COM_NO_COOR") == 0)
		return NO_COM_NO_COOR;
	if (strcmp(str, "COM_NO_COOR") == 0)
		return COM_NO_COOR;
	if (strcmp(str, "COM_COOR") == 0)
		return COM_COOR;
	fprintf(stderr, "Unknown scenario: %s\n", str);
	exit(1);
}
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
eInfoSum parse_info_sum(const char *str)
{
	if (strcmp(str, "LINEAR_SUM") == 0)
		return LINEAR_SUM;
	if (strcmp(str, "EXP_SUM") == 0)
		return EXP_SUM;
	fprintf(stderr, "Unknown info sum: %s\n", str);
	exit(1);
}

ePlumeModel parse_plume_model(const char *str)
{
	if (strcmp(str, "GAUSSIAN_PLUME") == 0)
		return GAUSSIAN_PLUME;
	if (strcmp(str, "DDPM") == 0)
		return DDPM;
	if (strcmp(str, "FDM_PDE") == 0)
		return FDM_PDE;
	fprintf(stderr, "Unknown plume model: %s\n", str);
	exit(1);
}

eWindInfo parse_wind_info(const char *str)
{
	if (strcmp(str, "WIND_OFFLINE") == 0)
		return WIND_OFFLINE;
	if (strcmp(str, "WIND_ONLINE") == 0)
		return WIND_ONLINE;
	fprintf(stderr, "Unknown wind info: %s\n", str);
	exit(1);
}

eWindEstimationMethod parse_wind_estimation_method(const char *str)
{
	if (strcmp(str, "GROUND_TRUTH") == 0)
		return GROUND_TRUTH;
	if (strcmp(str, "GAUSSIAN_KERNEL") == 0)
		return GAUSSIAN_KERNEL;
	if (strcmp(str, "BASEFLOW_CORRECTION") == 0)
		return BASEFLOW_CORRECTION;
	fprintf(stderr, "Unknown wind estimation method: %s\n", str);
	exit(1);
}

eModelUncertainty parse_model_uncertainty(const char *str)
{
	if (strcmp(str, "FIXED_UNCERTAINTY") == 0)
		return FIXED_UNCERTAINTY;
	if (strcmp(str, "DYNAMIC_UNCERTAINTY") == 0)
		return DYNAMIC_UNCERTAINTY;
	fprintf(stderr, "Unknown model uncertainty: %s\n", str);
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
eworkMode parse_work_mode(const char *str)
{
	if (strcmp(str, "EXPERIMENT") == 0)
		return EXPERIMENT;
	if (strcmp(str, "OFFLINE") == 0)
		return OFFLINE;
	fprintf(stderr, "Unknown work mode: %s\n", str);
	exit(1);
}
eSensorCalibration parse_calibration_mode(const char *str)
{
	if (strcmp(str, "ON") == 0)
		return ON;
	if (strcmp(str, "OFF") == 0)
		return OFF;
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
eTeamStrategy parse_team_strategy(const char* str) {
	if (strcmp(str, "INDIVIDUAL") == 0) return INDIVIDUAL;
	if (strcmp(str, "DISTRIBUTED") == 0) return DISTRIBUTED;
	if (strcmp(str, "CENTRALIZED") == 0) return CENTRALIZED;
	fprintf(stderr, "Unknown team strategy: %s\n", str);
	exit(1);
}

eMapCoverageStrategy parse_map_coverage_strategy(const char* str) {
	if (strcmp(str, "VORONOI") == 0) return VORONOI;
	if (strcmp(str, "LLOYD_D") == 0) return LLOYD_D;
	if (strcmp(str, "LLOYD_P") == 0) return LLOYD_P;
	if (strcmp(str, "REGION_GROWING_P") == 0) return REGION_GROWING_P;
	if (strcmp(str, "REGION_GROWING_D") == 0) return REGION_GROWING_D;
	if (strcmp(str, "NONE") == 0) return NONE;
	fprintf(stderr, "Unknown map coverage strategy: %s\n", str);
	exit(1);
}

eMapCoverageSeed parse_map_coverage_seed(const char* str) {
	if (strcmp(str, "MAX_CELL") == 0) return MAX_CELL;
	if (strcmp(str, "ROBOT_CELL") == 0) return ROBOT_CELL;
	fprintf(stderr, "Unknown map coverage seed: %s\n", str);
	exit(1);
}

eStopCondition parse_stop_condition(const char* str) {
	if (strcmp(str, "ENTROPY_ONLY") == 0) return ENTROPY_ONLY;
	if (strcmp(str, "SPREAD_ONLY") == 0) return SPREAD_ONLY;
	if (strcmp(str, "ENTROPY_SPREAD") == 0) return ENTROPY_SPREAD;
	if (strcmp(str, "ITERATION_ONLY") == 0) return ITERATION_ONLY;
	if (strcmp(str, "END_OF_FIX_PATH") == 0) return END_OF_FIX_PATH;
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


vector<int> parse_ID_list(const char *str)
{
	vector<int> id_list;
	char buf[1024];
	strncpy(buf, str, sizeof(buf));
	buf[sizeof(buf)-1] = '\0';
	char *token = strtok(buf, ",");
	while (token != NULL)
	{
		id_list.push_back(atoi(token));
		token = strtok(NULL, ",");
	}
	return id_list;
}

namespace {

typedef std::map<std::string, std::string> ConfigValues;

std::string trim(const std::string &text)
{
	size_t begin = 0;
	while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])))
		begin++;

	size_t end = text.size();
	while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])))
		end--;

	return text.substr(begin, end - begin);
}

std::string strip_inline_comment(const std::string &line)
{
	bool in_single_quote = false;
	bool in_double_quote = false;
	for (size_t i = 0; i < line.size(); i++)
	{
		char c = line[i];
		if (c == '\'' && !in_double_quote)
			in_single_quote = !in_single_quote;
		else if (c == '"' && !in_single_quote)
			in_double_quote = !in_double_quote;
		else if (c == '#' && !in_single_quote && !in_double_quote)
			return line.substr(0, i);
	}
	return line;
}

std::string normalize_yaml_value(const std::string &raw_value)
{
	std::string value = trim(raw_value);
	if (value.size() >= 2 &&
	    ((value.front() == '"' && value.back() == '"') ||
	     (value.front() == '\'' && value.back() == '\'')))
	{
		value = value.substr(1, value.size() - 2);
	}

	if (value.size() >= 2 && value.front() == '[' && value.back() == ']')
		value = value.substr(1, value.size() - 2);

	return trim(value);
}

std::string make_config_path(const std::vector<std::pair<int, std::string> > &stack,
			     const std::string &key)
{
	std::string path;
	for (size_t i = 0; i < stack.size(); i++)
	{
		if (!path.empty())
			path += ".";
		path += stack[i].second;
	}
	if (!path.empty())
		path += ".";
	path += key;
	return path;
}

void load_config_values(const char *filename, ConfigValues *values)
{
	std::ifstream file(filename);
	if (!file)
	{
		fprintf(stderr, "Error opening config file: %s\n", filename);
		exit(1);
	}

	std::vector<std::pair<int, std::string> > stack;
	std::string line;
	int line_number = 0;
	while (std::getline(file, line))
	{
		line_number++;
		std::string without_comment = strip_inline_comment(line);
		if (trim(without_comment).empty())
			continue;

		size_t indent = 0;
		while (indent < without_comment.size() &&
		       (without_comment[indent] == ' ' || without_comment[indent] == '\t'))
		{
			indent += (without_comment[indent] == '\t') ? 2 : 1;
		}

		std::string content = trim(without_comment);
		if (content == "---" || content == "...")
			continue;

		size_t separator = content.find(':');
		bool yaml_separator = true;
		if (separator == std::string::npos)
		{
			separator = content.find('=');
			yaml_separator = false;
		}
		if (separator == std::string::npos)
		{
			fprintf(stderr, "Invalid config line %d in %s: %s\n",
				line_number, filename, line.c_str());
			exit(1);
		}

		std::string key = trim(content.substr(0, separator));
		std::string value = trim(content.substr(separator + 1));
		if (key.empty())
		{
			fprintf(stderr, "Empty config key on line %d in %s\n", line_number, filename);
			exit(1);
		}

		while (!stack.empty() && static_cast<int>(indent) <= stack.back().first)
			stack.pop_back();

		if (yaml_separator && value.empty())
		{
			stack.push_back(std::make_pair(static_cast<int>(indent), key));
			continue;
		}

		(*values)[make_config_path(stack, key)] = normalize_yaml_value(value);
	}
}

bool get_config_value(const ConfigValues &values, const char *path, std::string *value)
{
	ConfigValues::const_iterator it = values.find(path);
	if (it == values.end())
		return false;
	*value = it->second;
	return true;
}

bool get_first_config_value(const ConfigValues &values,
			    std::initializer_list<const char *> paths,
			    std::string *value)
{
	for (std::initializer_list<const char *>::const_iterator it = paths.begin();
	     it != paths.end(); ++it)
	{
		if (get_config_value(values, *it, value))
			return true;
	}
	return false;
}

const char *sampling_mode_key(eSamplingMode mode)
{
	switch (mode)
	{
	case CONTINOUS_SAMPLING:
		return "CONTINOUS_SAMPLING";
	case STOP_AND_GO_SAMPLING:
		return "STOP_AND_GO_SAMPLING";
	case SENSE_WITHOUT_STOP:
		return "SENSE_WITHOUT_STOP";
	}
	return "CONTINOUS_SAMPLING";
}

const char *relative_method_key(eRelativeMethod method)
{
	switch (method)
	{
	case NORMALIZATION:
		return "NORMALIZATION";
	case Z_SCORE:
		return "Z_SCORE";
	case EMPIRICAL_CDF:
		return "EMPIRICAL_CDF";
	case EMPIRICAL_CDF_AND_SLOPE:
		return "EMPIRICAL_CDF_AND_SLOPE";
	case EMPIRICAL_CDF_NUM:
		return "EMPIRICAL_CDF_NUM";
	}
	return "EMPIRICAL_CDF";
}

const char *gas_hit_method_key(eGasHitMethod method)
{
	switch (method)
	{
	case FIXED_THRES:
		return "FIXED_THRES";
	case FIXED_THRES_NUM:
		return "FIXED_THRES_NUM";
	case MIN_MAX_THRES:
		return "MIN_MAX_THRES";
	case SLIDING_WINDOW_THRES:
		return "SLIDING_WINDOW_THRES";
	case SLIDING_WINDOW_THRES_NUM:
		return "SLIDING_WINDOW_THRES_NUM";
	}
	return "FIXED_THRES";
}

} // namespace

SimulationParameters::SimulationParameters()
{
	// default values
	// simulation parameters
	likelihood_error = 30;
	global_local_search_ratio = 0.5;
	MCMC_iteration_num = 1000;
	BayesianModelingMethod = EXHAUSTIVE;
	number_of_robots = 1;
	scenario = NO_COM_NO_COOR;
	map_nbr = 0;
	samplingMode = CONTINOUS_SAMPLING;
	processingMode = NO_PROCESSING;
	gasSensor = PID;
	planningStrategy = BELIEF_CLUSTERING;
	gasFeature = GASHIT;
	visitedList = visitedList_ON;
	infoFeature = VARIANCE;
	sampling_period = 1;
	cluster_entropy_ratio = 10;
	vel_factor = 10;
	sampling_dist_thresh = 0.1;
	relativeMethod = EMPIRICAL_CDF;
	gasHitMethod = SLIDING_WINDOW_THRES;
	gas_hit_threshold = 1000;
	sensorCalibration = OFF;
	workMode = EXPERIMENT;
	R0 = 100;
	likelihoodSigma = 1;
	lambda = 0.8;
	scoreSigma = 0.5;
	scoreFun = EXP;
	infoSum = LINEAR_SUM;
	planning_step = 10;
	weighting_entropy_ratio = 6;
	teamStrategy = CENTRALIZED;
	mapCoverageStrategy = NONE;
	mapCoverageSeed = MAX_CELL;
	epsilon_clusterize = 1e-9;
	stopCondition = ENTROPY_ONLY;
	wheel_speed = 400;
	plumeModel = DDPM;
	windInfo = WIND_OFFLINE;
	windEstimationMethod = GAUSSIAN_KERNEL;
	modelUncertainty = FIXED_UNCERTAINTY;
	diff_coefficient = 0.05f;
	wind_belief_weight = 1.0f;
	ID_list= {101, 102, 103, 104, 105, 106, 107, 108, 109, 110}; 
}

SimulationParameters::SimulationParameters(const char *filename)
	: SimulationParameters()
{
	load_simulation_parameters_from_config(filename);
}

void SimulationParameters::load_simulation_parameters_from_config(const char *filename)
{
	ConfigValues config;
	load_config_values(filename, &config);

	std::string value;
	if (get_first_config_value(config, {"bayesian_estimation.likelihood_error", "likelihood_error"}, &value))
		likelihood_error = atof(value.c_str());
	if (get_first_config_value(config, {"planning.global_local_search_ratio", "global_local_search_ratio"}, &value))
		global_local_search_ratio = atof(value.c_str());
	if (get_first_config_value(config, {"bayesian_estimation.BayesianModelingMethod", "BayesianModelingMethod"}, &value))
		BayesianModelingMethod = parse_evaluation_method(value.c_str());
	if (get_first_config_value(config, {"robot.number_of_robots", "number_of_robots"}, &value))
		number_of_robots = atoi(value.c_str());
	if (get_first_config_value(config, {"simulation.scenario", "scenario"}, &value))
		scenario = parse_scenario(value.c_str());
	if (get_first_config_value(config, {"simulation.map_nbr", "map_nbr"}, &value))
		map_nbr = atoi(value.c_str());
	if (get_first_config_value(config, {"bayesian_estimation.MCMC.MCMC_iteration_num", "MCMC_iteration_num"}, &value))
		MCMC_iteration_num = atoi(value.c_str());
	if (get_first_config_value(config, {"sensing.samplingMode", "samplingMode"}, &value))
		samplingMode = parse_sampling_mode(value.c_str());
	if (get_first_config_value(config, {"sensing.processingMode", "processingMode"}, &value))
		processingMode = parse_processing_mode(value.c_str());
	if (get_first_config_value(config, {"sensing.gasSensor", "gasSensor"}, &value))
		gasSensor = parse_gas_sensor(value.c_str());
	if (get_first_config_value(config, {"planning.planningStrategy", "planningStrategy"}, &value))
		planningStrategy = parse_planning_strategy(value.c_str());
	if (get_first_config_value(config, {"gas_feature.gasFeature", "gasFeature"}, &value))
		gasFeature = parse_gas_feature(value.c_str());
	if (get_first_config_value(config, {"planning.visitedList", "visitedList"}, &value))
		visitedList = parse_visited_list(value.c_str());
	if (get_first_config_value(config, {"planning.infoFeature", "infoFeature"}, &value))
		infoFeature = parse_info_feature(value.c_str());

	std::string sampling_period_path = std::string("sensing.") + sampling_mode_key(samplingMode) + ".sampling_period";
	if (get_first_config_value(config, {sampling_period_path.c_str(), "sensing.sampling_period", "sampling_period"}, &value))
		sampling_period = atoi(value.c_str());
	if (get_first_config_value(config, {"planning.BELIEF_CLUSTERING.cluster_entropy_ratio", "cluster_entropy_ratio"}, &value))
		cluster_entropy_ratio = atof(value.c_str());
	if (get_first_config_value(config, {"robot.vel_factor", "vel_factor"}, &value))
		vel_factor = atof(value.c_str());

	std::string sampling_dist_path = std::string("sensing.") + sampling_mode_key(samplingMode) + ".sampling_dist_thresh";
	if (get_first_config_value(config, {sampling_dist_path.c_str(), "sensing.sampling_dist_thresh", "sampling_dist_thresh"}, &value))
		sampling_dist_thresh = atof(value.c_str());

	if (get_first_config_value(config, {"gas_feature.RELATIVE_VALUE.relativeMethod", "relativeMethod"}, &value))
		relativeMethod = parse_relative_method(value.c_str());
	if (get_first_config_value(config, {"gas_feature.GASHIT.gasHitMethod", "gasHitMethod"}, &value))
		gasHitMethod = parse_gas_hit_method(value.c_str());

	std::string gas_hit_threshold_path = std::string("gas_feature.GASHIT.") + gas_hit_method_key(gasHitMethod) + ".gasHitThres";
	if (get_first_config_value(config,
				   {gas_hit_threshold_path.c_str(),
				    "gas_feature.GASHIT.FIXED_THRES.gasHitThres",
				    "gasHitThres"},
				   &value))
	{
		gas_hit_threshold = atoi(value.c_str());
	}

	if (get_first_config_value(config, {"sensing.sensorCalibration", "sensing.calibrationMode", "sensorCalibration"}, &value))
		sensorCalibration = parse_calibration_mode(value.c_str());
	if (get_first_config_value(config, {"sensing.NOLINEAR.R0", "R0"}, &value))
		R0 = atof(value.c_str());
	if (get_first_config_value(config, {"simulation.workMode", "simulation.simuMode", "workMode"}, &value))
		workMode = parse_work_mode(value.c_str());

	std::string relative_likelihood_path = std::string("gas_feature.RELATIVE_VALUE.") +
					       relative_method_key(relativeMethod) + ".likelihoodSigma";
	if (get_first_config_value(config,
				   {relative_likelihood_path.c_str(),
				    "gas_feature.RELATIVE_VALUE.likelihoodSigma",
				    "gas_feature.ABSOLUTE_VALUE.likelihoodSigma",
				    "likelihoodSigma"},
				   &value))
	{
		likelihoodSigma = atof(value.c_str());
	}

	std::string gas_hit_lambda_path = std::string("gas_feature.GASHIT.") + gas_hit_method_key(gasHitMethod) + ".lambda";
	if (get_first_config_value(config,
				   {gas_hit_lambda_path.c_str(),
				    "gas_feature.GASHIT.SLIDING_WINDOW_THRES.lambda",
				    "gas_feature.GASHIT.SLIDING_WINDOW_THRES_NUM.lambda",
				    "lambda"},
				   &value))
	{
		lambda = atof(value.c_str());
	}

	if (get_first_config_value(config, {"planning.scoreSigma", "scoreSigma"}, &value))
		scoreSigma = atof(value.c_str());
	if (get_first_config_value(config, {"planning.scoreFun", "scoreFun"}, &value))
		scoreFun = parse_score_fun(value.c_str());
	if (get_first_config_value(config, {"planning.infoSum", "infoSum"}, &value))
		infoSum = parse_info_sum(value.c_str());
	if (get_first_config_value(config, {"plume_model.plumeModel", "plumeModel", "plume_model"}, &value))
		plumeModel = parse_plume_model(value.c_str());
	if (get_first_config_value(config, {"wind.windInfo", "windInfo", "wind_info"}, &value))
		windInfo = parse_wind_info(value.c_str());
	if (get_first_config_value(config, {"wind.windEstimationMethod", "windEstimationMethod", "wind_estimation_method"}, &value))
		windEstimationMethod = parse_wind_estimation_method(value.c_str());
	if (get_first_config_value(config, {"wind.modelUncertainty", "modelUncertainty", "model_uncertainty"}, &value))
		modelUncertainty = parse_model_uncertainty(value.c_str());
	if (get_first_config_value(config, {"plume_model.FDM_PDE.diffCoefficient", "diffCoefficient", "diff_coefficient"}, &value))
		diff_coefficient = atof(value.c_str());
	if (get_first_config_value(config, {"wind.DYNAMIC_UNCERTAINTY.windBeliefWeight", "windBeliefWeight", "wind_belief_weight"}, &value))
		wind_belief_weight = atof(value.c_str());
	if (get_first_config_value(config, {"planning.planning_step", "planning_step"}, &value))
		planning_step = atoi(value.c_str());
	if (get_first_config_value(config, {"planning.SUM_DIJKSTRA_ADP.weighting_entropy_ratio", "weighting_entropy_ratio"}, &value))
		weighting_entropy_ratio = atof(value.c_str());
	if (get_first_config_value(config, {"team.teamStrategy", "teamStrategy"}, &value))
		teamStrategy = parse_team_strategy(value.c_str());
	if (get_first_config_value(config, {"team.mapCoverageStrategy", "mapCoverageStrategy"}, &value))
		mapCoverageStrategy = parse_map_coverage_strategy(value.c_str());
	if (get_first_config_value(config, {"team.mapCoverageSeed", "mapCoverageSeed"}, &value))
		mapCoverageSeed = parse_map_coverage_seed(value.c_str());
	if (get_first_config_value(config, {"team.epsilon_clusterize", "epsilon_clusterize"}, &value))
		epsilon_clusterize = atof(value.c_str());
	if (get_first_config_value(config, {"termination.stopCondition", "stopCondition"}, &value))
		stopCondition = parse_stop_condition(value.c_str());
	if (get_first_config_value(config, {"robot.wheel_speed", "wheel_speed"}, &value))
		wheel_speed = atoi(value.c_str());
	if (get_first_config_value(config, {"robot.ID_list", "team.ID_list", "ID_list"}, &value))
	{
		ID_list = parse_ID_list(value.c_str());
		for (size_t i = 0; i < ID_list.size(); i++)
			printf("ID_list[%zu] = %d\n", i, ID_list[i]);
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
		if (sim_param_file != NULL)
		{
			ok = fscanf(sim_param_file, "%d", &map_nbr);
			if (ok)
				printf("scenario : %d\n", map_nbr);
			fclose(sim_param_file);
		}
	}
}

void SimulationParameters::log_simulation_parameters(const char *config_logfile_name)
{
	// write simulation parameters to file
	FILE *sim_param_file;
	sim_param_file = fopen(config_logfile_name, "a");
	if (sim_param_file == NULL)
	{
		fprintf(stderr, "Error opening simulation parameter log file: %s\n", config_logfile_name);
		return;
	}
	fprintf(sim_param_file, "likelihood_error %f\n", likelihood_error);
	fprintf(sim_param_file, "global_local_search_ratio %f\n", global_local_search_ratio);
	fprintf(sim_param_file, "MCMC_iteration_num %d\n", MCMC_iteration_num);
	fprintf(sim_param_file, "BayesianModelingMethod %d\n", BayesianModelingMethod);
	fprintf(sim_param_file, "number_of_robots %d\n", number_of_robots);
	fprintf(sim_param_file, "scenario %d\n", scenario);
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
	fprintf(sim_param_file, "simu_mode %d\n", workMode);
	fprintf(sim_param_file, "sensor_calibration %d\n", sensorCalibration);
	fprintf(sim_param_file, "gas_hit_method %d\n", gasHitMethod);
	fprintf(sim_param_file, "gas_hit_threshold %d\n", gas_hit_threshold);
	fprintf(sim_param_file, "R0 %f\n", R0);
	fprintf(sim_param_file, "likelihood_sigma %f\n", likelihoodSigma);
	fprintf(sim_param_file, "lambda %f\n", lambda);
	fprintf(sim_param_file, "score_sigma %f\n", scoreSigma);
	fprintf(sim_param_file, "score_fun %d\n", scoreFun);
	fprintf(sim_param_file, "info_sum %d\n", infoSum);
	fprintf(sim_param_file, "plume_model %d\n", plumeModel);
	fprintf(sim_param_file, "wind_info %d\n", windInfo);
	fprintf(sim_param_file, "wind_estimation_method %d\n", windEstimationMethod);
	fprintf(sim_param_file, "model_uncertainty %d\n", modelUncertainty);
	fprintf(sim_param_file, "diff_coefficient %f\n", diff_coefficient);
	fprintf(sim_param_file, "wind_belief_weight %f\n", wind_belief_weight);
	fprintf(sim_param_file, "planning_step %d\n", planning_step);
	fprintf(sim_param_file, "weighting_entropy_ratio %f\n", weighting_entropy_ratio);
	fprintf(sim_param_file, "team_strategy %d\n", teamStrategy);
	fprintf(sim_param_file, "map_coverage_strategy %d\n", mapCoverageStrategy);
	fprintf(sim_param_file, "map_coverage_seed %d\n", mapCoverageSeed);
	fprintf(sim_param_file, "epsilon_clusterize %f\n", epsilon_clusterize);
	fprintf(sim_param_file, "stop_condition %d\n", stopCondition);
	fprintf(sim_param_file, "wheel_speed %d\n", wheel_speed);
	fprintf(sim_param_file, "ID_list ");
	for (size_t i = 0; i < ID_list.size(); i++) {
		fprintf(sim_param_file, "%d", ID_list[i]);
		if (i < ID_list.size() - 1) {
			fprintf(sim_param_file, ",");
		}
	}
	fprintf(sim_param_file, "\n");
	fclose(sim_param_file);
}

SimulationParameters::~SimulationParameters()
{
}
