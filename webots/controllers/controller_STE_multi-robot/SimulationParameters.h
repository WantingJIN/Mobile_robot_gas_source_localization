#ifndef SIMULATION_PARAMETER_H
#define SIMULATION_PARAMETER_H


typedef enum scenario{
	NO_COM_NO_COOR = 0,
	COM_NO_COOR = 1,
	COM_COOR = 2,
} eScenario;

typedef enum evaluationMethod{
	MCMC = 0,
	EXHAUSTIVE = 1,
} eEvaluationMethod;

// Mode of sampling
typedef enum samplingMode
{
	CONTINOUS_SAMPLING = 0,
	STOP_AND_GO_SAMPLING = 1,
	SENSE_WITHOUT_STOP = 2,
} eSamplingMode;

// Mode of Processing;
// in order to composente the slow reaction/recovery time of sensing
typedef enum processingMode
{
	NO_PROCESSING = 0,
	KERNEL_MAPPING = 1,
	DECONVOLUTION = 2,
	MAPPING_AND_DECONVOLUTION = 3,
} eProcessingMode;

typedef enum gasSensor
{
	PID = 0,
	MOX = 1,
	NOISY_MOX = 2,
	SLOW_MOX = 3,
	VERY_SLOW_MOX = 4,
	NOISY_SLOW_MOX = 5,
	NOISY_VERY_SLOW_MOX = 6,
} eGasSensor;

typedef enum planningStrategy
{
	GREEDY = 0,
	BALANCE_MAXIMAL_EXPECTED = 2,
	RANDOM = 3,
	LAWNMOWER = 4,
	BELIEF_CLUSTERING = 5,
	INFORMATIVE_KLD = 6,
	INFO_CLUSTERING = 7,
	WEIGHTED_CLUSTERING = 9,
	SUM_DIJKSTRA_FIX = 10,
	SUM_DIJKSTRA_ADP = 11,
	MULTI_DIJKSTRA = 12,
} ePlanningStrategy;

typedef enum scoreFun
{
      RATIO = 0,
	LOG = 1,
	SIGMOID = 2,
	SUM = 3,
	EXP = 4,
}eScoreFun;

typedef enum gasFeature
{
	ABSOLUTE_VALUE = 0,
	GASHIT = 1,
	RELATIVE_VALUE = 2,
} eGasFeature;

typedef enum visitedList
{
	visitedList_ON = 0,
	visitedList_OFF = 1,
} eVisitedList;

typedef enum infoFeature
{
	VARIANCE = 0,
	MEAN = 1,
	PDF = 2,
} eInfoFeature;

typedef enum relativeMethod
{
	NORMALIZATION = 0,
	EMPIRICAL_CDF = 3,
	EMPIRICAL_CDF_AND_SLOPE = 4,
	EMPIRICAL_CDF_NUM = 7,  // The likehood function is defined diffrently
} eRelativeMethod;

typedef enum simuMode{
	ONLINE = 0,
	OFFLINE = 1,
}eSimuMode;

typedef enum calibrationMode
{
	OFF = 0,
	LINEAR = 1,
	NOLINEAR = 2,
} eCalibrationMode;

typedef enum gasHitMethod
{
	FIXED_THRES = 0,
	MIN_MAX_THRES = 1,
	SLIDING_WINDOW_THRES = 2,
	SLIDING_WINDOW_THRES_NUM = 4,   // The likehood function is defined diffrently
	FIXED_THRES_NUM = 5, // The likehood function is defined diffrently
} eGasHitMethod;

typedef enum infoSum
{
	LINEAR_SUM = 0,
	EXP_SUM = 1,
} eInfoSum;

// parameters to estimate
struct parameters_set
{
	double Q;
	double source_position[2];
	double a_y;
	double b_y;
	// double a_z;
	// double b_z;
};

// simulation paramters
class SimulationParameters{
      public:
		int MCMC_iteration_num; 
		int number_of_robots; 
		int map_nbr; 
		int sampling_period; 
		float likelihood_error;  
		float global_local_search_ratio; 
		float cluster_entropy_ratio; 
		float vel_factor; 
		float sampling_dist_thresh; 
		int gas_hit_threshold;
		float R0;
	      float likelihoodSigma;
		float lambda;
		float scoreSigma;
		int planning_step;
		float weighting_entropy_ratio;

		eEvaluationMethod BayesianModelingMethod;
		eScenario scenario; 
		eSamplingMode samplingMode; 
		eProcessingMode processingMode; 
		eGasSensor gasSensor; 
		ePlanningStrategy planningStrategy; 
		eGasFeature gasFeature; 
		eVisitedList visitedList; 
		eInfoFeature infoFeature; 
		
		eRelativeMethod relativeMethod; 
		eSimuMode simuMode; 
		eCalibrationMode calibrationMode;
		eGasHitMethod gasHitMethod; 

	      eScoreFun scoreFun;

		eInfoSum infoSum;

		SimulationParameters();
		~SimulationParameters();

        	void load_simulation_parameters_from_config(const char *filename);
		void rewrite_parameters_from_file();
		void log_simulation_parameters(const char *filename);
};


#endif