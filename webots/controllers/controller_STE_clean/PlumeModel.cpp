#include <math.h>
#include "PlumeModel.h"


//constructor : default values
GaussianPlumeModel2D::GaussianPlumeModel2D(){
	//number of parameters to estimate
	nbr_param = 5;
	
	//allocate memory for the parameters
	param_list = (parameter*) malloc(nbr_param * sizeof(parameter));
	
	//hard-coded parameters {name, lower_limit, upper_limit, resolution, nbins, estimated, max_a_post}
	//Q
	param_list[0] = parameter {"Q", 			1, 		20, 	1.9, 	10, 	0, 0};
	//source_x
	param_list[1] = parameter {"source_x", 		0, 		20, 	0.5, 	40, 	0, 0};
	//source_y
	param_list[2] = parameter {"source_y", 		0,  	4, 		0.2, 	20, 	0, 0};
	//sigma_a
	param_list[3] = parameter {"sigma_y_a", 	0.01,  	0.05, 	0.004, 	10, 	0, 0};
	//sigma_b
	param_list[4] = parameter {"sigma_y_b",  	0.1,   	0.5,  	0.04, 	10, 	0, 0};
	
	//parameters read from files : TODO

	//copy addresses of expected and max a posteriori values in respective arrays
	expected_value = (double*) malloc(nbr_param * sizeof(double));
	max_a_post_value = (double*) malloc(nbr_param * sizeof(double));
	for (int i = 0; i < nbr_param; i++){
		expected_value[i] = 0;
		max_a_post_value[i] = 0;
	}
}

//destructor
GaussianPlumeModel2D::~GaussianPlumeModel2D(){
	free(param_list);
	free(expected_value);
	free(max_a_post_value);
}

void PlumeModel::get_expected_value(double* container){
	int i = 0;
	for(i = 0; i < nbr_param; i++){
		container[i] = expected_value[i];
	}
}

void PlumeModel::get_max_a_post_value(double* container){
	int i = 0;
	for(i = 0; i < nbr_param; i++){
		container[i] = max_a_post_value[i];
	}
}

void PlumeModel::set_expected_value(double* container){
	int i = 0;
	for(i = 0; i < nbr_param; i++){
		expected_value[i] = container[i];
	}
}

void PlumeModel::set_max_a_post_value(double* container){
	int i = 0;
	for(i = 0; i < nbr_param; i++){
		max_a_post_value[i]= container[i];
	}
}

//returns concentration for a set of parameters and a position
double GaussianPlumeModel2D::concentration(double robot_position[], double wind_intensity, double param_list[]){
	//ASSUMPTION : X is towards the wind speed
	//notation simplification (axis rotation, hence the signs)
	double x = param_list[1] - robot_position[0];
	double y = param_list[2] - robot_position[1];
	double z = 0;//robot_position[2] - parameters.source_position[2];

	// verify if the point is not before the source
	if(x < 0)
		return 0.0;
	else{
		//std on y and z
		double sigma_y = param_list[3] * x + param_list[4];
		double sigma_z = 1;//parameters.a_z * x + parameters.b_z;
		//double sigma_y = 0.000803  * x*x - 0.004406 * x + 0.1726;
    	//double sigma_z = 0.0006798 * x*x - 0.00244  * x + 0.1562;
		//double sigma_y = 0.01 * x + 0.1;
        //double sigma_z = 0.01 * x + 0.1;

		//model : C(x,y,z) = (Q/2pi.u.sigmaY.sigmaZ) * exp (-y^2/2sigmaY^2 - z^2/2sigmaZ^2)
		double model_value = (param_list[0]/(2*PI*wind_intensity*sigma_y*sigma_z)) * exp( -((y*y)/(2*sigma_y*sigma_y)) - ((z*z)/(2*sigma_z*sigma_z)) );
		
		return model_value;
	}
}

double GaussianPlumeModel2D::approx_concentration(double robot_position[], double wind_intensity){
	//ASSUMPTION : X is towards the wind speed
	//notation simplification (axis rotation, hence the signs)
	double x = expected_value[1] - robot_position[0];
	double y = expected_value[2] - robot_position[1];
	double z = 0;//robot_position[2] - parameters.source_position[2];

	// verify if the point is not before the source
	if(x < 0)
		return 0.0;
	else{
		//std on y and z
		double sigma_y = expected_value[3] * x + expected_value[4];
		double sigma_z = 1;//parameters.a_z * x + parameters.b_z;

		//model : C(x,y,z) = (Q/2pi.u.sigmaY.sigmaZ) * exp (-y^2/2sigmaY^2 - z^2/2sigmaZ^2)
		double model_value = (expected_value[0]/(2*PI*wind_intensity*sigma_y*sigma_z)) * exp( -((y*y)/(2*sigma_y*sigma_y)) - ((z*z)/(2*sigma_z*sigma_z)) );
		
		return model_value;
	}
}

void GaussianPlumeModel2D::print_expected_val(){
	printf("Expected value: Q=%f @(%f %f) sigma y: %f %f  \n", 
		expected_value[0], expected_value[1], expected_value[2], expected_value[3], expected_value[4]);
}

void GaussianPlumeModel2D::print_max_a_post_val(){
	printf("MaxApost value: Q=%f @(%f %f) sigma y: %f %f  \n", 
		max_a_post_value[0], max_a_post_value[1], max_a_post_value[2], max_a_post_value[3], max_a_post_value[4]);
}
