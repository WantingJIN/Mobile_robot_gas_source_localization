#ifndef PLUME_MODEL_H
#define PLUME_MODEL_H

#include <stdio.h>
#include <stdlib.h>

#define PI 	3.14159265358979

//parameter structure defines properties of a parameter. A model has an array of parameters.
typedef struct {
	char name[24];
	float lower_limit;
	float upper_limit;
	float resolution;
	int nbins;
	double expected_val;
	double max_a_post_val;
} parameter;



//The base class of plume models. It is an abstract class that defines the base of any plume model
class PlumeModel {
public:

	//number of parameters to estimate
	int nbr_param;
	//list of parameters
	parameter* param_list;
	//expected values of the parameters
	double* expected_value;
	//max a posteriori values of the parameters
	double* max_a_post_value;

	//function that returns concentration using parameters
	virtual double concentration(double robot_position[], double wind_intensity, double param_list[]) = 0;
	virtual double approx_concentration(double robot_position[], double wind_intensity) = 0;
	
	//print expected and max a posteriori values on screen
	virtual void print_expected_val() = 0;
	virtual void print_max_a_post_val() = 0;

	void get_expected_value(double* container);
	void get_max_a_post_value(double* container);
	void set_expected_value(double* container);
	void set_max_a_post_value(double* container);

};



//2D pseudo Gaussian plume model that was initially used in STE (ICRA2019)
class GaussianPlumeModel2D: public virtual PlumeModel {
public:

	//constructor : default values
    GaussianPlumeModel2D();

    //destructor
    ~GaussianPlumeModel2D();

    //returns concentration for a set of parameters and a position
	double concentration(double robot_position[], double wind_intensity, double param_list[]);
	double approx_concentration(double robot_position[], double wind_intensity);

	//print expected and max a posteriori values on screen
	void print_expected_val();
	void print_max_a_post_val();
/*
	void get_expected_value(double* container);
	void get_max_a_post_value(double* container);
	void set_expected_value(double* container);
	void set_max_a_post_value(double* container);*/
};

#endif