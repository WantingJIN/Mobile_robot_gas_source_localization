#include <math.h>
//#include "nvwa/debug_new.h"
#include "CellMeasurements.h"
#include <fstream>
#include "Constant.h"

CellMeasurements::CellMeasurements(){
	idx_x = 0;
	idx_y = 0;
	meas_sum = 0.0;
	meas_std = 0.0;
	meas_mean = 0.0;
	meas_num = 0;
}

CellMeasurements::CellMeasurements(int idx, int idy){
	idx_x = idx;
	idx_y = idy;
	px = source_x_min + idx_x * x_resol;
	py = source_y_min + idx_y * y_resol;
	meas_sum = 0.0;
	meas_std = 0.0;
	meas_mean = 0.0;
	meas_num = 0;

	hit_sum = 0.0;
	hit_std = 0.0;
	hit_mean = 0.0;
	gas_hit_thres = 2000;
}

void CellMeasurements::add_measurement(float meas){
	measurements.push_back(meas);
	meas_sum += meas;
	meas_num++;
}

void CellMeasurements::add_gas_hit(float meas){
    int gas_hit = 0;
    gas_hit = meas > gas_hit_thres ? 1 : 0;
	gas_hits.push_back(gas_hit);
	hit_sum += gas_hit;
	meas_num++;
}

void CellMeasurements::calculate_mean(){
	meas_mean = meas_sum / meas_num;
	hit_mean = hit_sum / meas_num;
}

void CellMeasurements::calculate_variance(){
	int i;
	float sum = 0.0;
	for(i = 0; i < meas_num; i++)
		sum += pow(measurements[i] - meas_mean, 2);
	meas_std = sqrt(sum / meas_num);
}

void CellMeasurements::calculate_weighted_variance(double *pdf){
	int i;
	float sum = 0.0;
	// TODO - check if the pdf is stored as the same order as the measurements (first y, then x)
	for(i = 0; i < meas_num; i++){
		sum += pow(measurements[i] - meas_mean, 2) * pdf[i];
	}
	meas_std = sqrt(sum);
}

void CellMeasurements::calculate_hit_weighted_variance(double *pdf){
	int i;
	float sum = 0.0;
	// TODO - check if the pdf is stored as the same order as the measurements (first y, then x)
	for(i = 0; i < meas_num; i++){
		sum += pow(gas_hits[i] - hit_mean, 2) * pdf[i];
	}
	hit_std = sqrt(sum);
}