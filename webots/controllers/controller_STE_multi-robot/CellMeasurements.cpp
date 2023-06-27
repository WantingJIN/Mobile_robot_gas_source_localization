#include <math.h>
//#include "nvwa/debug_new.h"
#include "CellMeasurements.h"

CellMeasurements::CellMeasurements(){
	idx_x = 0;
	idx_y = 0;
	meas_sum = 0.0;
	meas_std = 0.0;
	meas_mean = 0.0;
	meas_num = 0;
}

CellMeasurements::CellMeasurements(int idx_x, int idx_y){
	idx_x = idx_x;
	idx_y = idx_y;
	meas_sum = 0.0;
	meas_std = 0.0;
	meas_mean = 0.0;
	meas_num = 0;
}

void CellMeasurements::add_measurement(float meas){
	measurements.push_back(meas);
	meas_sum += meas;
	meas_num++;
}

void CellMeasurements::calculate_mean(){
	meas_mean = meas_sum / meas_num;
}

void CellMeasurements::calculate_variance(){
	int i;
	float sum = 0.0;
	for(i = 0; i < meas_num; i++)
		sum += pow(measurements[i] - meas_mean, 2);
	meas_std = sqrt(sum / meas_num);
}