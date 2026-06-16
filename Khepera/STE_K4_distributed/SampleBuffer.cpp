#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "SampleBuffer.h"
using namespace std;


SampleBuffer::SampleBuffer(){
	instant_concentration = 0;
	average_concentration = 0;
	instant_wind_intensity = 0;
	average_wind_intensity = 0;
	instant_wind_angle = 0;
	average_wind_angle = 0;
	instant_wind_vx = 0;
	average_wind_vx = 0;
	instant_wind_vy = 0;
	average_wind_vy = 0;
	instant_wind_vz = 0;
	average_wind_vz = 0;
	
	n_samples = 0;

	buffer_length = 10; //default value when no length mentioned

	concentration_buffer = (float*) calloc(buffer_length, sizeof(float));
	wind_intensity_buffer = (float*) calloc(buffer_length, sizeof(float));
	wind_angle_buffer = (float*) calloc(buffer_length, sizeof(float));
	wind_vx_buffer = (float*) calloc(buffer_length, sizeof(float));
	wind_vy_buffer = (float*) calloc(buffer_length, sizeof(float));
	wind_vz_buffer = (float*) calloc(buffer_length, sizeof(float));
}

SampleBuffer::SampleBuffer(int length){
	instant_concentration = 0;
	average_concentration = 0;
	instant_wind_intensity = 0;
	average_wind_intensity = 0;
	instant_wind_angle = 0;
	average_wind_angle = 0;
	instant_wind_vx = 0;
	average_wind_vx = 0;
	instant_wind_vy = 0;
	average_wind_vy = 0;
	instant_wind_vz = 0;
	average_wind_vz = 0;
	
	n_samples = 0;

	buffer_length = length;

	concentration_buffer = (float*) calloc(buffer_length, sizeof(float));
	wind_intensity_buffer = (float*) calloc(buffer_length, sizeof(float));
	wind_angle_buffer = (float*) calloc(buffer_length, sizeof(float));
	wind_vx_buffer = (float*) calloc(buffer_length, sizeof(float));
	wind_vy_buffer = (float*) calloc(buffer_length, sizeof(float));
	wind_vz_buffer = (float*) calloc(buffer_length, sizeof(float));
}

SampleBuffer::SampleBuffer(const SampleBuffer & to_copy_from){
	int i=0;
	instant_concentration = to_copy_from.instant_concentration;
	average_concentration = to_copy_from.average_concentration;
	instant_wind_intensity = to_copy_from.instant_wind_intensity;
	average_wind_intensity = to_copy_from.average_wind_intensity;
	instant_wind_angle = to_copy_from.instant_wind_angle;
	average_wind_angle = to_copy_from.average_wind_angle;
	instant_wind_vx = to_copy_from.instant_wind_vx;
	average_wind_vx = to_copy_from.average_wind_vx;
	instant_wind_vy = to_copy_from.instant_wind_vy;
	average_wind_vy = to_copy_from.average_wind_vy;
	instant_wind_vz = to_copy_from.instant_wind_vz;
	average_wind_vz = to_copy_from.average_wind_vz;
	
	n_samples = to_copy_from.n_samples;

	buffer_length = to_copy_from.buffer_length;

	concentration_buffer = (float*) calloc(buffer_length, sizeof(float));
	for(i = 0; i < buffer_length; i++)
		concentration_buffer[i] = to_copy_from.concentration_buffer[i];

	wind_intensity_buffer = (float*) calloc(buffer_length, sizeof(float));
	for(i = 0; i < buffer_length; i++)
		wind_intensity_buffer[i] = to_copy_from.wind_intensity_buffer[i];

	wind_angle_buffer = (float*) calloc(buffer_length, sizeof(float));
	for(i = 0; i < buffer_length; i++)
		wind_angle_buffer[i] = to_copy_from.wind_angle_buffer[i];

	wind_vx_buffer = (float*) calloc(buffer_length, sizeof(float));
	for(i = 0; i < buffer_length; i++)
		wind_vx_buffer[i] = to_copy_from.wind_vx_buffer[i];

	wind_vy_buffer = (float*) calloc(buffer_length, sizeof(float));
	for(i = 0; i < buffer_length; i++)
		wind_vy_buffer[i] = to_copy_from.wind_vy_buffer[i];

	wind_vz_buffer = (float*) calloc(buffer_length, sizeof(float));
	for(i = 0; i < buffer_length; i++)
		wind_vz_buffer[i] = to_copy_from.wind_vz_buffer[i];
}

SampleBuffer::~SampleBuffer(){
	free(concentration_buffer);
	free(wind_intensity_buffer);
	free(wind_angle_buffer);
	free(wind_vx_buffer);
	free(wind_vy_buffer);
	free(wind_vz_buffer);
}



void SampleBuffer::set_buffer_length(int length){
	buffer_length = length;

	concentration_buffer 	= (float*) realloc(concentration_buffer, 	buffer_length * sizeof(float));
	wind_intensity_buffer 	= (float*) realloc(wind_intensity_buffer, 	buffer_length * sizeof(float));
	wind_angle_buffer 		= (float*) realloc(wind_angle_buffer, 		buffer_length * sizeof(float));
	wind_vx_buffer 		= (float*) realloc(wind_vx_buffer, 		buffer_length * sizeof(float));
	wind_vy_buffer 		= (float*) realloc(wind_vy_buffer, 		buffer_length * sizeof(float));
	wind_vz_buffer 		= (float*) realloc(wind_vz_buffer, 		buffer_length * sizeof(float));
}

void SampleBuffer::reset_all(){
	instant_concentration = 0;
	average_concentration = 0;
	instant_wind_intensity = 0;
	average_wind_intensity = 0;
	instant_wind_angle = 0;
	average_wind_angle = 0;
	instant_wind_vx = 0;
	average_wind_vx = 0;
	instant_wind_vy = 0;
	average_wind_vy = 0;
	instant_wind_vz = 0;
	average_wind_vz = 0;
	
	n_samples = 0;
}



void SampleBuffer::add_all_to_buffer(){
	if (n_samples >= buffer_length)
		return;
	concentration_buffer[n_samples] = instant_concentration;
	average_concentration += instant_concentration;
	wind_intensity_buffer[n_samples] = instant_wind_intensity;
	average_wind_intensity += instant_wind_intensity;
	wind_angle_buffer[n_samples] = instant_wind_angle;
	wind_vx_buffer[n_samples] = instant_wind_vx;
	wind_vy_buffer[n_samples] = instant_wind_vy;
	wind_vz_buffer[n_samples] = instant_wind_vz;
	n_samples++;
}

void SampleBuffer::add_odor_to_buffer(){
	if (n_samples >= buffer_length)
		return;
	concentration_buffer[n_samples] = instant_concentration;
	average_concentration += instant_concentration; //use it as a sum temporarily
	n_samples++;
}

void SampleBuffer::add_wind_to_buffer(){
	if (n_samples >= buffer_length)
		return;
	wind_intensity_buffer[n_samples] = instant_wind_intensity;
	average_wind_intensity += instant_wind_intensity;
	wind_angle_buffer[n_samples] = instant_wind_angle;
	wind_vx_buffer[n_samples] = instant_wind_vx;
	wind_vy_buffer[n_samples] = instant_wind_vy;
	wind_vz_buffer[n_samples] = instant_wind_vz;
	n_samples++;
}



void SampleBuffer::average_all(){
	average_odor();
	average_wind();
}

void SampleBuffer::average_odor(){
	if (n_samples <= 0)
		return;
	//mean
	average_concentration /= n_samples;
	// standard deviation
	float variance = 0;
	for(int i = 0; i < n_samples; i++)
      	variance += pow(concentration_buffer[i] - average_concentration, 2);
    variance /= n_samples;
    stdDev_concentration = sqrt(variance);
}

void SampleBuffer::average_wind(){
	if (n_samples <= 0)
		return;
	int i=0;
	float tmp_sin = 0, tmp_cos = 0;

	average_wind_intensity /= n_samples;
	average_wind_vx = 0.0;
	average_wind_vy = 0.0;
	average_wind_vz = 0.0;

	for(i = 0; i < n_samples; i++){
		tmp_sin += sin(wind_angle_buffer[i]);
		tmp_cos += cos(wind_angle_buffer[i]);
		average_wind_vx += wind_vx_buffer[i];
		average_wind_vy += wind_vy_buffer[i];
		average_wind_vz += wind_vz_buffer[i];
	}
	average_wind_angle = atan2(tmp_sin/n_samples, tmp_cos/n_samples);
	average_wind_vx /= n_samples;
	average_wind_vy /= n_samples;
	average_wind_vz /= n_samples;
	//averaging technique from https://rosettacode.org/wiki/Averages/Mean_angle
}
