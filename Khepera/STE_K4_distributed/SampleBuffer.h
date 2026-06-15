#ifndef SAMPLEBUFFER_H
#define SAMPLEBUFFER_H

class SampleBuffer{ 
    public: 

    //buffer length
    int buffer_length;
    //sample counter
    int n_samples;
    //odor
    float* concentration_buffer;
    float instant_concentration;
    float average_concentration;
    float stdDev_concentration;
    //wind 
    float* wind_intensity_buffer;
    float instant_wind_intensity;
    float average_wind_intensity;
    float* wind_angle_buffer;
    float instant_wind_angle;
    float average_wind_angle;
    float* wind_vx_buffer;
    float instant_wind_vx;
    float average_wind_vx;
    float* wind_vy_buffer;
    float instant_wind_vy;
    float average_wind_vy;
    float* wind_vz_buffer;
    float instant_wind_vz;
    float average_wind_vz;
    //!\ angle must be in radian [0,2PI]

    //default constructor
    SampleBuffer();
    //constructor with parameter
    SampleBuffer(int length);
    //copy constructor
    SampleBuffer(const SampleBuffer & to_copy_from);

    //destructor
    ~SampleBuffer();

    //set the buffer length (after using the default constructor)
    void set_buffer_length(int length);
    //reset everything to 0
    void reset_all();

    //add element to buffer
    void add_all_to_buffer();
    void add_odor_to_buffer();
    void add_wind_to_buffer();

    //average calculator
    void average_all();
    void average_odor();
    void average_wind();
}; 

#endif
