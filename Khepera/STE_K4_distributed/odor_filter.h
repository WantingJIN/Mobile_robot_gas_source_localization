#ifndef ODOR_FILTER_H
#define ODOR_FILTER_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  float* buffer;
  float max;
  float last;
  int length;
  int current;
} sMaxFilter;

// Inits and returns a filter structure
sMaxFilter odor_filter_make(int length);

// Releases the filter memory
void odor_filter_free(sMaxFilter* f);

// Adds new value to the filter and returns max
float odor_filter_update(sMaxFilter* f, float x);

// returns the average of the buffer
float odor_filter_average(sMaxFilter* f);

// prints the values of the odor filter
void print_filter(sMaxFilter* f);

#endif
