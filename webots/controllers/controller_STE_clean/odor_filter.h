#ifndef MAXFILTER_H
#define MAXFILTER_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  double* buffer;
  double max;
  double last;
  int length;
  int current;
} sMaxFilter;

// Inits and returns a filter structure
sMaxFilter odor_filter_make(int length);

// Releases the filter memory
void odor_filter_free(sMaxFilter* f);

// Adds new value to the filter and returns max
double odor_filter_update(sMaxFilter* f, double x);

// returns the average of the buffer
double odor_filter_average(sMaxFilter* f);

// prints the values of the odor filter
void print_filter(sMaxFilter* f);

#endif
