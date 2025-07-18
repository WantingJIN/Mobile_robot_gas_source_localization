#include "odor_filter.h"

sMaxFilter odor_filter_make(int length) {
  sMaxFilter f;
  f.max = 0;        //the highest value in the array (not an index)
  f.current = 0;    //index of the position in the array where the new sample will be written
  f.length = length;//length of the array
  f.last = 0;       //the last value put in the array (not an index)
  f.buffer = (float*) calloc(length, sizeof (float)); //the array initialized to 0
  return f;
}

// Releases the filter memory
void odor_filter_free(sMaxFilter* f) {
  free(f->buffer);
}

// Adds new value to the filter and returns max
float odor_filter_update(sMaxFilter* f, float x) {
  //float pop = f->buffer[f->current];
  f->last = x;

  f->buffer[f->current++] = x;
  f->current %= f->length;

  /*// If value removed was current max
  if (f->max == pop) {
    float *p;
    f->max = 0;
    for (p = f->buffer; p < f->buffer + f->length; p++)
      f->max = (*p > f->max) ? *p : f->max;
  }*/ //This method would not update the max value every iteration /!\ *

  // update max
  float *p;
  f->max = 0;
  for (p = f->buffer; p < f->buffer + f->length; p++)
      f->max = (*p > f->max) ? *p : f->max;

  return f->max;
}

// returns the average of the buffer
float odor_filter_average(sMaxFilter* f) {
  float sum = 0;
  int i = 0;

  for(i=0; i<f->length; i++)
    sum += f->buffer[i];

  return (sum/f->length);
}

// prints the values of the odor filter
void print_filter(sMaxFilter* f) {
  int i=0;
  printf("Odor filter values : ");
  for (i = 0; i < f->length; i++){
    printf("%f ", f->buffer[i]);
  }
  printf("\n");
}
