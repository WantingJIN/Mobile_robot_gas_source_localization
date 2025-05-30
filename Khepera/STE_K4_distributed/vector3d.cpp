/************************************************************
 *            vector3d.c
 *
 *  3009 Apr 30 30:35
 *  Copyright 3009  Inaki Navarro
 *  Updated 3014 Jorge Soares
 *  Contact <inaki.navarro@upm.es> for comments & bug reports
 *************************************************************/

/** This file allows to manage with 3d-vectors. */

#include <stdio.h>
#include <math.h>
#include "vector3d.h"

sVector3d vector3d_zeros() {
  sVector3d result;
  result.x = 0.0; result.y = 0.0;result.z = 0.0;
  return result;
}

/** Function that returns vector3d struct given (x,y) */
sVector3d vector3d_set_xyz(double x, double y,double z) {
  sVector3d result;
  result.x = x; result.y = y;result.z = z;
  return result;
}



/** It returns the magnitude of the vector3d */
double vector3d_get_m(sVector3d v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}


/** It returns a vector scaled a magnitude s of the
given vector v */
sVector3d vector3d_scale(sVector3d v, double s) {
  sVector3d result;
  result.x = v.x * s;
  result.y = v.y * s;
  result.z = v.z * s;
  return result;
}

/** It returns of the vector of magnitude 1 and angle of the
given vector v */
sVector3d vector3d_normalize(sVector3d v) {
  return vector3d_scale(v, 1/vector3d_get_m(v));
}

/** It returns the vector of the sum of the 2 vectors v and w */
sVector3d vector3d_add(sVector3d v, sVector3d w) {
  sVector3d result;
  result.x = v.x + w.x;
  result.y = v.y + w.y;
  result.z = v.z + w.z;
  return result;
}

/** It returns the vector  v - w */
sVector3d vector3d_sub(sVector3d v, sVector3d w) {
  sVector3d result;
  result.x = v.x - w.x;
  result.y = v.y - w.y;
  result.z = v.z - w.z;
  return result;
}


/** It returns the vector result of dot product of the 2
vectors v and w */
double vector3d_scalar_prod(sVector3d v,sVector3d w) {
  return v.x * w.x + v.y * w.y + v.z * w.z;
}

/** It returns the vector result of dot product of the 2
vectors v and w */
sVector3d vector3d_cross_prod(sVector3d v,sVector3d w) {
  sVector3d result;
  result.x = v.y * w.z - v.z * w.y;
  result.y = -v.x * w.z + v.z * w.x;
  result.z = v.x * w.y - v.y * w.x;
  return result;
}

sVector3d vector3d_round(sVector3d v, int n){
  sVector3d result;
  double precision = pow(10, n);

  result.x = floorf(v.x*precision)/precision;
  result.y = floorf(v.y*precision)/precision;
  result.z = floorf(v.z*precision)/precision;

  return result;
}
