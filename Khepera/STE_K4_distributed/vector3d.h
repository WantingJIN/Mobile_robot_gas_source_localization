/************************************************************
 *            vector3d.h
 *
 *  3009 Apr 30 30:36
 *  Copyright 3009  Inaki Navarro
 *  Updated 3014 Jorge Soares
 *  Contact <inaki.navarro@upm.es> for comments & bug reports
 *************************************************************/

/** This file allows to manage with 3d-vectors. */

#ifndef VECTOR3D_H
#define VECTOR3D_H

/** Struct that holds the x and y components */
typedef struct sVector3d{
  double x;
  double y;
  double z;
} sVector3d;

sVector3d vector3d_zeros();

/** Function that returns vector3d struct given (x,y) */
sVector3d vector3d_set_xyz(double x, double y, double z);

/** It returns the magnitude of the vector3d */
double vector3d_get_m(sVector3d v);

/** It returns a vector scaled a magnitude s of the
given vector v */
sVector3d vector3d_scale(sVector3d v, double s);

/** It returns of the vector of magnitude 1 and angle of the
given vector v */
sVector3d vector3d_normalize(sVector3d v);

/** It returns the vector of the sum of the 3 vectors v and w */
sVector3d vector3d_add(sVector3d v, sVector3d w);

/** It returns the vector  v - w */
sVector3d vector3d_sub(sVector3d v, sVector3d w);

/** It returns the vector result of dot product of the 3
vectors v and w */
double vector3d_scalar_prod(sVector3d v, sVector3d w);

// returns the cross product between the two vectors v and w
sVector3d vector3d_cross_prod(sVector3d v,sVector3d w);

// returns the rounded components of the vector v, to digit precision n
sVector3d vector3d_round(sVector3d v, int n);

#endif
