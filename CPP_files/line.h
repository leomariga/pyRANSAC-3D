#ifndef __LINE__H
#define __LINE__H
#include <stdlib.h> 
#include <iostream>
#include <math.h>
#include <time.h>
#include <vector>
#include <random>

/*

Author: Guilherme Ferrari Fortino

Adaptation from https://github.com/jczamorac/Tracking_RANSAC

*/

extern "C" {

void swap(int &a, int &b);
void getPDF(double* charge, double Tcharge, int size, double* PDF);
void get_random(int &ind1, int &ind2, double (*data)[3], double* charge, double* PDF, double TCharge, double AvgCharge, double TwiceAvCharge, int mode, int size);
int Ransac(double (*data)[3], double *versor, double* pb, int *inliers, double* charge, int number_it, double min_dist, int size, int mode);
int Ransac_2(double (*data)[3], double *versor, double* pb, int *inliers, double* charge, int number_it, double min_dist, int size, int mode);
void Fit3D(double *vX, double *vY, double *vZ, double *vQ, double *versor, double *Pb, int size);
}
#endif
