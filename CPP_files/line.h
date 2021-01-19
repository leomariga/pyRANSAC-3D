#ifndef __LINE__H
#define __LINE__H
#include <stdlib.h> 
#include <iostream>
#include <math.h>
#include <time.h>
#include <vector>
extern "C" {

int Ransac(double (*data)[3], int number_it, double min_dist, double *versor, int *inliers, int size);
}
#endif