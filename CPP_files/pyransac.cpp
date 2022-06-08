#include <Python.h>
#include <numpy/arrayobject.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <random>
#include <numeric>
#include <algorithm>
// #include <thread>
// #include <omp.h>
#define PY_SSIZE_T_CLEAN
#define NPY_NO_DEPRECATED_API NPY_1_9_API_VERSION

/*

Author: Guilherme Ferrari Fortino

Adaptation from https://github.com/jczamorac/Tracking_RANSAC

*/

extern "C"{

#define M_PI 3.14159265358979323846

double norm(double* A){
	return (double)sqrt((double)pow(A[0], 2.) + (double)pow(A[1], 2.) + (double)pow(A[2], 2.));
}

void cross_prod(double* C, double* A, double* B){
	C[0] = (double) A[1]*B[2] - A[2]*B[1];
	C[1] = (double) -(A[0]*B[2] - A[2]*B[0]);
	C[2] = (double) A[0]*B[1] - A[1]*B[0];
}

int ransac_line(std::vector<std::vector<double>>& data, PyObject* PyInliers, PyObject* PyVersors, PyObject* PyPoints, int number_it, double min_dist, int min_inlier){
	int indice1, indice2, loop, j, size = data.size(), num_inliers_1 = 0, best = 0;
	std::vector <int> parcial;
	double versor[3], point[3];
	std::vector<double> pesos;
	double parcial_versor[3];
	double norma1, norma2;
	double point_A[3];
	double point_B[3];
	double aux[3];
	double cross[3];
	parcial.reserve(size);

	for(int i = 0; i < number_it; i++){

		indice1 = rand() % size;
		indice2 = rand() % size;
		while (indice1 == indice2) indice2 = rand() % size;
		for(loop = 0; loop < 3; loop++){
			point_A[loop]        = data[indice1][loop];
			point_B[loop]        = data[indice2][loop];
			parcial_versor[loop] = (double) point_B[loop] - point_A[loop];
		}
		norma1 = norm(parcial_versor);
		for(loop = 0; loop < 3; loop++) parcial_versor[loop] = (double) parcial_versor[loop]/norma1;
		for(j = 0; j < size; j++){
			for(loop = 0; loop < 3; loop++) aux[loop] = point_A[loop] - data[j][loop];
			
			cross_prod(cross, parcial_versor, aux);
			norma2 = norm(cross);

			if(fabs(norma2) <= min_dist){
				// parcial.push_back(j);
				num_inliers_1 += 1;
			} 
		}
		if (num_inliers_1 >= min_inlier && num_inliers_1 > best){
			best = num_inliers_1;
			for (loop = 0; loop < 3; loop++){
				versor[loop] = parcial_versor[loop];
				point[loop] = point_A[loop];
			}
		}
		num_inliers_1 = 0;
	}

	if (best == 0){
		return -1;
	}

	for(j = 0; j < size; j++){
		cross_prod(cross, versor, point);
		norma2 = norm(cross);
		if(fabs(norma2) <= min_dist) parcial.push_back(j); 
	}					

	// Guarda inliers, versor e ponto
	for(int m = 0; m < parcial.size(); m++) PyList_Append(PyInliers, PyLong_FromLong((long int) parcial[m]));
	for (loop = 0; loop < 3; loop++){
		PyList_Append(PyVersors, PyFloat_FromDouble(versor[loop]));
		PyList_Append(PyPoints, PyFloat_FromDouble(point[loop]));
	}
	return 0;
}

static PyObject* Ransac(PyObject* self, PyObject* args){
	// PyObject* list;
	PyArrayObject *p;
	int number_it, min_inliers;
	double min_dist;
	NpyIter *in_iter;
	// if(!PyArg_ParseTuple(args, "Oidii", &list, &number_it, &min_dist, &min_inliers, &mode)){
	if(!PyArg_ParseTuple(args, "O!idi", &PyArray_Type, &p, &number_it, &min_dist, &min_inliers)){
		return NULL;
	}
	if (PyArray_DESCR(p)->type_num != NPY_DOUBLE){
		PyErr_SetString(PyExc_TypeError, "Type np.float64 expected for array.");
		return NULL;
	}

    if (PyArray_NDIM(p)!=2){
		PyErr_SetString(PyExc_TypeError, "Array must be two dimensional.");
		return NULL;
	}
	// Py_ssize_t size = PyList_GET_SIZE(list);
	int rows = PyArray_DIM(p, 0);
	int cols = PyArray_DIM(p, 1);
	if (cols != 3){
		PyErr_SetString(PyExc_TypeError, "Array must have three columns.");
		return NULL;
	}
	std::vector<std::vector<double>> data;
	std::vector<long int> inliers;
	in_iter = NpyIter_New(p, NPY_ITER_READONLY, NPY_KEEPORDER, NPY_NO_CASTING, NULL);
	double ** in_dataptr = (double **) NpyIter_GetDataPtrArray(in_iter);
	NpyIter_IterNextFunc *in_iternext = NpyIter_GetIterNext(in_iter, NULL);
	// for(int i = 0; i < (int) size; i++){
	for(int i = 0; i < rows; i++){
		// PyObject* Point3D = PyList_GetItem(list, i);
		std::vector<double> PartVec(rows);
		for(int j = 0; j < cols; j++){
			// PartVec[j] = PyFloat_AsDouble(PyList_GetItem(Point3D, j));
			PartVec[j] = **in_dataptr;
			in_iternext(in_iter);
		}
		// charge[i] = PyFloat_AsDouble(PyList_GetItem(Point3D, 3));
		data.push_back(PartVec);
	};
	NpyIter_Deallocate(in_iter);
	PyObject* PyInliers = PyList_New(0);
	PyObject* PyVersor  = PyList_New(0);
	PyObject* PyPb      = PyList_New(0);
	ransac_line(data, PyInliers, PyVersor, PyPb, number_it, min_dist, min_inliers);
	PyObject * NPInliers = PyArray_FROM_OTF(PyInliers, NPY_INT, NPY_ARRAY_IN_ARRAY);
	Py_DecRef(PyInliers);
	return Py_BuildValue("(OOO)", NPInliers,
	 PyArray_FROM_OTF(PyVersor, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY),
	 PyArray_FROM_OTF(PyPb, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY));
}

static PyMethodDef myMethods[] = {
	{"ransac_line", Ransac, METH_VARARGS, "3D Line Ransac CPP implementation."},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef pyransac3d_wrapper = {
    PyModuleDef_HEAD_INIT,
    "pyransac3d_wrapper",
    "Wrapper for pyransac3D.",
    -1,
    myMethods    
};

PyMODINIT_FUNC PyInit_pyransac3d_wrapper(void){
	import_array();
    return PyModule_Create(&pyransac3d_wrapper);
}


}
