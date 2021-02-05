#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 10 17:58:45 2021

@author: Guilherme Ferrari Fortino

Adaptation from https://github.com/jczamorac/Tracking_RANSAC

"""

import numpy as np
import ctypes

class Line_c():
    """
    Ransac algorithm to fit a line with options to random sampling and
    criteria for the best model.
    mode = 0 -> Chooses two random points from cluster.
    mode = 1 -> Chooses two random points based on gaussian sampling.
    mode = 2 -> Chooses two random points based on weight/charge sampling.
    
    Parameters
    ----------
    
    data : numpy array
        Array with the cluster.
    number_it : int
        Number of iterations.
    min_dist : flaot
        Minimum distance from point to line.
    charge : numpy array
        Charge / weights from the dataset.
    mode : int, optional
        Random sampling mode. The default is 0.
    selection_mode : int, optional
        Parameter of ransac best model. If 0 the best model is the one with
        the most inliers. If 1 the best model is the one with the minor sum of
        the squared distances / number of inliers. The default is 0.
    
    Returns
    -------
    inliers: numpy array
        Inliers coefficients.
    versor: numpy array
        Versor of the best model.
    point: numpy array
        Reference point where it intercepts the line.
    """
    def __init__(self, data, number_it, min_dist, charge, mode: int = 0, selection_mode: int = 0):
        #print("\n\n", len(data), "\n")
        self.data      = data.copy().ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self.number_it = ctypes.c_int(number_it)
        self.min_dist  = ctypes.c_double(min_dist)
        self.charge    = charge.copy().ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self.versor    = np.zeros(3).ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self.inliers   = np.zeros(len(data)).ctypes.data_as(ctypes.POINTER(ctypes.c_int))
        self.pb        = np.zeros(3).ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self.size      = ctypes.c_int(len(data))
        self.mode      = ctypes.c_int(mode)
        self.sm        = ctypes.c_int(selection_mode)
        arq            = ctypes.CDLL("./line.so") #ctypes.CDLL("./teste.so")
        self.model     = None
        if selection_mode == 0:
            self.model = arq.Ransac
        else:
            self.model = arq.Ransac_2
        self.model.argtypes = [ctypes.POINTER(ctypes.c_double), # double (*data)[3]
                               ctypes.POINTER(ctypes.c_double), # double *versor
                               ctypes.POINTER(ctypes.c_double), # double* pb
                               ctypes.POINTER(ctypes.c_int),    # int *inliers
                               ctypes.POINTER(ctypes.c_double), # charge (weights)
                               ctypes.c_int,                    # int number_it
                               ctypes.c_double,                 # double min_dist
                               ctypes.c_int,                    # int size
                               ctypes.c_int]                    # int mode

        self.restype = ctypes.c_int
        
    def fit(self):
        num_inliers = int(self.model(self.data, self.versor, self.pb, self.inliers,
                                     self.charge, self.number_it, self.min_dist,
                                     self.size, self.mode, self.sm))
        inliers = np.array([self.inliers[i] for i in range(num_inliers)])
        versor  = np.array([float(self.versor[0]), float(self.versor[1]), float(self.versor[2])])
        pb      = np.array([float(self.pb[0]), float(self.pb[1]), float(self.pb[2])])
        return inliers, versor, pb
        
        
        
        
        
        
        
        
        
        
        
        
        
        