#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 10 17:58:45 2021

@author: Guilherme Ferrari Fortino
"""

import numpy as np
import matplotlib.pyplot as plt
import ctypes



plt.rcParams.update({'figure.max_open_warning': 0})
plt.rcParams.update({'figure.dpi': 140})
plt.rcParams.update({'figure.figsize': (10, 9)})
plt.rcParams.update({'axes.grid': True})

class Line():
    '''
    Receives:
    Data **double, int number_it, double min_dist, double *versor,
    int *inliers, int size
    '''
    def __init__(self, data, number_it, min_dist, versor, inliers):
        #print("\n\n", len(data), "\n")
        self.data      = data.copy().ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self.number_it = ctypes.c_int(number_it)
        self.min_dist  = ctypes.c_double(min_dist)
        self.versor    = versor.copy().ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self.inliers   = inliers.copy().ctypes.data_as(ctypes.POINTER(ctypes.c_int))
        self.size      = ctypes.c_int(len(data))
        arq            = ctypes.CDLL("./teste.so")
        self.model     = arq.Ransac
        self.model.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                               ctypes.c_double, ctypes.POINTER(ctypes.c_double),
                               ctypes.POINTER(ctypes.c_int), ctypes.c_int]
        self.restype = ctypes.c_int
        
    def fit(self):
        num_inliers = int(self.model(self.data, self.number_it, self.min_dist, 
                                 self.versor, self.inliers, self.size))
        #print(num_inliers)
        inliers = np.array([self.inliers[i] for i in range(num_inliers)])
        #print("Inliers python = ", num_inliers)
        #print(inliers)
        versor  = np.array([float(self.versor[0]), float(self.versor[1]), float(self.versor[2])])
        #print(versor)
        return inliers, versor
        
        
        
        
        
        
        
        
        
        
        
        
        
        