#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 10 17:58:45 2021

@author: Guilherme Ferrari Fortino

Adaptation from https://github.com/jczamorac/Tracking_RANSAC

"""

import numpy as np
import pyransac3d_wrapper as pyrsc_w

class Line_cpp:
    """ 
    Implementation for 3D Line RANSAC.

    This object finds the equation of a line in 3D space using RANSAC method. 
    This method uses 2 points from 3D space and computes a line. The selected candidate will be the line with more inliers inside the radius theshold. 

    ![3D line](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/line.gif "3D line")

    ---
    """

    def __init__(self):
        self.inliers = []
        self.A = []
        self.B = []

    def fit(self, pts, thresh=0.2, maxIteration=1000):
        """ 
        Find the best equation for the 3D line. The line in a 3d enviroment is defined as y = Ax+B, but A and B are vectors intead of scalars.

        :param pts: 3D point cloud as a `np.array (N,3)`.
        :param thresh: Threshold distance from the line which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :returns:
        - `A`: 3D slope of the line (angle) `np.array (1, 3)`
        - `B`: Axis interception as `np.array (1, 3)`
        - `inliers`: Inlier's index from the original point cloud. `np.array (1, M)`
        ---
        """
        self.inliers, self.A, self.B = pyrsc_w.ransac_line(pts, maxIteration, thresh, 2)
        return self.A, self.B, self.inliers
        
        
        
        
        
        
        
        
        
        
        
        
        