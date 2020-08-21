import open3d as o3d
import numpy as np
import random
import copy 

import open3d as o3d
import numpy as np
import copy

def drawPlane(plane):
	print(plane.rMatrix.T)
	box = o3d.geometry.TriangleMesh.create_box(width=plane.size[0], height=plane.size[1], depth=0.05).translate(plane.limits_f_plane[0, :])
	box = box.rotate(plane.rMatrix.T, center=(0,0,0)).translate(plane.tMatrix)
	return box
