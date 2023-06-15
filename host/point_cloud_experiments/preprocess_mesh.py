import open3d as o3d
import numpy as np 


# load mesh
mesh = o3d.io.read_triangle_mesh("office_chair.stl")

# center the mesh 
mesh.translate(-mesh.get_center())

# scale mesh 
max_size = 1.30 # size of the chair in meters along its longes extend 
points = mesh.vertices
max_extent = np.amax(points, axis=0)
min_extent = np.amin(points, axis=0)
size = max_extent - min_extent
scaling =  max_size / np.max(size)
mesh.scale(scaling, center=mesh.get_center())

# paint 
mesh.paint_uniform_color([0, 0, 1]) 

# save
o3d.io.write_triangle_mesh("56.obj", mesh)