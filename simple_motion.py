import open3d as o3d
import numpy as np
from numpy.linalg import norm

body = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
pos = np.array((0.0,0.0,0.0))
vel = np.array((2.0,2.0,0.0))
body.compute_vertex_normals()

mesh_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

tail = o3d.geometry.PointCloud()
tail.points = o3d.utility.Vector3dVector([pos]) 

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh_coord_frame)
vis.add_geometry(body)
vis.add_geometry(tail)

N = 200 # number of frames in the movie
dt = 0.01
for i in range(N):

    acc = np.array((0.0,-9.81,0.0))
    dv = acc * dt
    vel += dv
    dr = vel * dt
    pos += dr

    body.translate(dr)
    tail.points.extend([pos])
    
    vis.update_geometry(body)
    vis.update_geometry(tail)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()