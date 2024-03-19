import open3d as o3d
import numpy as np
from numpy.linalg import norm

body = o3d.geometry.TriangleMesh.create_icosahedron(radius=0.05)
r = np.array((0.0,1.0,0.0))
v = np.array((1.0,0.0,0.0))
body.translate(r)
body.compute_vertex_normals()

coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

tail = o3d.geometry.PointCloud()
tail.points = o3d.utility.Vector3dVector([r]) 

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(coordinate_frame)
vis.add_geometry(body)
vis.add_geometry(tail)

N = 1000 # number of frames in the movie
dt = 0.01

acc_centripetal = norm(v)**2/norm(r)

for i in range(N):

    # update position
    dr = v * dt
    r += dr
    body.translate(dr)

    #update velocity
    ur = r/norm(r)
    a = - ur * acc_centripetal
    v = v + a * dt
    
    
    tail.points.extend([r])
    vis.update_geometry(body)
    vis.update_geometry(tail)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()