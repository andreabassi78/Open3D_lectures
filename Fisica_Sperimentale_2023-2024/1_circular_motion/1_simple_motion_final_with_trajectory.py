import open3d as o3d
import numpy as np
from numpy.linalg import norm

body = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
body.compute_vertex_normals()

radius = 1.0
pos = np.array((radius,0.0,0.0))
body.translate(pos)

#create a reference frame with axis x,y,z
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0]) 

#create a Point Cloud showing the trajectory
tail = o3d.geometry.PointCloud()
tail.points = o3d.utility.Vector3dVector([pos]) 

vis = o3d.visualization.Visualizer()
vis.create_window(width=768, height=768)
vis.add_geometry(frame)
vis.add_geometry(body)
vis.add_geometry(tail)

N = 1250 # number of frames in the movie
dt = 0.004 # temporal sampling
f = 0.5 # Hz 
omega = np.array( (0.0,0.0,2*np.pi*f) )   # frequenza angolare [rad/s]
t = 0
for i in range(N):

    t = t + dt

    # update body position 
    pos = np.array([radius * np.cos(norm(omega)*t), radius* np.sin(norm(omega)*t),0])
    v = np.cross(omega,pos)
    dr = v *dt 
    body.translate(dr)
    
    #update tail showing the trajectory
    tail.points.extend([pos])
    
    #visualizer updates
    vis.update_geometry(body)
    vis.update_geometry(tail)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()