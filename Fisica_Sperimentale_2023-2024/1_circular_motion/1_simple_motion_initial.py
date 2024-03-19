import open3d as o3d
import numpy as np

body = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
body.compute_vertex_normals()

pos = np.array((0.0,0.0,0.0))
vel = np.array((1.0,0.0,0.0))

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

N = 250 # number of frames in the movie
dt = 0.004 # temporal sampling

for i in range(N):

    # update body position 
    dr = vel * dt
    pos = pos + dr
    body.translate(dr)

    #update tail showing the trajectory
    tail.points.extend([pos])
    
    #visualizer updates
    vis.update_geometry(body)
    vis.update_geometry(tail)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()