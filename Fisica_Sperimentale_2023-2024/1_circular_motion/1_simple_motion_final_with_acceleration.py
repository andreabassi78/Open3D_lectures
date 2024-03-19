import open3d as o3d
import numpy as np
from numpy.linalg import norm

body = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
body.compute_vertex_normals()
radius = 1.0
pos = np.array((radius,0.0,0.0))
vel = np.array((0.0,1.0,0.0))
body.translate(pos)

# show velocity vector
vel_arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.01, cone_radius=0.03, cylinder_height=0.3, cone_height=0.1,)
vel_arrow.translate(pos)
R = vel_arrow.get_rotation_matrix_from_axis_angle((-np.pi/2,0.0, 0))
vel_arrow.rotate(R,pos)

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
vis.add_geometry(vel_arrow)

N = 1000 # number of frames in the movie
dt = 0.02 # temporal sampling

acc_tang = 0.1

old_vel = vel

for i in range(N):

    # update body position 
    ur = pos/norm(pos)

    ut = vel/norm(vel) 

    acc_centripetal = norm(vel)**2 / radius # magnitude of the acceleration

    a = - ur * acc_centripetal + ut * acc_tang

    dv = a *dt

    vel = vel + dv

    dr = vel * dt

    pos = pos + dr

    body.translate(dr)
    

    # update velocity vector position and direction
    vel_arrow.translate(dr)
    delta_angle = norm(dv)/norm(vel)
    R = vel_arrow.get_rotation_matrix_from_axis_angle((0,0,delta_angle))
    vel_arrow.rotate(R,pos)
    vel_arrow.scale(norm(vel)/norm(old_vel),center=pos)
    old_vel = vel

    #update tail showing the trajectory
    tail.points.extend([pos])
    
    #visualizer updates
    vis.update_geometry(body)
    vis.update_geometry(tail)
    vis.update_geometry(vel_arrow)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()