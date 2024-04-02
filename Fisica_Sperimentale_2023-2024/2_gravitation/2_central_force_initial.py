import open3d as o3d
import numpy as np
from numpy.linalg import norm

G = 1.0 # Arbitrary gravitational constant.
dt = 0.001 # Set temporal sampling in seconds 

# define Moon
m = 1  # mass of the Moon 
radius_moon = 0.03 # size of the Moon
pos_moon = np.array((0.0, 2.0, 0.0)) 
vel = np.array((-20.0, 0.0, 0.0))
moon = o3d.geometry.TriangleMesh.create_sphere(radius=radius_moon)
moon.paint_uniform_color(np.array((0.3, 0.4, 0.4)))
moon.translate(pos_moon)
 
# define Earth 
M = 1000 # mass of the Earth
radius_earth = 0.1 # size of the Earth
pos_earth = np.array((0.0, 0.0, 0.0)) 
earth = o3d.geometry.TriangleMesh.create_sphere(radius=radius_earth) 
earth.paint_uniform_color(np.array((0.0, 0.6, 0.4)))
# We assume here that the Earth is not moving, only the Moon orbits around the Earth

# visualization
earth.compute_vertex_normals()
moon.compute_vertex_normals()
vis = o3d.visualization.Visualizer()
vis.create_window(width=768,height=768)
vis.add_geometry(earth)
vis.add_geometry(moon)

# create list of points to show the trajectory
moon_tail = o3d.geometry.PointCloud()
moon_tail.points = o3d.utility.Vector3dVector([pos_moon]) 
vis.add_geometry(moon_tail)

N = 1000 # number of frames in the movie
for i in range(N):

    dr = vel * dt
    moon.translate(dr)
    pos_moon += dr

    r = pos_moon-pos_earth # position vector of the Moon
    u_r = r/norm(r) # unit vector of the force
    force = - u_r * G*M*m/norm(r)**2 # gravitational like force force on the moon
    acc = force/m
    vel += acc * dt

    moon_tail.points.extend([pos_moon])
        
    vis.update_geometry(earth)
    vis.update_geometry(moon)
    vis.update_geometry(moon_tail)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()