import open3d as o3d
import numpy as np
from numpy.linalg import norm

G = 6.67408e-11 # Gravitational constant. All Units are in SI

dt = 3000 # Set temporal sampling in seconds 
   
m = 7.342e22
radius_moon = 10e6 # Exaggerated size of the Moon
pos_moon = np.array((0., 4.054e8,0.)) # Moon at apogee (m)
vel_moon = np.array((-970.,0.,0.)) # Moon velocity at apogee(m/s)
 
M = 5.972e24
radius_earth = 50e6 # Exaggerated size of the Earth
pos_earth = np.array((0.,0.,0.))
vel_earth =-vel_moon*m/M # Conservation of momentum

earth = o3d.geometry.TriangleMesh.create_sphere(radius=radius_earth) 
moon = o3d.geometry.TriangleMesh.create_sphere(radius=radius_moon) 
earth.compute_vertex_normals()
moon.compute_vertex_normals()
moon.translate(pos_moon)

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(earth)
vis.add_geometry(moon)

moon_tail = o3d.geometry.PointCloud()
moon_tail.points = o3d.utility.Vector3dVector([pos_moon]) 
vis.add_geometry(moon_tail)

N = 1000 # number of frames in the movie
for i in range(N):

    distance = pos_moon-pos_earth
    force = - G*M*m * distance/norm(distance)**3 # force on the moon
    acc_moon = force/m
    acc_earth = -force/M
    
    vel_moon += acc_moon * dt
    vel_earth += acc_earth * dt
    
    dr_moon = vel_moon * dt
    dr_earth = vel_earth * dt
    
    pos_moon += dr_moon
    pos_earth += dr_earth
    
    earth.translate(dr_earth)
    moon.translate(dr_moon)

    moon_tail.points.extend([pos_moon])
    
    vis.update_geometry(earth)
    vis.update_geometry(moon)
    vis.update_geometry(moon_tail)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()