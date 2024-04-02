import open3d as o3d
import numpy as np
from numpy.linalg import norm


def calculate_energy(m0,m1,r0,r1,v0,v1):
    K = 1/2 * m0 * norm(v0)**2 + 1/2 * m1 * norm(v1)**2 #kinetic energy of the system
    U = - G*m0*m1/norm(r0-r1) # TODO double check that this is the correct total potential energy of the system
    E = K+U
    return E


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
vel_earth = -vel*m/M # conservation of momentum

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

    dr_earth = vel_earth *dt
    earth.translate(dr_earth)
    pos_earth += dr_earth
    
    r = pos_moon-pos_earth # position vector of the Moon
    u_r = r/norm(r) # unit vector of the force
    force = - u_r * G*M*m/norm(r)**2.0 # gravitational like force force on the moon
    acc_moon = force/m
    acc_earth = -force/M

    vel += acc_moon * dt
    vel_earth += acc_earth *dt

    print(calculate_energy(m,M,pos_moon,pos_earth,vel,vel_earth))

    moon_tail.points.extend([pos_moon])
        
    vis.update_geometry(earth)
    vis.update_geometry(moon)
    vis.update_geometry(moon_tail)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()