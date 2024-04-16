import open3d as o3d
import numpy as np
from numpy.linalg import norm

YELLOW = np.array((0.7,0.6,0.0))
BLUE = np.array((0.0,0.6,0.8))


class Body:
    def __init__(self, radius, color, pos, vel, mass):
        self.radius = radius
        self.mesh = o3d.geometry.TriangleMesh.create_sphere(radius)
        self.mesh.compute_vertex_normals()
        self.mesh.paint_uniform_color(color)
        self.pos = pos    
        self.mesh.translate(pos)
        self.vel = vel
        self.acc = np.zeros(3)
        self.mass = mass

    def move(self,dt):
        dr = self.vel * dt
        self.mesh.translate(dr)
        self.pos = self.pos + dr    

body0 = Body(radius = 0.2,
             color = BLUE,
             pos = np.array((1.0,0.0,0.0)),
             vel = np.array((-1.0,0.0,0.0)),
             mass = 2.0
            ) 

body1 = Body(radius = 0.1,
             color = YELLOW,
             pos = np.array((-1.0,0.0,0.0)),
             vel = np.array((1.0,0.0,0.0)),
             mass = 1.0
            ) 

frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

vis = o3d.visualization.Visualizer()
vis.create_window(width=768,height =768)
vis.add_geometry(frame)
vis.add_geometry(body0.mesh)
vis.add_geometry(body1.mesh)

N = 300 # number of frames in the movie
dt = 0.01

for i in range(N):

    body0.move(dt)
    body1.move(dt)

    distance = body0.pos - body1.pos
    if norm(distance) < (body0.radius+body1.radius):
        print('collision!')
    
    vis.update_geometry(body0.mesh)
    vis.update_geometry(body1.mesh)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()