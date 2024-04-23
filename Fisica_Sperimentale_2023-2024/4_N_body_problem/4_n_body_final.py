import open3d as o3d
import numpy as np
from numpy.linalg import norm

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
        self.pos += dr
        self.vel += self.acc *dt    
               
vis = o3d.visualization.Visualizer()
vis.create_window(width=768,height =768)

frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
vis.add_geometry(frame)

np.random.seed(123)

N_bodies = 5
bodies = []
for idx in range(N_bodies):
    body = Body(radius = 0.05,
             color = np.array((0.9, 0.6, 0.0)),
             pos = np.random.uniform(low=-1,high=1,size=3),
             vel = np.random.uniform(low=-1,high=1,size=3),
             mass = 1.0
            )
    vis.add_geometry(body.mesh)
    bodies.append(body) 

N = 500 # number of frames in the movie

dt = 0.002

G = 1

for i in range(N):   

    for body in bodies:
        body.acc = np.zeros(3)
        for other_body in bodies:
            if body is not other_body:
                r = body.pos - other_body.pos
                ur = r/norm(r)
                body.acc += -ur * G * other_body.mass/norm(r)**2 # acceleration given by other body

        body.move(dt)
        vis.update_geometry(body.mesh)

    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()