import open3d as o3d
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt

class Body:
    def __init__(self, radius, pos, vel, mass):
        self.radius = radius
        self.pos = pos    
        self.vel = vel
        self.acc = np.zeros(3)
        self.mass = mass

    def move(self,dt):
        dr = self.vel * dt
        self.pos += dr
        self.vel += self.acc *dt 

def elastic_collision(b0,b1):
    vrel = b0.vel - b1.vel
    rrel = b0.pos - b1.pos
    distance = norm(rrel)
    ratio0 = 2 * b1.mass / (b0.mass + b1.mass) 
    ratio1 = 2 * b0.mass / (b0.mass + b1.mass) 
    b0.vel = b0.vel - ratio0 * np.dot(vrel,rrel) / distance**2 *rrel 
    b1.vel = b1.vel - ratio1 * np.dot(-vrel,-rrel) / distance**2 *(-rrel)

def bounce(body,d):
    pos = body.pos
    vel = body.vel
    for i in range(3):  # x, y, z dimensions
        over = np.abs(pos[i]) > d
        vel[over, i] *= -1  # Reverse velocity for bodies hitting the boundary


def create_box(size):
    # Create a box represented by lines for visualization
    box = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([
            [-size, -size, -size], [size, -size, -size], 
            [-size, size, -size], [size, size, -size],
            [-size, -size, size], [size, -size, size], 
            [-size, size, size], [size, size, size]]),
        lines=o3d.utility.Vector2iVector([
            [0, 1], [0, 2], [1, 3], [2, 3],
            [4, 5], [4, 6], [5, 7], [6, 7],
            [0, 4], [1, 5], [2, 6], [3, 7]]))
    box.paint_uniform_color([0, 0.7, 0])
    return box

vis = o3d.visualization.Visualizer()
vis.create_window(width=768,height =768)
vis.add_geometry(create_box(size=1.))

N_bodies = 500
bodies = []
for _index in range(N_bodies):
    pos = np.random.uniform(low=-1, high=1, size=(3))
    vel = np.random.uniform(low=-1, high=1, size=(3))
    body = Body(radius= 0.01, pos=pos, vel=vel, mass=1.0) 
    bodies.append(body) 

pcd = o3d.geometry.PointCloud()

xyz = np.zeros((N_bodies,3))
pcd.points = o3d.utility.Vector3dVector(xyz)
pcd.paint_uniform_color([0.4, 0.5, 0.6])

vis.add_geometry(pcd)

N = 100 # number of frames in the movie
dt = 0.01
xyz = np.zeros((N_bodies,3))
vxyz = np.zeros((N_bodies,3))
for i in range(N):          
    
    for i, body in enumerate(bodies): 
        body.move(dt)
        bounce(body, d=1.0)
        # for j in range(i + 1, len(bodies)):
        #     other_body = bodies[j]
        #     r = body.pos - other_body.pos
        #     if norm(r) < (body.radius + other_body.radius):
        #         elastic_collision(body, other_body)

    for index,body in enumerate(bodies):
        xyz[index,:] = body.pos
        vxyz[index,:] = body.pos


    # counts, bins = np.histogram(vxyz)
    # plt.stairs(counts, bins)
    # plt.show()
    
    pcd.points = o3d.utility.Vector3dVector(xyz)
    vis.update_geometry(pcd)

    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()