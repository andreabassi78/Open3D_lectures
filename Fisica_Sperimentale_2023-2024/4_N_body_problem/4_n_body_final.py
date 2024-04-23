import open3d as o3d
import numpy as np
from numpy.linalg import norm

def elastic_collision(b0,b1):
    vrel = b0.vel - b1.vel
    rrel = b0.pos - b1.pos
    distance = norm(rrel)
    ratio0 = 2 * b1.mass / (b0.mass + b1.mass) 
    ratio1 = 2 * b0.mass / (b0.mass + b1.mass) 
    b0.vel = b0.vel - ratio0 * np.dot(vrel,rrel) / distance**2 *rrel 
    b1.vel = b1.vel - ratio1 * np.dot(-vrel,-rrel) / distance**2 *(-rrel)

def completely_inelastic_collision(b0,b1):
    m0 = b0.mass
    m1 = b1.mass
    p = m0 * b0.vel + m1 * b1.vel
    vm =  p /(m0+m1)
    b0.vel = vm
    b1.vel = vm

def inelastic_collision(b0,b1,dt = 0.002):
    # ball is working as a springs with a frinction proportional to velocity                 
    K = 10 # elastic constant (N/m)
    B = 5 # damping (Ns/m)
    vrel = b0.vel - b1.vel
    rrel = b0.pos - b1.pos
    distance =  norm(rrel)
    F = + K * rrel / distance * (b0.radius+b1.radius-distance)  - B * vrel
    acceleration0 = +F / b0.mass
    acceleration1 = -F / b1.mass
    b0.vel = b0.vel + acceleration0*dt
    b1.vel = b1.vel + acceleration1*dt    


def calculate_momentum(bodies):
    p = np.zeros(3, dtype=float)
    for body in bodies:
        p += body.mass * body.vel
    return p

def calculate_cm(bodies):
    m_tot = 0
    for body in bodies:
        m_tot += body.mass 
    cm = calculate_momentum(bodies)/m_tot 
    return cm


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

np.random.seed(156)

N_bodies = 20
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

N = 2000 # number of frames in the movie

dt = 0.002

G = 1

c_m = calculate_cm(bodies)
tail = o3d.geometry.PointCloud()
tail.points = o3d.utility.Vector3dVector([c_m]) 
vis.add_geometry(tail)

for i in range(N):   

    for body in bodies:
        body.acc = np.zeros(3)
        for other_body in bodies:
            if body is not other_body:
                r = body.pos - other_body.pos
                if norm(r) < body.radius+other_body.radius:
                    inelastic_collision(body,other_body)
                else:
                    ur = r/norm(r)
                    body.acc += -ur * G * other_body.mass/norm(r)**2 # acceleration given by other body

    for body in bodies:
        body.move(dt)
        vis.update_geometry(body.mesh)

    print('Total momentum:', calculate_momentum(bodies) )
    # update tail showing the trajectory
    c_m = calculate_cm(bodies)
    tail.points.extend([c_m])
    vis.update_geometry(tail)

    vis.poll_events()
    vis.update_renderer()


vis.destroy_window()