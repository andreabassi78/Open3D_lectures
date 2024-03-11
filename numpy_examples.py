import numpy as np
from numpy.linalg import norm

v1 = np.array([0.0, 1.0, 0.0]) 

v2 = np.ones(3)

v3 = np.random.uniform(low=0.0, high=1.0, size=3)

v4 = v1 * v3 # elementwise product

s = np.dot(v1, v3) # scalar product

v5 = np.cross(v1, v3) # cross product

mag = np.sqrt(v3[0]**2+v3[1]**2+v3[2]**2) # magnitude of the vector using Pythagorean theorem

mag = norm(v3) # magnitude using the norm function

print(mag)
