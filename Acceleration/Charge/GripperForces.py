import numpy as np
import scipy as sp
import sympy as sym
import math

atotal = np.array([0.0, 0.0, 0.0])
alpha = np.array([0.0, 0.0, 0.0])
r = np.array([0.0, 0.0, 0.0])
omega = 0.0
alineal = np.array([0.0, 0.0, 0.0])
mass = 0.0
gravity = np.array([0.0, 0.0, 0.0])

def calculateGripperForces(atotal, alpha, r, omega, alineal, mass, gravity):
    # Gravitational force
    F_gravity = mass * gravity
    
    # Inertial force due to linear acceleration
    F_inertia = mass * alineal
    
    # Centripetal force due to angular velocity
    F_centripetal = mass * (omega**2 * r)
    
    # Coriolis force due to angular acceleration
    F_coriolis = 2 * mass * np.cross(omega, alpha)
    
    # Total force required by the gripper
    F_total = F_gravity + F_inertia + F_centripetal + F_coriolis
    
    return F_total

# Example usage
F_total = calculateGripperForces(atotal, alpha, r, omega, alineal, mass, gravity)
print(f"Total force required by the gripper: {F_total}")



