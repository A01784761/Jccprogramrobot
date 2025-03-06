import sympy as sp
from sympy.matrices import rot_axis3
# Para el ejemplo donde generamos la matriz DH
from spatialmath import *
from spatialmath.base import *
# Para poder Graficar
import matplotlib.pyplot as plt
import numpy as np
#Para usar el DH
import roboticstoolbox as rtb
np.set_printoptions(suppress=True, precision=4)


# Declaramos la matriz Denavit Hartenberg. 
robot=rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.550, a=0.2, alpha=np.pi/2, qlim=[np.deg2rad(-185), np.deg2rad(185)]),
        rtb.RevoluteDH(d=0, a=0.8, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-90), np.deg2rad(120)]),
        rtb.RevoluteDH(d=0, a=0.125, alpha=np.pi/2, qlim=[np.deg2rad(-184), np.deg2rad(67)]),
        rtb.RevoluteDH(d=0.815, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(180)]),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-120), np.deg2rad(120)]),
        rtb.RevoluteDH(d=0.145, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Robot Articc6-1969-60kg", base=SE3(0,0,0))
print(robot)

robot.qz=[np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)]
#robot.teach(robot.qz)

T_inicial = robot.fkine(robot.qz)
print(f"Matriz de posición inicial:\n{T_inicial}")

