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

robot=rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.128, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        rtb.RevoluteDH(d=0.176, a=0.612, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        rtb.RevoluteDH(d=-0.128, a=0.572, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        rtb.RevoluteDH(d=0.116, a=0.0, alpha=np.pi/2, offset=np.pi/2, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        rtb.RevoluteDH(d=0.116, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        rtb.RevoluteDH(d=0.092, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Ur10", base=SE3(0,0,0))
print(robot)

robot.qz=[np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)]
# robot.teach(robot.qz)

T_inicial = robot.fkine(robot.qz)
print(f"Matriz de posición inicial:\n{T_inicial}")

