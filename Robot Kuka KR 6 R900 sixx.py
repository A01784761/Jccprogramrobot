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
        rtb.RevoluteDH(d=0.4, a=0.025, alpha=np.pi/2, qlim=[np.deg2rad(-170), np.deg2rad(170)]),
        rtb.RevoluteDH(d=0, a=0.455, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-190), np.deg2rad(45)]),
        rtb.RevoluteDH(d=0, a=0.035, alpha=np.pi/2, qlim=[np.deg2rad(-120), np.deg2rad(156)]),
        rtb.RevoluteDH(d=0.5, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-185), np.deg2rad(185)]),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-120), np.deg2rad(120)]),
        rtb.RevoluteDH(d=0.08, a=0.0, alpha=0, qlim=[np.deg2rad(-350), np.deg2rad(350)]),
    ], name="Robot Kuka KR 6 R900 sixx", base=SE3(0,0,0))
print(robot)

robot.qz=[np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)]
# robot.teach(robot.qz)

T_inicial = robot.fkine(robot.qz)
print(f"Matriz de posición inicial:\n{T_inicial}")
