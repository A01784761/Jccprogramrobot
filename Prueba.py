import sympy as sp
from sympy.matrices import rot_axis3
from spatialmath import SE3
from spatialmath.base import *
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb

np.set_printoptions(suppress=True, precision=4)

# Definir el robot usando la matriz Denavit-Hartenberg
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(0.550, 0.2, alpha=np.pi/2, qlim=[np.deg2rad(-185), np.deg2rad(185)]),
        rtb.RevoluteDH(0, 0.8, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-90), np.deg2rad(120)]),
        rtb.RevoluteDH(d=0, a=0.125, alpha=np.pi/2, qlim=[np.deg2rad(-184), np.deg2rad(67)]),
        rtb.RevoluteDH(d=0.815, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(180)]),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-120), np.deg2rad(120)]),
        rtb.RevoluteDH(d=0.145, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Robot Articc6-1969-60kg", base=SE3(0, 0, 0)
)

# Alinear TCP con el brazo
robot.tool = SE3.OA([0, 1, 0], [0, 0, 1])
robot.configurations_str('ru')  # codo derecho arriba
robot.qz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Graficar robot
robot.plot(q=robot.qz, limits=[-1.737, 1.737, -1.737, 1.737, -2.3, 2.3], eeframe=True, backend='pyplot', shadow=True, jointaxes=False, block=True)

# Definir puntos de la trayectoria usando SE3
T = [
    SE3(-0.45, 0.45, 0.35) * SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 1 Abajo
    SE3(-0.45, 0.45, 0.50) * SE3.RPY(0.0, 0.34, 180, unit='deg'),  # 2 Arriba
    SE3(-0.45, -0.45, 0.50) * SE3.RPY(0.0, 0.34, 180, unit='deg'),  # 3 Arriba
    SE3(-0.45, -0.45, 0.35) * SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 4 Abajo
    SE3(0.45, -0.45, 0.35) * SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 5 Abajo
    SE3(0.45, -0.45, 0.50) * SE3.RPY(0.0, 0.34, 180, unit='deg'),  # 6 Arriba
    SE3(0.45, 0.45, 0.50) * SE3.RPY(0.0, 0.34, 180, unit='deg'),  # 7 Arriba
    SE3(0.45, 0.45, 0.35) * SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 8 Abajo
    SE3(-0.45, 0.45, 0.35) * SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 9 Abajo
]

# Extraer las matrices de transformación de los objetos SE3
T_matrices = np.array([p.A for p in T])

# Crear una lista vacía para almacenar los puntos de la trayectoria
via = np.empty((0, 3))

# Agregar los puntos a la trayectoria
for punto in T:
    xyz = np.array(punto.t)[:3]  # Extraer solo la parte de traslación
    via = np.vstack((via, xyz))  # Agregar el punto a la trayectoria

# Definir la velocidad máxima para cada articulación
qdmax = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

# Generar la trayectoria usando mstraj
xyz_traj = rtb.mstraj(via,qdmax, dt=0.02, tacc=0.2).q

# Graficar la trayectoria en 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(xyz_traj[:, 0], xyz_traj[:, 1], xyz_traj[:, 2])
ax.scatter(xyz_traj[0, 0], xyz_traj[0, 1], xyz_traj[0, 2], color='red', marker='*')  # Inicio
ax.scatter(xyz_traj[-1, 0], xyz_traj[-1, 1], xyz_traj[-1, 2], color='blue', marker='o')  # Final
plt.show()

# Resolver la cinemática inversa para cada punto de la trayectoria
q_traj = []
for T in T_matrices:
    sol = robot.ikine_LM(T, q0=q, mask=[1, 1, 1, 1, 1, 1])
    if sol.success:
        q_traj.append(sol.q)
    else:
        print("Cinemática inversa fallida para un punto")

q_traj = np.array(q_traj)

# Graficar la trayectoria del robot
robot.plot(q=q_traj, limits=[-1.737, 1.737, -1.737, 1.737, -2.3, 2.3], eeframe=True, backend='pyplot', shadow=True, jointaxes=False, block=True)