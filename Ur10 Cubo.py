import sympy as sp
from sympy.matrices import rot_axis3
# Para el ejemplo donde generamos la matriz DH
from spatialmath import *
from spatialmath.base import *
# Para poder Graficar
import matplotlib.pyplot as plt
import numpy as np
# Para usar el DH
import roboticstoolbox as rtb
from codigoVerTrayect import plot_robot_trajectory

np.set_printoptions(suppress=True, precision=4)

# Matriz DH del robot Ur10


robot=rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.128, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        rtb.RevoluteDH(d=0.176, a=0.612, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        rtb.RevoluteDH(d=-0.128, a=0.572, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        rtb.RevoluteDH(d=0.116, a=0.0, alpha=np.pi/2, offset=np.pi/2, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        rtb.RevoluteDH(d=0.116, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        rtb.RevoluteDH(d=0.092, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Ur10", base=SE3(0,0,0))


# Alinear TCP con el brazo.
robot.tool = SE3.OA([0, 1, 0], [0, 0, 1])
robot.configurations_str('ru')  # codo derecho arriba
robot.qz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# robot.teach(robot.qz)

# graficar robot
robot.plot(q=robot.qz, limits=[-1.737, 1.737, -1.737, 1.737, -2.3, 2.3])
           #eeframe=True, backend='pyplot', shadow=True, jointaxes=False, block=True)

# Tabla de puntos de cubo 
T = [
    SE3(-0.25, 0.25, 0.35)*SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 1 Abajo
    SE3(-0.25, 0.25, 0.90)*SE3.RPY(180, 0.34, 0, unit='deg'),  # 2 Arriba
    SE3(-0.25, -0.25, 0.90)*SE3.RPY(180, 0.34, 180, unit='deg'),  # 3 Arriba
    SE3(-0.25, -0.25, 0.35)*SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 4 Abajo
    SE3(0.25, -0.25, 0.35)*SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 5 Abajo
    SE3(0.25, -0.25, 0.90)*SE3.RPY(180, 0.34, 180, unit='deg'),  # 6 Arriba
    SE3(0.25, 0.25, 0.90)*SE3.RPY(180, 0.34, 180, unit='deg'),  # 7 Arriba
    SE3(0.25, 0.25, 0.35)*SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 8 Abajo
    SE3(-0.25, 0.25, 0.35)*SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 9 Abajo
    SE3(-0.25, 0.25, 0.90)*SE3.RPY(180, 0.34, 180, unit='deg'),  # 2 Arriba
    SE3(0.25, 0.25, 0.90)*SE3.RPY(180, 0.34, 180, unit='deg'),  # 7 Arriba
    SE3(0.25, 0.25, 0.35)*SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 8 Abajo
    SE3(0.25, -0.25, 0.35)*SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 5 Abajo
    SE3(0.25, -0.25, 0.90)*SE3.RPY(180, 0.34, 180, unit='deg'),  # 6 Arriba
    SE3(-0.25, -0.25, 0.90)*SE3.RPY(180, 0.34, 180, unit='deg'),  # 3 Arriba
    SE3(-0.25, -0.25, 0.35)*SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 4 Abajo
    SE3(-0.25, 0.25, 0.35)*SE3.RPY(180, 0.34, 0.0, unit='deg'),  # 1 Abajo
]

T_matrix = np.array([p.A for p in T])
print(type(T))
T_prueba = SE3.Trans(0, 0, 0.2)
T_cubo = [T_prueba * pose for pose in T]

q = [0, np.deg2rad(22.3), np.deg2rad(31.7), 0, np.deg2rad(0), 0]


T_angulos = robot.ikine_LM(np.array(T_cubo), q0=q).q


p_lim = [-1, 1, -1, 1, -0.15, 1.5]
plot_robot_trajectory(
    robot=robot,
    q_trajectory=np.array(T_angulos),
    limits=p_lim,
    eeframe=True,
    jointaxes=False,
    shadow=True,
    drawing_mode='continuous',  # o 'segments' si prefieres
    traj_color='r',             # Color de la trayectoria completa
    drawing_color='b',          # Color del trazo principal
    dt=1,
    block=True
)

#Primer método termina acá


# Segundo método empieza aquí


traj = []
for i in range(len(T)-1):
    segment = rtb.ctraj(T[i], T[i+1], 5)  # 5 puntos entre cada segmento
    traj.extend(segment)

qtraj = []
total_poses = len(traj)
q0 = robot.qz

for i, pose in enumerate(traj):
    # Mostrar progreso en la misma linea
    animation = "|/-\\"
    print(
        f"\rProgreso:{i+1}/{total_poses} ({(i+1)/total_poses*100:.1f}%) {animation[i % 4]}", end="")
    sol = robot.ikine_LM(pose, q0=q0, tol=1e-5, ilimit=500, slimit=25)

    if sol.success:
        qtraj.append(sol.q)
        q0 = sol.q

    else:
        print(f"\nError: Pose no alcanzable en paso {i+1}: {pose.t}")
        raise ValueError(f"No se alcanzó la pose {i+1} Cancelando ejecución ")

if not qtraj:
    raise ValueError("No se encontró configuración valida para qtraj")

qtraj = np.array(qtraj)


# Verificar que se hayan encontrado configuraciones válidas


plot_robot_trajectory(
    robot=robot,
    q_trajectory=qtraj,
    limits=p_lim,
    eeframe=True,
    jointaxes=False,
    shadow=True,
    drawing_mode='continuous',  # o 'segments' si prefieres
    traj_color='r',             # Color de la trayectoria completa
    drawing_color='b',          # Color del trazo principal
    dt=0.2,
    block=True
)

# Aca termina el segundo método
# Aca empieza el punto 2: Trayectoria en espacio articular. 

T2 = np.array([
    [0, 0, 0, 0, 0, 0],
    [25, 30, 35, 40, 45, 50],
    [20, 45, 70, 15, 22, 33],
    [34, 45, 56, 67, 78, 88],
    [10, 12, 14, 16, 18, 19],
    [4, 16, 60, 43, 46, 3], 
])
T2 = np.array([
    [0, 0, 0, 0, 0, 0],
    [np.deg2rad(25), np.deg2rad(30), np.deg2rad(35), np.deg2rad(40), np.deg2rad(45), np.deg2rad(50)],
    [np.deg2rad(20), np.deg2rad(45), np.deg2rad(70), np.deg2rad(15), np.deg2rad(22), np.deg2rad(33)],
    [np.deg2rad(34), np.deg2rad(45), np.deg2rad(56), np.deg2rad(67), np.deg2rad(78), np.deg2rad(88)],
    [np.deg2rad(10), np.deg2rad(12), np.deg2rad(14), np.deg2rad(16), np.deg2rad(18), np.deg2rad(19)],
    [np.deg2rad(4), np.deg2rad(16), np.deg2rad(60), np.deg2rad(43), np.deg2rad(46), np.deg2rad(3)], 
])
jointraj = []
for i in range(len(T2)-1):
    #Valor inicial incrementa con i 
    inicio = i* 0.5
    # Valor final incrementa con i 
    fin=(i+2) 
    
    t=np.arange(inicio,fin,0.5) # rango de 0 a 2 con intervalos de 50 milisegundos
    segment = rtb.jtraj(T2[i], T2[i+1], 20)  # 20 puntos entre cada segmento
    jointraj.append(segment.q)


jointraj = np.vstack(jointraj)
print("Jointraj")
print (len(jointraj))
print ((jointraj[0]))

    
plot_robot_trajectory(
   robot=robot,
    q_trajectory=jointraj,
    limits=p_lim,
    eeframe=True,
    jointaxes=False,
    shadow=True,
    drawing_mode='continuous',  # o 'segments' si prefieres
    traj_color='r',             # Color de la trayectoria completa
    drawing_color='b',          # Color del trazo principal
    dt=0.2,
    block=True
)
    