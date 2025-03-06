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
        rtb.RevoluteDH(0.550, 0.2, alpha=np.pi/2, qlim=[np.deg2rad(-185), np.deg2rad(185)]),
        rtb.RevoluteDH(0, 0.8, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-90), np.deg2rad(120)]),
        rtb.RevoluteDH(d=0, a=0.125, alpha=np.pi/2, qlim=[np.deg2rad(-184), np.deg2rad(67)]),
        rtb.RevoluteDH(d=0.815, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(180)]),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-120), np.deg2rad(120)]),
        rtb.RevoluteDH(d=0.145, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Robot Articc6-1969-60kg", base=SE3(0,0,0))
        
        
#Alinear TCP con el brazo. 
robot.tool=SE3.OA([0,1,0],[0,0,1])
robot.configurations_str('ru') #codo derecho arriba
robot.qz=[0.0,0.0,0.0,0.0,0.0,0.0] 

#robot.teach(robot.qz)

#graficar robot 
# Preguntar al profe si los límites estan bien declarados para generar el espacio tridimensional
#robot.plot(q=robot.qz, limits=[-1.737, 1.737, -1.737, 1.737, -2.3, 2.3],eeframe=True, backend='pyplot',shadow=True,jointaxes=False,block=True)

#Preguntar al profe si estan bien declarados los puntos
T= [
    SE3(-0.45,0.45,0.35)*SE3.RPY(180,0.34,0.0,unit='deg') , #1 Abajo
    SE3(-0.45,0.45,0.50)*SE3.RPY(0.0,0.34,180,unit='deg'), #2 Arriba
    SE3(-0.45,-0.45,0.50)*SE3.RPY(0.0,0.34,180,unit='deg'), #3 Arriba
    SE3(-0.45,-0.45,0.35)*SE3.RPY(180,0.34,0.0,unit='deg'),#4 Abajo
    SE3(0.45,-0.45,0.35)*SE3.RPY(180,0.34,0.0,unit='deg'),#5 Abajo
    SE3(0.45,-0.45,0.50)*SE3.RPY(0.0,0.34,180,unit='deg'),#6 Arriba
    SE3(0.45,0.45,0.50)*SE3.RPY(0.0,0.34,180,unit='deg'),#7 Arriba
    SE3(0.45,0.45,0.35)*SE3.RPY(180,0.34,0.0,unit='deg'),#8 Abajo
    SE3(-0.45,0.45,0.35)*SE3.RPY(180,0.34,0.0,unit='deg'),#9 Abajo
]

T_matrix= np.array([p.A for p in T])

robot.plot(q=T_matrix.q, limits=[-1.737, 1.737, -1.737, 1.737, -2.3, 2.3],eeframe=True, backend='pyplot',shadow=True,jointaxes=False,block=True)

#Preguntar al profe 
via= np.empty((0,3))

for punto in T: 
    xyz= np.array(punto) 
    print(xyz)
    via= np.vstack((via,xyz))  #append filas a puntos en vía. 
    
    xyz_traj=rtb.mstraj(via,qdmax=[0.5,0.5,0.5,0.5,0.5,0.5],dt=0.02,tacc=0.2).q

fig = plt.figure() 
ax=fig.add_subplot(111,projection='3d')
plt.plot(xyz_traj[:,0], xyz_traj[:,1],xyz_traj[:,2])
ax.scatter(xyz_traj[0,0],xyz_traj[0,1],xyz_traj[0,2],color='red',marker='*') #Inicio 
ax.scatter(xyz_traj[-1,0],xyz_traj[-1,1],xyz_traj[-1,2],color='blue',marker='o') #Final   
plt.show() 

sol=robot.ikine_LM(T_matrix,"lu")
print(sol.success)

robot.plot(q=sol.q,limits=[-1.737, 1.737, -1.737, 1.737, -2.3, 2.3],eeframe=True, backend='pyplot',shadow=True,jointaxes=False,block=True)
