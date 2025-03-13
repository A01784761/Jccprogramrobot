import numpy as np
import scipy as sp
import sympy as sym
import math

proporción=1*10**4
atotal_mm = proporción*np.array([1.8692, 1.7838, 0.7054]) # aceleración mm/seg^2

atotal_m=atotal_mm/1000 # aceleración m/seg^2



#print(atotal_m)

#0.368 kg

mass_kg= 0.368 # en kg

gravity= 9.75 #CDMX en m/seg^2

z_vector= np.array([0,0,1])

F_gripper_v= (mass_kg * atotal_m)+(mass_kg *gravity *z_vector) # Fuerza total en x,y,z
print("Vector de fuerza en x,y,z: ",F_gripper_v, "Newtons")



F_max_gripper= math.sqrt(F_gripper_v[0]**2 +F_gripper_v[1]**2+F_gripper_v[2]**2) # Magnitud de la fuerza total
print ("Magnitud de la fuerza total: ", F_max_gripper, "Newtons")








