import roboticstoolbox as rtb
import numpy as np 

print(f"Versión de roboticstoolbox: {rtb.__version__}")
print(f"Versión de numpy: {np.__version__}") 


robot=rtb.models.DH.Puma560()

#Variables articulares 
q = [0, np.deg2rad(30), -np.deg2rad(160), 0, 0, 0]

#visualizar 
#robot.plot(q, block=True, backend='pyplot')
#Si se downgradeo a matplotlib 3.8.3
robot.teach(q)


