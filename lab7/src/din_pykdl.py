import PyKDL
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import numpy as np

if __name__ == '__main__':

  # Lectura del modelo del robot a partir de URDF (parsing)
  robot = URDF.from_xml_file('../urdf/ur5_robot.urdf')
  ok, tree = treeFromUrdfModel(robot)
  chain = tree.getChain("base_link", "ee_link")

  # Grados de libertad
  ndof = chain.getNrOfJoints()
  
  q0 = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6])	# Configuracion articular
  dq0 = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])  # Velocidad articular
  ddq0 = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5]) # Aceleracion articular
  
  # Definir los arreglos de las configuraciones articulares
  q = PyKDL.JntArray(ndof)
  dq = PyKDL.JntArray(ndof)
  ddq = PyKDL.JntArray(ndof)
    
  # Rellenar los vectores de PyKDL
  for i in range(ndof):
    q[i]=q0[i]
    dq[i]=dq0[i]
    ddq[i]=ddq0[i]
    
  # Arrays PyKDL
  zeros = PyKDL.JntArray(ndof)           	# Vector de ceros
  tau   = PyKDL.JntArray(ndof)           	# Para torque
  g     = PyKDL.JntArray(ndof)    		# Para la gravedad
  c     = PyKDL.JntArray(ndof)    		# Para el vector de Coriolis+centrifuga
  M     = PyKDL.JntSpaceInertiaMatrix(ndof)  	# Para la matriz de inercia
  
  # Indicando la direccion de la gravedad
  gravity = PyKDL.Vector(0.0, 0.0, -9.81)
  
  # Solucionando la dinamica 
  dyn_solver = PyKDL.ChainDynParam(chain, gravity)
  
  
  # Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
  # y matriz M usando PyKDL
  
  
  
  # Parte 2: Calcular vector tau usando PyKDL
  
  
  
