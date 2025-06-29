import rbdl
import numpy as np

if __name__ == '__main__':

  # Lectura del modelo del robot a partir de URDF (parsing)
  modelo = rbdl.loadModel('../urdf/ur5_robot.urdf')
  # Grados de libertad
  ndof = modelo.q_size

  # Configuracion articular
  q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6])
  # Velocidad articular
  dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
  # Aceleracion articular
  ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])
  
  # Arrays numpy
  zeros = np.zeros(ndof)          # Vector de ceros
  tau   = np.zeros(ndof)          # Para torque
  g     = np.zeros(ndof)          # Para la gravedad
  c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
  M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
  e     = np.eye(6)               # Vector identidad
  
  # Torque dada la configuracion del robot
  rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
  
  # Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
  # y matriz M usando solamente InverseDynamics
