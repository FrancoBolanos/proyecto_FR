#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from funciones import *
from roslib import packages

import sys
sys.path.insert(0, '/home/user/lab_ws/src/rbdl/build/python')

if (True):
    import rbdl


# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('/home/user/lab_ws/src/proyecto/urdf/camara_movil_generated.urdf')
# Grados de libertad
ndof = modelo.q_size

# Configuracion articular
# (en radianes para revolucion y metros para prismatico)
q = np.array([1., 2.2, 0.1, 0.8, 0.2, 0.2, 0., 0., 0., 0., 0.])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0, 1.0, 0., 0., 0., 0.])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5, 0.5, 0., 0., 0., 0.])

# Definicion de vectores y matrices utilizadas en el modelo dinamico
zeros = np.zeros(ndof)          # Vector de ceros
tau = np.zeros(ndof)          # Para torque
g = np.zeros(ndof)          # Para la gravedad
c = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M = np.zeros([ndof, ndof])  # Para la matriz de inercia
e = np.eye(11)               # Vector identidad
b = np.zeros(ndof)          # Para la gravedad


# Torque dada la configuracion del robot
# Parte 1: Calcular vector de torques, vector de gravedad
# efectos no lineales y vector de coriolis
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
print("Vector de torques")
print(tau)
# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics
# Vector de gravedad
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
print("Vector de gravedad")
print(g)
# Vector de coriolis
rbdl.InverseDynamics(modelo, q, dq, zeros, b)
print("Efectos no lineales")
print(b)
print("Vector de coriolis")
c = b-g
print(c)

# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm. El resultado se almacena en M
M = np.zeros([ndof, ndof])  # Para matriz de inercia
# Matriz de inercia
rbdl.CompositeRigidBodyAlgorithm(modelo, q, M)
print("M:")
print(M)

# Parte 3: Vector de torques calculado de manera explicita
print("Vector de torques calculado de manera explicita")
print(M.dot(ddq)+c+g)
