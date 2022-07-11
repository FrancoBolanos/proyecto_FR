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


rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])
# Archivos donde se almacenara los datos
fqact = open("/tmp/qactual_pdg.txt", "w")
fqdes = open("/tmp/qdeseado_pdg.txt", "w")
fxact = open("/tmp/xactual_pdg.txt", "w")
fxdes = open("/tmp/xdeseado_pdg.txt", "w")

# Nombres de las articulaciones
# Joint names
jnames = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6',
          'q7', 'q01', 'q02', 'q03', 'q04']
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# =============================================================
# Configuracion articular inicial (en radianes para revolucion y metros para prismatico)
q = np.array([1., 2.2, 0.1, 0.8, 0.2, 0.2, 0., 0., 0., 0., 0.])
q0 = q
# Velocidad inicial
dq = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
# Configuracion articular deseada
qdes = np.array([-1.57, -1.57, 0.25, -1.57, 0.25, -
                1.57, -1.57, 0.0, 0.0, 0.0, 0.0])
#q = qdes
# =============================================================

# Posicion resultante de la configuracion articular deseada
xdes = fkine(qdes[0: 7])[0: 3, 3]
# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('/home/user/lab_ws/src/proyecto/urdf/camara_movil_generated.urdf')
ndof = modelo.q_size - 4     # Grados de libertad
# Se le resta 4 debido a que contiene 4 ruedas.
# entonces quedarian los 7 GDL del brazo robotico

# Frecuencia del envio (en Hz)
freq = 100
dt = 1.0/freq
rate = rospy.Rate(freq)
J = jacobian_position(q[0: 7])

# Simulador dinamico del robot
robot = Robot(q, dq, ndof + 4, dt)

# Se definen las ganancias del controlador
# Ganancia proporcional
valores = 8*np.array([0.2, 0.2, 0.2])
Kp = np.diag(valores)
# Ganancia derivativa
valoresd = 1*np.array([1.0, 2.0, 2.0, 2.0, 4.0,
                       2.0, 2.0, 0.0, 0.0, 0.0, 0.0])
Kd = np.diag(valoresd)
zeros = np.zeros(ndof)

# Bucle de ejecucion continua
t = 0.0
# Copia los 7 primeros valores de q correspondientes
# al brazo robotico
qc = q[0: 7]
while not rospy.is_shutdown():
    qc = q  # adquiere valor de la iteracion anterior
    # Leer valores del simulador
    q = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Se definen limites articulares en condicionales
    change = 1
    # Articulaciones prismaticas
    if (q[2] < -0.0 or q[2] > 0.25):
        q[2] = qc[2]
    # xd = xdp
        change = 0
    if (q[4] < -0.0 or q[4] > 0.25):
        q[4] = qc[4]
    # xd = xdp
        change = 0
    # Articulaciones de revolucion
    """Limites para articulaciones de revolucion
    if (q[0] < -6.28 or q[0] > 6.28):
        q[0] = qc[0]
    # xd = xdp
        change = 0
    # if (q[1] < -6.28 or q[1] > 6.28):
        #q[1] = qc[1]
    # xd = xdp
        change = 0
    if (q[3] < -6.28 or q[3] > 6.28):
        q[3] = qc[3]
    # xd = xdp
        change = 0
    if (q[5] < -6.28 or q[5] > 6.28):
        q[5] = qc[5]
    # xd = xdp
        change = 0
    if (q[6] < -6.28 or q[6] > 6.28):
        q[6] = qc[6]
    # xd = xdp
        change = 0
        """
    if (change == 1):
        q0 = np.concatenate((qc, np.array([0, 0, 0, 0])), axis=None)
    # xdp = xd
    # Jacobiano con la posicion actual
    J = jacobian_position(q[0: 7])
    # Posicion actual del efector final
    x = fkine(q[0: 7])[0: 3, 3]
    e = xdes-x  # Error de posicion
    x = fkine(q[0: 7])[0: 3, 3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' ' +
                str(q[2])+' ' + str(q[3])+' '+str(q[4])+' '+str(q[5])+' '+str(q[6])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' ' + str(
        qdes[2])+' ' + str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+' '+str(qdes[6])+'\n ')

    # CONTROL DINAMICO
    # Vector g de tamano igual al modelo, es decir 11
    g = np.zeros(ndof+4)
    rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
    # Se anaden ceros a la matriz jacobiana para darles las dimensiones del modelo
    # estos ceros ocuparan la posicion de las ruedas
    J1 = np.concatenate((J[0], np.array([0, 0, 0, 0])), axis=None)
    J2 = np.concatenate((J[1], np.array([0, 0, 0, 0])), axis=None)
    J3 = np.concatenate((J[2], np.array([0, 0, 0, 0])), axis=None)
    J = np.array([J1, J2, J3])
    # LEY DE CONTROL
    u = g+(J.T).dot(Kp.dot(xdes-x))-Kd.dot(dq.T)

    # Simulacion del robot
    robot.send_command(u)

    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    # Actualiza marcadores
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

fqact.close()
fqdes.close()
fxact.close()
fxdes.close()
