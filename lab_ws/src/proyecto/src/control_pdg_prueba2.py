#!/usr/bin/env python
# Importe de librerias
import rospy
from sensor_msgs.msg import JointState
from markers import *
from funciones import *
from roslib import packages
import sys
sys.path.insert(0, '/home/user/lab_ws/src/rbdl/build/python')
if (True):
    import rbdl


rospy.init_node("control_pdg")  # Se inicia el nodo
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
# define marcador para la posicion actual del efector final
bmarker_actual = BallMarker(color['RED'])
# define marcador para la posicion final del efector final
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
qdes = np.array([3.14, 0.7, 0.25, -2.4, 0.25, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0])
#q = qdes
# =============================================================

# Posicion resultante de la configuracion articular deseada
xdes = fkine(qdes[0:7])[0:3, 3]
# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('/home/user/lab_ws/src/proyecto/urdf/camara_movil_generated.urdf')
ndof = modelo.q_size     # Grados de libertad

# Frecuencia del envio (en Hz)
freq = 100
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)

# Se definen las ganancias del controlador
# Ganancia proporcional
valores = 1*np.array([2.0, 2.0, 2.0, 2.0, 4.0,
                      2.0, 2.0, 0.0, 0.0, 0.0, 0.0])
Kp = np.diag(valores)
# Ganancia derivativa
valoresd = 2*np.sqrt(np.array([2.0, 2.0, 2.0, 2.0, 4.0,
                               16.0, 0.5, 0.0, 0.0, 0.0, 0.0]))
Kd = np.diag(valoresd)
zeros = np.zeros(ndof)

t = 0.0
# vector auxiliar para guardar los valores articulares
# correspondientes al brazo robotico unicamente
qc = q[0:7]
# Bucle de ejecucion continua
while not rospy.is_shutdown():
    qc = q  # guarda valor de la iteracion anterior
    # Leer valores del simulador
    q = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Se agregan condiciones para los limites articulares
    change = 1
    # Articulaciones prismaticas
    if (q[2] < -0.25 or q[2] > 0.25):
        q[2] = qc[2]
    #xd = xdp
        change = 0
    if (q[4] < -0.25 or q[4] > 0.25):
        q[4] = qc[4]
    #xd = xdp
        change = 0
    # Articulaciones de revolucion
    if (q[0] < -6.28 or q[0] > 6.28):
        q[0] = qc[0]
    #xd = xdp
        change = 0
    if (q[1] < -6.28 or q[1] > 6.28):
        q[1] = qc[1]
    #xd = xdp
        change = 0
    if (q[3] < -6.28 or q[3] > 6.28):
        q[3] = qc[3]
    #xd = xdp
        change = 0
    if (q[5] < -6.28 or q[5] > 6.28):
        q[5] = qc[5]
    #xd = xdp
        change = 0
    if (q[6] < -6.28 or q[6] > 6.28):
        q[6] = qc[6]
    #xd = xdp
        change = 0
    if (change == 1):
        q0 = np.concatenate((qc, np.array([0, 0, 0, 0])), axis=None)
    #xdp = xd

    # Posicion actual del efector final
    x = fkine(q[0:7])[0:3, 3]
    e = xdes-x

    # Condiciones para evitar oscilaciones y disminuir el tiempo para el control
    if (np.linalg.norm(e) < 0.007):
        # Articulacion q5. Ultima articulacion de primsmatica
        if (np.linalg.norm(q[4]-qdes[4]) < 0.007):
            q[4] = qc[4]
        # Articulacion q6. Ultima articulacion de revolucion
        # antes del efector final
        if (np.linalg.norm(q[5]-qdes[5]) < 0.007):
            q[5] = qc[5]
            # Articulacion q7. Condicion para detener el control
            if (np.linalg.norm(q[6]-qdes[6]) < 0.007):
                q[6] = qc[6]
                break
        # Condicion para evitar oscilaciones innecesarias en
        # el valor articuales del efector final
        if (np.linalg.norm(q[6]-qdes[6]) < 0.007):
            q[6] = qc[6]
    # Nuevo calculo de la posicion
    x = fkine(q[0:7])[0:3, 3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' ' +
                str(q[2])+' ' + str(q[3])+' '+str(q[4])+' '+str(q[5])+' '+str(q[6])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' ' + str(
        qdes[2])+' ' + str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+' '+str(qdes[6])+'\n ')

    # Dinamica inversa
    g = np.zeros(ndof)  # Define vector de gravedad
    # Determina vector de gravedad con InverseDynamics
    rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
    u = g+Kp.dot(qdes-q)-Kd.dot(dq)  # LEY DE CONTROL

    # Simulacion del robot
    robot.send_command(u)

    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()
    # if (np.linalg.norm(e) < 0.007):
    # break

fqact.close()
fqdes.close()
fxact.close()
fxdes.close()
