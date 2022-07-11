import numpy as np
from copy import copy
import rbdl

pi = np.pi


class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('/home/user/lab_ws/src/proyecto/urdf/camara_movil_generated.urdf')


    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq


def dh(d, theta, a, alpha):
    """
    Matriz de transformacion homogenea asociada a los parametros DH.
    Retorna una matriz 4x4
    """
    sth = np.sin(theta)
    cth = np.cos(theta)
    sa = np.sin(alpha)
    ca = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                  [sth,  ca*cth, -sa*cth, a*sth],
                  [0.0,      sa,      ca,     d],
                  [0.0,     0.0,     0.0,   1.0]])
    return T


def fkine(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6, q7]

    T0 es la transformada que describe la base del robot manipulador con respecto al origen.
    Debido a que el robot es un robot manipulador unido a un robot movil, la posicion y
    orientacion de su base variaran debido al desplazamiento del robot movil.
    Por cuestiones de simplificacion se incluye dentro de la funcion para pruebas con el
    robot movil detenido. Cuando se desplace el robot movil, T0 debera multiplicarse luego
    de hallar la transformada del efector final con respecto a la base o pasarse como
    """
    # Matrices DH
    T0 = dh(0.175, 0, 0, 0)
    T1 = dh(0.32, q[0]+pi, 0, pi/2)
    T2 = dh(0.28, q[1]+pi, 0, pi/2)
    T3 = dh(q[2]+0.54, 0, 0, pi/2)
    T4 = dh(0.26, q[3]+pi, 0, pi/2)
    T5 = dh(q[4]+0.54, 0, 0, pi/2)
    T6 = dh(0.12, q[5]+pi, 0, pi/2)  # paramtero de la funcion
    T7 = dh(0.55, q[6], 0, 0)
    # Efector final con respecto al origen
    T = T0.dot(T1).dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7)
    return T


def jacobian_position(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x7 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6, q7].
    El jacobiano describe la derivada parcial de la posicion del efector final
    con respecto a la variacion de cada valor articular. 
    """
    J = np.zeros((3, 7))    # Se crea la matriz para almacenar el jacobiano, una columna por valor articular, 3 filas para los 3 componentes de posicion
    # Se halla la transformacion homogenea del efector final para usar su componente de posicion
    T = fkine(q)
    # Iteracion para la derivada de cada columna
    for i in xrange(7):  # Cada iteracion es cada columna (derivada parcial con respecto a articulacion i-esima)
        dq = copy(q)    # Se copia el vector de valores articulares
        # Incrementar delta a la articulacion i-esima, con respecto a la cual se calcula la derivada parcial.
        dq[i] += delta
        # Se halla la matriz de transformacion homogenea del efector final usando el vector articular con la variacion delta en la i-esima articulacion
        T_i = fkine(dq)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas (variacion de los componentes de posicion entre variacion del i-esimo valor articular)
        J[:, i] = (T_i[0:3, 3]-T[0:3, 3])/delta
    return J


def ikine(xdes, q0):
    epsilon = 0.001  # Error maximo con el que se detiene el calculo iterativo
    # Numero de iteraciones maximas en caso de no conseguir un error menor a epsilon
    max_iter = 1000
    delta = 0.00001  # Variacion angular en radianes para el calculo del jacobiano

    # Se abren los archivos en los que se guardan los valores del proceso iterativo
    fqiter = open(
        "/home/user/lab_ws/src/proyecto/logs/inverse_kinematics/NR/qiteracion.dat", "w")
    fxiter = open(
        "/home/user/lab_ws/src/proyecto/logs/inverse_kinematics/NR/xiteracion.dat", "w")
    fxdes = open(
        "/home/user/lab_ws/src/proyecto/logs/inverse_kinematics/NR/xdeseado.dat", "w")
    fxerr = open(
        "/home/user/lab_ws/src/proyecto/logs/inverse_kinematics/NR/xerror.dat", "w")
    ferr = open(
        "/home/user/lab_ws/src/proyecto/logs/inverse_kinematics/NR/error.dat", "w")

    # Se copia el valor del q inicial con el que se empieza el proceso iterativo
    q = copy(q0)
    for i in range(max_iter):  # Proceso iterativo para el calculo de valores articulares (cada iteracion se aproxima mas al valor deseado). Si se excede el limite de iteraciones, devuelve los valores articulares que se obtuvieron
        # Se calcula la transformada homogenea del efector final
        f = fkine(q)
        # Se extrae el componente de posicion de la transformada homogenea del efector final
        f = f[0:3, 3]
        e = xdes-f            # Se calcula el vector de error para x, y, z
        # Se halla el jacobiano de posicion (matriz de derivadas parciales de componentes de posicion con respecto a la variacion de valores articulares)
        J = jacobian_position(q, delta)
        # Se calcula el valor del nuevo q de acuerdo a la ecuacion 4.6 del informe, mas cercano al valor deseado
        q = q + np.dot(np.linalg.pinv(J), e)

        # Las articulaciones rotacionales no tienen limite de posicion angular
        # Limites para articulaciones prismaticas:
        if q[2] >= 0.25:
            q[2] = 0.25
        if q[2] <= 0:
            q[2] = 0
        if q[4] >= 0.25:
            q[4] = 0.25
        if q[4] <= 0:
            q[4] = 0

        # Se guardan los valores de la iteracion
        fqiter.write(str(i)+' '+str(q[0])+' '+str(q[1])+' '+str(q[2]) +
                     ' '+str(q[3])+' '+str(q[4])+' '+str(q[5])+' '+str(q[6])+'\n')
        fxiter.write(str(i)+' '+str(f[0])+' '+str(f[1])+' '+str(f[2])+'\n')
        fxdes.write(str(i)+' '+str(xdes[0]) +
                    ' '+str(xdes[1])+' '+str(xdes[2])+'\n')
        fxerr.write(str(i)+' '+str(e[0])+' '+str(e[1])+' '+str(e[2])+'\n')
        ferr.write(str(i)+' '+str(np.linalg.norm(e))+'\n')

        # Condicion de termino
        # Se calcula la norma del error, que es la distancia en linea recta entre la posicion deseada y la posicion actual obtenida mediante los valores articulares. Si esta distancia es menor a epsilon, se considera que el efector final se encuentra en la posicion deseada
        if (np.linalg.norm(e) < epsilon):
            break
    return q


def ik_gradient(xdes, q0):
    epsilon = 0.001  # Error maximo con el que se detiene el calculo iterativo
    # Numero de iteraciones maximas en caso de no conseguir un error menor a epsilon
    max_iter = 1000
    delta = 0.00001  # Variacion angular en radianes para el calculo del jacobiano
    alpha = 0.5  # Tamano de paso para la gradiente. Un valor mas alto permite convergencia en menor tiempo, pero puede generar oscilacion alrededor del valor deseado

    # Se abren los archivos en los que se guardan los valores del proceso iterativo
    fqiter = open(
        "/home/user/lab_ws/src/proyecto/logs/inverse_kinematics/gradiente/qiteracion.dat", "w")
    fxiter = open(
        "/home/user/lab_ws/src/proyecto/logs/inverse_kinematics/gradiente/xiteracion.dat", "w")
    fxdes = open(
        "/home/user/lab_ws/src/proyecto/logs/inverse_kinematics/gradiente/xdeseado.dat", "w")
    fxerr = open(
        "/home/user/lab_ws/src/proyecto/logs/inverse_kinematics/gradiente/xerror.dat", "w")
    ferr = open(
        "/home/user/lab_ws/src/proyecto/logs/inverse_kinematics/gradiente/error.dat", "w")

    # Se copia el valor del q inicial con el que se empieza el proceso iterativo
    q = copy(q0)
    for i in range(max_iter):  # Proceso iterativo para el calculo de valores articulares (cada iteracion se aproxima mas al valor deseado). Si se excede el limite de iteraciones, devuelve los valores articulares que se obtuvieron
        # Se calcula la transformada homogenea del efector final
        f = fkine(q)
        # Se extrae el componente de posicion de la transformada homogenea del efector final
        f = f[0:3, 3]
        e = xdes-f            # Se calcula el vector de error para x, y, z
        # Se halla el jacobiano de posicion (matriz de derivadas parciales de componentes de posicion con respecto a la variacion de valores articulares)
        J = jacobian_position(q, delta)
        # Se calcula el valor del nuevo q de acuerdo a la ecuacion 4.11 del informe, mas cercano al valor deseado
        q = q + alpha*np.dot(J.T, e)

        # Las articulaciones rotacionales no tienen limite de posicion angular
        # Limites para articulaciones prismaticas:
        if q[2] >= 0.25:
            q[2] = 0.25
        if q[2] <= 0:
            q[2] = 0
        if q[4] >= 0.25:
            q[4] = 0.25
        if q[4] <= 0:
            q[4] = 0

        # Se guardan los valores de la iteracion
        fqiter.write(str(i)+' '+str(q[0])+' '+str(q[1])+' '+str(q[2]) +
                     ' '+str(q[3])+' '+str(q[4])+' '+str(q[5])+' '+str(q[6])+'\n')
        fxiter.write(str(i)+' '+str(f[0])+' '+str(f[1])+' '+str(f[2])+'\n')
        fxdes.write(str(i)+' '+str(xdes[0]) +
                    ' '+str(xdes[1])+' '+str(xdes[2])+'\n')
        fxerr.write(str(i)+' '+str(e[0])+' '+str(e[1])+' '+str(e[2])+'\n')
        ferr.write(str(i)+' '+str(np.linalg.norm(e))+'\n')

        # Condicion de termino
        # Se calcula la norma del error, que es la distancia en linea recta entre la posicion deseada y la posicion actual obtenida mediante los valores articulares. Si esta distancia es menor a epsilon, se considera que el efector final se encuentra en la posicion deseada
        if (np.linalg.norm(e) < epsilon):
            break
    return q
