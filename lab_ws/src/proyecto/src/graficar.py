#!/usr/bin/env python3
# coding=utf-8

'''
NOTA: ESTE CODIGO FUE CORRIDO EN VISUAL STUDIO CODE. PARA GENERAR LAS GRAFICAS, COLOCAR EL CODIGO
EN UNA CARPETA EN Y COLOCAR LOS ARCHIVOS DE DATA GENERADOS POR LOS METODOS DE CINEMATICA INVERSA
EN LA MISMA CARPETA. ESTA DATA SE ENCUENTRA EN "proyecto/logs"
'''

import matplotlib.pyplot as plt
import numpy as np

iter = []
q1 = []
q2 = []
q3 = []
q4 = []
q5 = []
q6 = []
q7 = []
for line in open('qiteracion.dat', 'r'):
    lines = [i for i in line.split()]
    iter.append(float(lines[0]))
    q1.append(float(lines[1]))
    q2.append(float(lines[2]))
    q3.append(float(lines[3]))
    q4.append(float(lines[4]))
    q5.append(float(lines[5]))
    q6.append(float(lines[6]))
    q7.append(float(lines[7]))

x = []
y = []
z = []
for line in open('xiteracion.dat', 'r'):
    lines = [i for i in line.split()]
    x.append(float(lines[1]))
    y.append(float(lines[2]))
    z.append(float(lines[3]))

xd = []
yd = []
zd = []
for line in open('xdeseado.dat', 'r'):
    lines = [i for i in line.split()]
    xd.append(float(lines[1]))
    yd.append(float(lines[2]))
    zd.append(float(lines[3]))

xe = []
ye = []
ze = []
for line in open('xerror.dat', 'r'):
    lines = [i for i in line.split()]
    xe.append(float(lines[1]))
    ye.append(float(lines[2]))
    ze.append(float(lines[3]))

e = []
for line in open('error.dat', 'r'):
    lines = [i for i in line.split()]
    e.append(float(lines[1]))

plt.subplots_adjust(left=None, bottom=None, right=None,
                    top=None, wspace=None, hspace=0.5)

plt.subplot(3, 1, 1)
plt.scatter(iter, x, s=5)
plt.scatter(iter, xd, s=5)
plt.title("x")
plt.ylabel("Posición (m)")
plt.xlabel("Iteración")

plt.subplot(3, 1, 2)
plt.scatter(iter, y, s=5)
plt.scatter(iter, yd, s=5)
plt.title("y")
plt.ylabel("Posición (m)")
plt.xlabel("Iteración")

plt.subplot(3, 1, 3)
plt.scatter(iter, z, s=5)
plt.scatter(iter, zd, s=5)
plt.title("z")
plt.ylabel("Posición (m)")
plt.xlabel("Iteración")

plt.suptitle("Evolución temporal cartesiana")

plt.show()

plt.subplots_adjust(left=None, bottom=None, right=None,
                    top=None, wspace=None, hspace=0.3)

plt.subplot(2, 2, 1)
plt.scatter(iter, xe, s=5)
plt.title("xd-x")
plt.ylabel("Error de posición (m)")
plt.xlabel("Iteración")

plt.subplot(2, 2, 2)
plt.scatter(iter, ye, s=5)
plt.title("yd-y")
plt.ylabel("Error de posición (m)")
plt.xlabel("Iteración")

plt.subplot(2, 2, 3)
plt.scatter(iter, ze, s=5)
plt.title("zd-z")
plt.ylabel("Error de posición (m)")
plt.xlabel("Iteración")

plt.subplot(2, 2, 4)
plt.scatter(iter, e, s=5)
plt.title("Error")
plt.ylabel("Error")
plt.xlabel("Iteración")

plt.suptitle("Evolución temporal del error")

plt.show()

plt.subplots_adjust(left=None, bottom=None, right=None,
                    top=None, wspace=None, hspace=0.5)

plt.subplot(3, 3, 1)
plt.scatter(iter, q1, s=5)
plt.title("q1")
plt.ylabel("Ángulo (rad)")
plt.xlabel("Iteración")

plt.subplot(3, 3, 2)
plt.scatter(iter, q2, s=5)
plt.title("q2")
plt.ylabel("Ángulo (rad)")
plt.xlabel("Iteración")

plt.subplot(3, 3, 3)
plt.scatter(iter, q3, s=5)
plt.title("q3")
plt.ylabel("Desplazamiento (m)")
plt.xlabel("Iteración")

plt.subplot(3, 3, 4)
plt.scatter(iter, q4, s=5)
plt.title("q4")
plt.ylabel("Ángulo (rad)")
plt.xlabel("Iteración")

plt.subplot(3, 3, 5)
plt.scatter(iter, q5, s=5)
plt.title("q5")
plt.ylabel("Desplazamiento (m)")
plt.xlabel("Iteración")

plt.subplot(3, 3, 6)
plt.scatter(iter, q6, s=5)
plt.title("q6")
plt.ylabel("Ángulo (rad)")
plt.xlabel("Iteración")

plt.subplot(3, 3, 7)
plt.scatter(iter, q7, s=5)
plt.title("q7")
plt.ylabel("Ángulo (rad)")
plt.xlabel("Iteración")

plt.suptitle("Evolución temporal de las articulaciones")

plt.show()
