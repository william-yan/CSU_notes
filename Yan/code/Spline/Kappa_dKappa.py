#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May  4 20:24:09 2019

@author: self-driver
"""
import math
import numpy as np
import matplotlib.pyplot as plt

a0 = 1
a1 = 0.1
a2 = 0.01
a3 = 0.000001

t = np.linspace(0,20,200)
y = a0 + a1 * t + a2 * t**2 + a3 * t ** 3
y_d =  a1  + 2 * a2 * t +3 *  a3 * t ** 2
y_dd = 2 * a2 + 3 * 2 *  a3 * t
y_ddd = 2 * 3 * a3

a0 = 1
a1 = 0.5
a2 = 0.001
a3 = -0.001
x = a0 + a1 * t + a2 * t**2 + a3 * t ** 3
x_d =  a1  + 2 * a2 * t +3 *  a3 * t ** 2
x_dd = 2 * a2 + 3 * 2 *  a3 * t
x_ddd = 2 * 3 * a3

a = x_d*y_dd-y_d*x_dd
norm_square = x_d * x_d + y_d * y_d
norm = np.sqrt(norm_square);
b = norm * norm_square;



a = x_d * y_dd - y_d * x_dd;
b = x_d * y_ddd - y_d * x_ddd;
c = x_d * x_dd + y_d * y_dd;
d = x_d * x_d + y_d * y_d;


plt.figure(1)
plt.plot(a/b)
plt.title('kappa')
plt.figure(2)
plt.plot(x,y)
plt.axis("equal")
plt.title('x-y')
plt.figure(3)
plt.plot((b * d - 3.0 * a * c) / (d * d * d))
plt.title('dkappa')
plt.figure(3)
plt.plot(norm*(b * d - 3.0 * a * c) / (d * d * d))

plt.figure(3)
plt.plot((norm-1)*(b * d - 3.0 * a * c) / (d * d * d))
plt.legend({'kappa1','kappa2','dkappa1 - dkappa2'})
plt.figure(4)
plt.plot(t,x)
plt.title('t-x')
plt.figure(5)
plt.plot(t,y)
plt.title('t-y')





    