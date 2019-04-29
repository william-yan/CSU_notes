# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Apr 15 09:52:14 2019

@author: SelfDriving
"""

import math
import numpy as np
import pylab as pl
from scipy import interpolate
import matplotlib.pyplot as p
import bisect


N=3

class Spline:
    u"""
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)    #dx

        # calc coefficient c
        self.a = [iy for iy in y]
        
        # calc coefficient c

        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        u"""
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)         #找的t属于s坐标系的哪一段
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def __search_index(self, x):
        u"""
        search data segment index
        """
        idx = bisect.bisect(self.x, x)  #二分查找数值将会插入的位置并返回，而不会插入
        result = idx - 1
        return result

    def __calc_A(self, h):
        u"""
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        u"""
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        #  print(B)
        return B


class Spline2D:
    u"""
    2D Cubic Spline class
    """

    def __init__(self, x, y):         #构造函数
        self.s = self.__calc_s(x, y)
        #self.s=[i for i in range(len(x))]
        #print(self.s)
        self.sx = Spline(self.s, x)    #对（s,x）进行三次样条插值
        #print(self.sx)
        self.sy = Spline(self.s, y)   #对（s,y）进行三次样条插值
        #print(self.sy)

    def __calc_s(self, x, y):      #计算距离s值
        dx = np.diff(x)                            #[2.5, 2.5, 2.5, 2.5, -4.5, -4.]
        dy = np.diff(y)                            #[-6.7, 11., 1.5, -6.5, 5., -7.]
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)  #ds
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):   #通过s值计算xy
        u"""
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)
        return x, y

def bspline_planning(x, y, sn, t):    #2D B-Spline path
    x = np.array(x)
    y = np.array(y)
    t=np.array(t)
    #t = range(len(x))
    x_tup = interpolate.splrep(t, x, k=N)
    y_tup = interpolate.splrep(t, y, k=N)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]      #将插值样条转换为逼近样条？？？

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, t[-1], sn)
    rx = interpolate.splev(ipl_t, x_list)
    ry = interpolate.splev(ipl_t, y_list)

    return rx, ry


def generate_target_course(x, y):  
    csp = Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry = [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)

    return rx, ry, csp

def main():
    print("Spline 2D test")
    
    x = [0.0, 10.0, 20.5, 35.0, 70.5]
    y = [0.0, -6.0, 5.0, 6.5, 0.0]
    #对（x,y）进行1D三次样条插值
    flag=True
    for i in range(len(x)-1):
        if x[i+1]<=x[i]:
            flag=False
            break
    if flag:
        t=interpolate.splrep(x,y,k=3)
        x0=np.linspace(0,x[-1],1000)
        y0=interpolate.splev(x0,t)
        flg, ax = p.subplots(1)
        p.plot(x,y,color='blue',label="waypoints")
        p.plot(x0,y0,color='green',label="1D B-Spline path")
#        tt=list(t)
#        tt[1]=y + [0.0, 0.0, 0.0, 0.0]
#        y00=interpolate.splev(x0,np.array(tt))
#        p.plot(x0,y00,color='black',label="1D B-Spline path2")
        p.legend()
        p.show() 
    
    #利用自定义函数对（s,x）(s,y)进行2D三次样条插值
    tx, ty, csp = generate_target_course(x, y)
    flg, ax = p.subplots(1)
    p.plot(tx,ty,color='red',label="custom 2D B-Spline path by s")
    p.plot(x,y,color='blue',label="waypoints")
    p.legend()
    p.show()
    
    #利用库函数对（s,x）(s,y)进行2D三次样条插值
    flg, ax = p.subplots(1)
#    rx, ry = bspline_planning(x, y, 1000, [tt for tt in range(len(x))])
#    p.plot(rx, ry, color='orange', label="2D B-Spline path by len(x)")
#    rx1, ry1 = bspline_planning(x, y, 1000, csp.s)
#    p.plot(rx1, ry1, color='green', label="2D B-Spline path by s")
    
    t1=interpolate.splrep(csp.s,x,k=3)
    t2=interpolate.splrep(csp.s,y,k=3)
    t0=np.linspace(0,csp.s[-1],1000)
    x1=interpolate.splev(t0,t1)
    y1=interpolate.splev(t0,t2)
    p.plot(x1,y1,color='black', label="2D B-Spline path by s")
    
    t11=interpolate.splrep([tt for tt in range(len(x))],x,k=3)
    t21=interpolate.splrep([tt for tt in range(len(x))],y,k=3)
    t01=np.linspace(0,len(x)-1,1000)
    x11=interpolate.splev(t01,t11)
    y11=interpolate.splev(t01,t21)
    p.plot(x11,y11,color='red', label="2D B-Spline path by index")
    p.plot(x,y,color='blue',label="waypoints")  
    p.legend()
    p.show()
    
    
if __name__ == '__main__':
    main()