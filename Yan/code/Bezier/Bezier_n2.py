#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 26 21:30:26 2019

@author: self-driver
"""

import numpy as np
from matplotlib import pyplot as plt
import Bezier

    

def bezier_n_curve(points, n, nTimes=1000):   
    n = n
    np.set_printoptions(formatter={'float': '{: 0.3f}'.format})  
    tempresult = np.array([ Bezier.bezier_curve(points[i:i+n+1],nTimes) for i in range(0,len(points),n)])
    #print("points.shape is : ",points.shape)
    #print("tempresult.shape: ",tempresult.shape)
    #print("tempresult.shape: ",tempresult)  

    #  calcualte drivate
    #tempresult_d_f = np.array([ Bezier.bezier_curve(points[i:i+n],nTimes) for i in range(0,(len(points) - len(points)%n),n)])
    #print("points.shape is : ",points.shape)
    #print("tempresult_d_f.shape: ",tempresult_d_f.shape)
    
    #tempresult_d_r = np.array([ Bezier.bezier_curve(points[i+1:i+n+1],nTimes) for i in range(0,(len(points) - len(points)%n),n)])
    #print("points.shape is : ",points.shape)
    #print("tempresult_d_r.shape: ",tempresult_d_r.shape)
    #tempresult_d_ = tempresult_d_r - tempresult_d_f
    tempresult_d_ = np.array([ Bezier.bezier_curve(points[i+1:i+n+1]-points[i:i+n],nTimes) \
                             for i in range(0,(len(points) - len(points)%n),n)])
    #print("tempresult_d_.shape: ",tempresult_d_.shape)

    D = np.array([tempresult_d_[i][1]/tempresult_d_[i][0] for i in range(len(tempresult_d_))])
    #print ("D is :",D)
    
    plt.figure(2)
    for i in range(len(D)):
        plt.plot(tempresult[i][0],D[i])

    return tempresult




if __name__ == "__main__":
    plt.close()
    a0 = 1
    a1 = 0.5
    a2 = -0.01
    a3 = 0.000001
    x = np.linspace(0,20,10)
    y = a0 + a1 * x + a2 * x**2 + a3 * x ** 3
    Path = np.array([x,y])
    
    Path = np.transpose(Path)
    n = 4
    bezierPoint = bezier_n_curve(Path, n, 100)
    
    plt.figure(1)
    plt.plot(x,y,'ro')
    plt.axis("equal")
    for i in bezierPoint:
        plt.plot(i[0],i[1],'.')

    

    #plt.plot(bezierPoint[0],bezierPoint[1],'r.')
